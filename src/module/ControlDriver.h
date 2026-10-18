// Drives a control with one of the profile methods (AircraftProfile.h). The control setup uses
// it to try a method, and playback uses it for an aircraft whose profile has one, so a method
// that worked in the setup works the same way in a replay.
#pragma once

#include <MSFS/MSFS_WindowsTypes.h>
#include <MSFS/MSFS_Vars.h>
#include <SimConnect.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "AircraftProfile.h"

namespace br {

struct InputEventInfo {
    std::string name;
    std::string upper;  // for case-insensitive matching
    uint64_t hash;
};

class ControlDriver {
public:
    struct Context {
        HANDLE h = 0;
        const Frame* live = nullptr;  // the latest values from the sim
        const int* engines = nullptr;
        const int* flapPositions = nullptr;
        const double* throttleLowerLimit = nullptr;  // percent, negative with reversers
        const std::vector<InputEventInfo>* inputEvents = nullptr;
        const ChannelMask* clipChannels = nullptr;  // the playing clip's channels (empty otherwise)
    };
    void setContext(const Context& c) { ctx_ = c; }

    // Resolves what the method needs on this aircraft (its preset, L:var, simvars...). False,
    // with the reason, if it can't be used.
    bool begin(Control c, const ControlMethod& m, std::string& why);
    // The key-event throttle on an aircraft whose lever simvar isn't the lever: the clip's lever
    // variable (lv_thr_N) is turned back into the axis value through the setup's calibration.
    // After begin(); the clip must have recorded that variable (clipHasLever).
    void setLever(const LeverCalibration& cal);
    // How this aircraft's reverse thrust is commanded (the setup's sweep found it), for the
    // key-event throttle: "hold" holds THROTTLE_DECR while the clip's reversers are out; "events"
    // also puts the reverser in when the clip reversed without the reverse flag. After begin().
    void setReverseBy(ReverseBy by);
    // One THROTTLE_DECR, as a held "decrease throttle" key sends every frame (the setup's test).
    void throttleDecrease();
    // Drives the control from the frame's channels (methodSourceChannel). Values are sent when
    // they change, and every few frames anyway; `force` sends them now.
    void apply(Control c, const Frame& f, bool force);
    // The control setup, after trying an L:var method: put the L:var back as it was, so a
    // variable that only animates one panel (the PMDG's NGXLeftInbdTrailFlaps) isn't left moved.
    void restoreLVars(Control c);
    void end(Control c);
    void endAll();
    bool active(Control c) const { return slots_[static_cast<int>(c)].active; }
    // The throttle is driven through its simvars, so the hardware throttle has to be blocked.
    bool wantsThrottleMask() const;

private:
    static constexpr int kResendFrames = 15;
    struct SimVarTarget {
        int channel = -1;
        FsAVarId id = FS_VAR_INVALID_ID;
        FsUnitId unit = FS_INVALID_UNIT;
        unsigned index = 0;  // simvar index (":1"), 0 = none
    };
    struct Slot {
        bool active = false;
        ControlMethod method;
        int count = 1;  // engines (throttle), else 1
        FsBVarId setId[4] = {FS_VAR_INVALID_ID, FS_VAR_INVALID_ID, FS_VAR_INVALID_ID, FS_VAR_INVALID_ID};
        uint64_t hash[4] = {};
        FsLVarId lvar[4] = {FS_VAR_INVALID_ID, FS_VAR_INVALID_ID, FS_VAR_INVALID_ID, FS_VAR_INVALID_ID};
        double original[4] = {NAN, NAN, NAN, NAN};  // the L:vars when the method began
        SimVarTarget simvars[4];
        int simvarCount = 0;
        double sent[4] = {NAN, NAN, NAN, NAN};
        int since[4] = {};
        int errors = 0;
        int sends = 0;  // values sent, logged at the end (key events should be rare)
        LeverCalibration lever;  // throttle key events: replayed from the lever variable
        bool useLever = false;
        int leverFrames = 0;  // per engine, driven from the lever variable
        ReverseBy reverseBy = ReverseBy::Unknown;
        bool holding = false;     // THROTTLE_DECR held for reverse
        int holdFrames = 0;
        bool engaged[4] = {};     // the reverser put in by us (the clip reversed without its flag)
        int engageWait[4] = {};   // frames until it's put in again if the aircraft hasn't taken it
    };
    // Whether value v for output i is due (changed, forced or time to resend); records it.
    static bool due(Slot& s, int i, double v, bool force);
    // For key events, which act like moving the hardware: due only when forced or when v moved at
    // least minStep from what was last sent. No periodic resend, and no creeping in tiny steps:
    // every event makes the aircraft re-evaluate its lever.
    static bool dueOnChange(Slot& s, int i, double v, bool force, double minStep);
    // Like dueOnChange, for a value that isn't quite the lever: a change of nowStep or more is due
    // at once, one of minStep or more once it has lasted `frames` frames.
    static bool dueWhenLasting(Slot& s, int i, double v, bool force, double minStep, double nowStep, int frames);
    static constexpr int kLastingFrames = 30;
    void applyKeyEvent(Control c, Slot& s, const Frame& f, bool force);
    static constexpr int kNotchFrames = 8;       // FLAPS_INCR/DECR: at least this far apart
    static constexpr int kNotchWaitFrames = 60;  // and waiting this long for the handle to show it
    void stepFlaps(Slot& s, double target, bool force);
    static void keyEvent(int id, int32_t value);

    Context ctx_;
    Slot slots_[kControlCount];
    FsUnitId number_ = FS_INVALID_UNIT;
};

}  // namespace br
