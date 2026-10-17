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
    };
    void setContext(const Context& c) { ctx_ = c; }

    // Resolves what the method needs on this aircraft (its preset, L:var, simvars...). False,
    // with the reason, if it can't be used.
    bool begin(Control c, const ControlMethod& m, std::string& why);
    // Drives the control from the frame's channels (methodSourceChannel). Values are sent when
    // they change, and every few frames anyway; `force` sends them now.
    void apply(Control c, const Frame& f, bool force);
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
        SimVarTarget simvars[4];
        int simvarCount = 0;
        double sent[4] = {NAN, NAN, NAN, NAN};
        int since[4] = {};
        int errors = 0;
    };
    // Whether value v for output i is due (changed, forced or time to resend); records it.
    static bool due(Slot& s, int i, double v, bool force);
    void applyKeyEvent(Control c, Slot& s, const Frame& f, bool force);
    static void keyEvent(int id, int32_t value);

    Context ctx_;
    Slot slots_[kControlCount];
    FsUnitId number_ = FS_INVALID_UNIT;
};

}  // namespace br
