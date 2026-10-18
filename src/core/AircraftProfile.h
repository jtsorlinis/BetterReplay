// Per-aircraft control profiles: how playback drives the spoilers, flaps, throttle and gear on a
// given aircraft. The guided control setup (ControlSetup.h) builds them by watching the pilot
// move each control, replaying it with each candidate method and keeping the one that worked.
// They're kept in aircraft.json in the work folder.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Channels.h"

namespace br {

class JsonWriter;
struct JsonValue;

// The controls the setup teaches, in the order it asks for them.
enum class Control : uint8_t { Spoilers = 0, Flaps = 1, Throttle = 2, Gear = 3 };
constexpr int kControlCount = 4;

// How playback drives a control.
enum class MethodKind : uint8_t {
    None = 0,
    InputSet,    // the input event's "<NAME>_SET" preset (fsVarsBVarCall), the cockpit's own path;
                 // the argument is value * scale
    InputValue,  // SimConnect_SetInputEvent with the recorded value
    LVar,        // the aircraft's L:var, set to the recorded value
    SimVar,      // the control's simvars, written every frame
    KeyEvent,    // key events, like hardware bindings send. `var` names the family: empty for
                 // THROTTLEn_AXIS_SET_EX1, FLAPS_SET, SPOILERS_SET, GEAR_SET/UP/DOWN, or one of the
                 // kKey* variants below
};

// KeyEvent variants (ControlMethod::var), for aircraft that ignore FLAPS_SET and SPOILERS_SET but
// take what their hardware bindings send (the PMDG 737).
constexpr const char* kKeyFlapsNotches = "FLAPS_INCR";       // FLAPS_INCR/FLAPS_DECR, a notch at a time
constexpr const char* kKeyFlapsAxis = "AXIS_FLAPS_SET";      // -16383 (up) .. 16383 (full)
constexpr const char* kKeySpoilersAxis = "AXIS_SPOILER_SET";  // -16383 (stowed) .. 16383 (full)

struct ControlMethod {
    MethodKind kind = MethodKind::None;
    // InputSet/InputValue: the input event's name. LVar: the L:var's name. For the throttle, '#'
    // stands for the engine number (ENGINE_THROTTLE_#); a name without it is one variable for all
    // engines.
    std::string var;
    double scale = 1.0;  // InputSet: 0.01 when the event reads in percent but its preset takes 0..1

    bool operator==(const ControlMethod& o) const { return kind == o.kind && var == o.var && scale == o.scale; }
};

enum class ControlState : uint8_t {
    Pending = 0,  // not set up yet: playback drives it as it does without a profile
    Ok,           // set up: playback uses the profile's method
    Failed,       // nothing the setup tried worked: playback leaves it to the aircraft
    Skipped,      // the pilot skipped it: playback drives it as it does without a profile
    Absent,       // the aircraft doesn't have it (fixed gear, no spoilers or flaps)
};

// The throttle lever's own variable, for aircraft whose lever simvar isn't the lever: on the
// A320neo V2 GENERAL ENG THROTTLE LEVER POSITION is the autothrust's thrust demand, which creeps
// with speed while the lever sits in its detent. The setup finds the L:var that holds still when
// the pilot holds the lever, then steps the throttle axis through its range and notes where the
// lever went, so playback can turn the recorded lever back into the axis value that puts it
// there (detents included).
struct LeverPoint {
    double axis;   // THROTTLEn_AXIS_SET_EX1 as a fraction: 0 = -16384 (idle, or full reverse), 1 = full
    double lever;  // the lever variable there
};

// How the aircraft's reverse thrust can be commanded, as the setup's sweep found by trying.
enum class ReverseBy : uint8_t {
    Unknown = 0,  // not tried (profiles from before the reverse test)
    None,         // no reversers, or nothing tried deployed them
    Axis,         // the throttle axis has its own reverse zone
    Events,       // SET_THROTTLEn_REVERSE_THRUST_ON, then the axis sets the amount (the A350)
    Hold,         // "decrease throttle" held at idle (the PMDG 737); let go, it comes back to idle
};
const char* reverseByKey(ReverseBy r);  // "none", "axis", "events", "hold"; "" for Unknown
bool parseReverseByKey(const std::string& key, ReverseBy& out);

struct LeverCalibration {
    std::string var;  // the L:var; '#' stands for the engine number. Empty: none
    std::vector<LeverPoint> forward;  // reverser stowed, by ascending axis
    // With the reverser engaged (the axis then sets the reverse amount). Empty unless the reverse
    // events work (ControlProfile::reverseBy) and the lever variable moved.
    std::vector<LeverPoint> reverse;

    bool valid() const { return !var.empty() && forward.size() >= 2; }
};

struct ControlProfile {
    ControlState state = ControlState::Pending;
    ControlMethod method;
    bool confirmed = false;  // the pilot said the replay looked right (not only the automatic check)
    LeverCalibration lever;  // throttle only, with the key-event method
    ReverseBy reverseBy = ReverseBy::Unknown;  // throttle only, with the key-event method
};

// The key an aircraft's control profile is kept under, shared by its variants. `path` is the
// aircraft.cfg folder as Engine::setAircraftKey gets it (lower case, '/' separators), e.g.
// "simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config". The key is the
// aircraft's folder under SimObjects, so its presets (the PMDG's BW, SSW and BCF, the A350's
// -900ULR) share it, without the model's variant number: a dash and 3 or 4 digits after the
// model's own digits, so the PMDG 737-600/-700/-800/-900 are "simobjects/airplanes/pmdg 737" and
// an A350-900 and -1000 are both "a350". Anything else (a "title:" key) is returned as it is.
std::string aircraftFamily(const std::string& path);

struct AircraftProfile {
    std::string key;    // identifies the aircraft: aircraftFamily() of its path
    std::string title;  // for people reading aircraft.json
    int64_t updatedUtcMs = 0;
    bool declined = false;  // "never for this aircraft": the setup isn't offered again
    ControlProfile controls[kControlCount];

    ControlProfile& operator[](Control c) { return controls[static_cast<int>(c)]; }
    const ControlProfile& operator[](Control c) const { return controls[static_cast<int>(c)]; }
    bool anyPending() const;
    bool drives(Control c) const;  // Ok or Failed: playback follows the profile, not its defaults
};

// Which channels each control uses.
struct ControlSpec {
    const char* key;     // "spoilers", "flaps", "throttle", "gear" (aircraft.json, CommBus)
    const char* label;   // for prompts
    int inputChannel;    // PlayMode::Input channel (throttle: engine 1; engines 2-4 follow)
    int lvarChannel;     // PlayMode::LVar channel (throttle: engine 1)
    int keyChannel;      // what the KeyEvent method sends (throttle: engine 1)
    int simvarFirst;     // the Set channels the SimVar method writes, -1 if it has none
    int simvarCount;     // (the throttle's are per engine)
    bool perEngine;
};

const ControlSpec& controlSpec(Control c);
Control controlAt(int i);
bool parseControlKey(const std::string& key, Control& out);
const char* methodKey(MethodKind k);  // "input_set", ...
bool parseMethodKey(const std::string& key, MethodKind& out);
const char* stateKey(ControlState s);  // "pending", "ok", ...
bool parseStateKey(const std::string& key, ControlState& out);

// The variable for engine `engine` (1-based): '#' in the pattern replaced by its number.
std::string engineVar(const std::string& pattern, int engine);
// "input event HANDLING_FLAPS (preset)", "L:var A32NX_SPOILERS_HANDLE_POSITION", "key events", ...
std::string describeMethod(Control c, const ControlMethod& m);
// The method, and the throttle lever's variable if the profile has one.
std::string describeControl(Control c, const ControlProfile& cp);

// A sweep's (axis, lever) readings as a calibration table: sorted by axis, unreadable ones dropped.
std::vector<LeverPoint> leverTable(std::vector<LeverPoint> samples);
// The axis fraction that puts the lever at `lever`: the middle of a flat stretch at that value
// (a detent), else interpolated between the readings either side, clamped to the table's ends.
double invertLever(const std::vector<LeverPoint>& table, double lever);
// The axis fraction for a recorded lever value. clipReverser: the clip's reverser (rev_N) there,
// NaN if it wasn't recorded; liveReverse: the aircraft's reverser now. With a separate reverse
// table the clip's reverser events switch the aircraft's, and until it matches the lever is held
// at idle. False if the calibration can't place the value (reverse without a reverse table).
// Idle is the forward table's lowest axis reading; the lever variable may run either way from
// there (the PMDG 737's goes from 500 at idle down to 75 at full).
bool leverAxis(const LeverCalibration& cal, double lever, double clipReverser, bool liveReverse, double& axis);
// Whether engine `engine` (0-based) is in reverse in this frame, by what the reversers did: the
// reverse nozzles out, or the lever simvar below idle. Some aircraft reverse without setting
// GENERAL ENG REVERSE THRUST ENGAGED (the PMDG 737 with "decrease throttle" held), or set it
// without reversing.
bool reversing(const Frame& f, int engine);
// The channel a method takes its values from (engine is 0-based, throttle only), or -1.
int methodSourceChannel(Control c, MethodKind k, int engine);
// Every channel playback drives for this control when the profile drives it (any method):
// its Set channels, input event and L:var channels, and the gear command. Set in `out`.
void controlChannels(Control c, ChannelMask& out);

class ProfileStore {
public:
    const AircraftProfile* find(const std::string& key) const;
    AircraftProfile& upsert(const std::string& key);  // adds an empty profile when missing
    bool remove(const std::string& key);
    size_t size() const { return profiles_.size(); }
    const std::vector<AircraftProfile>& profiles() const { return profiles_; }

    // Re-keys profiles kept under an aircraft's own path (before variants shared one) by
    // aircraftFamily(). Where several variants had one, the most recently updated is kept.
    // Returns how many profiles changed or went.
    size_t groupVariants();

    void toJson(std::string& out) const;
    bool fromJson(const char* text, size_t length, std::string* error);

private:
    std::vector<AircraftProfile> profiles_;
};

}  // namespace br
