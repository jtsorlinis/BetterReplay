// Per-aircraft control profiles: how playback drives the spoilers, flaps, throttle and gear on a
// given aircraft. The guided control setup (ControlSetup.h) builds them by watching the pilot
// move each control, replaying it with each candidate method and keeping the one that worked.
// They're kept in aircraft.json in the work folder.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Channels.h"
#include "PlaybackController.h"
#include "StickFit.h"

namespace br {

class JsonWriter;
struct JsonValue;

// The controls the setup teaches, in the order it asks for them.
enum class Control : uint8_t { Spoilers = 0, Flaps = 1, Throttle = 2, Gear = 3 };
constexpr int kControlCount = 4;

// Which way a control goes: out (the gear down, more flap, the spoilers up: toward higher values)
// or back in. A profile can say how to move it each way (ControlProfile::ways).
enum class Direction : uint8_t { Extend = 0, Retract = 1 };
constexpr int kDirections = 2;

// How playback drives a control.
enum class MethodKind : uint8_t {
    None = 0,
    InputSet,    // the input event's "<NAME>_SET" preset (fsVarsBVarCall), the cockpit's own path;
                 // the argument is value * scale + offset
    InputValue,  // SimConnect_SetInputEvent with the recorded value * scale + offset
    LVar,        // the aircraft's L:var, set to the recorded value
    SimVar,      // the control's simvars, written every frame
    KeyEvent,    // key events, like hardware bindings send. `var` names the family: empty for
                 // THROTTLEn_AXIS_SET_EX1, FLAPS_SET, SPOILERS_SET, GEAR_SET/UP/DOWN, one of the
                 // kKey* variants below, or one event by its number ("#69927", EventMethod)
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
    double scale = 1.0;  // InputSet: 0.01 when the event reads in percent but its preset takes 0..1;
                         // an event by its number: see EventMethod
    // InputSet and InputValue: added after scaling, where the event takes another range than it
    // reads (the FSS E-Jets' EVT_LEVER_THROTTLE_B reads 0..1 but takes the axis's -1..1, written
    // directly: scale 2, offset -1). The setup finds both from what the event reads back.
    double offset = 0.0;

    bool operator==(const ControlMethod& o) const {
        return kind == o.kind && var == o.var && scale == o.scale && offset == o.offset;
    }
};

// A KeyEvent method with one event by its number ('#' starts its var): the aircraft's own event
// (kThirdPartyEventFirst and up, numbered as its SDK gives them to external apps) or a sim key
// event, from the aircraft's SDK (ownGearWays) or seen firing while the pilot moved the control
// in the setup. Not for the throttle. The var says how it drives the control's value (eventTarget):
//   "#69927"         a position: sent with round(value * scale) (the PMDG 777's gear lever, 0 UP,
//   "#70010@-16383"  1 DOWN); after '@', an offset added to that (an axis)
//   "#66587(29501)"  a toggle, sent with that parameter: each one flips the control (ROTOR_BRAKE
//                    with a PMDG cockpit click's code)
//   "#69940(536870912)/#69940(-2147483648)"
//                    a step each way: the first toward higher values (more flap, spoilers out,
//                    gear down), the second back (a click on a lever: left and right button)
//   "#69927(536870912,131072)"
//                    a click: after ',', a parameter sent right after the first, the same event
//                    again (a mouse button and its release, in the SDK's mouse flags); in a toggle
//                    or either step
// For a toggle or steps `scale` is how far one moves the control.
bool isOwnEvent(const ControlMethod& m);
constexpr uint32_t kThirdPartyEventFirst = 0x11000;  // THIRD_PARTY_EVENT_ID_MIN (69632)
struct EventMethod {
    enum class Mode : uint8_t { Position, Toggle, Steps };
    Mode mode = Mode::Position;
    uint32_t id = 0;        // Steps: the one toward higher values
    int32_t param = 0;      // Toggle, Steps
    bool click = false;     // and `release` sent right after it
    int32_t release = 0;
    uint32_t lowerId = 0;   // Steps: the one back
    int32_t lowerParam = 0;
    bool lowerClick = false;
    int32_t lowerRelease = 0;
    double offset = 0.0;    // Position
    double scale = 1.0;
};
bool parseEventMethod(const ControlMethod& m, EventMethod& out);
ControlMethod eventMethod(const EventMethod& e);
int32_t eventParam(const EventMethod& e, double value);  // a position's parameter for the control's value
// The control's value an event method goes by, in a clip's frame (the KeyEvent method's channel):
// the gear command (0 up, 1 down), the flap handle index, the spoiler handle (0..1). NaN for the
// throttle.
double eventTarget(Control c, const Frame& f);
// For the log: "#69927 (third-party +295)", "GEAR_TOGGLE (65570)", "key event 66587".
std::string keyEventName(uint32_t id);
// The ways the aircraft's own gear lever event moves the lever, for an aircraft whose SDK
// documents one (the PMDG 777's EVT_GEAR_LEVER, #69927): its position ("#69927", 0 UP, 1 DOWN), and
// a click on it as the pilot's mouse does, through the SDK's mouse flags ("#69927(536870912,131072)",
// the left button and its release, as a toggle). Empty for other aircraft. `aircraftKey` is the
// profile's key (aircraftFamily). The setup tries them when it didn't see the lever's own event
// fire as the pilot moved it; which works which way, it finds by trying.
std::vector<ControlMethod> ownGearWays(const std::string& aircraftKey);
// The L:var that shows whether the aircraft's own autothrottle is on, for an aircraft whose
// AUTO_THROTTLE_DISCONNECT toggles it (the FSS E-Jets': FSS_EXX_FGC_LT_AT), so a replay presses it
// by that (SimBridge::pressAutothrottleDisconnect). Null for other aircraft. `aircraftKey` is the
// profile's key (aircraftFamily). By the key, not by whether the L:var is there: an L:var stays
// registered for the rest of the sim session once an aircraft has made it, so an A320 loaded after
// an E190 has this one too.
const char* autothrottleLight(const std::string& aircraftKey);
// The L:var of the aircraft's own parking brake, for an aircraft whose handle doesn't move the sim's
// (the Fenix A319/A320/A321's: S_MIP_PARKING_BRAKE, 1 set; BRAKE PARKING POSITION stayed off with
// it set, so the setup never offered to start; the FlyByWire A320neo's and A380X's
// A32NX_PARK_BRAKE_LEVER_POS).
// The setup's "parking brake set" goes by it there.
// Null for other aircraft. By the key, as autothrottleLight.
const char* parkingBrakeLVar(const std::string& aircraftKey);
// Whether the aircraft's flap lever has no detent of its own for CONF 1+F, handle index 2 (the
// Fenix A319/A320/A321's and the FlyByWire A320neo's and A380X's: the lever has 0, 1, 2, 3 and
// FULL, the handle index 0 to 5), so FLAPS_INCR doesn't go a handle index at a time
// (FlapNotches::setOnePlusF), nor FLAPS_SET's range (flapLeverFraction). By the key, as
// autothrottleLight.
bool flapsOnePlusF(const std::string& aircraftKey);
// The L:var of the aircraft's flap lever, for an aircraft whose FLAPS HANDLE INDEX follows its flaps
// rather than the lever (the FlyByWire A320neo's and A380X's: A32NX_FLAPS_HANDLE_INDEX, 0 to 4; their
// systems write the handle index from where the flaps and slats are). On the A380X it showed CONF 2
// only once the flaps were three quarters of the way there, seconds after the lever, so the setup's
// replays, which send the clip's handle, sent nothing before they were given up (2026-11-08). The
// handle index is read from the lever there (flapsHandleFromLever): recorded, and what the setup
// and replays go by. Null for other aircraft. By the key, as autothrottleLight.
const char* flapsLeverLVar(const std::string& aircraftKey);
// The L:var a PlayMode::AircraftLVar channel records on this aircraft: the FlyByWire A380X's wing
// flex (wflex_l1 .. wflex_r4: A32NX_WING_FLEX_LEFT_INBOARD .. A32NX_WING_FLEX_RIGHT_OUTBOARD). Its
// wings bend by those, from its own structural model, not by the sim's flex; in a replay that model
// reads the frozen aircraft's motion (accelerations the sim makes of it, not settable), and its
// wings were 9.4 off the clip's on average through a takeoff. So they're recorded and written back
// (SimBridge::rewriteAircraftLVars). Null for other channels and aircraft. By the key, as
// autothrottleLight.
const char* aircraftLVar(const std::string& aircraftKey, int channel);
// That lever's detent (0 to 4) as a handle index on a lever with no 1+F detent (flapsOnePlusF):
// 0 is 0, 1 is 1+F's 2 (CONF 1 or 1+F, which the aircraft picks: going there a notch from 0 is
// one notch in the air too), 2 to 4 are CONF 2's 3 to FULL's 5. NaN for NaN.
double flapsHandleFromLever(double lever);
// FLAPS_SET's and AXIS_FLAPS_SET's fraction of their range for a handle index: the index over the
// highest (FLAPS NUM HANDLE POSITIONS). On a lever with no 1+F detent (`onePlusF`), the lever's
// detent over its highest, CONF 1 and 1+F both its detent 1 (the aircraft picks which): the FlyByWire
// A320neo's FLAPS_SET for 1+F as 2 of 5 (0.4) put its lever at 2, CONF 2, and its 2 from CONF 2
// left it there, so the setup's replay moved the flaps out and nothing brought them back
// (2026-11-08). NaN without positions or an index.
double flapLeverFraction(double index, int positions, bool onePlusF);

// The flap lever driven with FLAPS_INCR/DECR (kKeyFlapsNotches), like a hardware flap switch: a
// notch at a time toward the target handle index, going by where the notches sent left the lever.
// The handle can show that much later: the PMDG 777's follows its flaps, a detent every ten seconds
// or so, and going by it sent extra notches that ran them to 25. So the handle counts again only
// once it has held still for kSettleMs since it last moved and the last notch (a notch didn't
// take), or at once where it's gone further than the notches the way they went, up to the target.
// A lagging handle only shows where the notches have had the lever; one that moves anywhere else,
// other than further a notch's way just after it (kRespondMs), was moved by the pilot or the
// aircraft, and counts at once. Kept across replays: Stop steps the lever back from where the
// replay's notches left it.
class FlapNotches {
public:
    // A method beginning: the lever from the last notches, only if they were just now (a replay's
    // restore); since then the pilot may have moved it (above).
    void begin(int64_t nowMs);
    void forget() { setLever(NAN); dir_ = 0; }  // the aircraft was reset
    // A lever with no detent for 1+F (flapsOnePlusF: the Fenix's). Going up, a notch from 0 is 1+F
    // (index 2) on the ground, and one from CONF 1 (index 1) is CONF 2 (index 3): in the air at
    // the clip's CONF 2, the third notch the index count wanted took it to CONF 3, and its handle,
    // which showed 2 when that notch went, only got to 4 about 9 s later (2026-11-05). Going down
    // it goes an index a notch, 1+F to CONF 1 in the air as well.
    void setOnePlusF(bool on) { onePlusF_ = on; }
    // One frame, with the aircraft's handle index: +1 send FLAPS_INCR, -1 FLAPS_DECR, 0 none now.
    // Notches kFrames apart at least, unless `force`. `onGround` for setOnePlusF.
    int step(double target, double handle, int64_t nowMs, bool force, bool onGround = false);
    // Whether the notches sent have taken the lever to `target`, or as near as notches take it
    // (setOnePlusF: CONF 1 in the air for a clip at 1+F); nothing to go by counts as there.
    bool at(double target) const;
    double lever() const { return lever_; }  // the handle index the notches sent should give
    static constexpr int kFrames = 8;
    static constexpr int64_t kSettleMs = 30000;
    // How long after a notch the handle going further its way is the aircraft taking it there (the
    // Fenix's handle showed a notch about 9 s after it in the air).
    static constexpr int64_t kRespondMs = 10000;

private:
    double after(double from, int dir) const;  // the handle index a notch from `from` gives
    void setLever(double at) { lever_ = lo_ = hi_ = at; }  // the count starting from there
    bool onePlusF_ = false;
    bool ground_ = false;  // on the ground, as the last step saw it
    double lever_ = NAN;   // NaN: go by the handle
    double lo_ = NAN;      // the lowest and highest the lever has been since the count started
    double hi_ = NAN;
    double handle_ = NAN;  // as last seen
    int64_t movedMs_ = 0;  // when the handle last moved or a notch was sent
    int64_t notchMs_ = 0;  // when the last notch was sent
    int dir_ = 0;          // its way: +1 FLAPS_INCR, -1 FLAPS_DECR
    int since_ = 1 << 20;  // frames since it
};

// The flap lever driven with FLAPS_SET or AXIS_FLAPS_SET, which put it in any detent at once: a
// detent at a time toward the clip's, FlapNotches::kFrames apart, as a hand takes the lever through
// each. The A320neo V2 picks CONF 1 or 1+F by the detent its lever came from: sent from FULL
// straight to 1, its flaps stayed at FULL. A detent is sent like any key event, when the clip's
// differs from the last one sent (or `force`). The way there starts from the aircraft's handle and
// goes on from the detents sent until it gets there.
class FlapSteps {
public:
    // One frame, with the clip's detent and the aircraft's handle index: the detent to send now,
    // NaN for none.
    double step(double target, double handle, bool force);
    // Still on its way to `target`, a detent at a time.
    bool goingTo(double target) const { return midway_ && goal_ == std::round(target); }
    bool midway() const { return midway_; }

private:
    double lever_ = NAN;   // the last detent sent
    double goal_ = NAN;    // the clip's detent then
    bool midway_ = false;  // short of it
    int since_ = 1 << 20;  // frames since the lever was last sent to another detent
};

// Steps a control toward the clip's value with an EventMethod toggle or steps, as a pilot clicking
// would (FlapNotches does the same with FLAPS_INCR/DECR): one event when the
// aircraft's value is half a step or more off the clip's, the next once the aircraft shows it (or
// after `waitFrames`), kMinFrames apart at least. `force` skips only the latter: a toggle sent
// again before the aircraft shows the last one would undo it.
class EventStepper {
public:
    // +1: send the event toward higher values (a toggle: the toggle), -1: the one back, 0: none now.
    int step(double target, double live, double size, int waitFrames, bool force);
    static int waitFrames(Control c);  // the gear legs start moving seconds after the lever
    static constexpr int kMinFrames = 8;
    static constexpr int kWaitFrames = 60;
    static constexpr int kGearWaitFrames = 360;

private:
    double expect_ = NAN;  // where the last step should take the control
    int since_ = 1 << 20;  // frames since it
};

struct ControlProfile;

// Playback's pick among a control's ways (ControlProfile::ways), for the gear, flaps and spoilers.
// A command (the clip's value leaving the aircraft's, one way) starts with the first way of that
// direction's list. If the control hasn't moved that way after EventStepper::waitFrames, the next
// one takes over (round the list); the one that moved it is reported, once per command, and is
// tried first from then on (promoteWay does the same in the profile). A way that has moved the
// control is kept for the rest of that command. With one way each direction nothing changes.
class WayPicker {
public:
    void begin(Control c, const ControlProfile& cp);
    // The distinct ways, both directions' together, the profile's `method` first: the driver gets
    // each ready.
    const std::vector<ControlMethod>& ways() const { return ways_; }
    // Leaves way i out (the aircraft can't take it). False if a direction has none left.
    bool drop(size_t i);
    // More than one way: there's something to pick (else it's always way 0, as before).
    bool multi() const { return ways_.size() > 1; }
    const std::vector<int>& order(Direction d) const { return order_[static_cast<int>(d)]; }  // indices into ways()

    struct Pick {
        int way = 0;             // index into ways(): drive the control with it now
        bool fresh = false;      // not the way of the frame before: send its value now
        Direction dir = Direction::Extend;  // the latest command's way
        int gaveUp = -1;         // the way given up on this frame: the control hadn't moved
        int worked = -1;         // the way that has just moved the control (once per command)
        bool promoted = false;   // and it wasn't the first that way: it is now
    };
    // One frame: `target` the clip's value (eventTarget), `live` the aircraft as it is. The way a
    // command goes by the aircraft's value (the gear's command from its legs, the flap handle
    // index, the spoiler handle); it has moved when that has, or what the control moves (the legs,
    // the flap or spoiler surfaces) has gone that way.
    Pick step(double target, const Frame& live);
    double live() const { return live_; }  // the aircraft's value, as the last step saw it

private:
    Control c_ = Control::Gear;
    std::vector<ControlMethod> ways_;
    std::vector<int> order_[kDirections];  // indices into ways_
    int current_ = 0;
    bool pending_ = false;   // a command is under way
    Direction dir_ = Direction::Extend;
    size_t at_ = 0;          // its way's place in order_[dir_]
    int since_ = 0;          // frames since that way took over, or last moved the control
    double from_ = NAN;      // the aircraft then
    double fromOutcome_ = NAN;
    bool worked_ = false;    // a way has moved the control in this command
    double live_ = NAN;
    GearCommandFollower gear_;
};

enum class ControlState : uint8_t {
    Pending = 0,  // not set up yet: playback drives it as it does without a profile
    Ok,           // set up: playback uses the profile's method
    Failed,       // nothing the setup tried worked: playback leaves it to the aircraft
    Skipped,      // the pilot skipped it: playback drives it as it does without a profile
    Absent,       // the aircraft doesn't have it (fixed gear, no spoilers or flaps)
    Automatic,    // the aircraft works it itself (spoilers that don't come up on the ground, like the FSS
                  // E-Jets'): playback leaves it to the aircraft, as for Failed
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
    Events,       // SET_THROTTLEn_REVERSE_THRUST_ON, then the axis sets the amount
    Hold,         // "decrease throttle" held at idle (the PMDG 737); let go, it comes back to idle
    Toggle,       // "toggle reverse" (THROTTLE_REVERSE_THRUST_TOGGLE) at idle, then the axis sets the
                  // amount; toggled again at idle to come out (the A350, which ignores the events)
};
const char* reverseByKey(ReverseBy r);  // "none", "axis", "events", "hold", "toggle"; "" for Unknown
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
    // The throttle with the key-event method: its lever's own variable. The spoilers: where a key
    // event that takes a position (`var`: SPOILERS_SET or AXIS_SPOILER_SET) puts the handle, each
    // point the event's fraction and the handle there (spoilerPositionEvent): the A380 puts its
    // lever half way for the handle's 0.52 at full.
    LeverCalibration lever;
    ReverseBy reverseBy = ReverseBy::Unknown;  // throttle only, with the key-event method
    // The gear, flaps and spoilers, where one method doesn't move them both ways or from wherever
    // they are (the PMDG 777's gear): for each Direction, the ways to try, in order. Playback sends
    // the first, and the next if the control hasn't moved after a while (WayPicker); one that moved
    // it goes to the front (promoteWay). Empty: `method` that way, as in profiles from before.
    // `method` is still the one the clips' channels are recorded for (the first way out).
    std::vector<ControlMethod> ways[kDirections];
    // The gear, flaps and spoilers: replays drive the control from this far ahead in the clip, as
    // much later as the setup's replay moved it than the pilot's movement had (0: no later). The
    // PMDG 777's flap handle follows its flaps, so the clip has it moving when they did, seconds
    // after the lever; a notch sent then moves them seconds later again.
    double leadSec = 0.0;

    std::vector<ControlMethod> waysTo(Direction d) const;  // ways[d], or `method`
    bool perDirection() const { return !ways[0].empty() || !ways[1].empty(); }
};
// Puts `m` first among the ways that way, where a replay found it moving the control when the one
// before it didn't. False if it's first already or not in that list.
bool promoteWay(ControlProfile& cp, Direction d, const ControlMethod& m);
// ControlProfile::leadSec: less than kMinLeadSec isn't kept (a replay a frame or two behind), and
// none goes beyond kMaxLeadSec.
constexpr double kMinLeadSec = 0.5;
constexpr double kMaxLeadSec = 10.0;

// The key an aircraft's control profile is kept under, shared by its variants. `path` is the
// aircraft.cfg folder as Engine::setAircraftKey gets it (lower case, '/' separators), e.g.
// "simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config". The key is the
// aircraft's folder under SimObjects, so its presets (the PMDG's BW, SSW and BCF, the A350's
// -900ULR) share it, without the model's variant number: a dash and 3 or 4 digits after the
// model's own digits, so the PMDG 737-600/-700/-800/-900 are "simobjects/airplanes/pmdg 737" and
// an A350-900 and -1000 are both "a350". A few the dash rule doesn't join go by kFamilyAliases
// (AircraftProfile.cpp): the FSS E170, E175 and E195 (and E-Jet livery folders) under "e190", the
// PMDG 777F under "pmdg 777", the default A321LR under "microsoft-a320neo". Anything else (a
// "title:" key) is returned as it is.
std::string aircraftFamily(const std::string& path);
bool isTitleKey(const std::string& key);  // "title:<aircraft title>": the aircraft's path isn't known

// The aircraft.cfg folder in a path the sim gives, as Engine::setAircraftKey takes it: lower case,
// '/' separators, from "simobjects/" on, without the file. The AircraftLoaded event's path is a
// full one ("...\Content\SimObjects\Airplanes\a350\presets\...\config\aircraft.cfg"), the system
// state's starts at SimObjects; both come out the same.
std::string aircraftFolder(const std::string& raw);

// Which aircraft is loaded, from what the sim says about it: its title (polled), the AircraftLoaded
// event (sent as an aircraft loads, with its path) and the AircraftLoaded system state (asked for).
// The state lags: after a switch from the A350 to the A380, MSFS 2024 kept answering with the
// A350's path, and the A380 got the A350's control profile. So a state answer with the path that
// came with another title is stale and isn't used, and once the event has named the aircraft the
// state isn't needed. Until a path is trusted the aircraft goes by its title ("title:<title>").
class AircraftIdentity {
public:
    // Each returns true when key() changed. Paths as aircraftFolder() gives them.
    bool onTitle(const std::string& title);
    bool onEvent(const std::string& path);
    bool onState(const std::string& path);

    const std::string& key() const { return key_; }  // for Engine::setAircraftKey; empty until anything is known
    bool wantState() const { return source_ == Source::None || source_ == Source::Title; }  // no path trusted yet
    bool stale() const { return stale_; }  // the last state answer was another aircraft's path
    const std::string& path() const { return path_; }
    const std::string& pathTitle() const { return pathTitle_; }

private:
    enum class Source : uint8_t { None, Title, State, Event };
    bool trust(const std::string& path, Source source);

    std::string title_;
    std::string path_;       // the last trusted path
    std::string pathTitle_;  // the title it came with; empty while that title hasn't come yet
    Source source_ = Source::None;
    bool stale_ = false;
    std::string key_;
};

// Control surfaces an aircraft's fly-by-wire keeps moving itself in replays, which playback steers
// to the clip's by the stick instead (StickLoop): learned the first time it takes one back, so
// later replays steer it from the start ("stickSurfaces" in aircraft.json).
enum StickSurface : unsigned { kStickElevator = 1u, kStickAilerons = 2u, kStickRudder = 4u };
constexpr int kStickSurfaceCount = 3;
const char* stickSurfaceKey(int i);  // "elevator", "ailerons", "rudder" (bit 1 << i)

struct AircraftProfile {
    std::string key;    // identifies the aircraft: aircraftFamily() of its path
    std::string title;  // for people reading aircraft.json
    int64_t updatedUtcMs = 0;
    bool declined = false;  // "never for this aircraft": the setup isn't offered again
    unsigned stickSurfaces = 0;  // StickSurface bits
    // How each of those answers the stick (StickFit), where known: fitted from replays, by hand for
    // the included profiles, or as a replay fits them ("stickPlants" in aircraft.json).
    StickPlant stickPlants[kStickSurfaceCount];
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
// "down"/"up" for the gear, "extend"/"retract" for the rest (aircraft.json, the log).
const char* directionKey(Control c, Direction d);

// The variable for engine `engine` (1-based): '#' in the pattern replaced by its number.
std::string engineVar(const std::string& pattern, int engine);
// "input event HANDLING_FLAPS (preset)", "L:var A32NX_SPOILERS_HANDLE_POSITION", "key events", ...
std::string describeMethod(Control c, const ControlMethod& m);
// The method (or each direction's ways), and the throttle lever's variable if the profile has one.
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
// Whether a recorded lever value is in reverse by the calibration: beyond idle, the other way from
// full, where there's a reverse table. There the lever variable shows the reverse amount itself.
bool leverInReverse(const LeverCalibration& cal, double lever);
// Whether a recorded lever value is beyond idle, the other way from full, by more than
// kBelowIdleFraction of the lever's travel, with or without a reverse table: the pilot going into
// reverse on an aircraft whose lever runs on past idle for it (the Fenix's: 2 at idle, 5 at full,
// down to 0 in reverse), seconds before its reversers come out.
constexpr double kBelowIdleFraction = 0.05;
bool leverBelowIdle(const LeverCalibration& cal, double lever);
// Whether a lever value is at idle by the calibration: no more than kBelowIdleFraction of the
// lever's travel toward full from idle, or beyond idle the other way.
bool leverAtIdle(const LeverCalibration& cal, double lever);
// The axis fraction in reverse on an "events" aircraft (ReverseBy::Events), the clip and the
// aircraft both in reverse. `fromLever`: the lever's (leverAxis, or the hardware axis); leverShown:
// the clip's lever variable has shown reverse since its reversers came out (leverInReverse). Until
// it has, the clip's throttle simvar over THROTTLE LOWER LIMIT where that's more: with "decrease
// throttle" held the lever stays at idle and only the simvar shows the amount. After, the lever's:
// the A220's simvar sits at its lower limit all through reverse, reverse idle too.
double eventsReverseAmount(double fromLever, bool leverShown, double pct, double lower);
// The spoilers' key event that takes a position, for method `m` (SPOILERS_SET for the key event
// with no var, AXIS_SPOILER_SET), as a spoiler calibration names it; nullptr for any other.
const char* spoilerPositionEvent(const ControlMethod& m);
// The spoilers' handle value to send with `m` for the clip's `handle`: through the calibration
// where it was made with that event, else the handle as it is.
double spoilerCommand(const LeverCalibration& cal, const ControlMethod& m, double handle);

// Ground spoilers: where the clip's spoiler handle went further than the spoilers' event can put
// it (the top of its calibration), from a moment on the ground. The A380's lever, and SPOILERS_SET,
// take the handle to 0.52 at full (the speedbrake); armed, its ground spoilers took it to 1 at
// touchdown, the surfaces all the way out, and a replay through the lever got them half out.
struct GroundSpoilerStretch {
    double startT;  // clip time
    double endT;
};
// The furthest the calibrated event puts the handle; NaN without a calibration.
double spoilerReach(const LeverCalibration& cal);
std::vector<GroundSpoilerStretch> groundSpoilerStretches(const LeverCalibration& cal, const std::vector<Frame>& frames);
constexpr double kGroundSpoilerMargin = 0.05;  // beyond the reach by this much

// Ground spoilers in a replay: the spoilers armed ahead of each stretch, with the clip's lever
// retracted, and the handle left to the aircraft through it; back to the lever after, disarmed.
// If the aircraft's ground spoilers haven't come out kWaitSec into a stretch, the lever as the clip
// has it for the rest of it, as before. Arming is checked by SPOILERS ARMED, and tried once more
// with SPOILERS_ARM_ON if it didn't show.
class SpoilerArming {
public:
    enum class Mode : uint8_t {
        Lever,  // the clip's handle through the event
        Armed,  // armed, the handle left to the aircraft
    };
    struct Step {
        Mode mode = Mode::Lever;
        bool arm = false;       // SPOILERS_ARM_SET 1 now
        bool armAgain = false;  // SPOILERS_ARM_ON now: the first didn't show
        bool disarm = false;    // back to how they were before (SPOILERS_ARM_SET), then the lever at once
        bool deployed = false;  // the aircraft's ground spoilers came out
        bool gaveUp = false;    // they didn't: the lever from here
    };
    void begin(std::vector<GroundSpoilerStretch> stretches, double reach);
    void clear();  // no stretches, and nothing armed (put it back first: armedByUs)
    bool any() const { return !stretches_.empty(); }
    const std::vector<GroundSpoilerStretch>& stretches() const { return stretches_; }
    // One frame at clip time t. clipHandle: the clip's; liveHandle, liveArmed: the aircraft's
    // (SPOILERS ARMED, NaN if unknown). jumped: a seek or the start.
    Step step(double t, double clipHandle, double liveHandle, double liveArmed, bool jumped);
    bool armedByUs() const { return armed_; }

    static constexpr double kLeadSec = 30.0;     // armed this long before a stretch
    static constexpr double kRetractedMax = 0.1; // the clip's handle at most this there (the A380's armed: 0.06)
    static constexpr double kWaitSec = 3.0;      // for the ground spoilers, from the stretch or a jump into it
    static constexpr double kArmCheckSec = 1.0;  // for SPOILERS ARMED to show it

private:
    std::vector<GroundSpoilerStretch> stretches_;
    double reach_ = NAN;
    bool armed_ = false;
    bool deployed_ = false;
    bool gaveUp_ = false;
    bool armChecked_ = false;
    int stretch_ = -1;     // the stretch armed for
    double armedT_ = NAN;  // clip time armed at
    double waitFrom_ = NAN;
};
// Whether engine `engine` (0-based) is in reverse in this frame, by what the reversers did: the
// reverse nozzles out, or the lever simvar below idle. Some aircraft reverse without setting
// GENERAL ENG REVERSE THRUST ENGAGED (the PMDG 737 with "decrease throttle" held), or set it
// without reversing.
bool reversing(const Frame& f, int engine);

// Reverse thrust in a replay on a "toggle" aircraft (ReverseBy::Toggle). The toggle switches every
// engine in or out of reverse and the aircraft remembers it, but it may not show it at reverse idle
// (the A350's nozzles and throttle simvar only show the reverse amount). So this keeps which mode
// the aircraft is in: its own toggles, corrected by what the aircraft shows (reversing: it's in
// reverse; forward thrust while a reverse amount is sent: it isn't), but not while its throttle
// simvar is still heading back to idle (the A350's lags its lever by seconds). It toggles only
// with the levers back at idle, and holds them there while the toggle settles.
class ReverseToggle {
public:
    enum class Drive : uint8_t {
        Forward,  // the throttle as the clip has it, out of reverse
        Idle,     // the levers at idle, around a toggle
        Reverse,  // the clip's reverse amount (throttle simvar / THROTTLE LOWER LIMIT), at most `cap`
    };
    enum class Confirmed : uint8_t {
        No,
        Shown,     // the aircraft showed reverse
        NoThrust,  // the small first amount made no forward thrust, though it showed no reverse either
    };
    struct Step {
        Drive drive = Drive::Forward;
        // Until the aircraft is known to be in reverse, a small amount: if it didn't take the
        // toggle, that's a little forward thrust, which shows it.
        double cap = 1.0;
        bool toggle = false;     // send the toggle now, after the levers' idle
        bool corrected = false;  // the aircraft showed it isn't in the mode it was thought to be in
        Confirmed confirmed = Confirmed::No;  // known to be in reverse from this frame, and how
    };
    // One frame. clipReverse: the replay wants reverse here: the clip is in it (reversing(), any
    // engine), or will be within kLeadSec where that may be toggled into ahead; liveReverse: the
    // aircraft is; liveThrottle: its throttle simvar (the highest engine's), forward thrust above
    // kForwardPct. Back at idle is neither. leversIdle: the aircraft's levers are at idle by their
    // own variable, so going into reverse it's toggled without waiting for the simvar, which
    // trails them (the A380's comes down from approach thrust at about 5% a second: a landing
    // replay waited 0.4 s for it, though the A380 takes the toggle at 15%, as the pilot's clip
    // shows). amountNow: the clip itself is in reverse here, so the amount sent is the clip's; only
    // that tells whether the toggle took (ahead of it, at reverse idle, nothing shows either way).
    Step step(bool clipReverse, bool liveReverse, double liveThrottle, bool leversIdle = false, bool amountNow = true);
    bool on() const { return on_; }                 // the aircraft is in reverse, as far as can be told
    bool busy() const { return on_ || wait_ > 0; }  // or a toggle is still settling
    void forget() {                                 // the aircraft was reset: out of reverse
        on_ = confirmed_ = false;
        wait_ = probe_ = 0;
        recentCount_ = recentNext_ = 0;
    }
    int toggles() const { return toggles_; }
    int corrections() const { return corrections_; }

    static constexpr double kForwardPct = 5.0;
    static constexpr double kProbeAmount = 0.2;
    static constexpr double kHeadingPct = 0.01;  // a frame: the throttle simvar heading back to idle
    static constexpr int kHeadingFrames = 6;     // over this many frames (the A380's moves every third)
    // After a toggle: idle this long, or until it shows reverse. Then the first amount this long
    // without forward thrust, the simvar not rising: the toggle took. The A380 shows nothing at
    // either (a replay once stayed at the first amount through the clip's full reverse, its
    // reversing engines' N1 at idle), so it waited the whole of both: at 30 frames each, its
    // landing replays' reverse thrust came 1.9 s after the clip's. A toggle it didn't take shows as
    // forward lever in the throttle simvar within a frame or two of the first amount, long before
    // the engines answer, so a fraction of that is enough.
    static constexpr int kSettleFrames = 8;
    static constexpr int kProbeFrames = 10;
    static constexpr int kIdleFrames = 300;      // the longest wait for the throttle to come back to idle
    static constexpr int kEvidenceFrames = 10;   // the aircraft showing otherwise this long corrects the mode
    static constexpr int kTries = 3;             // toggles each time the clip goes in or out of reverse
    // How far ahead the clip is looked at: for toggling in at reverse idle before the clip's
    // reverse (reverseIdleInClip), and for the amount (reverseAmount). The pilot's own toggle comes
    // before the clip shows reverse: the A380 clip's throttle simvar went from 15.5% to 0 at the
    // press, and below -1% a second later.
    static constexpr double kLeadSec = 1.0;

private:
    bool on_ = false;
    bool confirmed_ = false;  // it has shown reverse since the toggle
    bool want_ = false;       // the clip's reverse in the last frame
    Drive last_ = Drive::Forward;
    double recent_[kHeadingFrames] = {};  // the last frames' simvar, a ring
    int recentCount_ = 0;
    int recentNext_ = 0;
    int wait_ = 0;
    int idle_ = 0;
    int evidence_ = 0;
    int probe_ = 0;  // frames at the first amount without forward thrust
    int tries_ = 0;
    int toggles_ = 0;
    int corrections_ = 0;
};

// Whether a clip shows its aircraft at reverse idle, which decides whether a replay may toggle
// into reverse ahead of the clip's (ReverseToggle::kLeadSec) without that showing. At each of the
// clip's reverse entries (reversing(), any engine): the reverse amount (the throttle simvar below
// -kAmountPct, any engine) against the reversers' nozzles (kNozzlePct, any engine), from up to
// kLookBackSec before the entry. Unseen: the amount came first everywhere, so the toggle alone
// showed nothing (the A380's, A350's, A340's, A330's and A320neo's landing clips: the nozzles
// under 2% as the amount began). Seen: somewhere the nozzles came first, together, or without an
// amount at all. Unknown: an entry with no nozzle values. NoReverse: none in the clip.
enum class ReverseIdle : uint8_t { NoReverse, Unseen, Seen, Unknown };
struct ReverseIdleCheck {
    static constexpr double kAmountPct = 0.5;
    static constexpr double kNozzlePct = 10.0;
    static constexpr double kLookBackSec = 3.0;
};
ReverseIdle reverseIdleInClip(const std::vector<Frame>& frames, int engines);

// Toggle reverse, once the aircraft is in it: the reverse amount to send, as a fraction of full
// reverse, for the clip's throttle simvar clipPct here and aheadPct ReverseToggle::kLeadSec
// later, with the aircraft's at livePct (percent, negative in reverse; lower: full reverse,
// negative). The clip's simvar trails the pilot's lever, and sent as the lever the aircraft's
// trails it again: on the A380, which lags about a second, a replay took 4 s from -1% to -17%,
// the clip 2. So where the aircraft's simvar trails the clip's by more than kPushFrom of full
// reverse, in the way the clip's is heading, the amount goes toward the clip's kLeadSec on: all
// the way at kPushBand more. Level with the clip's, or past it, it's the clip's own: an aircraft
// that keeps up gets just that, and one that lags is pushed only until it reaches the clip's.
// A frame's own step (the A380's 14% a second at 30 frames a second is 0.02) pushes nothing.
struct ReversePush {
    static constexpr double kPushFrom = 0.05;
    static constexpr double kPushBand = 0.15;
};
double reverseAmount(double clipPct, double aheadPct, double livePct, double lower);

// Reverse thrust in a replay on the FSS E-Jets (kLeverInputs in ControlDriver.cpp). Their throttle
// input event reads the pilot's throttle from idle up, and takes no reverse. A "decrease throttle"
// binding goes to their own input event (EVT_LEVER_THROTTLE_DEC), which takes their lever variable
// (kIdle at idle, below 0.17 reverse, 0 full) kStep lower each time; the sim's THROTTLE_DECR sent
// from the module doesn't reach it (a replay holding it left the levers at idle). A new value from
// the throttle input takes them back out, as the pilot's throttle moving a little did at the end
// of a landing's reverse. So while the clip's levers are below idle (its lever simvar: the
// reversers stow after them), the aircraft's decrease every frame, all or nothing like a held
// button, and the input isn't sent meanwhile. If the lever variable hasn't gone down kCheckFrames
// in, it's written instead, kStep lower a frame. When the clip's levers come out, the lever
// variable back to idle and the input as the clip has it; levers still in reverse kCutFrames on,
// the aircraft's throttle cut (idle), once.
class LeverReverseHold {
public:
    enum class Way : uint8_t { Decrease, Write };
    struct Step {
        bool holding = false;    // in the hold: the input not sent
        bool decrease = false;   // the aircraft's decrease now
        double write = NAN;      // the lever variable written to this now
        bool started = false;    // the hold began this frame
        bool switched = false;   // the decrease didn't move the lever variable: written from now on
        bool shown = false;      // the aircraft's levers show reverse, the first time in this hold
        bool letGo = false;      // the hold ended: the input as the clip has it now
        bool noReverse = false;  // and the aircraft's levers never showed reverse in it
        bool out = false;        // the aircraft's levers out of reverse after letting go
        bool cut = false;        // the throttle cut now: still in reverse kCutFrames after letting go
    };
    // One frame. clipOut: the clip's lever simvar below idle (any engine); liveLever: the
    // aircraft's; leverVar: the lever variable (NaN where it can't be read: the decrease then).
    Step step(bool clipOut, bool liveLever, double leverVar);
    // Holding, or let go with the levers still in reverse: ending the replay now would leave them there.
    bool pending() const { return holding_ || wait_ > 0; }
    Way way() const { return way_; }
    int held() const { return held_; }         // frames in this hold (the last)
    int sinceLetGo() const { return since_; }  // frames since letting go
    double startVar() const { return start_; }  // the lever variable when this hold began

    static constexpr double kIdle = 0.3;
    static constexpr double kStep = 0.01;
    static constexpr int kCheckFrames = 10;
    static constexpr int kShownFrames = 5;  // below idle this long in a row: in reverse
    static constexpr int kCutFrames = 30;

private:
    Way way_ = Way::Decrease;
    bool holding_ = false;
    bool shown_ = false;
    int held_ = 0;
    int below_ = 0;  // frames in a row the aircraft's levers are below idle
    double start_ = NAN;
    double target_ = NAN;  // Way::Write: where the lever variable goes next
    int wait_ = 0;  // frames left before the throttle cut, while the levers are still in reverse
    int since_ = 0;
};

// Autothrust on an aircraft whose levers stay put under it (an Airbus): the lever sits in its
// detent while the thrust demand (its lever simvar, see LeverCalibration) goes where the
// autothrust puts it. Replayed from the lever, the engines give the detent's own thrust (climb
// thrust all the way down an approach), so there playback follows the clip's thrust demand
// instead (ThrustFollower), the levers no further forward than the clip's.
struct AutothrustStretch {
    double startT;  // clip time
    double endT;
};
// The stretches of a clip where engine `engine` (0-based) had autothrust in control: its lever
// held still (within kAutothrustHoldAxis on the axis), above idle, for longer than
// kAutothrustSettleSec, while its thrust demand still moved more than kAutothrustRangePct after
// that: more than the demand catching up with a lever move, or creeping with speed in a takeoff
// detent, does. The whole hold counts, and the lever being pulled back from it towards idle
// after (kAutothrustRetardSec at most), as the autothrust then follows the lever down.
std::vector<AutothrustStretch> autothrustStretches(const LeverCalibration& cal, const std::vector<Frame>& frames,
                                                   int engine);
constexpr double kAutothrustHoldAxis = 0.01;
constexpr double kAutothrustSettleSec = 10.0;
constexpr double kAutothrustRangePct = 8.0;
constexpr double kAutothrustRetardSec = 3.0;
constexpr double kAutothrustGapSec = 1.0;  // a gap in the clip's lever this long ends a hold

// A throttle input event that reads the pilot's throttle, on an aircraft whose autothrottle moves
// the levers themselves: the FSS E-Jets', whose lever simvar is 100 x the input where the two
// agree (93.9% at the top). In a takeoff its A/T flew the levers sat at 87% with the pilot's
// throttle at 0.69; replayed through the input with the A/T off, they went to 69%. The stretches
// of a clip where engine `engine`'s lever was more than kAtLeverGap of its travel away from its
// input, and held there (not closing on it by kAtLeverCatchUp a second, as the lever following the
// pilot's throttle does) for kAtLeverHeldSec somewhere: there playback sends the lever's position
// through the input instead. A stretch runs from where the lever left the input to where it came
// back within kAtLeverGap, or the clip's reversers came out.
std::vector<AutothrustStretch> autothrottleLeverStretches(const std::vector<Frame>& frames, int engine);
constexpr double kAtLeverGap = 0.08;
constexpr double kAtLeverCatchUp = 0.01;
constexpr double kAtLeverLookSec = 0.5;  // the lever's movement over this long
constexpr double kAtLeverHeldSec = 2.0;

// Follows a clip's thrust demand with the throttle axis, by the demand the aircraft shows (its
// lever simvar) against the clip's at the same moment: a first guess from the demand itself (the
// simvar as a fraction of the axis above idle), corrected in proportion to the difference and by
// the difference accumulated, which learns how far off the guess is on this aircraft. The demand
// ramps up and settles after a lever move (on the A350 over 2 to 5 s, slower from ground idle),
// and the correction holds back while the axis is at a limit, so it doesn't overshoot.
class ThrustFollower {
public:
    void reset() { integral_ = 0.0; }
    // One frame: the axis fraction for the clip's demand `clip` (percent), given the aircraft's
    // `live`, within [lo, hi] (idle, and the clip's lever there). dt: seconds since the last.
    double step(double clip, double live, double lo, double hi, double dt);

    static constexpr double kGain = 0.01;       // axis per percent of demand off
    static constexpr double kIntegral = 0.005;  // axis per percent-second

private:
    double integral_ = 0.0;
};

// The thrust levers in a replay away from where it put them: an autothrottle that engaged itself
// again after the disconnect at the jump. The PMDG 777's, its A/T arm switches on, did at the
// clip's acceleration altitude and drove the levers against the replay's from there. One call a
// frame with `off`, how far the levers are from where the replay last sent them (the engine
// furthest off, as a fraction of the throttle axis; NaN where that can't be told, as in reverse).
// Disconnect: they've been more than kOffAxis off for kOffMs (back for less than kBackMs doesn't
// count: a lever the replay sends may jump back for a moment before the autothrottle has it
// again), kRetryMs or more since the last time, kTries at most a replay. Back: after a
// Disconnect, they've been where the replay put them for kBackMs.
class LeverWatch {
public:
    enum class Result : uint8_t { Quiet, Disconnect, Back };
    void reset() {
        offSinceMs_ = backSinceMs_ = firedMs_ = -1;
        waitingBack_ = false;
        tries_ = 0;
        worst_ = 0.0;
    }
    Result step(double off, int64_t nowMs);
    int tries() const { return tries_; }
    double worst() const { return worst_; }  // the furthest off since they went off
    int64_t offForMs(int64_t nowMs) const { return offSinceMs_ < 0 ? 0 : nowMs - offSinceMs_; }
    int64_t firedMs() const { return firedMs_; }  // the last Disconnect, -1 before any

    static constexpr double kOffAxis = 0.03;
    static constexpr int64_t kOffMs = 500;
    static constexpr int64_t kBackMs = 1000;
    static constexpr int64_t kRetryMs = 3000;
    static constexpr int kTries = 10;

private:
    int64_t offSinceMs_ = -1;
    int64_t backSinceMs_ = -1;
    int64_t firedMs_ = -1;
    bool waitingBack_ = false;
    int tries_ = 0;
    double worst_ = 0.0;
};

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
