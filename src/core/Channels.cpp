#include "Channels.h"

#include <cstring>

namespace br {

namespace {
constexpr Storage F64 = Storage::F64;
constexpr Storage F32 = Storage::F32;
constexpr Storage U8 = Storage::U8;
constexpr Interp LIN = Interp::Linear;
constexpr Interp A180 = Interp::Angle180;
constexpr Interp A360 = Interp::Angle360;
constexpr Interp STEP = Interp::Step;
constexpr PlayMode NONE = PlayMode::None;
constexpr PlayMode SET = PlayMode::Set;
constexpr PlayMode EVT = PlayMode::Event;
constexpr PlayMode SNAP = PlayMode::Snap;
constexpr PlayMode INPUT = PlayMode::Input;
constexpr PlayMode LVAR = PlayMode::LVar;
}  // namespace

// Names, units and settable flags verified against the MSFS 2024 SimVar documentation.
// Angles and lat/lon are requested in degrees (the documented default is radians), and
// vertical speed in feet/minute (the documented default is feet per second).
const ChannelDef kChannels[kChannelCount] = {
    {"lat", "PLANE LATITUDE", "degrees", F64, LIN, SET, nullptr},
    {"lon", "PLANE LONGITUDE", "degrees", F64, A180, SET, nullptr},
    {"alt", "PLANE ALTITUDE", "feet", F64, LIN, SET, nullptr},
    {"agl", "PLANE ALT ABOVE GROUND", "feet", F32, LIN, NONE, nullptr},
    {"pitch", "PLANE PITCH DEGREES", "degrees", F32, A180, SET, nullptr},
    {"bank", "PLANE BANK DEGREES", "degrees", F32, A180, SET, nullptr},
    {"hdg", "PLANE HEADING DEGREES TRUE", "degrees", F32, A360, SET, nullptr},
    // Velocities are written during playback so airspeed/VS indications follow the clip
    // (position and attitude are frozen, so they don't move the aircraft).
    {"vbx", "VELOCITY BODY X", "feet per second", F32, LIN, SET, nullptr},
    {"vby", "VELOCITY BODY Y", "feet per second", F32, LIN, SET, nullptr},
    {"vbz", "VELOCITY BODY Z", "feet per second", F32, LIN, SET, nullptr},
    {"rvx", "ROTATION VELOCITY BODY X", "radians per second", F32, LIN, SET, nullptr},
    {"rvy", "ROTATION VELOCITY BODY Y", "radians per second", F32, LIN, SET, nullptr},
    {"rvz", "ROTATION VELOCITY BODY Z", "radians per second", F32, LIN, SET, nullptr},
    {"gs", "GROUND VELOCITY", "knots", F32, LIN, NONE, nullptr},
    {"vs", "VERTICAL SPEED", "feet/minute", F32, LIN, NONE, nullptr},
    {"ongnd", "SIM ON GROUND", "bool", U8, STEP, NONE, nullptr},
    {"g", "G FORCE", "gforce", F32, LIN, NONE, nullptr},
    // Writing gear positions every frame fights the aircraft's gear system (visible flicker), so
    // playback commands the gear (on change) and lets the sim animate it. The positions are only
    // snapped after play/seek/stop, together with the command, so a jump doesn't leave the gear
    // animating from wherever it was. The command is derived from the recorded gear positions at
    // playback time (see PlaybackController), because some aircraft (e.g. the 2024 A320neo V2)
    // only update GEAR HANDLE POSITION once the gear has finished moving. Aircraft differ in which
    // gear events they honour, so all of them are sent.
    {"gear_c", "GEAR CENTER POSITION", "percent over 100", F32, LIN, SNAP, nullptr},
    {"gear_l", "GEAR LEFT POSITION", "percent over 100", F32, LIN, SNAP, nullptr},
    {"gear_r", "GEAR RIGHT POSITION", "percent over 100", F32, LIN, SNAP, nullptr},
    {"gear_h", "GEAR HANDLE POSITION", "percent over 100", F32, STEP, EVT, "GEAR_SET", "GEAR_DOWN", "GEAR_UP"},
    {"flap_l", "TRAILING EDGE FLAPS LEFT PERCENT", "percent over 100", F32, LIN, SET, nullptr},
    {"flap_r", "TRAILING EDGE FLAPS RIGHT PERCENT", "percent over 100", F32, LIN, SET, nullptr},
    {"flap_idx", "FLAPS HANDLE INDEX", "number", F32, STEP, SET, nullptr},
    {"spoil_h", "SPOILERS HANDLE POSITION", "percent over 100", F32, LIN, SET, nullptr},
    {"ail_l", "AILERON LEFT DEFLECTION", "radians", F32, LIN, SET, nullptr},
    {"ail_r", "AILERON RIGHT DEFLECTION", "radians", F32, LIN, SET, nullptr},
    {"elev", "ELEVATOR DEFLECTION", "radians", F32, LIN, SET, nullptr},
    {"rud", "RUDDER DEFLECTION", "radians", F32, LIN, SET, nullptr},
    {"ail_pos", "AILERON POSITION", "position", F32, LIN, NONE, nullptr},
    {"elev_pos", "ELEVATOR POSITION", "position", F32, LIN, NONE, nullptr},
    {"rud_pos", "RUDDER POSITION", "position", F32, LIN, NONE, nullptr},
    {"l_nav", "LIGHT NAV", "bool", U8, STEP, EVT, "NAV_LIGHTS_SET"},
    {"l_beacon", "LIGHT BEACON", "bool", U8, STEP, EVT, "BEACON_LIGHTS_SET"},
    {"l_strobe", "LIGHT STROBE", "bool", U8, STEP, EVT, "STROBES_SET"},
    {"l_land", "LIGHT LANDING", "bool", U8, STEP, EVT, "LANDING_LIGHTS_SET"},
    {"l_taxi", "LIGHT TAXI", "bool", U8, STEP, EVT, "TAXI_LIGHTS_SET"},
    // Throttle levers (percent; reverse can be negative). While playing, the hardware throttle
    // events are masked so the physical lever doesn't fight the replay. Aircraft that drive the
    // lever from their own input handling override this simvar; for those SimBridge switches to
    // sending throttle axis events instead.
    {"thr_1", "GENERAL ENG THROTTLE LEVER POSITION:1", "percent", F32, LIN, SET, nullptr, nullptr, nullptr, 1},
    {"thr_2", "GENERAL ENG THROTTLE LEVER POSITION:2", "percent", F32, LIN, SET, nullptr, nullptr, nullptr, 2},
    {"thr_3", "GENERAL ENG THROTTLE LEVER POSITION:3", "percent", F32, LIN, SET, nullptr, nullptr, nullptr, 3},
    {"thr_4", "GENERAL ENG THROTTLE LEVER POSITION:4", "percent", F32, LIN, SET, nullptr, nullptr, nullptr, 4},
    // Thrust reversers. The engaged flag isn't settable, so it's replayed with the reverse thrust
    // on/off events; the reverse amount is the (negative) throttle lever above. The nozzle
    // deployment isn't settable either and is recorded for analysis.
    {"rev_1", "GENERAL ENG REVERSE THRUST ENGAGED:1", "bool", U8, STEP, EVT, nullptr, "SET_THROTTLE1_REVERSE_THRUST_ON",
     "SET_THROTTLE1_REVERSE_THRUST_OFF", 1},
    {"rev_2", "GENERAL ENG REVERSE THRUST ENGAGED:2", "bool", U8, STEP, EVT, nullptr, "SET_THROTTLE2_REVERSE_THRUST_ON",
     "SET_THROTTLE2_REVERSE_THRUST_OFF", 2},
    {"rev_3", "GENERAL ENG REVERSE THRUST ENGAGED:3", "bool", U8, STEP, EVT, nullptr, "SET_THROTTLE3_REVERSE_THRUST_ON",
     "SET_THROTTLE3_REVERSE_THRUST_OFF", 3},
    {"rev_4", "GENERAL ENG REVERSE THRUST ENGAGED:4", "bool", U8, STEP, EVT, nullptr, "SET_THROTTLE4_REVERSE_THRUST_ON",
     "SET_THROTTLE4_REVERSE_THRUST_OFF", 4},
    {"rnoz_1", "TURB ENG REVERSE NOZZLE PERCENT:1", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 1},
    {"rnoz_2", "TURB ENG REVERSE NOZZLE PERCENT:2", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 2},
    {"rnoz_3", "TURB ENG REVERSE NOZZLE PERCENT:3", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 3},
    {"rnoz_4", "TURB ENG REVERSE NOZZLE PERCENT:4", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 4},
    // 2024 aircraft built on the Asobo templates keep their cockpit controls in input events and
    // overwrite the simvars above from them (the A320neo V2 ignores the throttle, gear, flap and
    // spoiler simvars and gear events). Their values are recorded as-is and set back, so they're
    // exact whatever their units. Names from the aircraft's input event list (logged at load).
    // The _SET argument differs per event. Seen on the A320neo V2: ENGINE_THROTTLE_n reads 0..100
    // (16.7 = idle) and _SET(0.86) moved the lever to ~81, so it takes a fraction; HANDLING_FLAPS
    // reads 0/25/50.. and _SET(0.25) gave 0, so it takes the value as read. The SDK's own example
    // sets HANDLING_SPEED_BRAKES_SET to 0.5.
    {"ie_thr_1", nullptr, nullptr, F32, LIN, INPUT, nullptr, nullptr, nullptr, 1, "ENGINE_THROTTLE_1", kThrottle1, 1, true},
    {"ie_thr_2", nullptr, nullptr, F32, LIN, INPUT, nullptr, nullptr, nullptr, 2, "ENGINE_THROTTLE_2", kThrottle2, 1, true},
    {"ie_thr_3", nullptr, nullptr, F32, LIN, INPUT, nullptr, nullptr, nullptr, 3, "ENGINE_THROTTLE_3", kThrottle3, 1, true},
    {"ie_thr_4", nullptr, nullptr, F32, LIN, INPUT, nullptr, nullptr, nullptr, 4, "ENGINE_THROTTLE_4", kThrottle4, 1, true},
    {"ie_gear", nullptr, nullptr, F32, STEP, INPUT, nullptr, nullptr, nullptr, 0, "LANDING_GEAR_GEAR", -1, 0, false},
    {"ie_flaps", nullptr, nullptr, F32, STEP, INPUT, nullptr, nullptr, nullptr, 0, "HANDLING_FLAPS", kFlapsLeft, 3, false},
    {"ie_spdbrk", nullptr, nullptr, F32, LIN, INPUT, nullptr, nullptr, nullptr, 0,
     "HANDLING_SPEED_BRAKES|HANDLING_SPOILERS|HANDLING_SPEEDBRAKES", kSpoilersHandle, 1, true},
    // What the controls did, for the control setup's checks (did the engines spool up, did the
    // spoilers come up?) and for analysis. Not settable.
    {"n1_1", "TURB ENG N1:1", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 1},
    {"n1_2", "TURB ENG N1:2", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 2},
    {"n1_3", "TURB ENG N1:3", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 3},
    {"n1_4", "TURB ENG N1:4", "percent", F32, LIN, NONE, nullptr, nullptr, nullptr, 4},
    {"rpm_1", "GENERAL ENG RPM:1", "rpm", F32, LIN, NONE, nullptr, nullptr, nullptr, 1},
    {"rpm_2", "GENERAL ENG RPM:2", "rpm", F32, LIN, NONE, nullptr, nullptr, nullptr, 2},
    {"rpm_3", "GENERAL ENG RPM:3", "rpm", F32, LIN, NONE, nullptr, nullptr, nullptr, 3},
    {"rpm_4", "GENERAL ENG RPM:4", "rpm", F32, LIN, NONE, nullptr, nullptr, nullptr, 4},
    {"spoil_l", "SPOILERS LEFT POSITION", "percent over 100", F32, LIN, NONE, nullptr},
    // L:vars the control setup found driving a control on this aircraft (its profile names them;
    // NaN everywhere else). Replayed only through the profile.
    {"lv_thr_1", nullptr, nullptr, F32, LIN, LVAR, nullptr, nullptr, nullptr, 1},
    {"lv_thr_2", nullptr, nullptr, F32, LIN, LVAR, nullptr, nullptr, nullptr, 2},
    {"lv_thr_3", nullptr, nullptr, F32, LIN, LVAR, nullptr, nullptr, nullptr, 3},
    {"lv_thr_4", nullptr, nullptr, F32, LIN, LVAR, nullptr, nullptr, nullptr, 4},
    {"lv_gear", nullptr, nullptr, F32, STEP, LVAR, nullptr},
    {"lv_flaps", nullptr, nullptr, F32, STEP, LVAR, nullptr},
    {"lv_spdbrk", nullptr, nullptr, F32, LIN, LVAR, nullptr},
};

int findChannel(const char* key) {
    for (int i = 0; i < kChannelCount; ++i) {
        if (std::strcmp(kChannels[i].key, key) == 0) return i;
    }
    return -1;
}

size_t storageBytes(Storage s) {
    switch (s) {
        case Storage::F64: return 8;
        case Storage::F32: return 4;
        case Storage::U8: return 1;
    }
    return 0;
}

}  // namespace br
