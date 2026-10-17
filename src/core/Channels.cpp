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
    // playback only commands the gear (on change) and lets the sim animate it. The command is
    // derived from the recorded gear positions at playback time (see PlaybackController), because
    // some aircraft (e.g. the 2024 A320neo V2) only update GEAR HANDLE POSITION once the gear has
    // finished moving. Aircraft differ in which gear events they honour, so all of them are sent.
    {"gear_c", "GEAR CENTER POSITION", "percent over 100", F32, LIN, NONE, nullptr},
    {"gear_l", "GEAR LEFT POSITION", "percent over 100", F32, LIN, NONE, nullptr},
    {"gear_r", "GEAR RIGHT POSITION", "percent over 100", F32, LIN, NONE, nullptr},
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
