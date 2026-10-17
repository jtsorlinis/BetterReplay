// Data-driven channel table. Every recorded value is described here once; the SimConnect
// data definitions, the clip file's channel list and playback all derive from this table.
//
// To add a channel: append an enumerator before kChannelCount and a matching row in
// Channels.cpp. Keys are stored in clip files, so never rename or reuse an existing key.
#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>

namespace br {

enum class Storage : uint8_t { F64 = 1, F32 = 2, U8 = 3 };

enum class Interp : uint8_t {
    Linear = 0,
    Angle180 = 1,  // degrees, shortest path, result in (-180, 180]
    Angle360 = 2,  // degrees, shortest path, result in [0, 360)
    Step = 3,      // hold the earlier sample (booleans, detents)
};

enum class PlayMode : uint8_t {
    None = 0,   // recorded only
    Set = 1,    // written every playback frame via SetDataOnSimObject
    Event = 2,  // sent as a key event when the value changes, and again after every play/seek/stop
    Snap = 3,   // written only right after play/seek/stop (so the aircraft starts in the recorded
                // state), then left to the sim to animate
    Input = 4,  // an aircraft input event (B: var, e.g. a 2024 aircraft's own throttle lever),
                // recorded from it and set back with SimConnect_SetInputEvent
    LVar = 5,   // an aircraft L:var named by the aircraft's control profile (AircraftProfile.h):
                // recorded and replayed only on aircraft whose setup chose it
};

struct ChannelDef {
    const char* key;        // stable id stored in clip files
    const char* simvar;     // SimConnect simulation variable (nullptr for PlayMode::Input)
    const char* units;      // SimConnect units string
    Storage storage;        // on-disk representation
    Interp interp;
    PlayMode play;
    const char* playEvent;     // key event for PlayMode::Event (sent with 1/0), otherwise nullptr
    const char* playEventOn = nullptr;   // event sent when the state turns on (e.g. GEAR_DOWN), in
                                         // addition to playEvent or instead of it
    const char* playEventOff = nullptr;  // event sent when the state turns off (e.g. GEAR_UP)
    uint8_t engine = 0;                  // 1-based engine index for per-engine channels; skipped on
                                         // aircraft with fewer engines (0 = not engine-specific)
    // PlayMode::Input: the input event's name, or several alternatives separated by '|'. Only
    // aircraft that have one of them record and replay the channel.
    const char* inputEvent = nullptr;
    // PlayMode::Input: the Set channels this input event drives, as a range [first, first+count).
    // While it's replayed they aren't written, since the aircraft derives them from the input.
    int16_t coversFirst = -1;
    uint8_t coversCount = 0;
    // PlayMode::Input: true if the "<NAME>_SET" preset takes a 0..1 fraction (values recorded in
    // percent are scaled), false if it takes the value as recorded.
    bool inputSetFraction = false;
};

enum Channel : uint16_t {
    kLat,
    kLon,
    kAlt,
    kAgl,
    kPitch,
    kBank,
    kHeading,
    kVelBodyX,
    kVelBodyY,
    kVelBodyZ,
    kRotVelBodyX,
    kRotVelBodyY,
    kRotVelBodyZ,
    kGroundSpeed,
    kVerticalSpeed,
    kOnGround,
    kGForce,
    kGearCenter,
    kGearLeft,
    kGearRight,
    kGearHandle,
    kFlapsLeft,
    kFlapsRight,
    kFlapsHandleIndex,
    kSpoilersHandle,
    kAileronLeft,
    kAileronRight,
    kElevator,
    kRudder,
    kAileronInput,
    kElevatorInput,
    kRudderInput,
    kLightNav,
    kLightBeacon,
    kLightStrobe,
    kLightLanding,
    kLightTaxi,
    kThrottle1,
    kThrottle2,
    kThrottle3,
    kThrottle4,
    kReverser1,
    kReverser2,
    kReverser3,
    kReverser4,
    kReverserNozzle1,
    kReverserNozzle2,
    kReverserNozzle3,
    kReverserNozzle4,
    kInputThrottle1,
    kInputThrottle2,
    kInputThrottle3,
    kInputThrottle4,
    kInputGear,
    kInputFlaps,
    kInputSpeedBrakes,
    kEngineN1_1,
    kEngineN1_2,
    kEngineN1_3,
    kEngineN1_4,
    kEngineRpm1,
    kEngineRpm2,
    kEngineRpm3,
    kEngineRpm4,
    kSpoilersLeft,
    kLVarThrottle1,
    kLVarThrottle2,
    kLVarThrottle3,
    kLVarThrottle4,
    kLVarGear,
    kLVarFlaps,
    kLVarSpeedBrakes,
    kThrottleAxis1,
    kThrottleAxis2,
    kThrottleAxis3,
    kThrottleAxis4,
    kEngineN2_1,
    kEngineN2_2,
    kEngineN2_3,
    kEngineN2_4,
    kChannelCount
};

extern const ChannelDef kChannels[kChannelCount];

using ChannelMask = std::bitset<kChannelCount>;

// Returns the slot for a stable key, or -1 if this build doesn't know it.
int findChannel(const char* key);

size_t storageBytes(Storage s);  // 0 for unknown values

// One recorded sample: time on the recording clock plus every channel's value.
struct Frame {
    double t = 0.0;
    double v[kChannelCount] = {};
};

}  // namespace br
