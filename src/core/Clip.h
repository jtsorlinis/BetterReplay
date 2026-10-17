#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "Channels.h"

namespace br {

enum class EventType : uint8_t { Manual = 0, Takeoff = 1, Landing = 2 };

const char* eventName(EventType e);  // "manual", "takeoff", "landing"
bool parseEventName(const std::string& name, EventType& out);

struct ClipMeta {
    EventType event = EventType::Manual;
    int64_t utcStartMs = 0;
    float sampleRateHz = 30.0f;
    uint32_t frames = 0;
    float eventOffsetSec = 0.0f;  // liftoff / first touchdown, relative to clip start
    float touchdownVsFpm = NAN;   // landings only
    float peakG = NAN;            // landings only
    uint16_t bounces = 0;
    std::string aircraft;
    // Channel key -> the aircraft variable it recorded, for input event and L:var channels
    // (their names depend on the aircraft and its control profile).
    std::vector<std::pair<std::string, std::string>> vars;
};

struct Clip {
    ClipMeta meta;
    std::vector<Frame> frames;  // t is relative to the clip start
    ChannelMask present;        // channels the file actually contained

    double duration() const { return frames.empty() ? 0.0 : frames.back().t - frames.front().t; }
};

}  // namespace br
