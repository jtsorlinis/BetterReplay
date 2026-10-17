// Playhead over a loaded clip: play/pause, seek, speed, and the interpolated frame to write.
#pragma once

#include <string>

#include "Clip.h"

namespace br {

// Replaces the clip's gear-handle channel with the gear command implied by the recorded gear
// positions: "up" from the moment the gear starts retracting, "down" from the moment it starts
// extending. Leaves the clip alone if it has no gear positions or the gear never moved (fixed gear).
// Returns true if the channel was rewritten.
bool deriveGearCommand(Clip& clip);

// Clears `present` for channels the clip has but never recorded a value for (all NaN), e.g. an
// input event the recording aircraft didn't have, so playback leaves them alone.
void dropEmptyChannels(Clip& clip);

class PlaybackController {
public:
    static constexpr double kMinSpeed = 0.25;
    static constexpr double kMaxSpeed = 4.0;

    void load(Clip&& clip, const std::string& file);
    void unload();

    bool loaded() const { return loaded_; }
    const std::string& file() const { return file_; }
    const Clip& clip() const { return clip_; }
    const ChannelMask& present() const { return clip_.present; }
    // Largest |value| of each channel over the whole clip (0 where it has none), e.g. to tell
    // whether an input event was recorded in percent or as a 0..1 fraction.
    const Frame& peak() const { return peak_; }

    void play();  // restarts from the beginning if at the end
    void pause() { playing_ = false; }
    bool playing() const { return playing_; }
    void seek(double t);
    void setSpeed(double speed);

    double speed() const { return speed_; }
    double time() const { return t_; }
    double duration() const { return duration_; }

    // Moves the playhead by dt * speed while playing; pauses automatically at the end.
    void advance(double dt);

    // Interpolated frame at the playhead (out.t = playhead time).
    void current(Frame& out);

private:
    Clip clip_;
    Frame peak_;
    std::string file_;
    bool loaded_ = false;
    bool playing_ = false;
    double t_ = 0.0;
    double duration_ = 0.0;
    double speed_ = 1.0;
    size_t hint_ = 0;
};

}  // namespace br
