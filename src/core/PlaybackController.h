// Playhead over a loaded clip: play/pause, seek, speed, and the interpolated frame to write.
#pragma once

#include <cstdint>
#include <string>

#include "Clip.h"

namespace br {

// Replaces the clip's gear-handle channel with the gear command implied by the recorded gear
// positions: "up" from the moment the gear starts retracting, "down" from the moment it starts
// extending. Leaves the clip alone if it has no gear positions or the gear never moved (fixed gear).
// Returns true if the channel was rewritten.
bool deriveGearCommand(Clip& clip);
// A clip recorded before the parking brake was gets it off throughout, so a replay started
// parked lets go of it (a takeoff or landing clip has it off nearly all the way). Returns true if
// the channel was filled in.
bool assumeParkingBrakeOff(Clip& clip);
// When the gear started coming down (the gear handle channel turning to down, so after
// deriveGearCommand), in seconds from the clip's start: on a landing the last time before the
// touchdown, otherwise the first. NaN if it never did.
double gearDownTime(const Clip& clip);
// The same, frame by frame, from the legs' mean extension as it comes (the live aircraft): 1 down,
// 0 up, NaN until the first reading.
class GearCommandFollower {
public:
    double update(double extension);
    double state() const { return state_; }

private:
    double state_ = NAN;
    double extreme_ = NAN;  // most extended (state 1) / most retracted (state 0) since the last flip
};

// A gear snap (the gear positions written for a few frames after a jump or Stop, so the gear doesn't
// animate from where it was), checked half a second after its last write: `before`, the aircraft's
// values when it began; `written`, what it wrote last; `after`, the aircraft's now. Rejected: the
// aircraft moved one back, more than kSnapBackTol from what was written (it keeps its own gear
// state, like the A320neo V2 and the PMDGs); `which` is that one. Held: they stuck, and the snap
// moved the gear (one value more than kSnapBackTol from where it was). Unknown: they stuck, but the
// gear was there already, which says nothing: the PMDG 737's held so at the start of a takeoff
// replay, and Stop, going by it, unfroze the aircraft on the ground with its gear still coming down.
enum class SnapVerdict : uint8_t { Unknown, Held, Rejected };
SnapVerdict snapVerdict(const double* before, const double* written, const double* after, int count,
                        int* which = nullptr);
constexpr double kSnapBackTol = 0.25;

// Clears `present` for channels the clip has but never recorded a value for (all NaN), e.g. an
// input event the recording aircraft didn't have, so playback leaves them alone.
void dropEmptyChannels(Clip& clip);

// Where the clip's flaps are going at clip time `t`: the frame where the movement they're in
// there ends (held still for kFlapsRestSec, turned back, or the clip's end), or the frame at `t`
// if they aren't moving. The PMDG 777's handle index follows its flaps, so a clip that starts
// with them still on their way to 30 has the handle at 25, though the pilot's lever is at 30.
Frame flapsDestination(const Clip& clip, double t);
constexpr double kFlapsRestSec = 1.0;
constexpr double kFlapsRestTol = 0.002;  // of the surfaces' travel (0 up, 1 fully out)

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
    double gearDownT() const { return gearDownT_; }  // gearDownTime of the loaded clip

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
    double gearDownT_ = NAN;
    double speed_ = 1.0;
    size_t hint_ = 0;
};

}  // namespace br
