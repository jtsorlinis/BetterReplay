#include "PlaybackController.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "Interp.h"

namespace br {

namespace {

// Mean extension (0 = up, 1 = down) of whichever gear legs the clip recorded, or NaN.
double gearExtension(const Frame& f, const ChannelMask& present) {
    static const int kLegs[] = {kGearCenter, kGearLeft, kGearRight};
    double sum = 0.0;
    int n = 0;
    for (int c : kLegs) {
        if (present.test(static_cast<size_t>(c)) && std::isfinite(f.v[c])) {
            sum += f.v[c];
            ++n;
        }
    }
    return n > 0 ? sum / n : NAN;
}

// Mean of the flap surfaces the clip recorded (0 up, 1 fully out), or NaN.
double flapsExtension(const Frame& f, const ChannelMask& present) {
    double sum = 0.0;
    int n = 0;
    for (int c : {static_cast<int>(kFlapsLeft), static_cast<int>(kFlapsRight)}) {
        if (present.test(static_cast<size_t>(c)) && std::isfinite(f.v[c])) {
            sum += f.v[c];
            ++n;
        }
    }
    return n > 0 ? sum / n : NAN;
}

}  // namespace

Frame flapsDestination(const Clip& clip, double t) {
    Frame at;
    const std::vector<Frame>& frames = clip.frames;
    if (frames.empty()) return at;
    sampleFrames(frames.data(), frames.size(), t, at, nullptr);
    double last = flapsExtension(at, clip.present);
    if (std::isnan(last)) return at;
    auto it = std::lower_bound(frames.begin(), frames.end(), t, [](const Frame& f, double v) { return f.t < v; });
    double movedT = t;
    int dir = 0;
    const Frame* end = nullptr;
    for (; it != frames.end(); ++it) {
        const double x = flapsExtension(*it, clip.present);
        if (std::isnan(x)) continue;
        if (std::fabs(x - last) > kFlapsRestTol) {
            const int d = x > last ? 1 : -1;
            if (dir != 0 && d != dir) break;  // turned back: this far, no further
            dir = d;
            last = x;
            movedT = it->t;
            end = &*it;
        } else if (it->t - movedT >= kFlapsRestSec) {
            if (dir != 0) end = &*it;  // at rest there (the 777's handle index caught up)
            break;
        }
    }
    return dir != 0 ? *end : at;
}

bool deriveGearCommand(Clip& clip) {
    if (clip.frames.empty()) return false;
    double lo = 1e9;
    double hi = -1e9;
    for (const Frame& f : clip.frames) {
        const double g = gearExtension(f, clip.present);
        if (std::isnan(g)) continue;
        lo = g < lo ? g : lo;
        hi = g > hi ? g : hi;
    }
    if (!(hi - lo > 0.1)) return false;  // no gear positions, or the gear never moved

    GearCommandFollower follow;
    for (Frame& f : clip.frames) {
        const double state = follow.update(gearExtension(f, clip.present));
        f.v[kGearHandle] = std::isnan(state) ? f.v[kGearHandle] : state;
    }
    clip.present.set(static_cast<size_t>(kGearHandle));
    return true;
}

bool assumeParkingBrakeOff(Clip& clip) {
    if (clip.present.test(static_cast<size_t>(kParkingBrake))) return false;
    for (Frame& f : clip.frames) f.v[kParkingBrake] = 0.0;
    clip.present.set(static_cast<size_t>(kParkingBrake));
    return true;
}

double gearDownTime(const Clip& clip) {
    if (clip.frames.size() < 2 || !clip.present.test(static_cast<size_t>(kGearHandle))) return NAN;
    const bool landing = clip.meta.event == EventType::Landing;
    const double t0 = clip.frames.front().t;
    double down = NAN;
    for (size_t i = 1; i < clip.frames.size(); ++i) {
        if (!(clip.frames[i - 1].v[kGearHandle] < 0.5 && clip.frames[i].v[kGearHandle] >= 0.5)) continue;
        const double t = clip.frames[i].t - t0;
        if (landing && t > clip.meta.eventOffsetSec) break;
        down = t;
        if (!landing) break;
    }
    return down;
}

// The command flips when the gear moves the other way by more than the threshold from its
// furthest point since the last flip, so a gear in mid-travel doesn't flip-flop.
double GearCommandFollower::update(double g) {
    constexpr double kMoveThreshold = 0.03;  // ignore noise at the end stops
    if (std::isnan(g)) return state_;
    if (std::isnan(state_)) {
        state_ = g >= 0.5 ? 1.0 : 0.0;
        extreme_ = g;
    } else if (state_ == 1.0) {
        if (g > extreme_) extreme_ = g;
        if (g < extreme_ - kMoveThreshold) {
            state_ = 0.0;  // started retracting
            extreme_ = g;
        }
    } else {
        if (g < extreme_) extreme_ = g;
        if (g > extreme_ + kMoveThreshold) {
            state_ = 1.0;  // started extending
            extreme_ = g;
        }
    }
    return state_;
}

void GearUpOnGround::sent(bool up, bool onGround) {
    waiting_ = up && onGround;
    tries_ = 0;
    airFromMs_ = 0;
    dueMs_ = 0;
}

bool GearUpOnGround::step(bool clipUp, bool onGround, double extension, int64_t nowMs) {
    if (!waiting_) return false;
    if (!clipUp) {
        waiting_ = false;  // the clip's gear went down again
        return false;
    }
    if (onGround) {
        airFromMs_ = 0;
        return false;
    }
    if (airFromMs_ == 0) airFromMs_ = nowMs;
    if (nowMs - airFromMs_ > kWatchMs) {
        waiting_ = false;  // the aircraft's gear came up
        return false;
    }
    const int64_t due = tries_ == 0 ? airFromMs_ + kAirMs : dueMs_;
    if (nowMs < due || extension < kDown) return false;
    ++tries_;
    dueMs_ = nowMs + kAgainMs;
    if (tries_ >= kTries) waiting_ = false;
    return true;
}

void JumpFollowCheck::sent(double clip, int64_t nowMs) {
    waiting_ = std::isfinite(clip);
    clip_ = clip;
    fromMs_ = nowMs;
    tries_ = 0;
}

bool JumpFollowCheck::step(double clip, double live, int64_t nowMs) {
    if (!waiting_) return false;
    if (!(std::fabs(clip - clip_) <= kClipMoved)) {
        waiting_ = false;
        return false;
    }
    if (nowMs - fromMs_ < kAfterMs || !std::isfinite(live)) return false;
    if (std::fabs(live - clip) <= kTol) {
        waiting_ = false;
        return false;
    }
    ++tries_;
    fromMs_ = nowMs;
    if (tries_ >= kTries) waiting_ = false;
    return true;
}

SnapVerdict snapVerdict(const double* before, const double* written, const double* after, int count, int* which) {
    bool moved = false;
    for (int i = 0; i < count; ++i) {
        if (std::isfinite(after[i]) && std::fabs(after[i] - written[i]) > kSnapBackTol) {
            if (which) *which = i;
            return SnapVerdict::Rejected;
        }
        moved |= std::isfinite(before[i]) && std::fabs(written[i] - before[i]) > kSnapBackTol;
    }
    return moved ? SnapVerdict::Held : SnapVerdict::Unknown;
}

void dropEmptyChannels(Clip& clip) {
    for (int c = 0; c < kChannelCount; ++c) {
        if (!clip.present.test(static_cast<size_t>(c))) continue;
        bool any = false;
        for (const Frame& f : clip.frames) {
            if (std::isfinite(f.v[c])) {
                any = true;
                break;
            }
        }
        if (!any) clip.present.reset(static_cast<size_t>(c));
    }
}

void PlaybackController::load(Clip&& clip, const std::string& file) {
    clip_ = std::move(clip);
    dropEmptyChannels(clip_);
    deriveGearCommand(clip_);
    assumeParkingBrakeOff(clip_);
    gearDownT_ = gearDownTime(clip_);
    peak_ = Frame();
    for (const Frame& f : clip_.frames) {
        for (int c = 0; c < kChannelCount; ++c) {
            const double a = std::fabs(f.v[c]);
            if (a > peak_.v[c]) peak_.v[c] = a;  // NaN compares false: skipped
        }
    }
    file_ = file;
    loaded_ = !clip_.frames.empty();
    duration_ = clip_.duration();
    t_ = loaded_ ? clip_.frames.front().t : 0.0;
    hint_ = 0;
    playing_ = false;
}

void PlaybackController::unload() {
    clip_ = Clip();
    file_.clear();
    loaded_ = false;
    playing_ = false;
    t_ = 0.0;
    duration_ = 0.0;
    gearDownT_ = NAN;
    hint_ = 0;
}

void PlaybackController::play() {
    if (!loaded_) return;
    if (t_ >= duration_) t_ = 0.0;
    playing_ = true;
}

void PlaybackController::seek(double t) {
    if (!std::isfinite(t)) return;
    t_ = t < 0.0 ? 0.0 : (t > duration_ ? duration_ : t);
}

void PlaybackController::setSpeed(double speed) {
    if (!std::isfinite(speed)) return;
    speed_ = speed < kMinSpeed ? kMinSpeed : (speed > kMaxSpeed ? kMaxSpeed : speed);
}

void PlaybackController::advance(double dt) {
    if (!loaded_ || !playing_ || !(dt > 0.0)) return;
    t_ += dt * speed_;
    if (t_ >= duration_) {
        t_ = duration_;
        playing_ = false;
    }
}

void PlaybackController::current(Frame& out) {
    if (!loaded_) return;
    sampleFrames(clip_.frames.data(), clip_.frames.size(), t_, out, &hint_);
}

}  // namespace br
