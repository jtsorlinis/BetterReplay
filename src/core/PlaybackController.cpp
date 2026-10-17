#include "PlaybackController.h"

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

}  // namespace

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

    // The command flips when the gear moves the other way by more than the threshold from its
    // furthest point since the last flip, so a gear in mid-travel doesn't flip-flop.
    constexpr double kMoveThreshold = 0.03;  // ignore noise at the end stops
    double state = NAN;                      // 1 = down, 0 = up
    double extreme = NAN;                    // most extended (state 1) / most retracted (state 0)
    for (Frame& f : clip.frames) {
        const double g = gearExtension(f, clip.present);
        if (!std::isnan(g)) {
            if (std::isnan(state)) {
                state = g >= 0.5 ? 1.0 : 0.0;
                extreme = g;
            } else if (state == 1.0) {
                if (g > extreme) extreme = g;
                if (g < extreme - kMoveThreshold) {
                    state = 0.0;  // started retracting
                    extreme = g;
                }
            } else {
                if (g < extreme) extreme = g;
                if (g > extreme + kMoveThreshold) {
                    state = 1.0;  // started extending
                    extreme = g;
                }
            }
        }
        f.v[kGearHandle] = std::isnan(state) ? f.v[kGearHandle] : state;
    }
    clip.present.set(static_cast<size_t>(kGearHandle));
    return true;
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
