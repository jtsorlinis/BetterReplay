#include "PlaybackController.h"

#include <cmath>
#include <utility>

#include "Interp.h"

namespace br {

void PlaybackController::load(Clip&& clip, const std::string& file) {
    clip_ = std::move(clip);
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
