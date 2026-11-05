#include "StickLoop.h"

#include <cmath>

namespace br {

namespace {
double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
double term(double coeff, double v) { return coeff != 0.0 && std::isfinite(v) ? coeff * v : 0.0; }
}  // namespace

void StickLoop::reset() {
    drive_ = 0.0;
    integral_ = 0.0;
    air_ = NAN;
    hold_ = ff_ = 0.0;
    failed_ = false;
    stuckFor_ = 0.0;
    stuckFrom_ = 0.0;
    event_ = Event::None;
}

void StickLoop::forget() {
    reset();
    sign_ = 1;
    flipped_ = false;
}

double StickLoop::update(double targetDeg, double aheadDeg, double aheadRate, double liveDeg, double dt, bool onGround,
                         const StickReadings& reads) {
    if (failed_ || !std::isfinite(targetDeg) || !std::isfinite(liveDeg) || !std::isfinite(dt) || dt <= 0.0) {
        return stick();
    }
    if (!std::isfinite(aheadDeg)) aheadDeg = targetDeg;
    dt = dt > 0.1 ? 0.1 : dt;  // a hitch: don't take one big step
    const double toward = onGround ? 0.0 : 1.0;
    if (!std::isfinite(air_)) {
        air_ = toward;  // the first frame: no ramp
    } else {
        const double ramp = cfg_.rampSec > 0.0 ? dt / cfg_.rampSec : 1.0;
        air_ += clamp(toward - air_, -ramp, ramp);
    }
    const double kp = cfg_.groundKp + (cfg_.airKp - cfg_.groundKp) * air_;
    const double ki = cfg_.groundKi + (cfg_.airKi - cfg_.groundKi) * air_;
    const double e = targetDeg - liveDeg;

    // The stick that asks for the motion the aircraft reads (it then holds the surface still) ...
    hold_ = air_ * (term(cfg_.holdG, reads.g - 1.0) + term(cfg_.holdPitchRate, reads.pitchRate) +
                    term(cfg_.holdRollRate, reads.rollRate) + term(cfg_.holdBeta, reads.betaDeg));
    // ... plus the clip's own movement, as it will be by the time the aircraft answers (the loop's
    // own terms go by where the clip is now: by the lead, they'd push it a second time) ...
    const double rate = std::isfinite(aheadRate) ? aheadRate : 0.0;
    const double air = cfg_.airK > 0.0 ? (cfg_.airDirect ? aheadDeg : rate) / cfg_.airK : 0.0;
    const double ground = cfg_.groundK > 0.0 ? aheadDeg / cfg_.groundK : 0.0;
    ff_ = air_ * air + (1.0 - air_) * ground;
    // ... and the loop's own terms for the rest. The integral only where it doesn't wind further
    // into full travel.
    const double base = hold_ + ff_ + kp * e;
    const double next = integral_ + ki * e * dt;
    const double want = base + next;
    if (!((want > 1.0 && e > 0.0) || (want < -1.0 && e < 0.0))) integral_ = clamp(next, -1.0, 1.0);
    const double target = clamp(base + integral_, -1.0, 1.0);
    const double step = cfg_.maxRate * dt;
    drive_ += clamp(target - drive_, -step, step);

    // At full travel the way the surface should go, and it isn't getting there. (While the clip's
    // surface moves fast, a hard input the stick can't keep up with isn't a sign of the stick being
    // the wrong way round.)
    const bool pinned = std::fabs(drive_) >= 0.98 && std::fabs(e) > cfg_.stuckDeg && drive_ * e > 0.0 &&
                        std::fabs(ff_ - (1.0 - air_) * ground) < 0.5;
    if (!pinned) {
        stuckFor_ = 0.0;
        return stick();
    }
    if (stuckFor_ == 0.0 || std::fabs(e) < stuckFrom_ - cfg_.closerDeg) {
        stuckFrom_ = std::fabs(e);  // (getting closer: the window starts again)
        stuckFor_ = 1e-9;
    }
    stuckFor_ += dt;
    if (stuckFor_ < cfg_.stuckSec) return stick();
    drive_ = 0.0;
    integral_ = 0.0;
    stuckFor_ = 0.0;
    if (!flipped_) {
        flipped_ = true;
        sign_ = -sign_;
        event_ = Event::Flipped;
    } else {
        failed_ = true;
        event_ = Event::Failed;
    }
    return stick();
}

}  // namespace br
