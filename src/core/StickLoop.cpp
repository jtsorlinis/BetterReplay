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
    built_ = NAN;
    failed_ = false;
    stuckFor_ = 0.0;
    stuckFrom_ = 0.0;
    event_ = Event::None;
    wigDir_ = 0;
    wigExt_ = wigPrev_ = NAN;
    wigCount_ = 0;
    damp_ = 1.0;
}

void StickLoop::forget() {
    reset();
    sign_ = 1;
    flipped_ = false;
    workedSec_ = 0.0;
}

double StickLoop::update(double targetDeg, double aheadDeg, double aheadRate, double liveDeg, double dt, bool onGround,
                         const StickReadings& reads) {
    if (failed_ || !std::isfinite(targetDeg) || !std::isfinite(liveDeg) || !std::isfinite(dt) || dt <= 0.0) {
        return stick();
    }
    if (!std::isfinite(aheadDeg)) aheadDeg = targetDeg;
    dt = dt > 0.1 ? 0.1 : dt;  // a hitch: don't take one big step
    ramp(dt, onGround);
    const double kp = cfg_.groundKp + (cfg_.airKp - cfg_.groundKp) * air_;
    const double ki = cfg_.groundKi + (cfg_.airKi - cfg_.groundKi) * air_;
    const double e = targetDeg - liveDeg;

    // The stick that asks for the motion the aircraft reads (it then holds the surface still) ...
    hold_ = air_ * (term(cfg_.holdG, reads.g - 1.0) + term(cfg_.holdPitchRate, reads.pitchRate) +
                    term(cfg_.holdRollRate, reads.rollRate) + term(cfg_.holdYawRate, reads.yawRate) +
                    term(cfg_.holdBeta, reads.betaDeg));
    // ... plus the clip's own movement, as it will be by the time the aircraft answers (the loop's
    // own terms go by where the clip is now: by the lead, they'd push it a second time) ...
    const double rate = std::isfinite(aheadRate) ? aheadRate : 0.0;
    const bool builds = cfg_.airDirect && cfg_.airRateK > 0.0;
    if (builds && (!std::isfinite(built_) || air_ <= 0.0)) built_ = liveDeg / cfg_.airRateK;  // where it is, all built up
    const double built = builds ? cfg_.airRateK * built_ : 0.0;
    const double air =
        cfg_.airK > 0.0 ? (cfg_.airDirect ? aheadDeg + cfg_.airSettleSec * rate - built : rate) / cfg_.airK : 0.0;
    const double ground = cfg_.groundK > 0.0 ? aheadDeg / cfg_.groundK : 0.0;
    ff_ = air_ * air + (1.0 - air_) * ground;
    // ... and the loop's own terms for the rest. The integral only where it doesn't wind further
    // into full travel. All but the hold less after a wiggle.
    const double base = hold_ + damp_ * (ff_ + kp * e);
    const double next = integral_ + damp_ * ki * e * dt;
    const double want = base + next;
    if (!((want > 1.0 && e > 0.0) || (want < -1.0 && e < 0.0))) integral_ = clamp(next, -1.0, 1.0);
    const double target = clamp(base + integral_, -1.0, 1.0);
    const double step = cfg_.maxRate * dt;
    drive_ += clamp(target - drive_, -step, step);
    if (builds) {
        built_ += air_ * (drive_ - hold_) * dt;
        // Kept to the surface: where it is, less what the stick puts it at once, is what the
        // aircraft has built up. The tally alone drifts wherever the hold isn't quite the
        // aircraft's: through an A350 landing it ran to 3.7 sticks' worth and the loop's integral
        // ran out against it (the rudder 13 degrees off, then 16 at the touchdown's blend).
        const double est = (liveDeg - cfg_.airK * (drive_ - hold_)) / cfg_.airRateK;
        built_ += (est - built_) * clamp(dt / StickLoopConfig::kBuiltSec, 0.0, 1.0);
        built_ = clamp(built_, -60.0 / cfg_.airRateK, 60.0 / cfg_.airRateK);
    }
    clock_ += dt;
    watchWiggle(liveDeg);
    if (std::fabs(e) < cfg_.workedDeg && std::fabs(rate) >= cfg_.workedRate) workedSec_ += dt;

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
    stuckFor_ = 0.0;
    if (worked()) {
        event_ = Event::Held;  // the aircraft's doing: kept on, as it is
        return stick();
    }
    drive_ = 0.0;
    integral_ = 0.0;
    if (!flipped_) {
        flipped_ = true;
        sign_ = -sign_;
        workedSec_ = 0.0;
        event_ = Event::Flipped;
    } else {
        failed_ = true;
        sign_ = -sign_;  // as it was: neither way round moved it
        flipped_ = false;
        event_ = Event::Failed;
    }
    return stick();
}

void StickLoop::ramp(double dt, bool onGround) {
    const double toward = onGround ? 0.0 : 1.0;
    if (!std::isfinite(air_)) {
        air_ = toward;  // the first frame: no ramp
    } else {
        const double step = cfg_.rampSec > 0.0 ? dt / cfg_.rampSec : 1.0;
        air_ += clamp(toward - air_, -step, step);
    }
}

double StickLoop::centre(double dt, bool onGround) {
    if (failed_ || !std::isfinite(dt) || dt <= 0.0) return stick();
    dt = dt > 0.1 ? 0.1 : dt;
    ramp(dt, onGround);  // (the laws change as ever)
    hold_ = ff_ = 0.0;
    integral_ = 0.0;
    built_ = NAN;  // from the surface again
    stuckFor_ = 0.0;
    // The surface steps with the pedals: where it went meanwhile isn't a wiggle.
    wigDir_ = 0;
    wigExt_ = wigPrev_ = NAN;
    const double step = cfg_.maxRate * dt;
    drive_ += clamp(-drive_, -step, step);
    clock_ += dt;
    return stick();
}

void StillStretches::find(const Frame* frames, size_t count, int channel, double minSec) {
    spans_.clear();
    if (!frames || channel < 0 || channel >= kChannelCount) return;
    size_t i = 0;
    while (i < count) {
        if (frames[i].v[channel] != 0.0) {
            ++i;
            continue;
        }
        size_t j = i;
        while (j + 1 < count && frames[j + 1].v[channel] == 0.0) ++j;
        if (frames[j].t - frames[i].t >= minSec) spans_.emplace_back(frames[i].t, frames[j].t);
        i = j + 1;
    }
}

bool StillStretches::at(double t) const {
    for (const auto& s : spans_) {
        if (t < s.first) return false;  // (in time order)
        if (t <= s.second) return true;
    }
    return false;
}

double StillStretches::seconds() const {
    double sum = 0.0;
    for (const auto& s : spans_) sum += s.second - s.first;
    return sum;
}

void StickLoop::watchWiggle(double liveDeg) {
    if (!std::isfinite(liveDeg)) return;
    if (!std::isfinite(wigExt_)) {
        wigExt_ = liveDeg;
        wigDir_ = 0;
        return;
    }
    // Going one way, the furthest it got; back turnDeg from there, a turn, counted where it swung
    // swingDeg or more from the turn before.
    bool turned = false;
    const double turn = cfg_.wiggleTurnDeg;
    if (wigDir_ == 0) {
        if (liveDeg > wigExt_ + turn || liveDeg < wigExt_ - turn) {
            wigDir_ = liveDeg > wigExt_ ? 1 : -1;
            wigPrev_ = wigExt_;
            wigExt_ = liveDeg;
        }
    } else if (wigDir_ * (liveDeg - wigExt_) > 0.0) {
        wigExt_ = liveDeg;
    } else if (wigDir_ * (wigExt_ - liveDeg) > turn) {
        turned = std::isfinite(wigPrev_) && std::fabs(wigExt_ - wigPrev_) >= cfg_.wiggleSwingDeg;
        wigPrev_ = wigExt_;
        wigExt_ = liveDeg;
        wigDir_ = -wigDir_;
    }
    if (!turned) return;
    // The turns within wiggleSec, newest last.
    constexpr int kCap = static_cast<int>(sizeof(wigTimes_) / sizeof(wigTimes_[0]));
    int keep = 0;
    for (int i = 0; i < wigCount_; ++i) {
        if (clock_ - wigTimes_[i] <= cfg_.wiggleSec) wigTimes_[keep++] = wigTimes_[i];
    }
    wigCount_ = keep;
    if (wigCount_ < kCap) wigTimes_[wigCount_++] = clock_;
    if (wigCount_ < cfg_.wiggleTurns || damp_ <= cfg_.minDamp) return;
    damp_ = std::fmax(cfg_.minDamp, damp_ * 0.5);
    wigCount_ = 0;
    if (event_ == Event::None) event_ = Event::Damped;
}

StickLoopConfig StickLoopConfig::forPlant(const StickPlant& plant) const {
    StickLoopConfig c = *this;
    if (!plant.valid()) return c;
    c.leadSec = plant.lagSec;
    if (plant.form == StickPlant::Form::Direct) {
        c.airDirect = true;
        c.airK = plant.gain;
        c.airSettleSec = plant.settleSec;
        c.airRateK = plant.rateGain > 0.0 ? plant.rateGain : 0.0;
        c.airKp = 0.0;
        c.airKi = clamp(0.9 / plant.gain, 0.005, 0.1);
    } else {
        // The products with the rate this config's gains were made for (the elevator's otherwise).
        // A surface that answers sooner takes quicker corrections, up to kFasterAtMost times: the
        // A220's elevator, at 0.07 s, drifted 2 to 3 degrees off for tens of seconds on the A380's
        // (0.3 s) products, and stayed closer on the A380's gains as they were.
        const StickLoopConfig like = airDirect ? StickLoopConfig::elevator() : *this;
        constexpr double kFasterAtMost = 3.0;
        const double faster = clamp(like.leadSec / std::fmax(plant.lagSec, 0.01), 1.0, kFasterAtMost);
        c.airDirect = false;
        c.airK = plant.gain;
        c.airSettleSec = 0.0;
        c.airRateK = 0.0;
        c.airKp = like.airKp * like.airK / plant.gain * faster;
        c.airKi = like.airKi * like.airK / plant.gain * faster;
    }
    if (plant.holdYawRate != 0.0) c.holdYawRate = plant.holdYawRate;
    const double holds = plant.holds ? clamp(plant.holdScale, 0.0, 2.0) : 0.0;
    c.holdYawRate *= holds;
    c.holdG *= holds;
    c.holdPitchRate *= holds;
    c.holdRollRate *= holds;
    c.holdBeta *= holds;
    return c;
}

}  // namespace br
