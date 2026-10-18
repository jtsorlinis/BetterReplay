#include "EventDetector.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>

#include "Interp.h"

namespace br {

namespace {

double distanceM(double lat1, double lon1, double lat2, double lon2) {
    constexpr double kEarthRadiusM = 6371000.0;
    constexpr double kDegToRad = 0.017453292519943295;
    const double x = wrap180(lon2 - lon1) * kDegToRad * std::cos((lat1 + lat2) * 0.5 * kDegToRad);
    const double y = (lat2 - lat1) * kDegToRad;
    return std::sqrt(x * x + y * y) * kEarthRadiusM;
}

double maxIgnoringNaN(double a, double b) {
    if (std::isnan(a)) return b;
    if (std::isnan(b)) return a;
    return a > b ? a : b;
}

constexpr double kPowerStepSec = 0.25;  // the engine power history's spacing

}  // namespace

EventDetector::EventDetector(const DetectorConfig& config) : cfg_(config) {
    power_.reset(static_cast<size_t>((cfg_.takeoffMaxPreRollSec + 10.0) / kPowerStepSec) + 8);
    disarm(0.0, nullptr);
}

void EventDetector::logf(const char* fmt, ...) {
    if (!log_) return;
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_(logCtx_, buf);
}

void EventDetector::disarm(double t, const char* reason) {
    const bool hadEvent = takeoff_.active || landing_.active;
    takeoff_ = Takeoff();
    landing_ = Landing();
    armed_ = false;
    armAt_ = t + cfg_.armDelaySec;
    holdNewUntil_ = -1e300;
    pendingContinuityCheck_ = false;
    haveTrack_ = false;
    groundRunStart_ = NAN;
    airRunStart_ = NAN;
    lastSlowGround_ = NAN;
    runwayEntry_ = NAN;
    gHistory_.clear();
    power_.clear();
    if (reason) {
        logf("detector disarmed (%s)%s; arming in %.0f s", reason, hadEvent ? ", in-progress event discarded" : "",
             cfg_.armDelaySec);
    }
}

void EventDetector::notifyResumedAfterPause() { pendingContinuityCheck_ = true; }

int EventDetector::finishInProgress(double t, DetectedEvent out[2]) {
    int n = 0;
    finishPending(t, out, n);
    return n;
}

void EventDetector::finishPending(double t, DetectedEvent* out, int& n) {
    if (landing_.active) {
        logf("landing ends here: the recording stops");
        finishLanding(t, out, n);
    }
    if (takeoff_.active && takeoff_.confirmed) {
        logf("takeoff ends here: the recording stops");
        finishTakeoff(t, out, n);
    }
    takeoff_ = Takeoff();
}

const char* EventDetector::phase() const {
    if (!armed_) return "disarmed";
    if (landing_.active) {
        if (landing_.bouncing) return "landing-bounce";
        return landing_.confirmed ? "landing-rollout" : "landing-touchdown";
    }
    if (takeoff_.active) return takeoff_.confirmed ? "takeoff-climb" : "takeoff-liftoff";
    return prevOnGround_ ? "ground" : "airborne";
}

int EventDetector::update(const DetectorSample& s, DetectedEvent out[2]) {
    int n = 0;

    if (pendingContinuityCheck_) {
        pendingContinuityCheck_ = false;
        if (haveTrack_) {
            const double moved = distanceM(last_.latDeg, last_.lonDeg, s.latDeg, s.lonDeg);
            const double climbed = std::fabs(s.altFt - last_.altFt);
            if (moved > cfg_.continuityMaxDistanceM || climbed > cfg_.continuityMaxAltFt) {
                logf("aircraft moved %.0f m / %.0f ft while paused", moved, climbed);
                finishPending(last_.t, out, n);
                disarm(s.t, "relocated during pause");
            } else if (cfg_.holdOffNewEventsAfterPause && armed_) {
                holdNewUntil_ = s.t + cfg_.armDelaySec;
                logf("resumed after pause: new events held off for %.0f s", cfg_.armDelaySec);
            }
        }
    }

    gHistory_.push({s.t, s.gForce});
    if (std::isfinite(s.enginePower) && (power_.empty() || s.t - power_.back().t >= kPowerStepSec - 1e-9)) {
        power_.push({s.t, s.enginePower});
    }

    const bool onGround = s.onGround;
    const bool tracked = haveTrack_;
    if (!haveTrack_) {
        haveTrack_ = true;
        prevOnGround_ = onGround;
        groundRunStart_ = onGround ? s.t : NAN;
        airRunStart_ = onGround ? NAN : s.t;
        lastSlowGround_ = NAN;
        runwayEntry_ = NAN;  // already on one (spawned there) isn't entering it
    }
    const bool wasOnGround = prevOnGround_;
    if (onGround && !wasOnGround) groundRunStart_ = s.t;
    if (!onGround && wasOnGround) airRunStart_ = s.t;
    if (onGround && s.groundSpeedKt < cfg_.takeoffSlowGroundSpeedKt) lastSlowGround_ = s.t;
    // Rolled onto a runway (landing on one doesn't count).
    if (tracked && onGround && wasOnGround && s.onRunway && !last_.onRunway) runwayEntry_ = s.t;

    if (!armed_) {
        if (s.t >= armAt_) {
            armed_ = true;
            logf("detector armed (%s, AGL %.0f ft, GS %.0f kt)", onGround ? "on ground" : "airborne", s.aglFt,
                 s.groundSpeedKt);
        }
        prevOnGround_ = onGround;
        last_ = s;
        return n;
    }

    if (wasOnGround && !onGround) {
        onLiftoff(s);
    } else if (!wasOnGround && onGround) {
        onTouchdown(s, out, n);
    }

    if (onGround) {
        groundTick(s, out, n);
    } else {
        airborneTick(s, out, n);
    }

    prevOnGround_ = onGround;
    last_ = s;
    return n;
}

void EventDetector::onLiftoff(const DetectorSample& s) {
    // A bounce, unless it stays airborne for landingGoAroundSec (airborneTick).
    if (landing_.active) {
        landing_.bouncing = true;
        landing_.vsRead = true;  // keep the first touchdown's rate, not a later bounce's
        landing_.bounceLiftoffT = s.t;
        logf("landing: airborne again %.1f s after touchdown", s.t - landing_.touchdownT);
    }

    if (!takeoff_.active && !newEventsHeld(s.t)) {
        const double ground = std::isnan(groundRunStart_) ? s.t : groundRunStart_;
        const double earliest = s.t - cfg_.takeoffMaxPreRollSec;
        const double spool = spoolUpStart(std::max(earliest + cfg_.takeoffPreSpoolSec, ground), s.t);
        const bool entered = !std::isnan(runwayEntry_) && runwayEntry_ >= ground;  // in this ground run
        double start;
        if (entered) {
            start = runwayEntry_ - cfg_.takeoffPreRunwaySec;
            if (!std::isnan(spool)) start = std::max(start, spool - cfg_.takeoffMaxWaitBeforeSpoolSec);
        } else if (!std::isnan(spool)) {
            start = spool - cfg_.takeoffPreSpoolSec;
        } else {
            // Nothing else to go by: from the last moment below takeoffSlowGroundSpeedKt.
            start = ground;
            if (!std::isnan(lastSlowGround_) && lastSlowGround_ > start) start = lastSlowGround_;
        }
        start = std::max(start, earliest);
        takeoff_ = Takeoff();
        takeoff_.active = true;
        takeoff_.liftoffT = s.t;
        takeoff_.startT = start;
        takeoff_.flapsOut = !s.flapsRetracted;
        if (landing_.active) return;
        const char* flaps = takeoff_.flapsOut ? "flaps out" : "flaps up";
        if (entered) {
            logf("liftoff: takeoff candidate (onto the runway %.1f s earlier, %s)", s.t - runwayEntry_, flaps);
        } else if (!std::isnan(spool)) {
            logf("liftoff: takeoff candidate (engines spooled up %.1f s earlier, %s)", s.t - spool, flaps);
        } else {
            logf("liftoff: takeoff candidate (roll began %.1f s earlier, %s)", s.t - start, flaps);
        }
    }
}

double EventDetector::spoolUpStart(double from, double to) const {
    const auto time = [](const PowerSample& p) { return p.t; };
    const size_t first = lowerBoundBy(power_, from, time);
    const size_t end = std::min(lowerBoundBy(power_, to + 1e-6, time), power_.size());
    if (end < first + 4) return NAN;
    double top = -INFINITY;
    for (size_t i = first; i < end; ++i) top = std::max(top, power_.at(i).p);
    if (!(top > 0.0)) return NAN;

    // Idle: the lowest the engines held steady, running (not shut down or starting) and well
    // below takeoff power (not a hold at it on the brakes). A short hold on the way up (40% N1 to
    // let the engines stabilise) is above idle, so it's part of the spool-up.
    const double band = cfg_.takeoffIdleBand * top;
    const double running = 0.1 * top;
    double idle = INFINITY;
    for (size_t i = first; i < end; ++i) {
        double lo = power_.at(i).p;
        double hi = lo;
        for (size_t j = i + 1; j < end; ++j) {
            lo = std::min(lo, power_.at(j).p);
            hi = std::max(hi, power_.at(j).p);
            if (hi - lo > band) break;
            if (power_.at(j).t - power_.at(i).t >= cfg_.takeoffIdleSteadySec) {
                if (lo >= running && hi <= 0.8 * top) idle = std::min(idle, hi);
                break;
            }
        }
    }
    if (std::isinf(idle)) {  // never steady: the lowest it ran at
        for (size_t i = first; i < end; ++i) {
            if (power_.at(i).p >= running) idle = std::min(idle, power_.at(i).p);
        }
    }
    if (std::isinf(idle) || top - idle < 0.1 * top) return NAN;  // no spool-up in here
    for (size_t i = end; i-- > first;) {
        if (power_.at(i).p <= idle + band) return power_.at(i).t;
    }
    return NAN;
}

double EventDetector::peakGAround(const DetectorSample& s) const {
    double peak = s.gForce;
    for (size_t i = 0; i < gHistory_.size(); ++i) {
        const GSample& g = gHistory_.at(i);
        if (g.t >= s.t - cfg_.peakGWindowBeforeTouchdownSec) peak = maxIgnoringNaN(peak, g.g);
    }
    return peak;
}

void EventDetector::onTouchdown(const DetectorSample& s, DetectedEvent* out, int& n) {
    if (takeoff_.active) {
        if (takeoff_.confirmed) {
            logf("takeoff: back on the ground before the end condition");
            finishTakeoff(s.t, out, n);
        } else {
            if (!landing_.active) logf("takeoff candidate dropped: back on the ground after %.1f s", s.t - takeoff_.liftoffT);
            takeoff_ = Takeoff();
        }
    }

    if (landing_.active && landing_.bouncing) {
        landing_.bouncing = false;
        ++landing_.bounces;
        landing_.peakG = maxIgnoringNaN(landing_.peakG, peakGAround(s));
        logf("landing: touchdown again (bounce %u)", static_cast<unsigned>(landing_.bounces));
        return;
    }

    if (!landing_.active && !newEventsHeld(s.t)) {
        const double airborneFor = std::isnan(airRunStart_) ? 1e300 : s.t - airRunStart_;
        if (airborneFor < cfg_.landingMinAirborneSec) {
            logf("touchdown after only %.1f s airborne: not a landing", airborneFor);
            return;
        }
        landing_ = Landing();
        landing_.active = true;
        landing_.touchdownT = s.t;
        landing_.touchdownVsFpm = last_.verticalSpeedFpm;  // fallback if the touchdown simvar is missing
        landing_.peakG = peakGAround(s);
        logf("touchdown: landing candidate (VS %.0f fpm, GS %.0f kt)", last_.verticalSpeedFpm, s.groundSpeedKt);
    }
}

void EventDetector::airborneTick(const DetectorSample& s, DetectedEvent* out, int& n) {
    if (landing_.active && s.t - landing_.bounceLiftoffT > cfg_.landingGoAroundSec) {  // only while bouncing
        logf("landing: airborne again for %.0f s", cfg_.landingGoAroundSec);
        finishLanding(s.t, out, n);
    }

    // Until then the liftoff may still be a bounce, so a takeoff from it waits.
    if (!takeoff_.active || landing_.active) return;
    const double sinceLiftoff = s.t - takeoff_.liftoffT;
    if (!takeoff_.confirmed) {
        if (sinceLiftoff >= cfg_.takeoffMinAirborneSec && s.aglFt > cfg_.takeoffMinAglFt) {
            takeoff_.confirmed = true;
            logf("takeoff confirmed (%.1f s airborne, AGL %.0f ft)", sinceLiftoff, s.aglFt);
        } else if (sinceLiftoff >= cfg_.takeoffAfterLiftoffSec) {
            logf("takeoff candidate dropped: still below %.0f ft AGL after %.0f s", cfg_.takeoffMinAglFt, sinceLiftoff);
            takeoff_ = Takeoff();
            return;
        }
    }
    if (!takeoff_.confirmed) return;
    const char* end = nullptr;
    if (takeoff_.flapsOut) {
        if (!s.flapsRetracted) {
            takeoff_.cleanT = NAN;  // (out again)
        } else if (std::isnan(takeoff_.cleanT)) {
            takeoff_.cleanT = s.t;
        }
        if (!std::isnan(takeoff_.cleanT) && s.t - takeoff_.cleanT >= cfg_.takeoffAfterCleanSec) end = "flaps up";
    } else if (sinceLiftoff >= cfg_.takeoffAfterLiftoffSec) {
        end = "flaps were up at liftoff: the time after it is up";
    }
    if (!end && sinceLiftoff >= cfg_.takeoffMaxAfterLiftoffSec) end = "flaps still out at the time limit";
    if (end) {
        logf("takeoff: %s", end);
        finishTakeoff(s.t, out, n);
    }
}

void EventDetector::groundTick(const DetectorSample& s, DetectedEvent* out, int& n) {
    if (!landing_.active || landing_.bouncing) return;
    landing_.peakG = maxIgnoringNaN(landing_.peakG, s.gForce);
    const double sinceTouchdown = s.t - landing_.touchdownT;

    if (!landing_.vsRead && sinceTouchdown >= cfg_.touchdownVsReadDelaySec) {
        landing_.vsRead = true;
        if (std::isfinite(s.touchdownNormalVelFps) && s.touchdownNormalVelFps != 0.0) {
            landing_.touchdownVsFpm = -std::fabs(s.touchdownNormalVelFps) * 60.0;
        }
    }

    if (!landing_.confirmed && s.t - groundRunStart_ >= cfg_.landingMinGroundSec) {
        landing_.confirmed = true;
        logf("landing confirmed (%.1f s on the ground, %u bounce(s))", s.t - groundRunStart_,
             static_cast<unsigned>(landing_.bounces));
    }
    if (s.onRunway) {
        landing_.runwaySeen = true;
        landing_.offRunwayT = NAN;  // (crossing another one starts the count again)
    } else if (landing_.runwaySeen && std::isnan(landing_.offRunwayT)) {
        landing_.offRunwayT = s.t;
    }
    const char* end = nullptr;
    if (s.groundSpeedKt < cfg_.landingStopGroundSpeedKt) {
        end = "stopped";
    } else if (!std::isnan(landing_.offRunwayT) && s.t - landing_.offRunwayT >= cfg_.landingOffRunwaySec) {
        end = "off the runway";
    } else if (sinceTouchdown >= cfg_.landingMaxAfterTouchdownSec) {
        end = "still rolling at the time limit";
    }
    if (end) {
        logf("landing: %s", end);
        finishLanding(s.t, out, n);
    }
}

void EventDetector::finishTakeoff(double endT, DetectedEvent* out, int& n) {
    if (n < 2) {
        DetectedEvent& e = out[n++];
        e = DetectedEvent();
        e.type = EventType::Takeoff;
        e.startT = takeoff_.startT;
        e.endT = endT;
        e.eventT = takeoff_.liftoffT;
        logf("takeoff finished: %.1f s clip (liftoff at +%.1f s)", e.endT - e.startT, e.eventT - e.startT);
    }
    takeoff_ = Takeoff();
}

void EventDetector::finishLanding(double endT, DetectedEvent* out, int& n) {
    if (n < 2) {
        DetectedEvent& e = out[n++];
        e = DetectedEvent();
        e.type = EventType::Landing;
        e.startT = landing_.touchdownT - cfg_.landingPreRollSec;
        e.endT = endT;
        e.eventT = landing_.touchdownT;
        e.touchdownVsFpm = static_cast<float>(landing_.touchdownVsFpm);
        e.peakG = static_cast<float>(landing_.peakG);
        e.bounces = landing_.bounces;
        logf("landing finished: %.1f s clip, VS %.0f fpm, peak %.2f G, %u bounce(s)", e.endT - e.startT,
             landing_.touchdownVsFpm, landing_.peakG, static_cast<unsigned>(landing_.bounces));
    }
    landing_ = Landing();
}

}  // namespace br
