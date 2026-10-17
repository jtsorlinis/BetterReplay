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

}  // namespace

EventDetector::EventDetector(const DetectorConfig& config) : cfg_(config) { disarm(0.0, nullptr); }

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
    gHistory_.clear();
    if (reason) {
        logf("detector disarmed (%s)%s; arming in %.0f s", reason, hadEvent ? ", in-progress event discarded" : "",
             cfg_.armDelaySec);
    }
}

void EventDetector::notifyResumedAfterPause() { pendingContinuityCheck_ = true; }

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
                disarm(s.t, "relocated during pause");
            } else if (cfg_.holdOffNewEventsAfterPause && armed_) {
                holdNewUntil_ = s.t + cfg_.armDelaySec;
                logf("resumed after pause: new events held off for %.0f s", cfg_.armDelaySec);
            }
        }
    }

    gHistory_.push({s.t, s.gForce});

    const bool onGround = s.onGround;
    if (!haveTrack_) {
        haveTrack_ = true;
        prevOnGround_ = onGround;
        groundRunStart_ = onGround ? s.t : NAN;
        airRunStart_ = onGround ? NAN : s.t;
        lastSlowGround_ = NAN;
    }
    const bool wasOnGround = prevOnGround_;
    if (onGround && !wasOnGround) groundRunStart_ = s.t;
    if (!onGround && wasOnGround) airRunStart_ = s.t;
    if (onGround && s.groundSpeedKt < cfg_.takeoffSlowGroundSpeedKt) lastSlowGround_ = s.t;

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
        onLiftoff(s, out, n);
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

void EventDetector::onLiftoff(const DetectorSample& s, DetectedEvent* out, int& n) {
    if (landing_.active) {
        if (landing_.confirmed) {
            logf("landing: airborne again");
            finishLanding(s.t, out, n);
        } else {
            landing_.bouncing = true;
            landing_.vsRead = true;  // keep the first touchdown's rate, not a later bounce's
            landing_.bounceLiftoffT = s.t;
            logf("landing: bounced, airborne %.1f s after touchdown", s.t - landing_.touchdownT);
        }
    }

    if (!takeoff_.active && !newEventsHeld(s.t)) {
        double start = std::isnan(groundRunStart_) ? s.t : groundRunStart_;
        if (!std::isnan(lastSlowGround_) && lastSlowGround_ > start) start = lastSlowGround_;
        start = std::max(start, s.t - cfg_.takeoffMaxPreRollSec);
        takeoff_ = Takeoff();
        takeoff_.active = true;
        takeoff_.liftoffT = s.t;
        takeoff_.startT = start;
        if (!landing_.active) logf("liftoff: takeoff candidate (roll began %.1f s earlier)", s.t - start);
    }
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
        landing_.peakG = maxIgnoringNaN(landing_.peakG, s.gForce);
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
        double peak = s.gForce;
        for (size_t i = 0; i < gHistory_.size(); ++i) {
            const GSample& g = gHistory_.at(i);
            if (g.t >= s.t - cfg_.peakGWindowBeforeTouchdownSec) peak = maxIgnoringNaN(peak, g.g);
        }
        landing_.peakG = peak;
        logf("touchdown: landing candidate (VS %.0f fpm, GS %.0f kt)", last_.verticalSpeedFpm, s.groundSpeedKt);
    }
}

void EventDetector::airborneTick(const DetectorSample& s, DetectedEvent* out, int& n) {
    if (landing_.active) {  // only possible while bouncing
        landing_.peakG = maxIgnoringNaN(landing_.peakG, s.gForce);
        if (s.t - landing_.bounceLiftoffT > cfg_.landingMaxBounceAirborneSec) {
            logf("landing candidate dropped: airborne %.0f s after touching down", s.t - landing_.bounceLiftoffT);
            landing_ = Landing();
        }
    }

    if (!takeoff_.active) return;
    const double sinceLiftoff = s.t - takeoff_.liftoffT;
    if (!takeoff_.confirmed) {
        if (sinceLiftoff >= cfg_.takeoffMinAirborneSec && s.aglFt > cfg_.takeoffMinAglFt) {
            takeoff_.confirmed = true;
            logf("takeoff confirmed (%.1f s airborne, AGL %.0f ft)", sinceLiftoff, s.aglFt);
            if (landing_.active) {
                logf("landing candidate dropped: climbed away before %.0f s on the ground", cfg_.landingMinGroundSec);
                landing_ = Landing();
            }
        } else if (sinceLiftoff >= cfg_.takeoffMaxAfterLiftoffSec) {
            logf("takeoff candidate dropped: still below %.0f ft AGL after %.0f s", cfg_.takeoffMinAglFt, sinceLiftoff);
            takeoff_ = Takeoff();
            return;
        }
    }
    if (takeoff_.confirmed &&
        (s.aglFt >= cfg_.takeoffEndAglFt || sinceLiftoff >= cfg_.takeoffMaxAfterLiftoffSec)) {
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
    if (landing_.confirmed &&
        (s.groundSpeedKt < cfg_.landingEndGroundSpeedKt || sinceTouchdown >= cfg_.landingMaxAfterTouchdownSec)) {
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
