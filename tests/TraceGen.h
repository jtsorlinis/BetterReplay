// Builds synthetic 30 Hz flight traces for detector and engine tests.
#pragma once

#include <cmath>
#include <vector>

#include "EventDetector.h"

namespace tracegen {

struct TraceGen {
    double dt = 1.0 / 30.0;
    double t = 0.0;
    double latDeg = -33.9461;  // somewhere near YSSY
    double lonDeg = 151.1772;
    double elevFt = 21.0;
    double aglFt = 0.0;
    double gsKt = 0.0;
    double vsFpm = 0.0;
    double tdNormalFps = NAN;
    std::vector<br::DetectorSample> samples;

    TraceGen(bool onGround, double agl, double gs) : aglFt(agl), gsKt(gs) { push(onGround); }

    void push(bool onGround) {
        br::DetectorSample s;
        s.t = t;
        s.onGround = onGround;
        s.aglFt = aglFt;
        s.groundSpeedKt = gsKt;
        s.verticalSpeedFpm = vsFpm;
        s.gForce = 1.0;
        s.latDeg = latDeg;
        s.lonDeg = lonDeg;
        s.altFt = elevFt + aglFt;
        s.touchdownNormalVelFps = tdNormalFps;
        samples.push_back(s);
    }

    // Ramps ground speed and AGL linearly to the end values over `sec`; vertical speed follows
    // the AGL ramp. The aircraft moves due south at the current ground speed.
    void segment(double sec, double gsEnd, double aglEnd, bool onGround) {
        const int steps = static_cast<int>(std::lround(sec / dt));
        const double gs0 = gsKt;
        const double agl0 = aglFt;
        vsFpm = (aglEnd - agl0) / sec * 60.0;
        for (int i = 1; i <= steps; ++i) {
            const double f = static_cast<double>(i) / steps;
            gsKt = gs0 + (gsEnd - gs0) * f;
            aglFt = agl0 + (aglEnd - agl0) * f;
            t += dt;
            latDeg -= gsKt * 0.514444 * dt / 111320.0;
            push(onGround);
        }
    }

    void hold(double sec, bool onGround) { segment(sec, gsKt, aglFt, onGround); }

    // Sets the G load of the most recent sample.
    void spike(double g) { samples.back().gForce = g; }

    double now() const { return samples.back().t; }
    size_t index() const { return samples.size(); }  // index of the next sample to be pushed
};

struct RunResult {
    std::vector<br::DetectedEvent> events;
};

// Feeds samples [from, to) into the detector and collects finished events.
inline void feed(br::EventDetector& d, const std::vector<br::DetectorSample>& s, size_t from, size_t to,
                 RunResult& r) {
    for (size_t i = from; i < to && i < s.size(); ++i) {
        br::DetectedEvent out[2];
        const int n = d.update(s[i], out);
        for (int k = 0; k < n; ++k) r.events.push_back(out[k]);
    }
}

inline RunResult runAll(br::EventDetector& d, const std::vector<br::DetectorSample>& s) {
    RunResult r;
    feed(d, s, 0, s.size(), r);
    return r;
}

// Standard approach from the current AGL down to the flare, touching down at `touchdownGs`.
// Returns the sample index of the first on-ground sample (the next one to be pushed).
inline size_t approachAndTouchdown(TraceGen& g, double touchdownGs) {
    g.segment(170.0, touchdownGs + 10.0, 50.0, false);
    g.segment(20.0, touchdownGs, 0.0, false);  // flare: 50 ft in 20 s = -150 fpm at contact
    return g.index();
}

}  // namespace tracegen
