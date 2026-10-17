#include "Interp.h"

#include <cmath>

namespace br {

double wrap180(double deg) {
    double r = std::fmod(deg, 360.0);
    if (r <= -180.0) r += 360.0;
    if (r > 180.0) r -= 360.0;
    return r;
}

double wrap360(double deg) {
    double r = std::fmod(deg, 360.0);
    if (r < 0.0) r += 360.0;
    if (r >= 360.0) r -= 360.0;
    return r;
}

double lerp(double a, double b, double f) { return a + (b - a) * f; }

double lerpAngle180(double a, double b, double f) { return wrap180(a + wrap180(b - a) * f); }

double lerpAngle360(double a, double b, double f) { return wrap360(a + wrap180(b - a) * f); }

double interpolate(Interp kind, double a, double b, double f) {
    if (std::isnan(a)) return b;
    if (std::isnan(b)) return a;
    switch (kind) {
        case Interp::Linear: return lerp(a, b, f);
        case Interp::Angle180: return lerpAngle180(a, b, f);
        case Interp::Angle360: return lerpAngle360(a, b, f);
        case Interp::Step: return a;
    }
    return a;
}

void sampleFrames(const Frame* frames, size_t count, double t, Frame& out, size_t* hint) {
    if (count == 0) return;
    if (count == 1 || t <= frames[0].t) {
        out = frames[0];
        out.t = t;
        if (hint) *hint = 0;
        return;
    }
    if (t >= frames[count - 1].t) {
        out = frames[count - 1];
        out.t = t;
        if (hint) *hint = count - 2;
        return;
    }

    // Find i such that frames[i].t <= t < frames[i + 1].t.
    size_t i = 0;
    const size_t cached = hint ? *hint : 0;
    if (cached + 1 < count && frames[cached].t <= t && t < frames[cached + 1].t) {
        i = cached;
    } else if (cached + 2 < count && frames[cached + 1].t <= t && t < frames[cached + 2].t) {
        i = cached + 1;
    } else {
        size_t lo = 0;
        size_t hi = count - 1;
        while (hi - lo > 1) {
            const size_t mid = lo + (hi - lo) / 2;
            if (frames[mid].t <= t) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        i = lo;
    }
    if (hint) *hint = i;

    const Frame& a = frames[i];
    const Frame& b = frames[i + 1];
    const double span = b.t - a.t;
    const double f = span > 0.0 ? (t - a.t) / span : 0.0;
    out.t = t;
    for (int c = 0; c < kChannelCount; ++c) {
        out.v[c] = interpolate(kChannels[c].interp, a.v[c], b.v[c], f);
    }
}

}  // namespace br
