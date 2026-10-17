#pragma once

#include <cstddef>

#include "Channels.h"

namespace br {

double wrap180(double deg);  // (-180, 180]
double wrap360(double deg);  // [0, 360)

double lerp(double a, double b, double f);
double lerpAngle180(double a, double b, double f);
double lerpAngle360(double a, double b, double f);

// Interpolates one value; if either side is NaN the other side is used.
double interpolate(Interp kind, double a, double b, double f);

// Interpolated frame at time t from frames sorted by t (clamped to the ends).
// `hint` caches the last segment index so sequential playback is O(1); may be null.
void sampleFrames(const Frame* frames, size_t count, double t, Frame& out, size_t* hint);

}  // namespace br
