// How a fly-by-wire surface answers the stick, fitted from what the stick loop sent and what the
// surface did, for StickLoop to steer it by (StickLoopConfig::forPlant). The loop's defaults are
// the A380's, fitted by hand from its replays; on the other fly-by-wire aircraft they didn't fit.
// The A220's ailerons and the 777's elevator go to where the stick puts them within a frame or two
// (the loop, made for a surface that moves at a rate, wiggled them 7 times a second). The A330's,
// A340's, A320neo's, A220's, 777's and 787's surfaces answer 2 to 5 times as fast and within 0.03
// to 0.2 s, against the A380's 0.3 to 0.4 (the loop, sending the clip's movement that far ahead,
// had them 0.3 to 1 s early and overshooting).
//
// Fitted on the surface's quick movement only: the stick and the surface each less their own
// average over kSmoothSec either side. The slower part is the clip's own motion and what the
// fly-by-wire does with it (the replay writes the clip's rates, which move with its surfaces, so
// a fit over everything credited the clip's surfaces to them). Two forms, each at every lag up to
// kMaxLagSec, by least squares over the last keepSec of usable samples:
//  - direct: the surface at gain degrees per full stick;
//  - rate: moving at a degrees a second per full stick, plus b times where it is. b well below 0
//    is a surface that settles at a position within kDirectBelowSec: taken as direct, -a/b per
//    full stick, settling after -1/b.
// The lag is how long the stick takes to show. Whichever reproduces the quick movement best over
// kWindowSec stretches is kept; fit = 1 - its rms off over the quick movement's rms (at least
// kMinFit to be used).
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace br {

struct StickSample {
    double dt = NAN;       // seconds since the last sample
    double stick = NAN;    // as sent, -1 .. 1, positive meant to raise the surface
    double surface = NAN;  // the aircraft's, degrees
    bool use = false;      // in the air, the stick steering it, not at full travel
};

// How a surface answers the stick, as the loop steers by it.
struct StickPlant {
    enum class Form : uint8_t { None, Rate, Direct };
    Form form = Form::None;
    double gain = NAN;    // rate: degrees a second per full stick; direct: degrees per full stick
    double lagSec = 0.0;  // how long the stick takes to show
    double settleSec = 0.0;  // direct: how long it takes to settle where the stick puts it (0: at once)
    // Direct: degrees a second per full stick it also moves at while the stick is held, beyond where
    // the stick puts it (a fly-by-wire's integral term, as the A350's rudder's; 0: none).
    double rateGain = 0.0;
    bool holds = true;    // the stick asking for the motion the aircraft reads (StickLoopConfig's hold)
    // That hold as a share of the A380's: how much this fly-by-wire reacts to the g and rates it
    // reads, against the A380's (the A220's elevator about half as much, from its replays).
    double holdScale = 1.0;
    // The pedals' hold on the yaw rate, stick per rad/s (0: none, as the A380's, whose pedals ask for
    // sideslip). The A350's ask for a yaw rate, about 10 degrees a second at full pedal, and its
    // fly-by-wire moves the rudder against the rest: a replay's yaw, written without the pedals that
    // made it, swung the rudder the other way to the clip's. Its traces: the rudder's rate went
    // Ki (k pedal - r) + Kp d(k pedal - r)/dt, Ki about 1.3, k about 10. Fitted by hand, not by
    // StickFit; kept with the rest of the figures.
    double holdYawRate = 0.0;
    double fit = NAN;     // how well it fitted, where it was fitted (NaN: set by hand)
    double seconds = 0.0;
    bool valid() const { return form != Form::None && std::isfinite(gain) && gain > 0.0; }
};
const char* stickPlantForm(StickPlant::Form f);  // "rate", "direct", "none"

class StickFit {
public:
    static constexpr double kKeepSec = 60.0;
    static constexpr double kMinSec = 15.0;         // of usable samples, before anything is fitted
    static constexpr double kMinQuickStick = 0.004;  // rms of the stick's quick movement
    static constexpr double kSmoothSec = 0.5;
    static constexpr double kMaxLagSec = 0.8;
    static constexpr double kWindowSec = 1.5;
    static constexpr double kDirectBelowSec = 0.35;
    static constexpr double kMinFit = 0.35;

    StickFit() = default;
    explicit StickFit(double keepSec) : keepSec_(keepSec) {}
    void add(const StickSample& s);
    void clear() { rows_.clear(); }
    double usableSec() const;
    // The best fit of the samples kept, or an invalid one (too few, too still, or none good enough).
    StickPlant fit() const;

private:
    struct Row {
        double t = 0.0;
        double s = NAN;
        double d = NAN;
        bool use = false;
    };
    double keepSec_ = kKeepSec;
    std::vector<Row> rows_;  // oldest first
};

}  // namespace br
