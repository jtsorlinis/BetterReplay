// Steers a control surface that the aircraft's fly-by-wire keeps moving itself to where the clip had
// it, by the pilot's stick or pedals. Writing the surface can't beat a fly-by-wire that writes it
// again later in the frame (the iniBuilds A380 does, every other frame), but it still listens to
// its stick: pushed forward, the A380's elevator came back from full nose-up.
//
// A fly-by-wire moves its surfaces from what the stick asks for and the motion it reads: in normal
// law the stick asks for g and pitch rate, roll rate, and (the pedals) sideslip. Frozen in a
// replay, it reads the clip's motion as the replay writes it, plus what freezing does to it: the
// A380 reads 0.35 to 1.3 g where the clip had about 1.0, 0.036 g less per degree of nose-up
// elevator (so left to itself the elevator runs away nose-up), and 5 to 11 degrees of sideslip
// (the wind isn't the clip's). So the stick sent is made of:
//  - hold: the stick that asks for the motion it reads, so it holds the surface still. Fitted from
//    A380 replays steered by an earlier version of this loop: 0.95 per g above 1 and -2 per rad/s
//    of pitch rate (R2 0.76), -4 per rad/s of roll rate (R2 0.88; Airbus normal law asks for about
//    15 degrees a second at full stick), and minus the sideslip over 16 degrees for the pedals (the
//    pedal held 0.94 R2 with it through a landing).
//  - feedforward: the clip's own movement. In the air the A380's elevator and ailerons move at a
//    rate (10 and 20 degrees a second at full stick), its rudder directly (45 degrees at full pedal,
//    0.17 s late); on the ground all three directly (ground law, taken as 30). Taken leadSec ahead
//    of the replay, for the time the aircraft takes to answer. Without it the loop only reacted
//    once a surface was off, by when the clip's motion had started and the fly-by-wire, reading
//    it, no longer answered: hard inputs lagged a second or two.
//  - a proportional and integral term on how far the surface is off, for the rest.
// Simulated against those fits with a hard-input clip's surfaces and motion, the elevator stays
// within about 2 degrees of the clip's (the loop without the hold and feedforward: 12, as in the
// sim); the ailerons within 13 at full stick in the hardest rolls (37); and the rudder, with no
// proportional term at its 0.17 s (it shook 3 times a second with one), without its wiggle.
//
// On the ground the gains blend to integral only (a proportional term on a surface the stick moves
// directly, a frame or two late, makes it shake), over rampSec after liftoff and touchdown, as the
// A380's laws change (its flight mode came in over about 4 s); the hold is the air's only.
//
// Which way round the stick works is assumed (positive raises the surface: pulled back for the
// elevator, as a 737's clips show; the A380's three all confirmed), and checked: held at full
// travel for stuckSec while the clip's surface isn't moving fast, more than stuckDeg off and not
// closerDeg closer, the surface went the wrong way (or not at all), so the other way is tried once;
// if that's stuck too the loop gives up and centres the stick.
#pragma once

#include <cmath>

namespace br {

// What the aircraft reads, for the hold: its g, pitch and roll rates (rad/s, body axes as the sim
// gives them) and sideslip (degrees). NaN: not fed in.
struct StickReadings {
    double g = NAN;
    double pitchRate = NAN;
    double rollRate = NAN;
    double betaDeg = NAN;
};

struct StickLoopConfig {
    bool airDirect = false;  // in the air the stick moves the surface directly (else at a rate)
    double airK = 10.0;      // in the air: degrees a second (or degrees) per full travel
    double groundK = 30.0;   // on the ground: degrees per full travel
    double leadSec = 0.3;    // the clip ahead, for the aircraft's lag (for the caller)
    double holdG = 0.95;     // stick per g above 1
    double holdPitchRate = -2.0;
    double holdRollRate = 0.0;
    double holdBeta = 0.0;   // per degree
    double airKp = 0.05;     // stick (full travel 1) per degree the surface is off
    double airKi = 0.02;     // stick per degree-second
    double groundKp = 0.0;
    double groundKi = 0.06;
    double rampSec = 4.0;
    double maxRate = 8.0;    // stick travel a second, at most
    double stuckSec = 2.0;
    double stuckDeg = 3.0;
    double closerDeg = 2.0;

    static StickLoopConfig elevator() { return StickLoopConfig(); }
    static StickLoopConfig ailerons() {
        StickLoopConfig c;
        c.airK = 20.0;
        c.leadSec = 0.4;
        c.holdG = c.holdPitchRate = 0.0;
        c.holdRollRate = -4.0;
        c.airKp = 0.05;
        c.airKi = 0.03;
        return c;
    }
    static StickLoopConfig rudder() {
        StickLoopConfig c;
        c.airDirect = true;
        c.airK = 45.0;
        c.leadSec = 0.17;
        c.holdG = c.holdPitchRate = 0.0;
        c.holdBeta = -1.0 / 16.0;
        c.airKp = 0.0;
        c.airKi = 0.02;
        return c;
    }
};

class StickLoop {
public:
    enum class Event { None, Flipped, Failed };

    explicit StickLoop(StickLoopConfig cfg = StickLoopConfig()) : cfg_(cfg) {}

    const StickLoopConfig& config() const { return cfg_; }
    // A new replay: the stick from centre, the integral emptied. The way round found earlier on
    // this aircraft is kept; forget() starts over (another aircraft).
    void reset();
    void forget();
    // After a jump: the surface is somewhere else now, so the stuck check starts again.
    void onJump() { stuckFor_ = 0.0; }

    // One frame: the clip's surface now, and leadSec ahead with how fast it moves there (degrees,
    // degrees a second), the aircraft's surface (degrees), dt in seconds, whether the aircraft is
    // on the ground, and what it reads. Returns the stick, -1 .. 1, positive the way that raises
    // the surface (for the elevator: pulled back). A gap in the clip or the aircraft's surface
    // holds it; one ahead goes by the clip now.
    double update(double targetDeg, double aheadDeg, double aheadRate, double liveDeg, double dt, bool onGround,
                  const StickReadings& reads = StickReadings());

    double stick() const { return failed_ ? 0.0 : sign_ * drive_; }
    double airWeight() const { return air_; }  // 0 ground gains .. 1 air gains (NaN before the first frame)
    double lastHold() const { return hold_; }  // this frame's terms, for the trace
    double lastFeedforward() const { return ff_; }
    int sign() const { return sign_; }
    bool flipped() const { return flipped_; }
    bool failed() const { return failed_; }
    // What the last update decided, once.
    Event takeEvent() {
        const Event e = event_;
        event_ = Event::None;
        return e;
    }

private:
    StickLoopConfig cfg_;
    double drive_ = 0.0;     // -1 .. 1, positive the way that raises the surface (as believed)
    double integral_ = 0.0;
    double air_ = NAN;
    double hold_ = 0.0;
    double ff_ = 0.0;
    int sign_ = 1;          // the stick for a positive drive_: 1 pulled back
    bool flipped_ = false;   // the other way round has been tried
    bool failed_ = false;
    double stuckFor_ = 0.0;
    double stuckFrom_ = 0.0;
    Event event_ = Event::None;
};

}  // namespace br
