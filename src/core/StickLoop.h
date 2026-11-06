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
// if that's stuck too the loop gives up and centres the stick, and the way round is as it was for
// the next replay (neither way moved it: that doesn't say which is right). Once the stick has
// steered the surface on this aircraft (within workedDeg of the clip's while the clip's moves at
// workedRate or more, for workedSec in all: a stick the wrong way round can't follow a moving
// surface, and a still one says nothing; on the A340's flown clip that took 60 s, the A380's
// landing 48, the others' less), full travel that doesn't get there is the aircraft's own doing,
// not the stick the wrong way round: the A340's elevator went to 30 degrees against a full push
// in a hard push-over (its protections, with the replay's g lower than the clip's), the loop took
// that for the stick reversed, gave up, and the next replay, its landing, began reversed and gave
// up within 2 s. Then it stays on (Event::Held).
//
// Those figures are the A380's. Another aircraft's (StickPlant: how its surface answers, fitted by
// StickFit from its replays, kept in its profile) replace the feedforward, the lead and the gains
// (forPlant); the hold, in stick, is the control law's (an Airbus asks for the same g and rates per
// stick), kept. And whatever the figures: a surface swinging a degree or more one way and back
// wiggleTurns times within wiggleSec has the loop's own terms halved (Event::Damped, down to a
// quarter): the A220's ailerons and the 777's elevator swung 16 to 34 times in 2 s on the A380's
// figures, the rest at most 11, even in hard manoeuvres.
//
// Where the clip's rudder sits still at exactly 0 for 2 s or more, the pedals are centred and the
// loop's terms emptied (centreStill, StillStretches). The A350's fly-by-wire leaves the rudder to
// the pedals below about 100 ft: the clip's sat at 0 from there to the touchdown, the replay's
// switched over with it, and the sideslip hold, still in the pedals, put it 5.6 degrees off for
// 2 s. The A380's, A320neo's and A330's do the same from 100 to 200 ft down and from the liftoff to
// about 200 ft: there their rudders followed the pedals alone (22 to 35 degrees per full pedal,
// R2 0.99, nothing from the sideslip), and the A380's was up to 10 off.
#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "Channels.h"
#include "StickFit.h"

namespace br {

// What the aircraft reads, for the hold: its g, pitch, roll and yaw rates (rad/s, body axes as the
// sim gives them) and sideslip (degrees). NaN: not fed in.
struct StickReadings {
    double g = NAN;
    double pitchRate = NAN;
    double rollRate = NAN;
    double yawRate = NAN;
    double betaDeg = NAN;
};

struct StickLoopConfig {
    bool airDirect = false;  // in the air the stick moves the surface directly (else at a rate)
    double airK = 10.0;      // in the air: degrees a second (or degrees) per full travel
    double airSettleSec = 0.0;  // direct: how long the surface takes to settle (the feedforward that much ahead)
    // Direct: degrees a second per stick it also moves at, as long as the stick is held (0: none).
    // A fly-by-wire with a proportional and an integral term on the stick, as the A350's on its
    // pedals: the feedforward then leaves out what the stick sent so far has built up (the loop's
    // own tally of it), or the surface runs past the clip's.
    double airRateK = 0.0;
    static constexpr double kBuiltSec = 1.0;  // how quickly that tally is brought to the surface's
    double groundK = 30.0;   // on the ground: degrees per full travel
    double leadSec = 0.3;    // the clip ahead, for the aircraft's lag (for the caller)
    double holdG = 0.95;     // stick per g above 1
    double holdPitchRate = -2.0;
    double holdRollRate = 0.0;
    double holdBeta = 0.0;   // per degree
    // Per rad/s of yaw rate: none on the A380, whose pedals ask for sideslip. The A350's ask for a
    // yaw rate (StickPlant::holdYawRate).
    double holdYawRate = 0.0;
    double airKp = 0.05;     // stick (full travel 1) per degree the surface is off
    double airKi = 0.02;     // stick per degree-second
    double groundKp = 0.0;
    double groundKi = 0.06;
    double rampSec = 4.0;
    double maxRate = 8.0;    // stick travel a second, at most
    double stuckSec = 2.0;
    double stuckDeg = 3.0;
    double closerDeg = 2.0;
    double workedSec = 3.0;
    double workedDeg = 1.5;
    double workedRate = 1.0;  // degrees a second
    double wiggleTurnDeg = 0.5;   // back this far from where it went: a turn
    double wiggleSwingDeg = 1.0;  // counted where it swung this far
    double wiggleSec = 2.0;
    int wiggleTurns = 12;
    double minDamp = 0.25;
    // Centred where the clip's surface sits still at 0 (StillStretches): the pedals, whose surface
    // reads exactly 0 only with nothing moving it.
    bool centreStill = false;

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
        c.centreStill = true;
        return c;
    }
    // This config with a surface's own figures: its feedforward, lead and gains. A rate's
    // proportional and integral gains keep their product with the rate (this config's, if it's a
    // rate, else the elevator's); a direct surface's are integral only, as the A380's rudder (0.9 a
    // second over its degrees per stick). The hold scaled by plant.holdScale (none without holds).
    StickLoopConfig forPlant(const StickPlant& plant) const;
};

class StickLoop {
public:
    enum class Event { None, Flipped, Failed, Held, Damped };

    explicit StickLoop(StickLoopConfig cfg = StickLoopConfig()) : cfg_(cfg) {}

    const StickLoopConfig& config() const { return cfg_; }
    // Other figures from here on (StickLoopConfig::forPlant), the stick where it is.
    void setConfig(const StickLoopConfig& cfg) { cfg_ = cfg; }
    double damping() const { return damp_; }  // 1, or less after a wiggle (Event::Damped)
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
    // One frame where the clip's surface sits still at 0 (config().centreStill, StillStretches):
    // the stick to centre, as fast as it goes, and the loop's terms emptied, so it steers from
    // there again where the clip's moves. Returns the stick.
    double centre(double dt, bool onGround);

    double stick() const { return failed_ ? 0.0 : sign_ * drive_; }
    double airWeight() const { return air_; }  // 0 ground gains .. 1 air gains (NaN before the first frame)
    double lastHold() const { return hold_; }  // this frame's terms, for the trace
    double lastFeedforward() const { return ff_; }
    int sign() const { return sign_; }
    bool flipped() const { return flipped_; }
    bool worked() const { return workedSec_ >= cfg_.workedSec; }  // the stick has steered it, this way round
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
    double built_ = NAN;    // airRateK: the stick-seconds sent beyond the hold (NaN: from the surface)
    int sign_ = 1;          // the stick for a positive drive_: 1 pulled back
    bool flipped_ = false;   // the other way round has been tried
    double workedSec_ = 0.0;  // how long it has steered the surface, this way round (kept like sign_)
    bool failed_ = false;
    double stuckFor_ = 0.0;
    double stuckFrom_ = 0.0;
    Event event_ = Event::None;
    // The wiggle check: where the surface last turned, which way it's going, and when it swung.
    double clock_ = 0.0;
    int wigDir_ = 0;
    double wigExt_ = NAN;
    double wigPrev_ = NAN;
    double wigTimes_[32] = {};
    int wigCount_ = 0;
    double damp_ = 1.0;
    void ramp(double dt, bool onGround);
    void watchWiggle(double liveDeg);
};

// Where a clip's surface sits still at exactly 0 for kMinSec or more (clip seconds, first to last
// such frame). A rudder reads exactly 0 only with the pedals centred and nothing else moving it:
// the Airbuses' below 100 to 200 ft, to the touchdown and from the liftoff (their fly-by-wire
// leaves it to the pedals there). A shorter stretch can be the fly-by-wire's own still moment
// (the A340's and A330's, up to 1.4 s, in the air).
class StillStretches {
public:
    static constexpr double kMinSec = 2.0;

    void find(const Frame* frames, size_t count, int channel, double minSec = kMinSec);
    void clear() { spans_.clear(); }
    bool at(double t) const;
    size_t count() const { return spans_.size(); }
    double seconds() const;

private:
    std::vector<std::pair<double, double>> spans_;
};

}  // namespace br
