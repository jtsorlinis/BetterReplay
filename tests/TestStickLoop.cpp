#include <cmath>
#include <vector>

#include "StickLoop.h"
#include "TestHarness.h"

using namespace br;

namespace {

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

constexpr double kDt = 1.0 / 30.0;
constexpr double kPi = 3.14159265358979323846;

// The stick reaching the aircraft n frames after it's sent.
template <int N>
struct Late {
    double q[N] = {};
    double push(double sent) {
        const double out = q[0];
        for (int i = 0; i + 1 < N; ++i) q[i] = q[i + 1];
        q[N - 1] = sent;
        return out;
    }
};

// The iniBuilds A380's pitch frozen in a replay, as fitted from replays steered by the stick: its
// elevator moves at 12 degrees a second per full travel and 11 the other way per g it reads below
// 1, and the g it reads is 0.036 g lower per degree of nose-up elevator, 0.15 g low to start with.
// direct: ground law, the stick moves the elevator, 30 degrees at full travel.
struct A380Pitch {
    double surface = 18.0;  // full nose-up, as left to itself
    double wobble = 0.0;    // the rest of what moves its g
    int way = 1;            // -1: the stick works the other way round
    bool ignores = false;   // the stick does nothing
    bool direct = false;
    Late<9> late;           // 0.3 s
    double g() const { return direct ? 1.0 : 1.0 - 0.036 * surface - 0.15 + wobble; }
    StickReadings reads() const {
        StickReadings r;
        r.g = g();
        return r;
    }
    void step(double sent, double dt) {
        const double s = ignores ? 0.0 : way * late.push(sent);
        if (direct) {
            surface += clampd(30.0 * s - surface, -40.0 * dt, 40.0 * dt);
        } else {
            surface = clampd(surface + (12.0 * s - 11.0 * (g() - 1.0)) * dt, -18.0, 18.0);
        }
    }
};

// Wiggle: how much a surface moves about its own half-second average, rms.
struct Wiggle {
    double h[16] = {};
    int n = 0;
    double sum = 0.0;
    int count = 0;
    void add(double v) {
        h[n % 16] = v;
        if (++n < 16) return;
        double m = 0.0;
        for (double x : h) m += x;
        m /= 16.0;
        const double mid = h[(n - 8) % 16];
        sum += (mid - m) * (mid - m);
        ++count;
    }
    double rms() const { return count ? std::sqrt(sum / count) : 0.0; }
};

}  // namespace

TEST(StickLoop_BringsAFlyByWireElevatorBackAndHoldsIt) {
    for (const double from : {18.0, 0.0}) {
        A380Pitch a;
        a.surface = from;
        StickLoop loop(StickLoopConfig::elevator());
        const double lead = loop.config().leadSec;
        double worst = 0.0;
        double sum = 0.0;
        int n = 0;
        for (int i = 0; i < 30 * 90; ++i) {
            const double t = i * kDt;
            a.wobble = 0.08 * std::sin(t * 2.0 * kPi / 5.0);
            auto clip = [](double t) {
                return 0.3 + 0.8 * std::sin(t * 2.0 * kPi / 8.0) + 0.4 * std::sin(t * 2.0 * kPi / 2.7);
            };
            const double rate = (clip(t + lead + 0.05) - clip(t + lead - 0.05)) / 0.1;
            a.step(loop.update(clip(t), clip(t + lead), rate, a.surface, kDt, false, a.reads()), kDt);
            if (t < 6.0) continue;
            worst = std::fmax(worst, std::fabs(a.surface - clip(t)));
            sum += std::fabs(a.surface - clip(t));
            ++n;
        }
        CHECK(worst < (from > 0.0 ? 3.0 : 1.5));  // (from full nose-up: the last of its way back)
        CHECK(sum / n < 0.5);
        CHECK(!loop.flipped());
        CHECK(!loop.failed());
    }
}

// A hard input: the clip's elevator 10 degrees nose-down in under a second and back. Without the
// clip's movement fed forward, the loop only answers once the elevator is off.
TEST(StickLoop_KeepsUpWithAHardInput) {
    auto clip = [](double t) { return t < 2.0 ? 0.0 : (t < 2.8 ? -12.5 * (t - 2.0) : (t < 4.0 ? -10.0 : (t < 4.8 ? -10.0 + 12.5 * (t - 4.0) : 0.0))); };
    double worst[2] = {};
    for (int withFf = 0; withFf < 2; ++withFf) {
        StickLoopConfig cfg = StickLoopConfig::elevator();
        if (!withFf) cfg.airK = 0.0;  // no feedforward
        StickLoop loop(cfg);
        A380Pitch a;
        a.surface = 0.0;
        for (int i = 0; i < 30 * 8; ++i) {
            const double t = i * kDt;
            const double lead = cfg.leadSec;
            const double rate = (clip(t + lead + 0.05) - clip(t + lead - 0.05)) / 0.1;
            a.step(loop.update(clip(t), clip(t + lead), rate, a.surface, kDt, false, a.reads()), kDt);
            worst[withFf] = std::fmax(worst[withFf], std::fabs(a.surface - clip(t)));
        }
    }
    CHECK(worst[1] < 3.0);
    CHECK(worst[0] > 2.0 * worst[1]);
}

TEST(StickLoop_WithoutTheHoldItCyclesAsTheFirstTestBuildDid) {
    StickLoopConfig cfg;
    cfg.airKp = 0.045;
    cfg.airKi = 0.035;
    cfg.holdG = cfg.holdPitchRate = 0.0;
    cfg.airK = 0.0;
    StickLoop loop(cfg);
    A380Pitch a;
    double worst = 0.0;
    for (int i = 0; i < 30 * 90; ++i) {
        a.step(loop.update(0.0, 0.0, 0.0, a.surface, kDt, false, a.reads()), kDt);
        if (i * kDt > 30.0) worst = std::fmax(worst, std::fabs(a.surface));
    }
    CHECK(worst > 5.0);
}

// The A380's rudder: 45 degrees per full pedal, 5 frames late, and the pedal asks for sideslip, 16
// degrees at full pedal, so the sideslip it reads (5 to 11 degrees in replays) moves it as much.
TEST(StickLoop_HoldsTheRudderWithoutShakingIt) {
    double shake[2] = {};
    double worst[2] = {};
    for (int old = 0; old < 2; ++old) {
        StickLoopConfig cfg = StickLoopConfig::rudder();
        if (old) {  // the second test build's: proportional, no hold or feedforward
            cfg.airKp = 0.03;
            cfg.airKi = 0.04;
            cfg.holdBeta = 0.0;
            cfg.airK = 0.0;
        }
        StickLoop loop(cfg);
        double rudder = 0.0;
        Late<5> late;
        Wiggle w;
        for (int i = 0; i < 30 * 60; ++i) {
            const double t = i * kDt;
            const double beta = -9.0 + 1.5 * std::sin(t * 2.0 * kPi / 20.0);
            auto clip = [](double t) { return 0.4 * std::sin(t * 2.0 * kPi / 6.0); };
            StickReadings r;
            r.betaDeg = beta;
            const double lead = cfg.leadSec;
            const double pedal = late.push(loop.update(clip(t), clip(t + lead), 0.0, rudder, kDt, false, r));
            rudder += clampd(clampd(45.0 * (pedal + beta / 16.0), -30.0, 30.0) - rudder, -60.0 * kDt, 60.0 * kDt);
            if (t < 10.0) continue;
            w.add(rudder);
            worst[old] = std::fmax(worst[old], std::fabs(rudder - clip(t)));
        }
        shake[old] = w.rms();
    }
    CHECK(shake[0] < 0.2);
    CHECK(worst[0] < 1.5);
    CHECK(shake[1] > 1.0);  // it shook
}

// The A350's rudder: its pedals ask for a yaw rate (10 degrees a second at full pedal) and its
// fly-by-wire moves the rudder by the error, as fitted from its replays: the rudder's rate
// Ki (k pedal - r) + Kp d(k pedal - r)/dt. The clip: the pilot's pedals yawing the aircraft. The
// replay writes the clip's yaw without the pedals that made it, so the fly-by-wire, left to
// itself, moves the rudder against it: on the flown clip it went +9 degrees where the clip's was
// -8.6. With the hold on the yaw rate the stick asks for the yaw it reads.
TEST(StickLoop_HoldsAYawRateRudderAgainstTheYawItReads) {
    constexpr double k = 10.0, ki = 1.3, kp = 2.0;  // deg/s per pedal; rudder deg/s and deg per deg/s
    constexpr int n = 30 * 40;
    // The clip: the pilot's pedal doublets, the rudder the fly-by-wire made of them, the yaw it gave.
    static double clipRudder[n], clipYaw[n];  // degrees, deg/s
    {
        double rudder = 0.0, yaw = 0.0, ePrev = 0.0;
        for (int i = 0; i < n; ++i) {
            const double t = i * kDt;
            const double pedal = (t > 4.0 && t < 24.0) ? 0.6 * std::sin(t * 2.0 * kPi / 8.0) : 0.0;
            const double e = k * pedal - yaw;
            rudder = clampd(rudder + ki * e * kDt + kp * (e - ePrev), -30.0, 30.0);
            ePrev = e;
            yaw += (0.6 * rudder - yaw) * kDt / 1.2;  // the aircraft yawing to its rudder
            clipRudder[i] = rudder;
            clipYaw[i] = yaw;
        }
    }
    auto at = [](const double* a, double t) {
        const int i = static_cast<int>(t / kDt);
        return a[i < 0 ? 0 : (i >= n ? n - 1 : i)];
    };
    // The A350's figures: 15 degrees at full pedal at once (its traces gave 10 to 30) and 13 a
    // second more while held.
    StickPlant plant;
    plant.form = StickPlant::Form::Direct;
    plant.gain = 15.0;
    plant.rateGain = ki * k;
    plant.lagSec = 0.1;
    double worst[2] = {}, against[2] = {}, biased = 0.0;
    for (int run = 0; run < 3; ++run) {
        const int withHold = run == 0 ? 0 : 1;
        // The third: the fly-by-wire also answering something the hold doesn't know (sideslip it
        // reads, 0.6 degrees a second of rudder): a tally of the stick alone drifted off.
        const double bias = run == 2 ? 0.6 / ki : 0.0;
        plant.holdYawRate = withHold ? 180.0 / kPi / k : 0.0;
        StickLoopConfig cfg = StickLoopConfig::rudder().forPlant(plant);
        cfg.holdBeta = 0.0;  // no sideslip here
        StickLoop loop(cfg);
        Late<3> late;
        double rudder = 0.0, ePrev = 0.0;
        for (int i = 0; i < n; ++i) {
            const double t = i * kDt;
            const double lead = cfg.leadSec;
            StickReadings r;
            r.yawRate = at(clipYaw, t + lead) * kPi / 180.0;  // written: the clip's, ahead
            const double rate = (at(clipRudder, t + lead + 0.05) - at(clipRudder, t + lead - 0.05)) / 0.1;
            const double pedal = late.push(loop.update(at(clipRudder, t), at(clipRudder, t + lead), rate, rudder, kDt, false, r));
            const double e = k * pedal - at(clipYaw, t) + bias;
            rudder = clampd(rudder + ki * e * kDt + kp * (e - ePrev), -30.0, 30.0);
            ePrev = e;
            if (t < 6.0) continue;
            const double off = rudder - at(clipRudder, t);
            if (run == 2) {
                biased = std::fmax(biased, std::fabs(off));
                continue;
            }
            worst[withHold] = std::fmax(worst[withHold], std::fabs(off));
            if (std::fabs(at(clipRudder, t)) > 2.0 && rudder * at(clipRudder, t) < 0.0) {
                against[withHold] = std::fmax(against[withHold], std::fabs(rudder));
            }
        }
    }
    std::printf("  A350-like rudder: at most %.2f deg off with the yaw-rate hold (%.2f without, %.2f with a sideslip "
                "it doesn't know), at most %.2f deg the other way to the clip's (%.2f without)\n",
                worst[1], worst[0], biased, against[1], against[0]);
    CHECK(worst[1] < 2.5);  // (on figures a quarter off the aircraft's: 15 at once where it has 20)
    CHECK(against[1] < 1.0);
    CHECK(worst[0] > 2.0 * worst[1]);  // (the tally kept to the surface catches some of it without)
    CHECK(biased < 3.0);
}

// The A380's ailerons: 20 degrees a second per full stick, 0.4 s late, against the roll rate it
// reads (the stick asks for about 15 degrees a second at full travel).
TEST(StickLoop_FollowsTheAileronsThroughARoll) {
    StickLoop loop(StickLoopConfig::ailerons());
    const double lead = loop.config().leadSec;
    double aileron = 0.0;
    Late<12> late;
    double worst = 0.0;
    for (int i = 0; i < 30 * 30; ++i) {
        const double t = i * kDt;
        auto clip = [](double t) { return 8.0 * std::sin(t * 2.0 * kPi / 6.0); };
        // The clip's roll rate, behind its ailerons; the replay writes it, so it's known ahead.
        auto rollRate = [](double t) { return 0.1 * std::sin(t * 2.0 * kPi / 6.0 - 1.0); };  // rad/s
        StickReadings r;
        r.rollRate = rollRate(t + lead);  // what it will read when the stick reaches it
        const double rate = (clip(t + lead + 0.05) - clip(t + lead - 0.05)) / 0.1;
        const double stick = late.push(loop.update(clip(t), clip(t + lead), rate, aileron, kDt, false, r));
        aileron = clampd(aileron + (20.0 * stick + 80.0 * rollRate(t)) * kDt, -30.0, 30.0);
        if (t > 5.0) worst = std::fmax(worst, std::fabs(aileron - clip(t)));
    }
    CHECK(worst < 2.0);
    CHECK(!loop.flipped());
}

TEST(StickLoop_MovesASurfaceTheStickDrivesDirectlyOnTheGround) {
    A380Pitch a;
    a.direct = true;
    a.surface = 0.0;
    StickLoop loop(StickLoopConfig::elevator());
    for (int i = 0; i < 30 * 10; ++i) a.step(loop.update(6.0, 6.0, 0.0, a.surface, kDt, true, a.reads()), kDt);
    CHECK_NEAR(a.surface, 6.0, 0.2);
    CHECK_NEAR(loop.stick(), 0.2, 0.01);
    CHECK_EQ(loop.airWeight(), 0.0);
}

TEST(StickLoop_BlendsItsGainsAfterLiftoff) {
    StickLoop loop;
    StickReadings low;
    low.g = 0.5;
    loop.update(0.0, 0.0, 0.0, 0.0, 0.1, true, low);
    CHECK_EQ(loop.airWeight(), 0.0);
    CHECK_EQ(loop.stick(), 0.0);  // no hold on the ground
    for (int i = 0; i < 20; ++i) loop.update(0.0, 0.0, 0.0, 0.0, 0.1, false);
    CHECK_NEAR(loop.airWeight(), 0.5, 1e-9);  // 2 s of the 4
    for (int i = 0; i < 30; ++i) loop.update(0.0, 0.0, 0.0, 0.0, 0.1, false);
    CHECK_EQ(loop.airWeight(), 1.0);
    loop.reset();
    loop.update(0.0, 0.0, 0.0, 0.0, 0.1, false, low);
    CHECK_EQ(loop.airWeight(), 1.0);  // a replay starting in the air starts with the air gains
    CHECK(loop.stick() < 0.0);        // and pushes for the g it's short
}

TEST(StickLoop_TriesTheOtherWayRoundOnce) {
    A380Pitch a;
    a.way = -1;
    StickLoop loop;
    bool flippedSeen = false;
    for (int i = 0; i < 30 * 30; ++i) {
        a.step(loop.update(0.0, 0.0, 0.0, a.surface, kDt, false, a.reads()), kDt);
        if (loop.takeEvent() == StickLoop::Event::Flipped) flippedSeen = true;
    }
    CHECK(flippedSeen);
    CHECK_EQ(loop.sign(), -1);
    CHECK(!loop.failed());
    CHECK_NEAR(a.surface, 0.0, 1.0);
    // Kept for the next replay on this aircraft.
    loop.reset();
    CHECK_EQ(loop.sign(), -1);
    loop.forget();
    CHECK_EQ(loop.sign(), 1);
}

TEST(StickLoop_GivesUpWhenTheStickDoesNothing) {
    A380Pitch a;
    a.ignores = true;
    StickLoop loop;
    int flips = 0;
    int fails = 0;
    for (int i = 0; i < 30 * 20; ++i) {
        a.step(loop.update(0.0, 0.0, 0.0, a.surface, kDt, false, a.reads()), kDt);
        const StickLoop::Event e = loop.takeEvent();
        flips += e == StickLoop::Event::Flipped;
        fails += e == StickLoop::Event::Failed;
    }
    CHECK_EQ(flips, 1);
    CHECK_EQ(fails, 1);
    CHECK(loop.failed());
    CHECK_EQ(loop.stick(), 0.0);
    // Neither way round moved it, which doesn't say which is right: the next replay starts the
    // first way round again (not the other, to give up on at once).
    CHECK_EQ(loop.sign(), 1);
    CHECK(!loop.flipped());
}

namespace {

// The clip's elevator in a loop test: moving (up to about 1.6 degrees a second), so the stick
// following it shows it works.
double movingClip(double t) { return 0.3 + 0.8 * std::sin(t * 2.0 * kPi / 8.0) + 0.4 * std::sin(t * 2.0 * kPi / 2.7); }

struct LoopRun {
    int flips = 0, fails = 0, helds = 0;
    double worst = 0.0;  // the surface off the clip's, at most, where measured
    void play(StickLoop& loop, A380Pitch& a, double from, double seconds, double measureFrom = 1e9) {
        const double lead = loop.config().leadSec;
        for (int i = 0; i < static_cast<int>(seconds * 30.0); ++i) {
            const double t = from + i * kDt;
            const double rate = (movingClip(t + lead + 0.05) - movingClip(t + lead - 0.05)) / 0.1;
            a.step(loop.update(movingClip(t), movingClip(t + lead), rate, a.surface, kDt, false, a.reads()), kDt);
            const StickLoop::Event e = loop.takeEvent();
            flips += e == StickLoop::Event::Flipped;
            fails += e == StickLoop::Event::Failed;
            helds += e == StickLoop::Event::Held;
            if (t >= measureFrom) worst = std::fmax(worst, std::fabs(a.surface - movingClip(t)));
        }
    }
};

}  // namespace

// The A340's elevator, in a hard push-over on a flown clip: its fly-by-wire took it to 30 degrees
// against a full push (its protections, the replay's g lower than the clip's). The stick had
// steered it for a minute and a half, but the loop took that for the stick the wrong way round,
// flipped it, gave up, and began the next replay reversed. Once the stick has followed the clip's
// moving surface it's kept on, the right way round, and follows the clip again after.
TEST(StickLoop_KeepsItsWayRoundThroughTheAircraftsLimits) {
    A380Pitch a;
    a.surface = 0.0;
    StickLoop loop(StickLoopConfig::elevator());
    LoopRun run;
    run.play(loop, a, 0.0, 30.0);
    REQUIRE(loop.worked());
    a.ignores = true;  // the aircraft's own: nose-up to its stop whatever the stick does
    run.play(loop, a, 30.0, 10.0);
    CHECK(a.surface > 15.0);
    a.ignores = false;
    run.play(loop, a, 40.0, 30.0, 55.0);
    CHECK_EQ(run.flips, 0);
    CHECK_EQ(run.fails, 0);
    CHECK(run.helds >= 1);
    CHECK_EQ(loop.sign(), 1);
    CHECK(!loop.failed());
    CHECK(run.worst < 2.0);
    // And the next replay starts the right way round, still known to work.
    loop.reset();
    CHECK(loop.worked());
    CHECK_EQ(loop.sign(), 1);
}

// The same before the stick has shown anything (from the start of a clip): tried the other way
// round, given up on for that replay; the next replay starts the first way round and steers it.
TEST(StickLoop_AReplayAfterAFailureStartsTheFirstWayRound) {
    A380Pitch a;
    a.ignores = true;
    StickLoop loop(StickLoopConfig::elevator());
    LoopRun first;
    first.play(loop, a, 0.0, 20.0);
    CHECK_EQ(first.flips, 1);
    CHECK_EQ(first.fails, 1);
    loop.reset();
    a.ignores = false;
    a.surface = 0.0;
    LoopRun next;
    next.play(loop, a, 0.0, 40.0, 10.0);
    CHECK_EQ(next.flips, 0);
    CHECK_EQ(next.fails, 0);
    CHECK_EQ(loop.sign(), 1);
    CHECK(next.worst < 2.0);
}

TEST(StickLoop_HoldsTheStickThroughGaps) {
    StickLoop loop;
    for (int i = 0; i < 10; ++i) loop.update(5.0, 5.0, 0.0, 0.0, kDt, false);
    const double held = loop.stick();
    CHECK(held > 0.0);
    CHECK_EQ(loop.update(NAN, NAN, 0.0, 0.0, kDt, false), held);
    CHECK_EQ(loop.update(5.0, 5.0, 0.0, NAN, kDt, false), held);
    CHECK_EQ(loop.update(5.0, 5.0, 0.0, 0.0, 0.0, false), held);
}

namespace {

// A surface that goes where the stick puts it, `gain` degrees per full travel, 2 frames on (the
// A220's ailerons: 15 to 25 degrees; at 25 the A380's figures wiggle it), at most `rateLimit`
// degrees a second.
struct DirectSurface {
    double surface = 0.0;
    double gain = 25.0;
    double rateLimit = 200.0;
    Late<2> late;
    void step(double sent, double dt) {
        const double want = gain * late.push(sent);
        surface += clampd(want - surface, -rateLimit * dt, rateLimit * dt);
    }
};

// A surface that moves at 30 degrees a second per full travel, 0.2 s on.
struct RateSurface {
    double surface = 0.0;
    Late<6> late;
    void step(double sent, double dt) { surface = clampd(surface + 30.0 * late.push(sent) * dt, -25.0, 25.0); }
};

// The clip's ailerons in a loop test: rolls each way (up to about 10 degrees, 8 degrees a second).
double rollClip(double t) { return 6.0 * std::sin(t * 2.0 * kPi / 6.0) + 3.0 * std::sin(t * 2.0 * kPi / 2.3); }

struct SurfaceRun {
    StickFit fit;
    int damped = 0;
    double worst = 0.0;   // off the clip, at most, where measured
    double wiggle = 0.0;  // rms about its own half-second average, where measured
    template <class Plant>
    void play(StickLoop& loop, Plant& plant, double (*clip)(double), double from, double seconds, double measureFrom) {
        const double lead = loop.config().leadSec;
        Wiggle w;
        for (int i = 0; i < static_cast<int>(seconds * 30.0); ++i) {
            const double t = from + i * kDt;
            const double rate = (clip(t + lead + 0.05) - clip(t + lead - 0.05)) / 0.1;
            const double s = loop.update(clip(t), clip(t + lead), rate, plant.surface, kDt, false);
            plant.step(s, kDt);
            StickSample sample;
            sample.dt = kDt;
            sample.stick = s;
            sample.surface = plant.surface;
            sample.use = std::fabs(s) < 0.97;
            fit.add(sample);
            if (loop.takeEvent() == StickLoop::Event::Damped) ++damped;
            if (t < measureFrom) continue;
            worst = std::fmax(worst, std::fabs(plant.surface - clip(t)));
            w.add(plant.surface);
        }
        wiggle = w.rms();
    }
};

}  // namespace

// The A220's ailerons on the A380's figures: each correction overshoots, a wiggle several times a
// second. The loop calms itself; the fit, from that, finds a surface that goes where the stick
// puts it, and steered by those figures the next replay follows the clip without the wiggle.
TEST(StickLoop_CalmsAWiggleAndFitsTheSurface) {
    DirectSurface plant;
    StickLoop loop(StickLoopConfig::ailerons());
    SurfaceRun first;
    first.play(loop, plant, rollClip, 0.0, 40.0, 1e9);
    CHECK(first.damped >= 1);
    CHECK(loop.damping() < 1.0);
    const StickPlant p = first.fit.fit();
    std::printf("  A220-like ailerons: fitted %s %.1f, %.2f s late, fit %.2f\n", stickPlantForm(p.form), p.gain, p.lagSec,
                p.fit);
    REQUIRE(p.valid());
    CHECK(p.form == StickPlant::Form::Direct);
    CHECK_NEAR(p.gain, 25.0, 6.0);
    CHECK(p.lagSec < 0.2);
    StickLoop next(StickLoopConfig::ailerons().forPlant(p));
    DirectSurface again;
    SurfaceRun run;
    run.play(next, again, rollClip, 0.0, 30.0, 5.0);
    std::printf("  then by its figures: at most %.2f deg off, wiggle %.2f deg\n", run.worst, run.wiggle);
    CHECK_EQ(run.damped, 0);
    CHECK(run.worst < 2.5);
}

// The A380's own figures give the loop it has always had; a faster surface's gains come down with
// its rate, and without holds there are none.
TEST(StickLoop_TheA380sOwnFiguresAreItsDefaults) {
    StickPlant elev;
    elev.form = StickPlant::Form::Rate;
    elev.gain = 10.0;
    elev.lagSec = 0.3;
    const StickLoopConfig a = StickLoopConfig::elevator();
    const StickLoopConfig b = a.forPlant(elev);
    CHECK_EQ(b.airDirect, a.airDirect);
    CHECK_NEAR(b.airK, a.airK, 1e-9);
    CHECK_NEAR(b.airKp, a.airKp, 1e-9);
    CHECK_NEAR(b.airKi, a.airKi, 1e-9);
    CHECK_NEAR(b.leadSec, a.leadSec, 1e-9);
    CHECK_NEAR(b.holdG, a.holdG, 1e-9);
    StickPlant rud;
    rud.form = StickPlant::Form::Direct;
    rud.gain = 45.0;
    rud.lagSec = 0.17;
    const StickLoopConfig r = StickLoopConfig::rudder().forPlant(rud);
    CHECK_NEAR(r.airKi, StickLoopConfig::rudder().airKi, 1e-9);
    CHECK_NEAR(r.airKp, 0.0, 1e-12);
    CHECK_NEAR(r.holdBeta, StickLoopConfig::rudder().holdBeta, 1e-12);
    StickPlant fast = elev;
    fast.gain = 40.0;
    fast.holds = false;
    const StickLoopConfig f = a.forPlant(fast);
    CHECK_NEAR(f.airKp * f.airK, a.airKp * a.airK, 1e-9);
    CHECK_NEAR(f.holdG, 0.0, 1e-12);
    CHECK_NEAR(f.holdPitchRate, 0.0, 1e-12);
    // A fly-by-wire that reacts half as much to what it reads: half the hold.
    StickPlant half = elev;
    half.holdScale = 0.55;
    const StickLoopConfig h = a.forPlant(half);
    CHECK_NEAR(h.holdG, 0.55 * a.holdG, 1e-12);
    CHECK_NEAR(h.holdPitchRate, 0.55 * a.holdPitchRate, 1e-12);
}

// A surface that moves at a rate, fitted from the loop steering it: a rate, about its gain and lag.
TEST(StickFit_FindsASurfaceThatMovesAtARate) {
    RateSurface plant;
    StickLoop loop(StickLoopConfig::ailerons());
    SurfaceRun run;
    run.play(loop, plant, rollClip, 0.0, 60.0, 1e9);
    const StickPlant p = run.fit.fit();
    std::printf("  rate surface (30 deg/s, 0.2 s late): fitted %s %.1f, %.2f s late, fit %.2f\n", stickPlantForm(p.form),
                p.gain, p.lagSec, p.fit);
    REQUIRE(p.valid());
    CHECK(p.form == StickPlant::Form::Rate);
    CHECK_NEAR(p.gain, 30.0, 9.0);
    CHECK_NEAR(p.lagSec, 0.2, 0.1);
}

// Too little to go by (a still stick, or not long enough): no fit, and the A380's figures stay.
TEST(StickFit_NoFitFromTooLittle) {
    StickFit still;
    for (int i = 0; i < 30 * 60; ++i) {
        StickSample s;
        s.dt = kDt;
        s.stick = 0.1;
        s.surface = 2.0;
        s.use = true;
        still.add(s);
    }
    CHECK(!still.fit().valid());
    StickFit brief;
    DirectSurface plant;
    for (int i = 0; i < 30 * 5; ++i) {
        StickSample s;
        s.dt = kDt;
        s.stick = 0.3 * std::sin(i * 0.7);
        plant.step(s.stick, kDt);
        s.surface = plant.surface;
        s.use = true;
        brief.add(s);
    }
    CHECK(!brief.fit().valid());
}

// The A220's elevator: at a rate, 60 degrees a second per full travel, 2 frames late, with the
// fly-by-wire's own drift the hold doesn't account for (a few degrees a second, slowly changing).
// By its figures, with corrections as quick as it answers, it stays within a couple of degrees;
// with the A380's slower products it drifted off, as the first replay by its figures did.
TEST(StickLoop_AFastSurfaceGetsQuickerCorrections) {
    struct FastElevator {
        double surface = 0.0;
        double t = 0.0;
        Late<2> late;
        void step(double sent, double dt) {
            t += dt;
            const double drift = 3.0 * std::sin(t * 2.0 * kPi / 14.0) + 1.5;
            surface = clampd(surface + (60.0 * late.push(sent) + drift) * dt, -25.0, 25.0);
        }
    };
    StickPlant p;
    p.form = StickPlant::Form::Rate;
    p.gain = 60.0;
    p.lagSec = 0.07;
    const StickLoopConfig quick = StickLoopConfig::elevator().forPlant(p);
    StickLoopConfig slow = quick;
    slow.airKp = StickLoopConfig::elevator().airKp * StickLoopConfig::elevator().airK / p.gain;
    slow.airKi = StickLoopConfig::elevator().airKi * StickLoopConfig::elevator().airK / p.gain;
    CHECK(quick.airKp > 2.5 * slow.airKp);
    double worst[2] = {};
    double wiggle[2] = {};
    for (int k = 0; k < 2; ++k) {
        StickLoop loop(k == 0 ? quick : slow);
        FastElevator plant;
        SurfaceRun run;
        run.play(loop, plant, rollClip, 0.0, 60.0, 10.0);
        worst[k] = run.worst;
        wiggle[k] = run.wiggle;
        CHECK_EQ(run.damped, 0);
    }
    std::printf("  fast elevator: at most %.2f deg off by its figures (%.2f on the slower products), wiggle %.2f\n",
                worst[0], worst[1], wiggle[0]);
    CHECK(worst[0] < 2.5);
    CHECK(worst[0] < 0.6 * worst[1]);
}


TEST(StillStretches_FindsWhereTheClipsSurfaceSitsStillAtZero) {
    constexpr int n = 30 * 15;
    std::vector<Frame> frames(n);
    for (int i = 0; i < n; ++i) {
        frames[i].t = i * kDt;
        const bool still = (i >= 60 && i <= 150) || (i >= 200 && i <= 225) || (i >= 300 && i <= 400);
        frames[i].v[kRudder] = still ? 0.0 : 0.01;
    }
    frames[350].v[kRudder] = NAN;  // a gap: two stretches of 1.6 s, too short
    StillStretches s;
    s.find(frames.data(), frames.size(), kRudder);
    CHECK_EQ(static_cast<int>(s.count()), 1);
    CHECK_NEAR(s.seconds(), 3.0, 0.01);
    CHECK(s.at(2.0));
    CHECK(s.at(3.5));
    CHECK(s.at(5.0));
    CHECK(!s.at(1.99));
    CHECK(!s.at(5.02));
    CHECK(!s.at(7.0));   // 0.8 s
    CHECK(!s.at(11.0));  // the gap's
    s.find(frames.data(), frames.size(), kRudder, 1.5);
    CHECK_EQ(static_cast<int>(s.count()), 3);
    s.find(nullptr, 0, kRudder);
    CHECK_EQ(static_cast<int>(s.count()), 0);
}

// The A350's rudder through a landing: in normal law its fly-by-wire also answers the sideslip it
// reads (the hold, in the pedals, takes that out), and below 100 ft it leaves the rudder to the
// pedals, which the pilot had centred: the clip's sits at exactly 0 there, with 3 degrees of
// sideslip, and the replay's law switches with it. Steered there, the hold put the rudder 5.6
// degrees off; centred, it stays at the clip's, and the loop steers again where the law comes back.
TEST(StickLoop_CentresThePedalsWhereTheClipsRudderSitsStill) {
    constexpr int n = 30 * 24;
    constexpr double kFrom = 8.0, kTo = 16.0, kBeta = 3.0, kGain = 30.0;
    std::vector<Frame> frames(n);
    for (int i = 0; i < n; ++i) {
        const double t = i * kDt;
        frames[i].t = t;
        const bool pedalsOnly = t >= kFrom && t <= kTo;
        frames[i].v[kRudder] = pedalsOnly ? 0.0 : 3.0 * std::sin(t * 2.0 * kPi / 5.0) * kPi / 180.0;
    }
    auto clip = [&](double t) {
        const int i = static_cast<int>(std::lround(t / kDt));
        return frames[i < 0 ? 0 : (i >= n ? n - 1 : i)].v[kRudder] * 180.0 / kPi;
    };
    StickPlant plant;
    plant.form = StickPlant::Form::Direct;
    plant.gain = kGain;
    plant.lagSec = 0.11;  // (a little over the 3 frames the pedal takes here, so no rounding loses one)
    double during[2] = {}, after[2] = {};
    for (int centring = 0; centring < 2; ++centring) {
        StickLoopConfig cfg = StickLoopConfig::rudder().forPlant(plant);
        CHECK(cfg.centreStill);
        cfg.centreStill = centring == 1;
        StickLoop loop(cfg);
        StillStretches still;
        if (cfg.centreStill) still.find(frames.data(), frames.size(), kRudder);
        Late<3> late;
        double rudder = 0.0;
        for (int i = 0; i < n; ++i) {
            const double t = i * kDt;
            StickReadings r;
            r.betaDeg = kBeta;
            const double ahead = t + cfg.leadSec;
            double sent;
            if (cfg.centreStill && still.at(ahead)) {
                sent = loop.centre(kDt, false);
            } else {
                const double rate = (clip(ahead + 0.05) - clip(ahead - 0.05)) / 0.1;
                sent = loop.update(clip(t), clip(ahead), rate, rudder, kDt, false, r);
            }
            CHECK(std::isfinite(sent));
            const double pedal = late.push(sent);
            const bool pedalsOnly = t >= kFrom && t <= kTo;
            rudder = kGain * (pedalsOnly ? pedal : pedal + kBeta / 16.0);
            const double off = std::fabs(rudder - clip(t));
            // (Its last frame left out: the lead a little over the lag, the pedal steered again a
            // frame before the law comes back.)
            if (t >= kFrom && t < kTo - kDt / 2.0) during[centring] = std::fmax(during[centring], off);
            if (t >= kTo + 1.0) after[centring] = std::fmax(after[centring], off);
        }
    }
    std::printf("  rudder where the clip's sat still: at most %.2f deg off centred (%.2f steered), %.2f after "
                "(%.2f)\n",
                during[1], during[0], after[1], after[0]);
    CHECK(during[0] > 4.0);
    CHECK(during[1] < 0.5);
    CHECK(after[1] < 2.0);
}

// The A350's figures (a rudder that also builds up while the pedal's held) come back from a
// centred stretch without a kick: their tally starts again from where the rudder is.
TEST(StickLoop_ARudderThatBuildsUpComesBackFromCentred) {
    StickPlant plant;
    plant.form = StickPlant::Form::Direct;
    plant.gain = 15.0;
    plant.rateGain = 13.0;
    plant.lagSec = 0.1;
    StickLoop loop(StickLoopConfig::rudder().forPlant(plant));
    for (int i = 0; i < 90; ++i) loop.update(2.0, 2.0, 0.0, 1.0, kDt, false);
    CHECK(std::fabs(loop.stick()) > 0.0);
    for (int i = 0; i < 30; ++i) loop.centre(kDt, false);
    CHECK_EQ(loop.stick(), 0.0);
    CHECK_EQ(loop.lastHold(), 0.0);
    CHECK_EQ(loop.lastFeedforward(), 0.0);
    // Back where the rudder is the clip's: no more than a small correction.
    const double s = loop.update(0.0, 0.0, 0.0, 0.0, kDt, false);
    CHECK(std::isfinite(s));
    CHECK(std::fabs(s) < 0.05);
}
