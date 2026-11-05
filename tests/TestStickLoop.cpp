#include <cmath>

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
