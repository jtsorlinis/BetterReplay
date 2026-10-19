#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "EventDetector.h"
#include "TestHarness.h"
#include "TraceGen.h"

using namespace br;
using namespace tracegen;

namespace {

constexpr double kTol = 0.05;  // a bit over one 30 Hz sample
// The engine power history is kept every ~0.27 s, and the spool-up is seen once N1 is 3% of
// takeoff power above idle: a fraction of a second after it starts.
constexpr double kSpoolTol = 0.5;

bool g_verbose = false;
void testLog(void*, const char* msg) {
    if (g_verbose) std::printf("         [detector] %s\n", msg);
}

EventDetector makeDetector(const DetectorConfig& cfg = DetectorConfig()) {
    EventDetector d(cfg);
    d.setLogger(&testLog, nullptr);
    return d;
}

// Departure from a standstill without engine readings: 30 s parked, taxi, 10 s lined up, 30 s
// roll to 70 kt, then a climb for 140 s. Returns {lastSlowT, liftoffT} for the checks.
struct TakeoffTimes {
    double lastSlow;
    double liftoff;
};

TakeoffTimes departure(TraceGen& g) {
    g.hold(30.0, true);
    g.segment(5.0, 8.0, 0.0, true);
    g.hold(20.0, true);
    g.segment(3.0, 0.0, 0.0, true);
    g.hold(10.0, true);
    const double rollStart = g.now();
    g.segment(30.0, 70.0, 0.0, true);  // crosses 10 kt after 30 * 10/70 s
    const double liftoff = g.now() + g.dt;
    g.segment(40.0, 90.0, 1200.0, false);
    g.segment(100.0, 120.0, 3000.0, false);
    return {rollStart + 30.0 * 10.0 / 70.0, liftoff};
}

// The same with N1: the engines are started less than 3 minutes before liftoff, a breakaway
// bump to 35% gets it taxiing, then idle, a stop on the runway, 40% held for 4 s to let the
// engines stabilise and on to 95% on the brakes. Returns {spool-up start, liftoff}.
struct SpoolTimes {
    double spool;
    double liftoff;
};

SpoolTimes poweredDeparture(TraceGen& g) {
    g.power = 0.0;
    g.hold(10.0, true);         // engines off
    g.spool(30.0, 20.0, true);  // engine start, to idle
    g.hold(20.0, true);
    g.spool(2.0, 35.0, true);  // breakaway
    g.segment(5.0, 8.0, 0.0, true);
    g.spool(2.0, 20.0, true);  // idle taxi
    g.hold(20.0, true);
    g.segment(3.0, 0.0, 0.0, true);  // stopped on the runway
    g.hold(10.0, true);
    const double spool = g.now();
    g.spool(3.0, 40.0, true);  // stabilise
    g.hold(4.0, true);
    g.spool(4.0, 95.0, true);
    g.segment(30.0, 70.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(40.0, 90.0, 1200.0, false);
    g.segment(100.0, 120.0, 3000.0, false);
    return {spool, liftoff};
}

// Index of the first sample at or after `i` with ground speed below `kt`.
size_t firstBelow(const TraceGen& g, size_t i, double kt) {
    while (i < g.samples.size() && !(g.samples[i].groundSpeedKt < kt)) ++i;
    return i;
}

}  // namespace

// ---------------------------------------------------------------------------- takeoff

TEST(Detector_TakeoffWithoutARunwayEntryStartsFiveSecondsBeforeTheSpoolUp) {
    // TraceGen starts on a runway, so it never rolls onto one (as if spawned there).
    TraceGen g(true, 0.0, 0.0);
    const SpoolTimes st = poweredDeparture(g);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    CHECK(e.type == EventType::Takeoff);
    // Not the engine start, the breakaway or the 40% hold: the start of the spool-up for takeoff.
    CHECK(e.startT >= st.spool - 5.0 - kTol);
    CHECK(e.startT <= st.spool - 5.0 + kSpoolTol);
    CHECK_NEAR(e.eventT, st.liftoff, kTol);
    CHECK_NEAR(e.endT, st.liftoff + 120.0, kTol);  // flaps up at liftoff
}

TEST(Detector_TakeoffSpoolUpFromATurbopropsHighIdle) {
    // Gas generator idle around 65%, takeoff 100%: still well below takeoff power.
    TraceGen g(true, 0.0, 0.0);
    g.power = 64.0;
    g.hold(20.0, true);
    g.spool(1.0, 66.0, true);  // idle wanders a little
    g.segment(20.0, 10.0, 0.0, true);
    g.segment(5.0, 0.0, 0.0, true);
    g.hold(5.0, true);
    const double spool = g.now();
    g.spool(6.0, 100.0, true);
    g.segment(25.0, 85.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(130.0, 120.0, 2500.0, false);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].startT >= spool - 5.0 - kTol);
    CHECK(r.events[0].startT <= spool - 5.0 + kSpoolTol);
    CHECK_NEAR(r.events[0].endT, liftoff + 120.0, kTol);
}

TEST(Detector_TakeoffWithoutEngineReadingsStartsAtTheRoll) {
    TraceGen g(true, 0.0, 0.0);
    const TakeoffTimes tt = departure(g);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    CHECK(e.type == EventType::Takeoff);
    CHECK_NEAR(e.startT, tt.lastSlow, kTol);
    CHECK_NEAR(e.eventT, tt.liftoff, kTol);
    CHECK_NEAR(e.endT, tt.liftoff + 120.0, kTol);
}

TEST(Detector_TakeoffWithFlapsUpEndsTwoMinutesAfterLiftoff) {
    TraceGen g(true, 0.0, 0.0);
    g.hold(15.0, true);
    g.segment(25.0, 60.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(150.0, 80.0, 400.0, false);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Takeoff);
    CHECK_NEAR(r.events[0].endT, liftoff + 120.0, kTol);
}

TEST(Detector_TakeoffEndsWhenBackOnTheGroundSooner) {
    // A low circuit: back on the runway 90 s after liftoff, then a landing to a stop.
    TraceGen g(true, 0.0, 0.0);
    g.hold(15.0, true);
    g.segment(25.0, 60.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(30.0, 80.0, 500.0, false);
    g.segment(60.0, 70.0, 0.0, false);
    const size_t td = g.index();
    g.segment(20.0, 0.0, 0.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 2);
    CHECK(r.events[0].type == EventType::Takeoff);
    CHECK_NEAR(r.events[0].eventT, liftoff, kTol);
    CHECK_NEAR(r.events[0].endT, g.samples[td].t, 1e-9);
    CHECK(r.events[1].type == EventType::Landing);
    CHECK_NEAR(r.events[1].eventT, g.samples[td].t, 1e-9);
}

TEST(Detector_TakeoffStartsThirtySecondsBeforeEnteringTheRunwayAndEndsClean) {
    // Taxiing out across another runway, onto the departure runway, lined up briefly, off with
    // the flaps out and cleaned up 90 s after liftoff.
    TraceGen g(true, 0.0, 0.0);
    g.onRunway = false;
    g.flapsUp = false;
    g.power = 20.0;
    g.hold(20.0, true);
    g.segment(5.0, 12.0, 0.0, true);
    g.onRunway = true;  // crossing
    g.hold(6.0, true);
    g.onRunway = false;
    g.hold(30.0, true);
    const double entry = g.now() + g.dt;
    g.onRunway = true;
    g.segment(5.0, 0.0, 0.0, true);
    g.hold(5.0, true);
    g.spool(4.0, 95.0, true);
    g.segment(30.0, 150.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(90.0, 220.0, 3000.0, false);
    g.flapsUp = true;
    const double clean = g.now() + g.dt;
    g.segment(60.0, 250.0, 5000.0, false);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    CHECK(e.type == EventType::Takeoff);
    CHECK_NEAR(e.startT, entry - 30.0, 1e-6);
    CHECK_NEAR(e.eventT, liftoff, kTol);
    CHECK_NEAR(e.endT, clean + 10.0, kTol);
}

TEST(Detector_TakeoffAfterALongWaitOnTheRunwayStartsSixtySecondsBeforeTheSpoolUp) {
    TraceGen g(true, 0.0, 0.0);
    g.onRunway = false;
    g.power = 20.0;
    g.hold(15.0, true);
    g.segment(5.0, 10.0, 0.0, true);
    g.onRunway = true;
    g.segment(5.0, 0.0, 0.0, true);
    g.hold(120.0, true);  // lined up and waiting
    const double spool = g.now();
    g.spool(4.0, 95.0, true);
    g.segment(30.0, 150.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(130.0, 200.0, 3000.0, false);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].startT >= spool - 60.0 - kTol);
    CHECK(r.events[0].startT <= spool - 60.0 + kSpoolTol);
    CHECK_NEAR(r.events[0].endT, liftoff + 120.0, kTol);
}

TEST(Detector_TakeoffNeverCleanEndsFiveMinutesAfterLiftoff) {
    TraceGen g(true, 0.0, 0.0);
    g.flapsUp = false;
    g.hold(15.0, true);
    g.segment(25.0, 60.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(320.0, 90.0, 3000.0, false);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK_NEAR(r.events[0].endT, liftoff + 300.0, kTol);
}

// ---------------------------------------------------------------------------- landing

TEST(Detector_NormalLandingEndsAtTheStop) {
    TraceGen g(false, 3000.0, 140.0);
    g.segment(240.0, 130.0, 1000.0, false);
    const size_t td = approachAndTouchdown(g, 120.0);
    g.segment(30.0, 0.0, 0.0, true);  // rolls to a stop, still on the runway
    g.samples[td].gForce = 1.35;
    g.hold(20.0, true);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    const double tdT = g.samples[td].t;
    CHECK(e.type == EventType::Landing);
    CHECK_NEAR(e.eventT, tdT, 1e-9);
    CHECK_NEAR(e.startT, tdT - 180.0, 1e-9);
    CHECK_NEAR(e.endT, g.samples[firstBelow(g, td, 1.0)].t, 1e-9);
    CHECK_NEAR(e.touchdownVsFpm, -150.0, 1.0);  // VS just before contact (no touchdown simvar)
    CHECK_NEAR(e.peakG, 1.35, 1e-6);
    CHECK_EQ(e.bounces, 0);
}

TEST(Detector_LandingEndsTenSecondsOffTheRunway) {
    // Turned off with the flaps still out.
    {
        TraceGen g(false, 1500.0, 120.0);
        g.flapsUp = false;
        const size_t td = approachAndTouchdown(g, 115.0);
        g.segment(25.0, 15.0, 0.0, true);
        g.onRunway = false;
        const size_t exit = g.index();
        g.hold(30.0, true);
        EventDetector d = makeDetector();
        const RunResult r = runAll(d, g.samples);
        REQUIRE(r.events.size() == 1);
        CHECK_NEAR(r.events[0].eventT, g.samples[td].t, 1e-9);
        CHECK_NEAR(r.events[0].endT, g.samples[exit].t + 10.0, kTol);
    }
    // Across another runway on the way: 10 s after leaving that one.
    {
        TraceGen g(false, 1500.0, 120.0);
        approachAndTouchdown(g, 115.0);
        g.segment(25.0, 15.0, 0.0, true);
        g.onRunway = false;
        g.hold(5.0, true);
        g.onRunway = true;
        g.hold(4.0, true);
        g.onRunway = false;
        const size_t exit = g.index();
        g.hold(30.0, true);
        EventDetector d = makeDetector();
        const RunResult r = runAll(d, g.samples);
        REQUIRE(r.events.size() == 1);
        CHECK_NEAR(r.events[0].endT, g.samples[exit].t + 10.0, kTol);
    }
}

TEST(Detector_LandingAwayFromARunwayEndsAtTheStop) {
    // A strip the sim doesn't know as a runway, landed with the flaps up: never "off the runway".
    TraceGen g(false, 1500.0, 90.0);
    g.onRunway = false;
    const size_t td = approachAndTouchdown(g, 60.0);
    g.segment(20.0, 0.0, 0.0, true);
    g.hold(10.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK_NEAR(r.events[0].endT, g.samples[firstBelow(g, td, 1.0)].t, 1e-9);
}

TEST(Detector_LandingEndsAtTheTimeLimitWhenRollingOn) {
    // Rolling on along the runway without stopping (a long backtrack).
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(25.0, 15.0, 0.0, true);
    g.hold(200.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK_NEAR(r.events[0].endT, g.samples[td].t + 170.0, kTol);
}

TEST(Detector_LandingPrefersTouchdownNormalVelocity) {
    TraceGen g(false, 1500.0, 120.0);
    approachAndTouchdown(g, 110.0);
    g.tdNormalFps = 2.2;  // the sim latches this at touchdown
    g.segment(30.0, 0.0, 0.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK_NEAR(r.events[0].touchdownVsFpm, -132.0, 1e-3);
}

TEST(Detector_BouncedLandingIsOneEvent) {
    TraceGen g(false, 1500.0, 120.0);
    const size_t td1 = approachAndTouchdown(g, 115.0);
    g.segment(0.6, 113.0, 0.0, true);
    g.samples[td1].gForce = 1.6;
    g.segment(1.0, 112.0, 8.0, false);  // bounce up to 8 ft
    g.segment(1.2, 110.0, 0.0, false);
    const size_t td2 = g.index();
    g.segment(30.0, 0.0, 0.0, true);
    g.samples[td2].gForce = 1.25;
    g.hold(10.0, true);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    CHECK(e.type == EventType::Landing);
    CHECK_EQ(e.bounces, 1);
    CHECK_NEAR(e.eventT, g.samples[td1].t, 1e-9);  // clip anchored on the first touchdown
    CHECK_NEAR(e.touchdownVsFpm, -150.0, 1.0);    // first touchdown's rate, not the bounce's
    CHECK_NEAR(e.peakG, 1.6, 1e-6);
    CHECK_NEAR(e.endT, g.samples[firstBelow(g, td2, 1.0)].t, 1e-9);
}

TEST(Detector_TouchAndGoGivesLandingThenTakeoff) {
    TraceGen g(false, 1500.0, 110.0);
    const size_t td = approachAndTouchdown(g, 95.0);
    g.segment(4.0, 70.0, 0.0, true);
    g.segment(6.0, 80.0, 0.0, true);  // never stops, 10 s on the ground
    const double liftoff = g.now() + g.dt;
    g.segment(40.0, 95.0, 1200.0, false);
    g.hold(100.0, false);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 2);
    const DetectedEvent& land = r.events[0];
    const DetectedEvent& to = r.events[1];
    CHECK(land.type == EventType::Landing);
    CHECK_NEAR(land.eventT, g.samples[td].t, 1e-9);
    CHECK_NEAR(land.endT, liftoff + 15.0, kTol);  // ends once airborne again for 15 s
    CHECK_EQ(land.bounces, 0);
    CHECK(to.type == EventType::Takeoff);
    CHECK_NEAR(to.startT, g.samples[td].t, kTol);  // no engine readings, never below 10 kt
    CHECK_NEAR(to.eventT, liftoff, kTol);
    CHECK_NEAR(to.endT, liftoff + 120.0, kTol);
}

TEST(Detector_TouchAndGoTakeoffStartsBeforeItsSpoolUp) {
    // Approach power held long enough to look steady; idle from the flare, only 3 s of it on the
    // ground: the spool-up is found from the ground run alone.
    TraceGen g(false, 1500.0, 110.0);
    g.power = 55.0;
    g.segment(170.0, 105.0, 50.0, false);
    g.spool(2.0, 20.0, false);
    g.segment(18.0, 95.0, 0.0, false);
    const size_t td = g.index();
    g.segment(3.0, 80.0, 0.0, true);
    const double spool = g.now();
    g.spool(4.0, 95.0, true);
    g.segment(8.0, 85.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(140.0, 120.0, 2500.0, false);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 2);
    CHECK(r.events[0].type == EventType::Landing);
    CHECK_NEAR(r.events[0].eventT, g.samples[td].t, 1e-9);
    const DetectedEvent& to = r.events[1];
    CHECK(to.type == EventType::Takeoff);
    CHECK(to.startT >= spool - 5.0 - kTol);
    CHECK(to.startT <= spool - 5.0 + kSpoolTol);
    CHECK_NEAR(to.endT, liftoff + 120.0, kTol);
}

TEST(Detector_BounceIntoAGoAroundEndsTheLandingAfterFifteenSeconds) {
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(0.5, 113.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(40.0, 120.0, 1000.0, false);  // never back on the ground
    g.hold(120.0, false);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 2);
    CHECK(r.events[0].type == EventType::Landing);
    CHECK_NEAR(r.events[0].eventT, g.samples[td].t, 1e-9);
    CHECK_NEAR(r.events[0].endT, liftoff + 15.0, kTol);
    CHECK_EQ(r.events[0].bounces, 0);
    CHECK(r.events[1].type == EventType::Takeoff);
    CHECK_NEAR(r.events[1].eventT, liftoff, kTol);
}

TEST(Detector_ShortBouncesStayOneLanding) {
    // Airborne for 12 s after the first touchdown: still a bounce.
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(0.5, 113.0, 0.0, true);
    g.segment(6.0, 110.0, 40.0, false);
    g.segment(6.0, 105.0, 0.0, false);
    g.segment(30.0, 0.0, 0.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Landing);
    CHECK_NEAR(r.events[0].eventT, g.samples[td].t, 1e-9);
    CHECK_EQ(r.events[0].bounces, 1);
}

// ---------------------------------------------------------------------------- arming, pauses

TEST(Detector_SpawnOnRunwayTriggersNothing) {
    // The aircraft is placed slightly above the runway and settles, with SIM ON GROUND flickering.
    TraceGen g(false, 4.0, 0.0);
    g.segment(0.2, 0.0, 0.0, false);
    g.hold(0.3, true);
    g.hold(0.2, false);
    g.hold(1.5, true);
    g.hold(120.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    CHECK(r.events.empty());
    CHECK(d.armed());
}

TEST(Detector_SpawnOnRunwayThenDepart) {
    TraceGen g(false, 3.0, 0.0);
    g.hold(0.3, false);
    g.hold(1.0, true);
    const TakeoffTimes tt = departure(g);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Takeoff);
    CHECK_NEAR(r.events[0].startT, tt.lastSlow, kTol);
}

TEST(Detector_SpawnAirborneTriggersNothing) {
    // First frame of a fresh flight can report on-ground garbage before the state settles.
    TraceGen g(true, 2500.0, 0.0);
    g.segment(1.0, 120.0, 2500.0, false);
    g.hold(180.0, false);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    CHECK(r.events.empty());
}

TEST(Detector_LiftoffBeforeArmingIsIgnored) {
    TraceGen g(true, 0.0, 0.0);
    g.segment(5.0, 60.0, 0.0, true);
    g.segment(30.0, 90.0, 1500.0, false);  // airborne at t=5, arms at t=10 while climbing
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    CHECK(r.events.empty());
}

TEST(Detector_PauseMidLandingKeepsTheLanding) {
    // The recording clock stops while paused, so a pause is just a resume notification between
    // two consecutive samples.
    auto build = [](TraceGen& g) {
        const size_t td = approachAndTouchdown(g, 115.0);
        g.segment(30.0, 0.0, 0.0, true);
        g.hold(10.0, true);
        return td;
    };
    TraceGen base(false, 1500.0, 120.0);
    build(base);
    EventDetector d0 = makeDetector();
    const RunResult expected = runAll(d0, base.samples);
    REQUIRE(expected.events.size() == 1);

    TraceGen g(false, 1500.0, 120.0);
    const size_t td = build(g);
    const size_t pauseAt = td + 45;  // 1.5 s into the rollout, before the landing is confirmed
    EventDetector d = makeDetector();
    RunResult r;
    feed(d, g.samples, 0, pauseAt, r);
    d.notifyResumedAfterPause();
    feed(d, g.samples, pauseAt, g.samples.size(), r);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Landing);
    CHECK_NEAR(r.events[0].eventT, expected.events[0].eventT, 1e-9);
    CHECK_NEAR(r.events[0].endT, expected.events[0].endT, 1e-9);
    CHECK_NEAR(r.events[0].touchdownVsFpm, expected.events[0].touchdownVsFpm, 1e-6);
}

TEST(Detector_PauseWithRelocationEndsTheLandingThere) {
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(1.5, 105.0, 0.0, true);
    const size_t pauseAt = g.index();
    g.latDeg += 0.1;  // moved ~11 km from the pause menu, parked
    g.gsKt = 0.0;
    g.hold(60.0, true);

    EventDetector d = makeDetector();
    RunResult r;
    feed(d, g.samples, 0, pauseAt, r);
    CHECK(std::string(d.phase()) == "landing-touchdown");
    d.notifyResumedAfterPause();
    feed(d, g.samples, pauseAt, pauseAt + 1, r);
    CHECK(!d.armed());
    REQUIRE(r.events.size() == 1);  // saved up to the pause, not lost
    CHECK(r.events[0].type == EventType::Landing);
    CHECK_NEAR(r.events[0].eventT, g.samples[td].t, 1e-9);
    CHECK_NEAR(r.events[0].endT, g.samples[pauseAt - 1].t, 1e-9);
    feed(d, g.samples, pauseAt + 1, g.samples.size(), r);
    CHECK_EQ(r.events.size(), 1u);
    CHECK(d.armed());
}

TEST(Detector_FinishInProgressEndsALandingOrConfirmedTakeoffThere) {
    // Mid-rollout (the flight ends, say): the landing ends at the last sample.
    {
        TraceGen g(false, 1500.0, 120.0);
        const size_t td = approachAndTouchdown(g, 115.0);
        g.segment(10.0, 60.0, 0.0, true);
        EventDetector d = makeDetector();
        RunResult r = runAll(d, g.samples);
        CHECK(r.events.empty());
        DetectedEvent out[2];
        REQUIRE(d.finishInProgress(g.now(), out) == 1);
        CHECK(out[0].type == EventType::Landing);
        CHECK_NEAR(out[0].eventT, g.samples[td].t, 1e-9);
        CHECK_NEAR(out[0].endT, g.now(), 1e-9);
        CHECK_EQ(d.finishInProgress(g.now(), out), 0);  // only once
    }
    // A confirmed takeoff too; one that isn't confirmed yet is dropped.
    {
        TraceGen g(true, 0.0, 0.0);
        g.hold(15.0, true);
        g.segment(25.0, 60.0, 0.0, true);
        g.segment(3.0, 65.0, 30.0, false);
        EventDetector d = makeDetector();
        RunResult r = runAll(d, g.samples);
        DetectedEvent out[2];
        CHECK_EQ(d.finishInProgress(g.now(), out), 0);
        g.segment(30.0, 90.0, 900.0, false);
        EventDetector d2 = makeDetector();
        r = runAll(d2, g.samples);
        REQUIRE(d2.finishInProgress(g.now(), out) == 1);
        CHECK(out[0].type == EventType::Takeoff);
        CHECK_NEAR(out[0].endT, g.now(), 1e-9);
    }
}

TEST(Detector_PauseOnShortFinal) {
    auto build = [](TraceGen& g) {
        g.segment(60.0, 120.0, 200.0, false);
        const size_t pauseAt = g.index();
        g.segment(8.0, 115.0, 0.0, false);  // touches down 8 s after the pause
        g.segment(30.0, 0.0, 0.0, true);
        return pauseAt;
    };

    // Default: no hold-off after a pause without movement, so the landing is kept.
    {
        TraceGen g(false, 600.0, 125.0);
        const size_t pauseAt = build(g);
        EventDetector d = makeDetector();
        RunResult r;
        feed(d, g.samples, 0, pauseAt, r);
        d.notifyResumedAfterPause();
        feed(d, g.samples, pauseAt, g.samples.size(), r);
        REQUIRE(r.events.size() == 1);
        CHECK(r.events[0].type == EventType::Landing);
    }
    // Strict config: new events are held off for 10 s after every pause, so this one is lost.
    {
        DetectorConfig cfg;
        cfg.holdOffNewEventsAfterPause = true;
        TraceGen g(false, 600.0, 125.0);
        const size_t pauseAt = build(g);
        EventDetector d = makeDetector(cfg);
        RunResult r;
        feed(d, g.samples, 0, pauseAt, r);
        d.notifyResumedAfterPause();
        feed(d, g.samples, pauseAt, g.samples.size(), r);
        CHECK(r.events.empty());
    }
}

TEST(Detector_StrictHoldOffStillKeepsInProgressLanding) {
    DetectorConfig cfg;
    cfg.holdOffNewEventsAfterPause = true;
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(30.0, 0.0, 0.0, true);
    EventDetector d = makeDetector(cfg);
    RunResult r;
    feed(d, g.samples, 0, td + 30, r);
    d.notifyResumedAfterPause();
    feed(d, g.samples, td + 30, g.samples.size(), r);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Landing);
}

TEST(Detector_DisarmMidTakeoffDiscardsIt) {
    TraceGen g(true, 0.0, 0.0);
    g.hold(15.0, true);
    g.segment(25.0, 60.0, 0.0, true);
    g.segment(3.0, 70.0, 100.0, false);
    const size_t slewAt = g.index();
    g.segment(60.0, 90.0, 1500.0, false);
    EventDetector d = makeDetector();
    RunResult r;
    feed(d, g.samples, 0, slewAt, r);
    CHECK(std::string(d.phase()) == "takeoff-liftoff");
    d.disarm(g.samples[slewAt].t, "slew");
    feed(d, g.samples, slewAt, g.samples.size(), r);
    CHECK(r.events.empty());
}

TEST(Detector_ShortHopIsNeitherTakeoffNorLanding) {
    TraceGen g(true, 0.0, 0.0);
    g.hold(15.0, true);
    g.segment(10.0, 50.0, 0.0, true);
    g.segment(2.0, 52.0, 10.0, false);  // airborne 4 s, never above 50 ft
    g.segment(2.0, 50.0, 0.0, false);
    g.segment(20.0, 0.0, 0.0, true);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    CHECK(r.events.empty());
}

TEST(Detector_AbortedTakeoffAfterLongerFloatIsALanding) {
    TraceGen g(true, 0.0, 0.0);
    g.hold(15.0, true);
    g.segment(15.0, 60.0, 0.0, true);
    g.segment(4.0, 65.0, 40.0, false);  // 8 s airborne below 50 ft: not a takeoff...
    g.segment(4.0, 60.0, 0.0, false);
    g.segment(20.0, 0.0, 0.0, true);  // ...but long enough that putting it back down is a landing
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Landing);
}
