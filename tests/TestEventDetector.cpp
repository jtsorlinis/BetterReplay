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

bool g_verbose = false;
void testLog(void*, const char* msg) {
    if (g_verbose) std::printf("         [detector] %s\n", msg);
}

EventDetector makeDetector(const DetectorConfig& cfg = DetectorConfig()) {
    EventDetector d(cfg);
    d.setLogger(&testLog, nullptr);
    return d;
}

// Departure from a standstill: 30 s parked, taxi, 10 s lined up, 30 s roll to 70 kt, then a
// 1800 fpm climb. Returns {lastSlowT, liftoffT, reach1000T} for the checks.
struct TakeoffTimes {
    double lastSlow;
    double liftoff;
    double at1000;
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
    g.segment(40.0, 90.0, 1200.0, false);  // 1000 ft after 40 * 1000/1200 s
    g.segment(60.0, 120.0, 3000.0, false);
    return {rollStart + 30.0 * 10.0 / 70.0, liftoff, liftoff - g.dt + 40.0 * 1000.0 / 1200.0};
}

}  // namespace

TEST(Detector_NormalTakeoff) {
    TraceGen g(true, 0.0, 0.0);
    const TakeoffTimes tt = departure(g);
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    CHECK(e.type == EventType::Takeoff);
    CHECK_NEAR(e.startT, tt.lastSlow, kTol);
    CHECK_NEAR(e.eventT, tt.liftoff, kTol);
    CHECK_NEAR(e.endT, tt.at1000, kTol);
}

TEST(Detector_TakeoffEndsSixtySecondsAfterLiftoffWhenClimbingSlowly) {
    TraceGen g(true, 0.0, 0.0);
    g.hold(15.0, true);
    g.segment(25.0, 60.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(120.0, 80.0, 900.0, false);  // never reaches 1000 ft AGL inside 60 s
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Takeoff);
    CHECK_NEAR(r.events[0].endT, liftoff + 60.0, kTol);
}

TEST(Detector_NormalLanding) {
    TraceGen g(false, 3000.0, 140.0);
    g.segment(240.0, 130.0, 1000.0, false);
    const size_t td = approachAndTouchdown(g, 120.0);
    g.segment(25.0, 20.0, 0.0, true);  // 120 -> 20 kt; below 30 kt after 22.5 s
    g.samples[td].gForce = 1.35;
    g.segment(10.0, 15.0, 0.0, true);
    g.hold(20.0, true);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    const DetectedEvent& e = r.events[0];
    const double tdT = g.samples[td].t;
    CHECK(e.type == EventType::Landing);
    CHECK_NEAR(e.eventT, tdT, 1e-9);
    CHECK_NEAR(e.startT, tdT - 120.0, 1e-9);
    CHECK_NEAR(e.endT, tdT + 22.5, kTol);
    CHECK_NEAR(e.touchdownVsFpm, -150.0, 1.0);  // VS just before contact (no touchdown simvar)
    CHECK_NEAR(e.peakG, 1.35, 1e-6);
    CHECK_EQ(e.bounces, 0);
}

TEST(Detector_LandingPrefersTouchdownNormalVelocity) {
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 110.0);
    g.tdNormalFps = 2.2;  // the sim latches this at touchdown
    g.segment(25.0, 15.0, 0.0, true);
    (void)td;
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
    g.segment(25.0, 15.0, 0.0, true);
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
}

TEST(Detector_TouchAndGoGivesLandingThenTakeoff) {
    TraceGen g(false, 1500.0, 110.0);
    const size_t td = approachAndTouchdown(g, 95.0);
    g.segment(4.0, 70.0, 0.0, true);
    g.segment(6.0, 80.0, 0.0, true);  // never below 30 kt, 10 s on the ground
    const double liftoff = g.now() + g.dt;
    g.segment(40.0, 95.0, 1200.0, false);
    g.hold(30.0, false);

    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 2);
    const DetectedEvent& land = r.events[0];
    const DetectedEvent& to = r.events[1];
    CHECK(land.type == EventType::Landing);
    CHECK_NEAR(land.eventT, g.samples[td].t, 1e-9);
    CHECK_NEAR(land.endT, liftoff, kTol);  // ends when airborne again
    CHECK(to.type == EventType::Takeoff);
    CHECK_NEAR(to.startT, g.samples[td].t, kTol);  // GS never < 10 kt: start of this ground run
    CHECK_NEAR(to.eventT, liftoff, kTol);
    CHECK_NEAR(to.endT, liftoff + 40.0 * 1000.0 / 1200.0, kTol);
}

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
        g.segment(25.0, 15.0, 0.0, true);
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

TEST(Detector_PauseWithRelocationDropsTheLanding) {
    TraceGen g(false, 1500.0, 120.0);
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(1.5, 105.0, 0.0, true);
    const size_t pauseAt = g.index();
    g.latDeg += 0.1;  // moved ~11 km from the pause menu, parked
    g.gsKt = 0.0;
    g.hold(60.0, true);
    (void)td;

    EventDetector d = makeDetector();
    RunResult r;
    feed(d, g.samples, 0, pauseAt, r);
    CHECK(std::string(d.phase()) == "landing-touchdown");
    d.notifyResumedAfterPause();
    feed(d, g.samples, pauseAt, pauseAt + 1, r);
    CHECK(!d.armed());
    feed(d, g.samples, pauseAt + 1, g.samples.size(), r);
    CHECK(r.events.empty());
    CHECK(d.armed());
}

TEST(Detector_PauseOnShortFinal) {
    auto build = [](TraceGen& g) {
        g.segment(60.0, 120.0, 200.0, false);
        const size_t pauseAt = g.index();
        g.segment(8.0, 115.0, 0.0, false);  // touches down 8 s after the pause
        g.segment(25.0, 15.0, 0.0, true);
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
    g.segment(25.0, 15.0, 0.0, true);
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
    g.segment(20.0, 5.0, 0.0, true);
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
    g.segment(20.0, 5.0, 0.0, true);  // ...but long enough that putting it back down is a landing
    EventDetector d = makeDetector();
    const RunResult r = runAll(d, g.samples);
    REQUIRE(r.events.size() == 1);
    CHECK(r.events[0].type == EventType::Landing);
}
