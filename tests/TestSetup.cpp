// Control setup and profiles, against a simulated aircraft whose controls only respond to one
// method each (like the A320neo V2, which ignores the simvars and legacy events).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "AircraftProfile.h"
#include "ClipFormat.h"
#include "ControlSetup.h"
#include "Engine.h"
#include "Json.h"
#include "TestHarness.h"

using namespace br;

namespace {

constexpr double kDt = 1.0 / 60.0;

bool g_verbose = std::getenv("BR_VERBOSE") != nullptr;
void testLog(void*, int level, const char* msg) {
    if (g_verbose || level >= 2) std::printf("         [setup %d] %s\n", level, msg);
}

double approach(double v, double target, double rate, double dt) {
    const double step = rate * dt;
    if (std::fabs(target - v) <= step) return target;
    return v + (target > v ? step : -step);
}

// An airliner-like aircraft. Its own spoiler lever is an input event with an unusual name, its
// throttles are ENGINE_THROTTLE_1/2 (reading 16.7 at idle, like the A320neo V2), and the gear
// legs only start moving 2 s after the gear lever (doors first).
struct FakeAircraft : IControlSim {
    // state
    double spoilerHandle = 0.0, spoilerSurface = 0.0;
    double thr[2] = {0.0, 0.0};
    double n1[2] = {20.0, 20.0};
    double gearLever = 1.0, gearPos = 1.0, gearLeverChangedAt = -100.0, gearHandleSimvar = 1.0;
    double t = 0.0;
    bool onGround = true;
    double gs = 0.0;
    int engines = 2;
    bool rollWhenDriven = false;  // full thrust from a replay makes it roll (parking brake not holding)

    // the one method each control follows when replayed
    ControlMethod spoilersWork{MethodKind::InputSet, "AIRLINER_SPOILER_LEVER", 0.01};
    ControlMethod throttleWorks{MethodKind::InputSet, "ENGINE_THROTTLE_#", 0.01};
    ControlMethod gearWorks{MethodKind::InputSet, "LANDING_GEAR_GEAR", 1.0};

    // what the setup did
    std::vector<ControlMethod> tried;
    int scans = 0, scanReads = 0, scanEnds = 0, ends = 0;
    bool driving = false;
    Control drivenControl = Control::Spoilers;
    ControlMethod driven;
    bool refuseInputValue = false;

    void setGearLever(double v) {
        if (v != gearLever) gearLeverChangedAt = t;
        gearLever = v;
    }
    double ieThrottle(int e) const { return 16.7 + 0.833 * thr[e]; }

    void step(double dt) {
        t += dt;
        spoilerSurface = approach(spoilerSurface, spoilerHandle, 2.0, dt);
        for (int e = 0; e < 2; ++e) n1[e] = approach(n1[e], 20.0 + 0.75 * thr[e], 10.0, dt);
        if (t - gearLeverChangedAt >= 2.0) gearPos = approach(gearPos, gearLever, 0.25, dt);
        if (gearPos >= 0.999) gearHandleSimvar = 1.0;  // the handle simvar lags, as on the A320
        if (gearPos <= 0.001) gearHandleSimvar = 0.0;
        if (rollWhenDriven && driving && drivenControl == Control::Throttle && (thr[0] > 50.0 || thr[1] > 50.0)) gs = 5.0;
    }

    Frame live() const {
        Frame f;
        for (double& v : f.v) v = 0.0;
        for (int c = 0; c < kChannelCount; ++c) {
            if (kChannels[c].play == PlayMode::LVar) f.v[c] = NAN;
        }
        f.v[kOnGround] = onGround ? 1.0 : 0.0;
        f.v[kGroundSpeed] = gs;
        f.v[kAgl] = onGround ? 0.0 : 800.0;
        f.v[kSpoilersHandle] = spoilerHandle;
        f.v[kSpoilersLeft] = spoilerSurface;
        for (int e = 0; e < 2; ++e) {
            f.v[kThrottle1 + e] = thr[e];
            f.v[kEngineN1_1 + e] = n1[e];
            f.v[kInputThrottle1 + e] = ieThrottle(e);
        }
        f.v[kGearCenter] = f.v[kGearLeft] = f.v[kGearRight] = gearPos;
        f.v[kGearHandle] = gearHandleSimvar;
        f.v[kInputGear] = gearLever;
        return f;
    }

    void scanBegin(std::vector<ScanVar>& vars) override {
        ++scans;
        vars = {{"AIRLINER_SPOILER_LEVER", false}, {"ENGINE_THROTTLE_1", false}, {"ENGINE_THROTTLE_2", false},
                {"LANDING_GEAR_GEAR", false},      {"UNRELATED_SWITCH", false},  {"SIM_TIMER", true},
                {"CONSTANT", true},                {"SPOILER_ANIMATION", true}};
    }
    void scanRead(std::vector<double>& v) override {
        ++scanReads;
        v.assign(8, 0.0);
        v[0] = spoilerHandle * 100.0;
        v[1] = ieThrottle(0);
        v[2] = ieThrottle(1);
        v[3] = gearLever;
        v[4] = 1.0;
        v[5] = t * 1000.0;
        v[6] = 42.0;
        v[7] = spoilerSurface;
    }
    void scanEnd() override { ++scanEnds; }

    bool beginControl(Control c, const ControlMethod& m, std::string& why) override {
        tried.push_back(m);
        if (refuseInputValue && m.kind == MethodKind::InputValue) {
            why = "no such input event";
            return false;
        }
        driving = true;
        drivenControl = c;
        driven = m;
        return true;
    }
    void applyControl(Control c, const Frame& f, bool) override {
        if (!driving || c != drivenControl) return;
        if (c == Control::Spoilers && driven == spoilersWork) {
            double v = NAN;
            if (driven.kind == MethodKind::InputSet || driven.kind == MethodKind::InputValue) v = f.v[kInputSpeedBrakes] * driven.scale;
            if (driven.kind == MethodKind::KeyEvent || driven.kind == MethodKind::SimVar) v = f.v[kSpoilersHandle];
            if (std::isfinite(v) && v >= 0.0 && v <= 1.0) spoilerHandle = v;  // out of range: ignored
        } else if (c == Control::Throttle && driven == throttleWorks) {
            for (int e = 0; e < engines; ++e) {
                const double arg = f.v[kInputThrottle1 + e] * driven.scale;
                if (std::isfinite(arg) && arg >= 0.0 && arg <= 1.0) {
                    const double pct = (arg * 100.0 - 16.7) / 0.833;
                    thr[e] = pct < 0.0 ? 0.0 : (pct > 100.0 ? 100.0 : pct);
                }
            }
        } else if (c == Control::Gear && driven == gearWorks) {
            const double v = driven.kind == MethodKind::KeyEvent ? f.v[kGearHandle] : f.v[kInputGear];
            if (std::isfinite(v)) setGearLever(v >= 0.5 ? 1.0 : 0.0);
        }
    }
    void endControl(Control) override {
        driving = false;
        ++ends;
    }
};

SetupFacts airlinerFacts() {
    SetupFacts f;
    f.onRunway = true;
    f.parkingBrake = true;
    f.engineRunning = true;
    f.engineType = 1;
    f.engines = 2;
    f.gearRetractable = true;
    f.hasSpoilers = true;
    f.flapPositions = 0;  // no flaps in these tests: one step fewer
    return f;
}

struct SetupHarness {
    FakeAircraft aircraft;
    SetupFlow flow;
    SetupFacts facts = airlinerFacts();
    double t = 0.0;
    int asks = 0;
    double firstReplayAt = -1.0;  // harness time when the first replay started
    std::string lastPhase;

    SetupHarness() {
        flow.setLogger(&testLog, nullptr);
        flow.setSim(&aircraft);
    }

    bool start() {
        std::string err;
        const bool ok = flow.start(AircraftProfile(), aircraft.live(), facts, err);
        if (!ok) std::printf("         start failed: %s\n", err.c_str());
        return ok;
    }

    JsonValue status() const {
        std::string out;
        JsonWriter w(out);
        w.beginObject();
        flow.statusJson(w);
        w.endObject();
        JsonValue v;
        std::string err;
        parseJson(out.data(), out.size(), v, &err);
        return v;
    }
    std::string phase() const { return flow.active() ? status().getString("phase", "") : "idle"; }

    // Runs the sim for `seconds`; `pilot(t)` moves the controls (t from the start of this run)
    // until the setup starts replaying: hands off from then on. Stops early once the setup
    // reaches `until` (a phase name), if given.
    bool run(double seconds, const std::function<void(double)>& pilot = nullptr, const char* until = nullptr) {
        for (double s = 0.0; s < seconds; s += kDt) {
            const std::string before = phase();
            if (pilot && (before == "prepare" || before == "user" || before == "takeoff")) pilot(s);
            aircraft.step(kDt);
            flow.onFrame(kDt, aircraft.live(), facts);
            t += kDt;
            const std::string p = phase();
            if (p == "ask" && lastPhase != "ask") ++asks;
            if (p == "replay" && firstReplayAt < 0.0) firstReplayAt = t;
            lastPhase = p;
            if (until && p == until) return true;
        }
        return !until;
    }

    bool answer(const char* a) {
        std::string err;
        const bool ok = flow.answer(a, err);
        if (!ok) std::printf("         answer %s failed: %s\n", a, err.c_str());
        lastPhase = phase();
        return ok;
    }
};

// Deploy the spoilers over 0.5 s at t0, hold 1 s, retract over 0.5 s.
std::function<void(double)> deploySpoilers(FakeAircraft& a, double t0) {
    return [&a, t0](double s) {
        const double x = s - t0;
        if (x < 0.0) a.spoilerHandle = 0.0;
        else if (x < 0.5) a.spoilerHandle = x / 0.5;
        else if (x < 1.5) a.spoilerHandle = 1.0;
        else if (x < 2.0) a.spoilerHandle = 1.0 - (x - 1.5) / 0.5;
        else a.spoilerHandle = 0.0;
    };
}

}  // namespace

TEST(Setup_FindsTheSpoilerInputEventByItsMovementAndAsksOnce) {
    SetupHarness h;
    h.facts.gearRetractable = false;
    REQUIRE(h.start());
    CHECK_EQ(h.phase(), std::string("prepare"));
    CHECK(h.flow.profile()[Control::Flaps].state == ControlState::Absent);
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Absent);

    CHECK(h.run(30.0, deploySpoilers(h.aircraft, 2.0), "ask"));
    CHECK_EQ(h.asks, 1);
    CHECK_EQ(h.aircraft.scans, 1);
    CHECK_EQ(h.aircraft.scanEnds, 1);
    REQUIRE(!h.aircraft.tried.empty());
    // The unusual name was found by what moved, and the percent value is scaled to a fraction.
    CHECK(h.aircraft.tried[0] == (ControlMethod{MethodKind::InputSet, "AIRLINER_SPOILER_LEVER", 0.01}));
    CHECK_EQ(h.aircraft.tried.size(), size_t(1));
    CHECK_EQ(h.status().getString("step", ""), std::string("spoilers"));

    REQUIRE(h.answer("yes"));
    const ControlProfile& sp = h.flow.profile()[Control::Spoilers];
    CHECK(sp.state == ControlState::Ok);
    CHECK(sp.confirmed);
    CHECK(sp.method == h.aircraft.spoilersWork);
    CHECK(h.flow.takeChanged());
    CHECK(!h.flow.takeChanged());

    // Throttle next; it needs the parking brake.
    h.facts.parkingBrake = false;
    h.run(0.5);
    CHECK_EQ(h.phase(), std::string("prepare"));
    CHECK_EQ(h.status().getString("detail", ""), std::string("Set the parking brake first."));
    const JsonValue prepare = h.status();
    const JsonValue* buttons = prepare.get("buttons");
    REQUIRE(buttons && buttons->isArray() && !buttons->array.empty());
    CHECK_EQ(buttons->array[0].string, std::string("ready"));
    REQUIRE(h.answer("ready"));  // e.g. an aircraft whose parking brake isn't the simvar
    CHECK_EQ(h.phase(), std::string("user"));
    REQUIRE(h.answer("skip"));
    CHECK_EQ(h.phase(), std::string("done"));
    CHECK(h.flow.profile()[Control::Throttle].state == ControlState::Skipped);
    REQUIRE(h.answer("close"));
    CHECK(!h.flow.active());
}

TEST(Setup_FailingMethodsAreSkippedWithoutAskingAndANoTriesTheNext) {
    SetupHarness h;
    h.facts.gearRetractable = false;
    h.aircraft.spoilersWork = ControlMethod{MethodKind::KeyEvent, "", 1.0};
    REQUIRE(h.start());
    CHECK(h.run(60.0, deploySpoilers(h.aircraft, 2.0), "ask"));
    CHECK_EQ(h.asks, 1);
    // Both preset scalings failed the automatic check before the key event passed it.
    REQUIRE(h.aircraft.tried.size() == 3);
    CHECK(h.aircraft.tried[0].kind == MethodKind::InputSet);
    CHECK(h.aircraft.tried[1].kind == MethodKind::InputSet);
    CHECK(h.aircraft.tried[2].kind == MethodKind::KeyEvent);

    REQUIRE(h.answer("again"));  // watch it once more
    CHECK(h.run(30.0, nullptr, "ask"));
    REQUIRE(h.aircraft.tried.size() == 4);
    CHECK(h.aircraft.tried[3].kind == MethodKind::KeyEvent);

    // "No": the rest are tried and fail, so the spoilers end up not replayable.
    REQUIRE(h.answer("no"));
    h.run(60.0, nullptr, "prepare");
    CHECK_EQ(h.asks, 2);
    CHECK(h.flow.profile()[Control::Spoilers].state == ControlState::Failed);
    std::vector<MethodKind> rest;
    for (size_t i = 4; i < h.aircraft.tried.size(); ++i) rest.push_back(h.aircraft.tried[i].kind);
    REQUIRE(rest.size() == 3);
    CHECK(rest[0] == MethodKind::SimVar);
    CHECK(rest[1] == MethodKind::LVar);
    CHECK(h.aircraft.tried[5].var == "SPOILER_ANIMATION");
    CHECK(rest[2] == MethodKind::InputValue);
    CHECK_EQ(h.aircraft.ends, static_cast<int>(h.aircraft.tried.size()));
    CHECK(h.status().getString("note", "").find("Couldn't replay the spoilers") != std::string::npos);
}

TEST(Setup_ThrottleUsesOnePatternForBothEnginesAndWaitsForTheSpool) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.gearRetractable = false;
    REQUIRE(h.start());
    CHECK_EQ(h.status().getString("step", ""), std::string("throttle"));
    auto pilot = [&](double s) {
        const double x = s - 2.0;
        double v = 0.0;
        if (x >= 0.0 && x < 2.0) v = 50.0 * x;
        else if (x >= 2.0 && x < 4.0) v = 100.0;
        else if (x >= 4.0 && x < 5.0) v = 100.0 - 100.0 * (x - 4.0);
        h.aircraft.thr[0] = h.aircraft.thr[1] = v;
    };
    CHECK(h.run(90.0, pilot, "ask"));
    // The lever is back at idle at 7 s, but the pilot's part only ends once N1 is back down (~13 s).
    CHECK(h.firstReplayAt > 11.0);
    REQUIRE(!h.aircraft.tried.empty());
    CHECK(h.aircraft.tried[0] == h.aircraft.throttleWorks);
    CHECK_NEAR(h.aircraft.thr[0], 0.0, 1.0);
    REQUIRE(h.answer("yes"));
    CHECK(h.flow.profile()[Control::Throttle].method.var == "ENGINE_THROTTLE_#");
}

TEST(Setup_ThrottleReplayStopsWhenTheAircraftRolls) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.gearRetractable = false;
    h.aircraft.rollWhenDriven = true;
    REQUIRE(h.start());
    auto pilot = [&](double s) {
        const double v = s > 2.0 && s < 6.0 ? 100.0 : 0.0;
        if (h.phase() == "user" || h.phase() == "prepare") h.aircraft.thr[0] = h.aircraft.thr[1] = v;
    };
    CHECK(h.run(90.0, pilot, "ask"));
    CHECK(h.status().getString("note", "").find("roll") != std::string::npos);
    CHECK_NEAR(h.aircraft.thr[0], 0.0, 1e-6);  // put back to idle
    CHECK(!h.aircraft.driving);
}

TEST(Setup_GearAfterTakeoffFindsTheLeverAheadOfTheLegs) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    REQUIRE(h.start());
    CHECK_EQ(h.status().getNumber("steps", 0), 2.0);
    REQUIRE(h.answer("skip"));  // throttle
    CHECK_EQ(h.phase(), std::string("takeoff"));
    h.run(1.0);
    CHECK_EQ(h.phase(), std::string("takeoff"));

    h.aircraft.onGround = false;
    h.run(0.1);
    CHECK_EQ(h.phase(), std::string("user"));
    // Lever up at 2 s (the legs follow from 4 s), down again at 12 s.
    auto pilot = [&](double s) { h.aircraft.setGearLever(s >= 2.0 && s < 12.0 ? 0.0 : 1.0); };
    CHECK(h.run(120.0, pilot, "ask"));
    REQUIRE(!h.aircraft.tried.empty());
    CHECK(h.aircraft.tried[0] == h.aircraft.gearWorks);
    CHECK_NEAR(h.aircraft.gearPos, 1.0, 1e-6);
    REQUIRE(h.answer("yes"));
    CHECK_EQ(h.phase(), std::string("done"));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Ok);
}

TEST(Setup_InTheAirOnlyTheGear) {
    SetupHarness h;
    h.aircraft.onGround = false;
    REQUIRE(h.start());
    CHECK_EQ(h.status().getNumber("steps", 0), 1.0);
    CHECK_EQ(h.status().getString("step", ""), std::string("gear"));

    SetupHarness fixed;
    fixed.aircraft.onGround = false;
    fixed.facts.gearRetractable = false;
    std::string err;
    CHECK(!fixed.flow.start(AircraftProfile(), fixed.aircraft.live(), fixed.facts, err));
    CHECK(!err.empty());
}

TEST(Setup_NoMovementTimesOutToRetry) {
    SetupHarness h;
    h.facts.gearRetractable = false;
    REQUIRE(h.start());
    CHECK(h.run(95.0, nullptr, "retry"));
    CHECK(h.status().getString("note", "").find("Didn't see the spoilers move") != std::string::npos);
    CHECK_EQ(h.aircraft.scanEnds, 1);
    REQUIRE(h.answer("retry"));
    CHECK_EQ(h.phase(), std::string("user"));
    CHECK(h.run(30.0, deploySpoilers(h.aircraft, 1.0), "ask"));
    REQUIRE(h.answer("cancel"));
    CHECK(!h.flow.active());
    CHECK(!h.aircraft.driving);
    std::string err;
    CHECK(!h.flow.answer("yes", err));
}

TEST(Setup_PausedSimHoldsTheClock) {
    SetupHarness h;
    h.facts.gearRetractable = false;
    REQUIRE(h.start());
    h.run(0.1);
    for (int i = 0; i < 60 * 200; ++i) h.flow.onFrame(0.0, h.aircraft.live(), h.facts);  // 200 s paused
    CHECK_EQ(h.phase(), std::string("user"));
}

TEST(Setup_TrackerIgnoresTimersAndChangesBeforeTheMovement) {
    CandidateTracker t;
    t.begin({{"LEVER", false}, {"TIMER", true}, {"EARLY_FLAG", true}, {"STILL", true}});
    std::vector<double> v(4);
    for (int s = 0; s <= 100; ++s) {
        const double time = s * 0.1;
        const double lever = time >= 6.0 && time < 8.0 ? 1.0 : 0.0;
        v[0] = lever * 50.0;
        v[1] = time;                    // counts up: never comes back
        v[2] = time >= 0.5 ? 1.0 : 0.0;  // flipped long before
        v[3] = 3.0;
        t.sample(time, v, lever, lever);
        if (s == 55) t.dropChangedBefore(1.0);
    }
    const auto found = t.find(5.0, 10.0, false, false, nullptr, 4);
    REQUIRE(found.size() == 1);
    CHECK_EQ(t.var(found[0].index).name, std::string("LEVER"));
    CHECK_NEAR(found[0].peak, 50.0, 1e-9);
    CHECK(t.find(5.0, 10.0, true, false, nullptr, 4).empty());
    CHECK(!t.changed(2));
    CHECK(!t.changed(3));
    CHECK_NEAR(t.valueAt(0, 7.0), 50.0, 1e-6);
    CHECK_NEAR(t.valueAt(0, 2.0), 0.0, 1e-6);
    CHECK_NEAR(t.firstChange(0), 6.0, 1e-9);
}

// ---------------------------------------------------------------------------- profiles

TEST(Profile_JsonRoundTrip) {
    ProfileStore store;
    store.upsert("c172").declined = true;
    AircraftProfile& a = store.upsert("simobjects/airplanes/a320neo");
    a.title = "Airbus A320neo \"V2\"";
    a.updatedUtcMs = 1792247525000;
    a[Control::Spoilers] = {ControlState::Failed, ControlMethod(), false};
    a[Control::Flaps] = {ControlState::Ok, ControlMethod{MethodKind::InputSet, "HANDLING_FLAPS", 1.0}, true};
    a[Control::Throttle] = {ControlState::Ok, ControlMethod{MethodKind::InputSet, "ENGINE_THROTTLE_#", 0.01}, true};
    a[Control::Gear] = {ControlState::Pending, ControlMethod(), false};

    std::string json;
    store.toJson(json);
    ProfileStore back;
    std::string err;
    REQUIRE(back.fromJson(json.data(), json.size(), &err));
    REQUIRE(back.size() == 2);
    const AircraftProfile* b = back.find("simobjects/airplanes/a320neo");
    REQUIRE(b);
    CHECK_EQ(b->title, a.title);
    CHECK_EQ(b->updatedUtcMs, a.updatedUtcMs);
    CHECK((*b)[Control::Spoilers].state == ControlState::Failed);
    CHECK((*b)[Control::Flaps].method == a[Control::Flaps].method);
    CHECK((*b)[Control::Throttle].method == a[Control::Throttle].method);
    CHECK((*b)[Control::Throttle].confirmed);
    CHECK(b->anyPending());
    CHECK(b->drives(Control::Spoilers));
    CHECK(!b->drives(Control::Gear));
    CHECK(back.find("c172")->declined);
    CHECK(!back.find("nope"));
    CHECK(back.remove("c172"));
    CHECK(!back.remove("c172"));

    CHECK(!back.fromJson("{\"version\":1}", 13, &err));
    const char* bad = "{\"aircraft\":[{\"key\":\"x\",\"controls\":{\"gear\":{\"state\":\"ok\",\"method\":\"none\"}}}]}";
    REQUIRE(back.fromJson(bad, std::strlen(bad), &err));
    CHECK(back.find("x")->controls[3].state == ControlState::Pending);  // "ok" without a method
}

TEST(Profile_ChannelsAndNames) {
    CHECK_EQ(engineVar("ENGINE_THROTTLE_#", 2), std::string("ENGINE_THROTTLE_2"));
    CHECK_EQ(engineVar("A32NX_THROTTLE", 2), std::string("A32NX_THROTTLE"));
    CHECK_EQ(methodSourceChannel(Control::Throttle, MethodKind::InputSet, 1), static_cast<int>(kInputThrottle2));
    CHECK_EQ(methodSourceChannel(Control::Throttle, MethodKind::LVar, 3), static_cast<int>(kLVarThrottle4));
    CHECK_EQ(methodSourceChannel(Control::Flaps, MethodKind::KeyEvent, 0), static_cast<int>(kFlapsHandleIndex));
    CHECK_EQ(methodSourceChannel(Control::Gear, MethodKind::SimVar, 0), -1);
    CHECK_EQ(methodSourceChannel(Control::Spoilers, MethodKind::LVar, 2), static_cast<int>(kLVarSpeedBrakes));
    ChannelMask flaps;
    controlChannels(Control::Flaps, flaps);
    CHECK(flaps.test(kFlapsLeft) && flaps.test(kFlapsRight) && flaps.test(kFlapsHandleIndex));
    CHECK(flaps.test(kInputFlaps) && flaps.test(kLVarFlaps));
    CHECK(!flaps.test(kSpoilersHandle));
    Control c;
    CHECK(parseControlKey("gear", c) && c == Control::Gear);
    CHECK(!parseControlKey("rudder", c));
}

TEST(Profile_ClipSupportsTheMethodOnlyWhenRecordedFromTheSameVariable) {
    Clip clip;
    clip.present.set(kInputThrottle1);
    clip.present.set(kInputThrottle2);
    clip.present.set(kThrottle1);
    clip.present.set(kThrottle2);
    clip.present.set(kLVarSpeedBrakes);
    std::string why;
    const ControlMethod thr{MethodKind::InputSet, "ENGINE_THROTTLE_#", 0.01};
    // Older clips don't list their variables; the default input event names are assumed.
    CHECK(clipSupports(Control::Throttle, thr, clip, 2, &why));
    CHECK(!clipSupports(Control::Throttle, ControlMethod{MethodKind::InputSet, "MY_THR_#", 0.01}, clip, 2, &why));
    CHECK(!clipSupports(Control::Throttle, thr, clip, 3, &why));  // no third engine in the clip
    CHECK(clipSupports(Control::Throttle, ControlMethod{MethodKind::KeyEvent, "", 1.0}, clip, 2, &why));
    CHECK(!clipSupports(Control::Flaps, ControlMethod{MethodKind::SimVar, "", 1.0}, clip, 1, &why));
    CHECK(clipSupports(Control::Gear, ControlMethod(), clip, 1, &why));  // Failed: nothing to read

    const ControlMethod lv{MethodKind::LVar, "MY_SPOILER", 1.0};
    CHECK(!clipSupports(Control::Spoilers, lv, clip, 1, &why));
    clip.meta.vars.push_back({"lv_spdbrk", "MY_SPOILER"});
    CHECK(clipSupports(Control::Spoilers, lv, clip, 1, &why));
    clip.meta.vars.push_back({"ie_thr_1", "OTHER_THROTTLE_1"});
    CHECK(!clipSupports(Control::Throttle, thr, clip, 2, &why));
    CHECK(why.find("OTHER_THROTTLE_1") != std::string::npos);
}

TEST(ClipFormat_VarsChunkRoundTrip) {
    ClipMeta meta;
    meta.aircraft = "A320";
    meta.vars = {{"ie_flaps", "HANDLING_FLAPS"}, {"lv_spdbrk", "XMLVAR_SPOILERS"}};
    std::vector<Frame> frames(3);
    for (size_t i = 0; i < frames.size(); ++i) {
        for (double& v : frames[i].v) v = 1.0;
        frames[i].t = static_cast<double>(i) / 30.0;
    }
    std::vector<uint8_t> bytes;
    writeClip(meta, VectorFrameSource(frames), bytes);
    Clip clip;
    std::string err;
    REQUIRE(readClip(bytes.data(), bytes.size(), clip, &err));
    REQUIRE(clip.meta.vars.size() == 2);
    CHECK_EQ(clip.meta.vars[1].first, std::string("lv_spdbrk"));
    CHECK_EQ(clip.meta.vars[1].second, std::string("XMLVAR_SPOILERS"));
    CHECK_EQ(clip.meta.aircraft, std::string("A320"));
}

// ---------------------------------------------------------------------------- engine

namespace {

struct MemStore : IStorage {
    std::map<std::string, std::vector<uint8_t>> files;
    bool writeFile(const std::string& n, const uint8_t* d, size_t s) override {
        files[n].assign(d, d + s);
        return true;
    }
    bool readFile(const std::string& n, std::vector<uint8_t>& out) override {
        const auto it = files.find(n);
        if (it == files.end()) return false;
        out = it->second;
        return true;
    }
    bool removeFile(const std::string& n) override { return files.erase(n) > 0; }
    bool replaceFile(const std::string& a, const std::string& b) override {
        const auto it = files.find(a);
        if (it == files.end()) return false;
        files[b] = it->second;
        files.erase(a);
        return true;
    }
    bool listFiles(std::vector<std::string>& names) override {
        names.clear();
        for (const auto& kv : files) names.push_back(kv.first);
        return true;
    }
};

struct ProfileSpy : ISimOutput {
    int applied = 0;
    bool haveProfile = false;
    AircraftProfile last;
    void beginPlayback(const Clip&, const Frame&) override {}
    void writePlaybackFrame(const Frame&, bool) override {}
    void sendChannelEvent(int, double) override {}
    void endPlayback(const Frame&, bool) override {}
    void applyProfile(const AircraftProfile* p) override {
        ++applied;
        haveProfile = p != nullptr;
        if (p) last = *p;
    }
};

struct EngineSetupHarness {
    MemStore storage;
    FakeAircraft aircraft;
    ProfileSpy spy;
    Engine engine{storage};
    SetupFacts facts = airlinerFacts();
    double simTime = 100.0;

    EngineSetupHarness() {
        engine.setLogger(&testLog, nullptr);
        engine.setSimOutput(&spy);
        engine.setControlSim(&aircraft);
        engine.init();
        engine.setSimRunning(true);
        engine.setAircraftTitle("Test Airliner");
        engine.setAircraftKey("simobjects/airplanes/test_airliner");
    }
    void run(double seconds, const std::function<void(double)>& pilot = nullptr) {
        for (double s = 0.0; s < seconds; s += kDt) {
            if (pilot) {
                const std::string phase = setupStatus().getString("phase", "");
                if (phase == "prepare" || phase == "user") pilot(s);  // hands off during replays
            }
            aircraft.step(kDt);
            FrameInput in;
            in.raw = aircraft.live();
            in.simTimeSec = simTime;
            in.facts = facts;
            simTime += kDt;
            engine.onFrame(in, 1792247525000 + static_cast<int64_t>(simTime * 1000.0));
        }
    }
    JsonValue setupStatus() {
        std::string out;
        engine.statusJson(out);
        JsonValue v;
        std::string err;
        parseJson(out.data(), out.size(), v, &err);
        const JsonValue* s = v.get("setup");
        return s ? *s : JsonValue();
    }
};

}  // namespace

TEST(EngineSetup_OfferedOnTheRunwayWithEnginesRunning) {
    EngineSetupHarness h;
    h.run(0.5);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("new"));
    CHECK_EQ(h.setupStatus().getString("have", "?"), std::string("none"));

    h.facts.onRunway = false;  // at the gate
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
    h.facts.onRunway = true;
    h.facts.engineRunning = false;
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
    h.facts.engineRunning = true;

    std::string err;
    CHECK(h.engine.setupLater(err));
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
    h.engine.onFlightLoaded("test");  // a new flight offers it again
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("new"));

    CHECK(h.engine.setupNever(err));
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
    CHECK_EQ(h.setupStatus().getString("have", "?"), std::string("declined"));
    CHECK(h.storage.files.count("aircraft.json") == 1);
    CHECK(h.engine.setupReset(err));
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("new"));
    CHECK(!h.spy.haveProfile);
}

TEST(EngineSetup_StepsAreSavedAndAppliedAndSurviveARestart) {
    EngineSetupHarness h;
    h.facts.gearRetractable = false;
    h.run(0.2);
    std::string err;
    REQUIRE(h.engine.setupStart(err));
    CHECK(h.engine.setupActive());
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
    CHECK(!h.engine.play("x.rpl", err));  // unknown clip
    h.run(30.0, deploySpoilers(h.aircraft, 2.0));
    CHECK_EQ(h.setupStatus().getString("phase", ""), std::string("ask"));
    REQUIRE(h.engine.setupAnswer("yes", err));
    CHECK(h.spy.haveProfile);
    CHECK(h.spy.last[Control::Spoilers].state == ControlState::Ok);
    CHECK_EQ(h.spy.last.title, std::string("Test Airliner"));
    REQUIRE(h.engine.setupAnswer("skip", err));  // throttle
    REQUIRE(h.engine.setupAnswer("close", err));
    CHECK(!h.engine.setupActive());
    const JsonValue s = h.setupStatus();
    CHECK_EQ(s.getString("have", ""), std::string("complete"));
    const JsonValue* controls = s.get("controls");
    REQUIRE(controls && controls->isArray() && controls->array.size() == 4);
    CHECK_EQ(controls->array[0].getString("state", ""), std::string("ok"));
    CHECK(controls->array[0].getString("method", "").find("AIRLINER_SPOILER_LEVER") != std::string::npos);

    // A new engine on the same work folder knows the aircraft.
    Engine again(h.storage);
    again.init();
    again.setAircraftKey("simobjects/airplanes/test_airliner");
    REQUIRE(again.currentProfile());
    CHECK((*again.currentProfile())[Control::Spoilers].method == h.aircraft.spoilersWork);
}

TEST(EngineSetup_ClipsListTheVariablesTheirChannelsRecorded) {
    MemStore storage;
    Engine engine(storage);
    engine.init();
    engine.setSimRunning(true);
    engine.setChannelVar(kInputFlaps, "HANDLING_FLAPS");
    engine.setChannelVar(kLVarSpeedBrakes, "XMLVAR_SPOILERS");
    FrameInput in;
    for (int i = 0; i < 90; ++i) {
        for (double& v : in.raw.v) v = 0.0;
        in.simTimeSec = i / 30.0;
        engine.onFrame(in, 1792247525000 + i * 33);
    }
    std::string err;
    REQUIRE(engine.saveLast(1792247530000, err));
    REQUIRE(engine.index().size() == 1);
    Clip clip;
    const std::vector<uint8_t>& bytes = storage.files[engine.index().clips()[0].file];
    REQUIRE(readClip(bytes.data(), bytes.size(), clip, &err));
    REQUIRE(clip.meta.vars.size() == 2);
    CHECK_EQ(clip.meta.vars[0].first, std::string("ie_flaps"));
    CHECK_EQ(clip.meta.vars[1].second, std::string("XMLVAR_SPOILERS"));
}
