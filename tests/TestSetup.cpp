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

    // An Airbus-like throttle instead (the A320neo V2): the pilot moves an axis (0..1), the lever
    // has detents (idle, CL, FLX/MCT, TOGA) and reads in INI_ENGn_THROTTLE_POS, and the lever
    // simvar is the thrust demand, which follows the lever with a lag (a thrust output follows
    // that). Replayed, only the throttle key events move the lever (as THROTTLEn_AXIS_SET_EX1 would).
    bool airbus = false;
    double demandLagSec = 1.2;    // 0: the demand settles at once (no creep to see)
    double demandCreepPct = 0.0;  // and it keeps creeping up this much a second while off idle
    bool hideLeverVars = false;   // the INI_ L:vars don't move (the lever can't be found from them)
    bool atRequest = false;       // and an autothrust request that reads the same as the lever (the A350)
    double axis[2] = {0.0, 0.0};
    double out[2] = {0.0, 0.0};
    double creep[2] = {0.0, 0.0};
    bool reversers = true;       // the reverse thrust events engage them; the axis then sets the amount
    bool reverseZone = false;    // the axis's bottom fifth is reverse instead
    bool reverser = false;
    int reverserCalls = 0;
    // The PMDG 737 instead: the reverse events don't reverse it (its levers go forward with the
    // axis), but "decrease throttle" held at idle does, without the reverse flag, and let go it
    // comes back to idle. Its lever variable runs from 500 at idle down to 75 at full.
    bool reverseByHold = false;
    bool leverDown = false;
    bool holdReverse = false;
    double decreaseAt = -100.0;
    int decreases = 0;
    bool axisRecorded = false;      // the key-event handler sees the pilot's axis (thr_axis_N)
    bool hideThrottleVars = false;  // nothing but the simvar moves with the throttle

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

    // The lever (TLA, degrees) where the axis puts it: detents at idle (0), CL (25), FLX/MCT (35)
    // and TOGA (45); reverse down to -20.
    double tla(int e) const {
        if (holdReverse) return -20.0;
        double a = axis[e];
        if (reverser) return -20.0 * a;
        if (reverseZone) {
            if (a < 0.2) return -20.0 + 100.0 * a;
            a = (a - 0.2) / 0.8;
        }
        if (a < 0.05) return 0.0;
        if (a < 0.45) return (a - 0.05) / 0.4 * 22.5;
        if (a < 0.55) return 25.0;
        if (a < 0.7) return 27.0 + (a - 0.55) / 0.15 * 6.0;
        if (a < 0.8) return 35.0;
        if (a < 0.9) return 37.0 + (a - 0.8) / 0.1 * 6.0;
        return 45.0;
    }
    static constexpr double kLeverUnits = 51.9 / 45.0;  // INI_ENGn_THROTTLE_POS per degree
    static constexpr double kLeverDownUnits = 425.0 / 45.0;
    double leverVar(int e) const { return leverDown ? 500.0 - tla(e) * kLeverDownUnits : tla(e) * kLeverUnits; }

    void step(double dt) {
        t += dt;
        if (holdReverse && t - decreaseAt > 0.2) holdReverse = false;  // let go
        spoilerSurface = approach(spoilerSurface, spoilerHandle, 2.0, dt);
        if (airbus) {
            for (int e = 0; e < 2; ++e) {
                const double l = tla(e);
                creep[e] = l > 0.0 ? creep[e] + demandCreepPct * dt : 0.0;
                const double demand = (l >= 0.0 ? l / 45.0 * 85.0 : l) + creep[e];
                thr[e] += (demand - thr[e]) * (dt < demandLagSec ? dt / demandLagSec : 1.0);
                out[e] += (thr[e] - out[e]) * (dt / 0.6 < 1.0 ? dt / 0.6 : 1.0);
            }
        }
        for (int e = 0; e < 2; ++e) n1[e] = approach(n1[e], 20.0 + 0.75 * std::fabs(thr[e]), 10.0, dt);
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
            f.v[kThrottle1 + e] = !airbus && holdReverse ? -37.0 : thr[e];  // (the PMDG's simvar in full reverse)
            f.v[kEngineN1_1 + e] = n1[e];
            f.v[kInputThrottle1 + e] = ieThrottle(e);
            f.v[kReverser1 + e] = reverser ? 1.0 : 0.0;
            f.v[kReverserNozzle1 + e] = reverser || holdReverse ? 100.0 : 0.0;
            f.v[kThrottleAxis1 + e] = airbus && axisRecorded ? axis[e] : NAN;
        }
        f.v[kGearCenter] = f.v[kGearLeft] = f.v[kGearRight] = gearPos;
        f.v[kGearHandle] = gearHandleSimvar;
        f.v[kInputGear] = gearLever;
        return f;
    }

    std::vector<ScanVar> scanVars() const {
        std::vector<ScanVar> vars = {{"AIRLINER_SPOILER_LEVER", false}, {"ENGINE_THROTTLE_1", false},
                                     {"ENGINE_THROTTLE_2", false},      {"LANDING_GEAR_GEAR", false},
                                     {"UNRELATED_SWITCH", false},       {"SIM_TIMER", true},
                                     {"CONSTANT", true},                {"SPOILER_ANIMATION", true}};
        if (airbus) {
            // the lever per engine; a flag; a thrust output that follows the demand
            vars.insert(vars.end(), {{"INI_ENG1_THROTTLE_POS", true}, {"INI_ENG2_THROTTLE_POS", true},
                                     {"INI_LEVER_IN_TOGA", true},     {"INI_THROTTLE_OUTPUT_ENGINE1", true},
                                     {"INI_THROTTLE_OUTPUT_ENGINE2", true}});
            if (atRequest) {
                vars.insert(vars.end(), {{"INI_AUTOTHROTTLE_ENGINE1_AT_REQUEST", true},
                                         {"INI_AUTOTHROTTLE_ENGINE2_AT_REQUEST", true}});
            }
        }
        return vars;
    }
    std::vector<double> scanValues() const {
        std::vector<double> v = {spoilerHandle * 100.0, ieThrottle(0), ieThrottle(1), gearLever, 1.0, t * 1000.0, 42.0,
                                 spoilerSurface};
        if (hideThrottleVars) v[1] = v[2] = 16.7;
        if (airbus && hideLeverVars) v.insert(v.end(), {0.0, 0.0, 0.0, 0.0, 0.0});
        else if (airbus) v.insert(v.end(), {leverVar(0), leverVar(1), tla(0) >= 44.9 ? 1.0 : 0.0, out[0], out[1]});
        if (airbus && atRequest) v.insert(v.end(), {leverVar(0), leverVar(1)});
        return v;
    }
    void scanBegin(std::vector<ScanVar>& vars) override {
        ++scans;
        vars = scanVars();
    }
    void scanRead(std::vector<double>& v) override {
        ++scanReads;
        v = scanValues();
    }
    void scanEnd() override { ++scanEnds; }
    double readVar(const ScanVar& var) override {
        const std::vector<ScanVar> vars = scanVars();
        const std::vector<double> values = scanValues();
        for (size_t i = 0; i < vars.size(); ++i) {
            if (vars[i].name == var.name && vars[i].lvar == var.lvar) return values[i];
        }
        return NAN;
    }
    void setReverser(bool on) override {
        ++reverserCalls;
        if (reversers) reverser = on;
    }
    void throttleDecrease() override {
        ++decreases;
        decreaseAt = t;
        if (reverseByHold && (airbus ? axis[0] < 0.05 : thr[0] < 1.0)) holdReverse = true;
    }

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
        } else if (c == Control::Throttle && driven == throttleWorks && driven.kind == MethodKind::KeyEvent) {
            // Like ControlDriver: the recorded hardware axis, else the lever simvar as a fraction.
            for (int e = 0; e < engines; ++e) {
                double a = f.v[kThrottleAxis1 + e];
                if (!std::isfinite(a)) a = f.v[kThrottle1 + e] / 100.0;
                if (!std::isfinite(a)) continue;
                a = a < 0.0 ? 0.0 : (a > 1.0 ? 1.0 : a);
                if (airbus) {
                    axis[e] = a;
                } else {
                    thr[e] = reverser ? -25.0 * a : 100.0 * a;  // with the reverser in, the reverse amount
                }
            }
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
    std::vector<std::string> prompts;  // every main text shown, in order
    bool sawTaskDone = false;          // the last instruction was shown as done

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
            if (pilot && (before == "prepare" || before == "user" || before == "takeoff" || before == "positions")) pilot(s);
            aircraft.step(kDt);
            flow.onFrame(kDt, aircraft.live(), facts);
            t += kDt;
            const JsonValue st = status();
            const std::string p = flow.active() ? st.getString("phase", "") : "idle";
            const std::string prompt = st.getString("prompt", "");
            if (!prompt.empty() && (prompts.empty() || prompts.back() != prompt)) prompts.push_back(prompt);
            if (st.getBool("taskDone", false)) sawTaskDone = true;
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

// Whether `want` appear in `seen` in this order (other prompts may come between).
bool inOrder(const std::vector<std::string>& seen, const std::vector<std::string>& want) {
    size_t i = 0;
    for (const std::string& s : seen) {
        if (i < want.size() && s == want[i]) ++i;
    }
    if (i < want.size()) {
        std::printf("         prompts seen:");
        for (const std::string& s : seen) std::printf(" [%s]", s.c_str());
        std::printf("\n");
    }
    return i == want.size();
}

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

// A pilot who follows the throttle's instructions from 1 s in: to `half`, then `full`, each held
// until the setup ticks it, then back to `idle` (the lever simvar in percent, or on the Airbus the
// axis), at full travel a second. Records when the throttle was back at idle.
struct ThrottlePilot {
    double idle, half, full;
    double backAt = -1.0;
    void operator()(SetupHarness& h, double s) {
        double& v = h.aircraft.airbus ? h.aircraft.axis[0] : h.aircraft.thr[0];
        if (s >= 1.0) {
            const JsonValue st = h.status();
            const std::string phase = st.getString("phase", "");
            const int task = static_cast<int>(st.getNumber("task", 0));
            const bool pilot = phase == "user" || phase == "positions";
            const double target = !pilot || task >= 2 ? idle : (task == 0 ? half : full);
            v = approach(v, target, (h.aircraft.airbus ? 1.0 : 100.0), kDt);
            if (pilot && task >= 2 && v == idle && backAt < 0.0) backAt = h.t;
        }
        (h.aircraft.airbus ? h.aircraft.axis[1] : h.aircraft.thr[1]) = v;
    }
};

std::function<void(double)> followThrottle(SetupHarness& h, ThrottlePilot& p) {
    return [&h, &p](double s) { p(h, s); };
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
    // One instruction at a time, the last shown as done while things settle.
    CHECK(inOrder(h.prompts, {"Deploy the spoilers", "Retract the spoilers", "Watch the spoilers",
                              "Did the spoilers come up and go back down?"}));
    CHECK(h.sawTaskDone);
    CHECK_EQ(h.aircraft.scans, 1);
    CHECK_EQ(h.aircraft.scanEnds, 1);
    REQUIRE(!h.aircraft.tried.empty());
    // The unusual name was found by what moved, and the percent value is scaled to a fraction.
    CHECK(h.aircraft.tried[0] == (ControlMethod{MethodKind::InputSet, "AIRLINER_SPOILER_LEVER", 0.01}));
    CHECK_EQ(h.aircraft.tried.size(), size_t(1));
    CHECK_EQ(h.status().getString("step", ""), std::string("spoilers"));

    REQUIRE(h.answer("yes"));
    const JsonValue afterSpoilers = h.status();
    const JsonValue* plan = afterSpoilers.get("plan");
    REQUIRE(plan && plan->isArray() && plan->array.size() == 2);
    CHECK_EQ(plan->array[0].getString("control", ""), std::string("spoilers"));
    CHECK_EQ(plan->array[0].getString("state", ""), std::string("ok"));
    CHECK_EQ(plan->array[1].getString("control", ""), std::string("throttle"));
    CHECK_EQ(plan->array[1].getString("state", ""), std::string("current"));
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
    CHECK_EQ(h.status().getString("prompt", ""), std::string("Set the parking brake"));
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
    {
        const JsonValue st = h.status();
        const JsonValue* attempts = st.get("attempts");
        REQUIRE(attempts && attempts->isArray() && attempts->array.size() == 2);
        CHECK_EQ(attempts->array[0].string, std::string("failed"));
        CHECK_EQ(attempts->array[1].string, std::string("failed"));
        CHECK_EQ(st.getNumber("method", 0), 3.0);
    }
    CHECK(h.aircraft.tried[0].kind == MethodKind::InputSet);
    CHECK(h.aircraft.tried[1].kind == MethodKind::InputSet);
    CHECK(h.aircraft.tried[2].kind == MethodKind::KeyEvent);

    REQUIRE(h.answer("again"));  // watch it once more
    CHECK(h.run(30.0, nullptr, "ask"));
    REQUIRE(h.aircraft.tried.size() == 4);
    CHECK(h.aircraft.tried[3].kind == MethodKind::KeyEvent);

    // "No": the rest are tried and fail, so the spoilers end up not replayable.
    REQUIRE(h.answer("no"));
    {
        const JsonValue st = h.status();
        const JsonValue* attempts = st.get("attempts");
        REQUIRE(attempts && attempts->array.size() == 3);
        CHECK_EQ(attempts->array[2].string, std::string("rejected"));  // the pilot said no
    }
    h.run(60.0, nullptr, "prepare");
    CHECK_EQ(h.asks, 2);
    CHECK(h.flow.profile()[Control::Spoilers].state == ControlState::Failed);
    std::vector<MethodKind> rest;
    for (size_t i = 4; i < h.aircraft.tried.size(); ++i) rest.push_back(h.aircraft.tried[i].kind);
    REQUIRE(rest.size() == 4);
    CHECK(rest[0] == MethodKind::KeyEvent);
    CHECK(h.aircraft.tried[4].var == kKeySpoilersAxis);
    CHECK(rest[1] == MethodKind::SimVar);
    CHECK(rest[2] == MethodKind::LVar);
    CHECK(h.aircraft.tried[6].var == "SPOILER_ANIMATION");
    CHECK(rest[3] == MethodKind::InputValue);
    CHECK_EQ(h.aircraft.ends, static_cast<int>(h.aircraft.tried.size()));
    CHECK(h.status().getString("note", "").find("Couldn't replay the spoilers") != std::string::npos);
}

// The PMDG 737 ignores SPOILERS_SET but takes AXIS_SPOILER_SET, what a hardware speedbrake axis
// sends: it's tried after the default key event, before the simvars and the L:vars.
TEST(Setup_SpoilersFallBackToTheHardwareAxisEvent) {
    SetupHarness h;
    h.facts.gearRetractable = false;
    h.aircraft.spoilersWork = ControlMethod{MethodKind::KeyEvent, kKeySpoilersAxis, 1.0};
    REQUIRE(h.start());
    CHECK(h.run(90.0, deploySpoilers(h.aircraft, 2.0), "ask"));
    REQUIRE(h.aircraft.tried.size() == 4);
    CHECK(h.aircraft.tried[0].kind == MethodKind::InputSet);
    CHECK(h.aircraft.tried[1].kind == MethodKind::InputSet);
    CHECK(h.aircraft.tried[2] == (ControlMethod{MethodKind::KeyEvent, "", 1.0}));
    CHECK(h.aircraft.tried[3] == h.aircraft.spoilersWork);
    REQUIRE(h.answer("yes"));
    const ControlProfile& sp = h.flow.profile()[Control::Spoilers];
    CHECK(sp.state == ControlState::Ok);
    CHECK(sp.method == h.aircraft.spoilersWork);
    CHECK_EQ(describeMethod(Control::Spoilers, sp.method), std::string("key event AXIS_SPOILER_SET"));
    CHECK_EQ(describeMethod(Control::Flaps, ControlMethod{MethodKind::KeyEvent, kKeyFlapsNotches, 1.0}),
             std::string("key events FLAPS_INCR/FLAPS_DECR"));
    CHECK_EQ(describeMethod(Control::Flaps, ControlMethod{MethodKind::KeyEvent, "", 1.0}), std::string("key event FLAPS_SET"));
}

TEST(Setup_ThrottleUsesOnePatternForBothEnginesAndWaitsForTheSpool) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.gearRetractable = false;
    REQUIRE(h.start());
    CHECK_EQ(h.status().getString("step", ""), std::string("throttle"));
    // The holds: the detail says how long, and while one is showing it says how far it is.
    double holdSeen = 0.0;
    bool holdTask = false, detail = false;
    ThrottlePilot p{0.0, 50.0, 100.0};
    auto pilot = [&](double s) {
        p(h, s);
        const JsonValue st = h.status();
        if (st.getString("phase", "") != "user") return;
        holdSeen = std::max(holdSeen, st.getNumber("hold", 0.0));
        holdTask |= st.getBool("holdTask", false);
        detail |= st.getString("detail", "").find("until it's ticked") != std::string::npos;
    };
    CHECK(h.run(90.0, pilot, "ask"));
    CHECK(holdTask && detail);
    CHECK(holdSeen > 0.9);
    // The lever is back at idle well before the pilot's part ends: that waits for N1 to come down.
    REQUIRE(p.backAt > 0.0);
    CHECK(h.firstReplayAt > p.backAt + 5.0);
    CHECK(inOrder(h.prompts, {"Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle",
                              "Wait for the engines to spool down", "Listen to the engines"}));
    REQUIRE(!h.aircraft.tried.empty());
    CHECK(h.aircraft.tried[0] == h.aircraft.throttleWorks);
    CHECK_NEAR(h.aircraft.thr[0], 0.0, 1.0);
    REQUIRE(h.answer("yes"));
    CHECK(h.flow.profile()[Control::Throttle].method.var == "ENGINE_THROTTLE_#");
}

// ---------------------------------------------------------------------------- throttle lever

namespace {

// A throttle-only setup on an aircraft whose throttle replays with the key events.
void throttleOnly(SetupHarness& h) {
    h.facts.hasSpoilers = false;
    h.facts.gearRetractable = false;
    h.aircraft.throttleWorks = ControlMethod{MethodKind::KeyEvent, "", 1.0};
}

// The pilot's throttle: halfway and full as the setup asks, on the Airbus the CL and TOGA
// detents (idle at `idle` on the axis, the rest of it above).
ThrottlePilot throttlePilot(SetupHarness& h, double idle = 0.0) {
    if (!h.aircraft.airbus) return ThrottlePilot{0.0, 50.0, 100.0};
    return ThrottlePilot{idle, idle + (1.0 - idle) * 0.5, idle + (1.0 - idle) * 0.95};
}

// Through the throttle's movement and its replays to the method's question, then "yes".
void confirmKeyEvents(SetupHarness& h, ThrottlePilot& p) {
    REQUIRE(h.run(240.0, followThrottle(h, p), "ask"));
    REQUIRE(!h.aircraft.tried.empty());
    CHECK(h.aircraft.tried.back().kind == MethodKind::KeyEvent);
    REQUIRE(h.answer("yes"));
}

}  // namespace

TEST(Setup_ThrottleLeverStepSkippedWhenTheSimvarHoldsStill) {
    SetupHarness h;
    throttleOnly(h);
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h);
    confirmKeyEvents(h, p);
    // The simvar is the lever here: no further questions.
    CHECK_EQ(h.phase(), std::string("done"));
    const ControlProfile& tp = h.flow.profile()[Control::Throttle];
    CHECK(tp.state == ControlState::Ok);
    CHECK(tp.method.kind == MethodKind::KeyEvent);
    CHECK(tp.lever.var.empty());
    for (const std::string& s : h.prompts) CHECK(s.find("Hands off") == std::string::npos);
}

// Without the holds the movement doesn't count: the instruction stays up until the throttle is held.
TEST(Setup_ThrottleMovementWaitsForTheHolds) {
    SetupHarness h;
    throttleOnly(h);
    REQUIRE(h.start());
    auto slam = [&](double s) { h.aircraft.thr[0] = h.aircraft.thr[1] = s > 2.0 && s < 3.0 ? 100.0 : 0.0; };
    h.run(20.0, slam);
    CHECK_EQ(h.phase(), std::string("user"));
    CHECK_EQ(h.status().getString("prompt", ""), std::string("Move the throttle to about halfway and hold it"));
    CHECK_EQ(h.status().getNumber("tasks", 0), 4.0);
    // Full first, then halfway, also does it.
    ThrottlePilot p{0.0, 100.0, 50.0};
    REQUIRE(h.run(240.0, followThrottle(h, p), "ask"));
    CHECK(inOrder(h.prompts, {"Now to full and hold it", "Back to idle", "Wait for the engines to spool down"}));
    REQUIRE(h.answer("yes"));
    CHECK_EQ(h.phase(), std::string("done"));
    CHECK(h.flow.profile()[Control::Throttle].lever.var.empty());
}

// A thrust demand that settles at once holds still too, but short of 100% at full: asked.
TEST(Setup_ThrottleLeverAskedWhenTheMovementDoesntTell) {
    for (const bool yes : {false, true}) {
        SetupHarness h;
        throttleOnly(h);
        h.aircraft.airbus = true;
        h.aircraft.demandLagSec = 0.0;
        REQUIRE(h.start());
        ThrottlePilot p = throttlePilot(h);
        confirmKeyEvents(h, p);
        CHECK_EQ(h.phase(), std::string("query"));
        CHECK_EQ(h.status().getString("prompt", ""), std::string("Do the levers stay put under autothrust?"));
        CHECK(!h.status().getString("detail", "").empty());
        REQUIRE(h.answer(yes ? "yes" : "no"));
        if (!yes) {
            CHECK_EQ(h.phase(), std::string("done"));
            CHECK(h.flow.profile()[Control::Throttle].state == ControlState::Ok);
            CHECK(h.flow.profile()[Control::Throttle].lever.var.empty());
            continue;
        }
        // "Yes": the lever's variable from the holds already made.
        CHECK_EQ(h.phase(), std::string("notice"));
        REQUIRE(h.answer("begin"));
        REQUIRE(h.run(120.0, nullptr, "done"));
        CHECK_EQ(h.flow.profile()[Control::Throttle].lever.var, std::string("INI_ENG#_THROTTLE_POS"));
        CHECK(h.flow.profile()[Control::Throttle].lever.valid());
    }
}

TEST(Setup_ThrottleLeverFoundAndCalibratedWhenTheSimvarCreeps) {
    SetupHarness h;
    throttleOnly(h);
    h.aircraft.airbus = true;
    h.aircraft.atRequest = true;  // holds exactly as still as the lever: the lever's name wins
    h.facts.hasReverse = true;
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h);
    confirmKeyEvents(h, p);
    // The thrust demand kept moving while the lever sat in CL and TOGA, and the lever's own
    // variable held: straight on to its calibration, with no second pass of the throttle.
    CHECK_EQ(h.phase(), std::string("notice"));
    CHECK(inOrder(h.prompts, {"Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle",
                              "Wait for the engines to spool down", "Listen to the engines",
                              "Did the engines spool up and back down?"}));
    const JsonValue notice = h.status();
    CHECK_EQ(notice.getString("prompt", ""), std::string("Hands off the throttle"));
    CHECK(notice.getString("detail", "").find("20 seconds") != std::string::npos);
    const JsonValue* buttons = notice.get("buttons");
    REQUIRE(buttons && buttons->isArray() && !buttons->array.empty());
    CHECK_EQ(buttons->array[0].string, std::string("begin"));

    REQUIRE(h.answer("begin"));
    CHECK_EQ(h.phase(), std::string("sweep"));
    REQUIRE(h.run(120.0, nullptr, "done"));
    CHECK(!h.aircraft.driving);
    CHECK(!h.aircraft.reverser);  // stowed again
    CHECK_EQ(h.aircraft.reverserCalls, 2);
    CHECK_NEAR(h.aircraft.axis[0], 0.025, 0.02);  // left at idle

    const ControlProfile& tp = h.flow.profile()[Control::Throttle];
    CHECK(tp.state == ControlState::Ok);
    CHECK(tp.method.kind == MethodKind::KeyEvent);
    // The lever itself, as one pattern for both engines: not the creeping thrust output or the flag.
    CHECK_EQ(tp.lever.var, std::string("INI_ENG#_THROTTLE_POS"));
    REQUIRE(tp.lever.valid());
    CHECK(tp.lever.forward.size() >= 30);
    CHECK(tp.lever.reverse.size() >= 8);
    CHECK(tp.reverseBy == ReverseBy::Events);  // the reverse events deployed the reversers
    CHECK_EQ(h.aircraft.decreases, 0);
    const double u = FakeAircraft::kLeverUnits;
    double a = NAN;
    // Detents come back as the middle of their stretch of the axis, the rest interpolated.
    CHECK(leverAxis(tp.lever, 25.0 * u, 0.0, false, a));
    CHECK_NEAR(a, 0.5, 0.03);
    CHECK(leverAxis(tp.lever, 35.0 * u, 0.0, false, a));
    CHECK_NEAR(a, 0.75, 0.03);
    CHECK(leverAxis(tp.lever, 45.0 * u, 0.0, false, a));
    CHECK_NEAR(a, 0.95, 0.03);
    CHECK(leverAxis(tp.lever, 30.0 * u, 0.0, false, a));
    CHECK_NEAR(a, 0.625, 0.02);
    CHECK(leverAxis(tp.lever, 0.0, 0.0, false, a));
    CHECK(a < 0.05);
    // Reverse: the amount, once the aircraft's reverser is engaged too.
    CHECK(leverAxis(tp.lever, -20.0 * u, 1.0, true, a));
    CHECK_NEAR(a, 1.0, 0.03);
    CHECK(leverAxis(tp.lever, -10.0 * u, 1.0, true, a));
    CHECK_NEAR(a, 0.5, 0.03);
    CHECK(leverAxis(tp.lever, -10.0 * u, NAN, true, a));  // by value when the clip has no rev_N
    CHECK_NEAR(a, 0.5, 0.03);
    CHECK(leverAxis(tp.lever, -10.0 * u, 1.0, false, a));  // the reverser not engaged yet: idle
    CHECK(a < 0.05);
}

// A thrust demand that keeps creeping (1.5% a second) while the lever sits still: the holds still
// tick, and the creep is what shows it isn't the lever.
TEST(Setup_ThrottleHoldsTickWhileTheThrustDemandCreeps) {
    SetupHarness h;
    throttleOnly(h);
    h.aircraft.airbus = true;
    h.aircraft.demandLagSec = 0.3;
    h.aircraft.demandCreepPct = 1.5;
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h);
    confirmKeyEvents(h, p);
    CHECK_EQ(h.phase(), std::string("notice"));
    CHECK(inOrder(h.prompts, {"Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle"}));
    REQUIRE(h.answer("begin"));
    REQUIRE(h.run(120.0, nullptr, "done"));
    CHECK_EQ(h.flow.profile()[Control::Throttle].lever.var, std::string("INI_ENG#_THROTTLE_POS"));
}

TEST(Setup_ThrottleLeverAxisWithItsOwnReverseZone) {
    SetupHarness h;
    throttleOnly(h);
    h.aircraft.airbus = true;
    h.aircraft.reverseZone = true;
    // Replayed from the simvar the bottom of the axis would be full reverse, so this aircraft's
    // key events only pass the check with the pilot's own axis recorded.
    h.aircraft.axisRecorded = true;
    h.facts.hasReverse = true;
    // Idle is at axis 0.2 here.
    h.aircraft.axis[0] = h.aircraft.axis[1] = 0.2;
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h, 0.2);
    confirmKeyEvents(h, p);
    REQUIRE(h.phase() == "notice");
    REQUIRE(h.answer("begin"));
    REQUIRE(h.run(120.0, nullptr, "done"));
    const ControlProfile& tp = h.flow.profile()[Control::Throttle];
    REQUIRE(tp.lever.valid());
    CHECK(tp.lever.reverse.empty());  // the axis covers reverse itself
    CHECK(tp.reverseBy == ReverseBy::Axis);
    CHECK_EQ(h.aircraft.reverserCalls, 0);
    CHECK_NEAR(h.aircraft.axis[0], 0.22, 0.03);  // idle, not the bottom of the axis
    double a = NAN;
    CHECK(leverAxis(tp.lever, -20.0 * FakeAircraft::kLeverUnits, 1.0, true, a));
    CHECK_NEAR(a, 0.0, 0.03);
    CHECK(leverAxis(tp.lever, -10.0 * FakeAircraft::kLeverUnits, 1.0, true, a));
    CHECK_NEAR(a, 0.1, 0.03);
    CHECK(leverAxis(tp.lever, 25.0 * FakeAircraft::kLeverUnits, 0.0, false, a));
    CHECK_NEAR(a, 0.6, 0.03);
}

// The movement showed the thrust demand creeping, but not the lever's own variable: the holds once
// more, for it alone.
TEST(Setup_ThrottleLeverHeldAgainWhenTheMovementDidntShowIt) {
    SetupHarness h;
    throttleOnly(h);
    h.aircraft.airbus = true;
    h.aircraft.hideLeverVars = true;
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h);
    confirmKeyEvents(h, p);
    // Nothing lever-like but the input events, which follow the demand: the movement can't tell.
    REQUIRE(h.phase() == "query");
    REQUIRE(h.answer("yes"));
    CHECK_EQ(h.phase(), std::string("positions"));
    const JsonValue st = h.status();
    CHECK_EQ(st.getString("prompt", ""), std::string("Move the throttle to about halfway and hold it"));
    CHECK_EQ(st.getNumber("tasks", 0), 3.0);
    CHECK(st.getBool("holdTask", false));
    CHECK(st.getString("note", "").find("Once more") != std::string::npos);
    CHECK(st.getString("detail", "").find("until it's ticked") != std::string::npos);
    const JsonValue* buttons = st.get("buttons");
    REQUIRE(buttons && buttons->isArray() && !buttons->array.empty());
    CHECK_EQ(buttons->array[0].string, std::string("skip"));

    h.aircraft.hideLeverVars = false;
    h.prompts.clear();
    ThrottlePilot again = throttlePilot(h);
    REQUIRE(h.run(90.0, followThrottle(h, again), "notice"));
    CHECK(inOrder(h.prompts, {"Move the throttle to about halfway and hold it", "Now to full and hold it", "Back to idle",
                              "Hands off the throttle"}));
    CHECK(h.sawTaskDone);
    REQUIRE(h.answer("begin"));
    REQUIRE(h.run(120.0, nullptr, "done"));
    CHECK_EQ(h.flow.profile()[Control::Throttle].lever.var, std::string("INI_ENG#_THROTTLE_POS"));
    CHECK(h.flow.profile()[Control::Throttle].lever.valid());
}

// The PMDG 737: its lever variable runs down from idle to full, and the reverse events don't
// reverse it (its levers go forward with the axis), so the sweep lets them off at half the axis
// and tries "decrease throttle" held at idle; that deploys the reversers, and let go they stow.
// An aircraft that takes neither gets no reverse.
TEST(Setup_ThrottleReverseByHoldingDecreaseOnALeverThatRunsDown) {
    for (const bool holdWorks : {true, false}) {
        SetupHarness h;
        throttleOnly(h);
        h.aircraft.airbus = true;
        h.aircraft.leverDown = true;
        h.aircraft.reversers = false;
        h.aircraft.reverseByHold = holdWorks;
        h.facts.hasReverse = true;
        REQUIRE(h.start());
        ThrottlePilot p = throttlePilot(h);
        confirmKeyEvents(h, p);
        REQUIRE(h.phase() == "notice");
        REQUIRE(h.answer("begin"));
        REQUIRE(h.run(150.0, nullptr, "done"));
        CHECK(!h.aircraft.driving);
        CHECK(!h.aircraft.holdReverse);
        CHECK(h.aircraft.axis[0] < 0.05);  // left at idle
        CHECK_EQ(h.aircraft.reverserCalls, 2);  // the reverse events: tried, and stowed
        CHECK(h.aircraft.decreases > 0);
        const LeverCalibration& cal = h.flow.profile()[Control::Throttle].lever;
        REQUIRE(cal.valid());
        CHECK_EQ(cal.var, std::string("INI_ENG#_THROTTLE_POS"));
        CHECK(h.flow.profile()[Control::Throttle].reverseBy == (holdWorks ? ReverseBy::Hold : ReverseBy::None));
        CHECK(cal.reverse.empty());  // what the reverse events did was the forward levers
        CHECK_NEAR(cal.forward.front().lever, 500.0, 1.0);
        CHECK(cal.forward.back().lever < 100.0);
        double a = NAN;
        CHECK(leverAxis(cal, 500.0, NAN, false, a));
        CHECK(a < 0.06);
        CHECK(leverAxis(cal, 500.0 - 25.0 * FakeAircraft::kLeverDownUnits, 0.0, false, a));  // CL
        CHECK_NEAR(a, 0.5, 0.03);
        CHECK(leverAxis(cal, 500.0 - 45.0 * FakeAircraft::kLeverDownUnits, 0.0, false, a));  // TOGA
        CHECK_NEAR(a, 0.95, 0.03);
        if (holdWorks) {
            CHECK(describeControl(Control::Throttle, h.flow.profile()[Control::Throttle]).find("holding THROTTLE_DECR") !=
                  std::string::npos);
        }
    }
}

// The PMDG 737 again, this time with its lever simvar as the lever (or the pilot saying its levers
// move with the thrust): no lever calibration, but reverse thrust is still tried. And an aircraft
// that takes the reverse events.
TEST(Setup_ThrottleReverseTestedWithoutALeverCalibration) {
    for (const bool hold : {true, false}) {
        SetupHarness h;
        throttleOnly(h);
        h.aircraft.reverseByHold = hold;
        h.aircraft.reversers = !hold;
        h.facts.hasReverse = true;
        REQUIRE(h.start());
        ThrottlePilot p = throttlePilot(h);
        confirmKeyEvents(h, p);
        REQUIRE(h.phase() == "notice");
        CHECK(h.status().getString("detail", "").find("tries reverse thrust") != std::string::npos);
        REQUIRE(h.answer("begin"));
        CHECK(h.status().getString("detail", "").find("tries reverse thrust") != std::string::npos);
        REQUIRE(h.run(60.0, nullptr, "done"));
        const ControlProfile& tp = h.flow.profile()[Control::Throttle];
        CHECK(tp.state == ControlState::Ok);
        CHECK(tp.lever.var.empty());
        CHECK(tp.reverseBy == (hold ? ReverseBy::Hold : ReverseBy::Events));
        CHECK_EQ(h.aircraft.reverserCalls, 2);
        CHECK(hold ? h.aircraft.decreases > 0 : h.aircraft.decreases == 0);
        CHECK(!h.aircraft.reverser && !h.aircraft.holdReverse);
        CHECK_NEAR(h.aircraft.thr[0], 0.0, 1e-6);  // left at idle
        CHECK(describeControl(Control::Throttle, tp).find(hold ? "holding THROTTLE_DECR" : "REVERSE_THRUST_ON") !=
              std::string::npos);
    }
}

TEST(Setup_ThrottleLeverSweepStopsWhenTheAircraftRolls) {
    SetupHarness h;
    throttleOnly(h);
    h.aircraft.airbus = true;
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h);
    confirmKeyEvents(h, p);
    REQUIRE(h.phase() == "notice");
    REQUIRE(h.answer("begin"));
    h.run(3.0);
    h.aircraft.gs = 5.0;  // the parking brake isn't holding
    REQUIRE(h.run(1.0, nullptr, "retry"));
    CHECK(h.status().getString("note", "").find("roll") != std::string::npos);
    CHECK(!h.aircraft.driving);
    CHECK(h.aircraft.axis[0] < 0.05);  // put back to idle
    h.aircraft.gs = 0.0;
    // Skipping keeps the confirmed method, without a lever.
    REQUIRE(h.answer("skip"));
    CHECK_EQ(h.phase(), std::string("done"));
    CHECK(h.flow.profile()[Control::Throttle].state == ControlState::Ok);
    CHECK(h.flow.profile()[Control::Throttle].method.kind == MethodKind::KeyEvent);
    CHECK(!h.flow.profile()[Control::Throttle].lever.valid());
}

TEST(Setup_SkippingAStepOnALaterRunKeepsItsEarlierSetup) {
    SetupHarness h;
    throttleOnly(h);
    AircraftProfile before;
    before[Control::Throttle].state = ControlState::Ok;
    before[Control::Throttle].method = ControlMethod{MethodKind::KeyEvent, "", 1.0};
    before[Control::Throttle].lever.var = "MY_LEVER";
    before[Control::Throttle].lever.forward = {{0.0, 0.0}, {1.0, 45.0}};
    std::string err;
    REQUIRE(h.flow.start(before, h.aircraft.live(), h.facts, err));
    REQUIRE(h.answer("skip"));
    CHECK_EQ(h.phase(), std::string("done"));
    const ControlProfile& tp = h.flow.profile()[Control::Throttle];
    CHECK(tp.state == ControlState::Ok);
    CHECK_EQ(tp.lever.var, std::string("MY_LEVER"));
}

TEST(Setup_TrackerHoldCheckSeesASimvarCreepWhileTheLeverHolds) {
    // The lever goes to 45 over 1 s and stays; the simvar follows with a lag (still creeping
    // seconds later), or exactly. A switch nearby settles before the throttle moves and stays put.
    for (const bool lag : {true, false}) {
        CandidateTracker t;
        t.begin({{"LEVER", true}, {"FLAG", true}, {"SWITCH", true}});
        std::vector<double> v(3);
        double simvar = 0.0;
        for (int s = 0; s <= 120; ++s) {
            const double time = s * 0.1;
            const double lever = time < 2.0 ? 0.0 : (time < 3.0 ? 45.0 * (time - 2.0) : (time < 8.0 ? 45.0 : 0.0));
            simvar = lag ? simvar + (lever / 45.0 * 85.0 - simvar) * 0.08 : lever / 45.0 * 85.0;
            v[0] = lever;
            v[1] = lever > 40.0 ? 1.0 : 0.0;
            v[2] = std::round(std::min(time, 1.5) * 10.0);
            t.sample(time, v, simvar, simvar);
        }
        const CandidateTracker::HoldCheck hc = t.holdCheck(0.0, 12.0, {0, 1, 2}, 0.0, 20.0, 0.5, 2.0, 0.75, 42.5);
        if (lag) {
            CHECK(hc.creep > 0.5);
            CHECK_EQ(hc.by, size_t(0));  // the flag takes two values: not a lever
        } else {
            CHECK(hc.creep < 0.5);
            CHECK(hc.steady);
        }
        // Without the limit the switch, still through the whole movement, looks like a creep.
        CHECK(t.holdCheck(0.0, 12.0, {2}, 0.0, 20.0, 0.5, 2.0, 0.75).creep > 80.0);
    }
}

// A simvar that jumps (some aircraft move it in steps) isn't held until it's been still for the
// hold's time after the jump, whenever it lands between the setup's samples.
TEST(Setup_ThrottleHoldStartsAfterAJump) {
    SetupHarness h;
    throttleOnly(h);
    REQUIRE(h.start());
    double jumpAt = -1.0, tickAt = -1.0;
    auto pilot = [&](double s) {
        if (s >= 3.05 && jumpAt < 0.0) {
            h.aircraft.thr[0] = h.aircraft.thr[1] = 50.0;
            jumpAt = h.t;
        }
        if (tickAt < 0.0 && h.status().getNumber("task", 0) >= 1.0) tickAt = h.t;
    };
    h.run(8.0, pilot);
    REQUIRE(jumpAt > 0.0 && tickAt > 0.0);
    CHECK(tickAt - jumpAt >= 2.0);
    CHECK(tickAt - jumpAt < 3.5);
}

TEST(Profile_LeverCalibrationTables) {
    // A forward sweep with a flat stretch (a detent) and readings out of order.
    LeverCalibration cal;
    cal.var = "LEVER_#";
    cal.forward = leverTable({{0.5, 25.0}, {0.0, 0.0}, {0.25, 12.5}, {0.4, 25.0}, {0.6, 25.0}, {1.0, 45.0}, {0.8, NAN}});
    REQUIRE(cal.forward.size() == 6);
    CHECK_NEAR(cal.forward[1].axis, 0.25, 1e-12);
    CHECK_NEAR(invertLever(cal.forward, 25.0), 0.5, 1e-9);     // the detent's middle (0.4..0.6)
    CHECK_NEAR(invertLever(cal.forward, 6.25), 0.125, 1e-9);   // interpolated
    CHECK_NEAR(invertLever(cal.forward, 35.0), 0.8, 1e-9);     // between the detent and full
    CHECK_NEAR(invertLever(cal.forward, 60.0), 1.0, 1e-9);     // clamped
    CHECK_NEAR(invertLever(cal.forward, -5.0), 0.0, 1e-9);
    double a = NAN;
    CHECK(leverAxis(cal, 25.0, NAN, false, a));
    CHECK_NEAR(a, 0.5, 1e-9);
    CHECK(!leverAxis(cal, NAN, NAN, false, a));
    // Without a reverse table a reverse value can't be placed (playback falls back to the simvar).
    CHECK(!leverAxis(cal, -10.0, 1.0, true, a));
    // A reverse table: the amount 0..1 takes the lever from 0 to -20.
    cal.reverse = leverTable({{0.0, 0.0}, {0.5, -10.0}, {1.0, -20.0}});
    CHECK(leverAxis(cal, -15.0, 1.0, true, a));
    CHECK_NEAR(a, 0.75, 1e-9);
    CHECK(leverAxis(cal, -15.0, NAN, true, a));  // below idle: reverse, by value
    CHECK_NEAR(a, 0.75, 1e-9);
    CHECK(leverAxis(cal, -15.0, 1.0, false, a));  // not engaged yet: forward idle
    CHECK_NEAR(a, 0.0, 1e-9);
    CHECK(leverAxis(cal, 25.0, 0.0, true, a));  // still engaged: reverse idle
    CHECK_NEAR(a, 0.0, 1e-9);
    CHECK(!LeverCalibration().valid());

    // A lever variable that runs down from idle (500) to full (75): idle is still its lowest-axis
    // reading, and reverse is beyond it the other way (above 500).
    LeverCalibration down;
    down.var = "switch_683_73X";
    down.forward = leverTable({{0.0, 500.0}, {0.5, 264.0}, {0.875, 75.0}, {1.0, 75.0}});
    down.reverse = leverTable({{0.0, 500.0}, {1.0, 700.0}});
    CHECK(leverAxis(down, 300.0, 1.0, false, a));  // the clip reversing, the aircraft not yet: idle
    CHECK_NEAR(a, 0.0, 1e-9);
    CHECK(leverAxis(down, 600.0, NAN, true, a));  // beyond idle: reverse, by value
    CHECK_NEAR(a, 0.5, 1e-9);
    CHECK(leverAxis(down, 600.0, 0.0, true, a));  // the aircraft still reversed: reverse idle
    CHECK_NEAR(a, 0.0, 1e-9);
    CHECK(leverAxis(down, 264.0, 0.0, false, a));
    CHECK_NEAR(a, 0.5, 1e-9);
    down.reverse.clear();
    CHECK(!leverAxis(down, 600.0, 1.0, false, a));  // reverse without a table: can't be placed
    CHECK(leverAxis(down, 75.0, 1.0, false, a));     // full forward with the flag set: forward
    CHECK_NEAR(a, 0.9375, 1e-9);

    // Reversing, by what the reversers did rather than the flag.
    Frame f;
    for (double& v : f.v) v = 0.0;
    CHECK(!reversing(f, 0));
    f.v[kReverser1] = 1.0;  // the flag alone (the PMDG with "toggle reverse" and the throttle forward)
    CHECK(!reversing(f, 0));
    f.v[kThrottle1] = -3.2;  // the lever simvar below idle
    CHECK(reversing(f, 0));
    f.v[kThrottle1] = 0.0;
    f.v[kReverserNozzle1 + 1] = 77.0;  // engine 2's nozzles out
    CHECK(reversing(f, 1));
    CHECK(!reversing(f, 0));
    f.v[kReverserNozzle1] = NAN;  // not recorded
    CHECK(!reversing(f, 0));
}

TEST(Setup_ThrottleReplayStopsWhenTheAircraftRolls) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.gearRetractable = false;
    h.aircraft.rollWhenDriven = true;
    REQUIRE(h.start());
    ThrottlePilot p{0.0, 50.0, 100.0};
    CHECK(h.run(90.0, followThrottle(h, p), "ask"));
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
    CHECK(inOrder(h.prompts, {"Take off", "Raise the gear", "Lower the gear", "Watch the gear"}));
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

TEST(Profile_VariantsShareOneKey) {
    const std::string pmdg = "simobjects/airplanes/pmdg 737";
    CHECK_EQ(aircraftFamily("simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config"), pmdg);
    CHECK_EQ(aircraftFamily("simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 ssw/config"), pmdg);
    CHECK_EQ(aircraftFamily("simobjects/airplanes/pmdg 737-600/presets/pmdg/pmdg 737-600/config"), pmdg);
    CHECK_EQ(aircraftFamily("simobjects/airplanes/pmdg 737-700/presets/pmdg/pmdg 737-700 bbj/config"), pmdg);
    CHECK_EQ(aircraftFamily("simobjects/airplanes/pmdg 737-900er/presets/pmdg/pmdg 737-900er/config"), pmdg);
    CHECK_EQ(aircraftFamily(pmdg), pmdg);
    const std::string a350 = "simobjects/airplanes/a350";
    CHECK_EQ(aircraftFamily("simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin/config"), a350);
    CHECK_EQ(aircraftFamily("simobjects/airplanes/a350/presets/inibuilds/a350-900ulr/config"), a350);
    CHECK_EQ(aircraftFamily("simobjects/airplanes/a350-1000/presets/inibuilds/a350-1000/config"), a350);
    // Different aircraft stay apart; a dash that isn't a variant number stays.
    CHECK_EQ(aircraftFamily("simobjects/airplanes/microsoft-a320neo/presets/inibuilds/a20n/config"),
             std::string("simobjects/airplanes/microsoft-a320neo"));
    CHECK_EQ(aircraftFamily("simobjects/airplanes/asobo_pc-12/presets/asobo/pc12/config"),
             std::string("simobjects/airplanes/asobo_pc-12"));
    CHECK_EQ(aircraftFamily("simobjects/airplanes/an-225/presets/x/config"), std::string("simobjects/airplanes/an-225"));
    CHECK_EQ(aircraftFamily("title:Some Jet"), std::string("title:Some Jet"));
    CHECK_EQ(aircraftFamily("c:/sim/simobjects/airplanes/pmdg 737-800/config"), "c:/sim/" + pmdg);

    // Profiles kept per variant before: one per family, the most recently updated.
    ProfileStore store;
    store.upsert("simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config").updatedUtcMs = 2;
    store.upsert("simobjects/airplanes/pmdg 737-700/presets/pmdg/pmdg 737-700/config").updatedUtcMs = 5;
    store.upsert("simobjects/airplanes/asobo_cj4/presets/asobo/cj4/config").updatedUtcMs = 1;
    store.upsert("simobjects/airplanes/a350").updatedUtcMs = 3;
    CHECK_EQ(store.groupVariants(), 4u);  // three re-keyed, one -700/-800 pair merged
    REQUIRE(store.size() == 3);
    const AircraftProfile* p = store.find(pmdg);
    REQUIRE(p != nullptr);
    CHECK_EQ(p->updatedUtcMs, static_cast<int64_t>(5));
    CHECK(store.find("simobjects/airplanes/asobo_cj4") != nullptr);
    CHECK(store.find(a350) != nullptr);
    CHECK_EQ(store.groupVariants(), 0u);
}

TEST(Profile_JsonRoundTrip) {
    ProfileStore store;
    store.upsert("c172").declined = true;
    {
        AircraftProfile& v2 = store.upsert("simobjects/airplanes/a320neo-v2");
        v2[Control::Throttle] = {ControlState::Ok, ControlMethod{MethodKind::KeyEvent, "", 1.0}, true};
        v2[Control::Throttle].lever.var = "INI_ENG#_THROTTLE_POS";
        v2[Control::Throttle].lever.forward = {{0.0, 0.0}, {0.5, 28.83}, {0.525, 28.83}, {1.0, 51.9}};
        v2[Control::Throttle].lever.reverse = {{0.0, 0.0}, {1.0, -23.07}};
        v2[Control::Throttle].reverseBy = ReverseBy::Hold;
        v2[Control::Flaps] = {ControlState::Ok, ControlMethod{MethodKind::KeyEvent, kKeyFlapsNotches, 1.0}, true};
    }
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
    REQUIRE(back.size() == 3);
    const AircraftProfile* b2 = back.find("simobjects/airplanes/a320neo-v2");
    REQUIRE(b2);
    const LeverCalibration& lv = (*b2)[Control::Throttle].lever;
    CHECK_EQ(lv.var, std::string("INI_ENG#_THROTTLE_POS"));
    REQUIRE(lv.forward.size() == 4 && lv.reverse.size() == 2);
    CHECK_NEAR(lv.forward[2].axis, 0.525, 1e-9);
    CHECK_NEAR(lv.forward[3].lever, 51.9, 1e-9);
    CHECK((*b2)[Control::Flaps].method == (ControlMethod{MethodKind::KeyEvent, kKeyFlapsNotches, 1.0}));
    CHECK_NEAR(lv.reverse[1].lever, -23.07, 1e-9);
    CHECK((*b2)[Control::Throttle].reverseBy == ReverseBy::Hold);
    CHECK(json.find("\"reverseBy\":\"hold\"") != std::string::npos);
    {
        // Where it was first kept: in the lever object.
        const char* old = "{\"aircraft\":[{\"key\":\"y\",\"controls\":{\"throttle\":{\"state\":\"ok\",\"method\":\"key_event\","
                          "\"lever\":{\"var\":\"L_#\",\"forward\":[[0,0],[1,1]],\"reverse\":[],\"reverseBy\":\"events\"}}}}]}";
        ProfileStore s;
        REQUIRE(s.fromJson(old, std::strlen(old), &err));
        REQUIRE(s.find("y"));
        CHECK((*s.find("y"))[Control::Throttle].reverseBy == ReverseBy::Events);
    }
    CHECK(describeControl(Control::Throttle, (*b2)[Control::Throttle]).find("INI_ENG#_THROTTLE_POS") != std::string::npos);
    CHECK(json.find("\"lever\"") == json.rfind("\"lever\""));  // only the profile that has one
    const AircraftProfile* b = back.find("simobjects/airplanes/a320neo");
    REQUIRE(b);
    CHECK(!(*b)[Control::Throttle].lever.valid());
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

    // The throttle lever's own variable: per engine, from the same L:var.
    LeverCalibration lever;
    lever.var = "INI_ENG#_THROTTLE_POS";
    lever.forward = {{0.0, 0.0}, {1.0, 51.9}};
    CHECK(!clipHasLever(lever, clip, 2, &why));  // recorded before the lever setup
    clip.present.set(kLVarThrottle1);
    clip.present.set(kLVarThrottle2);
    clip.meta.vars.push_back({"lv_thr_1", "INI_ENG1_THROTTLE_POS"});
    CHECK(!clipHasLever(lever, clip, 2, &why));
    CHECK(why.find("lv_thr_2") != std::string::npos);
    clip.meta.vars.push_back({"lv_thr_2", "ini_eng2_throttle_pos"});
    CHECK(clipHasLever(lever, clip, 2, &why));
    lever.var = "INI_THROTTLE_ALL";  // one variable for all engines
    CHECK(!clipHasLever(lever, clip, 2, &why));
    CHECK(!clipHasLever(LeverCalibration(), clip, 2, &why));
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

    // Without a profile it's always offered; until the aircraft is ready it waits, saying what's
    // missing.
    auto ready = [&](const char* key) {
        const JsonValue s = h.setupStatus();
        const JsonValue* r = s.get("ready");
        return r && r->getBool(key, false);
    };
    CHECK(ready("parkingBrake") && ready("onRunway") && ready("engines"));
    h.facts.onRunway = false;  // at the gate
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("wait"));
    CHECK(!ready("onRunway") && ready("parkingBrake"));
    h.facts.onRunway = true;
    h.facts.engineRunning = false;
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("wait"));
    CHECK(!ready("engines") && ready("onRunway"));
    h.facts.engineRunning = true;
    h.facts.parkingBrake = false;
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("wait"));
    CHECK(!ready("parkingBrake") && ready("engines"));
    h.facts.parkingBrake = true;
    h.aircraft.onGround = false;  // in the air: not on a runway
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("wait"));
    CHECK(!ready("onRunway"));
    h.aircraft.onGround = true;
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("new"));

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
