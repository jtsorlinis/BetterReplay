// Control setup and profiles, against a simulated aircraft whose controls only respond to one
// method each (like the A320neo V2, which ignores the simvars and legacy events).
#include <algorithm>
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
#include "PlaybackController.h"
#include "TestHarness.h"

using namespace br;

namespace {

constexpr double kDt = 1.0 / 60.0;

bool g_verbose = std::getenv("BR_VERBOSE") != nullptr;
std::vector<std::string> g_setupLog;  // every line the setup logged (cleared by each SetupHarness)
void testLog(void*, int level, const char* msg) {
    g_setupLog.push_back(msg);
    if (g_verbose || level >= 2) std::printf("         [setup %d] %s\n", level, msg);
}
bool logged(const std::string& part) {
    for (const std::string& line : g_setupLog) {
        if (line.find(part) != std::string::npos) return true;
    }
    std::printf("         not in the setup's log: %s\n", part.c_str());
    return false;
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
    // The A350 instead: neither the reverse events nor "decrease throttle" reverse it, but
    // "toggle reverse" with the levers at idle does, without the reverse flag; the axis then sets
    // the amount, and its nozzles and simvar show only the amount (nothing at reverse idle).
    bool reverseByToggle = false;
    bool toggled = false;
    int toggles = 0;
    int ignoreToggle = 0;           // that toggle (1-based) does nothing
    bool toggleStuck = false;       // once in reverse, toggles do nothing
    bool axisRecorded = false;      // the key-event handler sees the pilot's axis (thr_axis_N)
    bool hideThrottleVars = false;  // nothing but the simvar moves with the throttle
    // The PMDG 777's gear instead: no variable the scan sees is its lever, but each leg has an
    // L:var animating it (0..1000), which moves with the gear and moves nothing else when written.
    bool pmdgGear = false;
    // And as the real one took the replays: its lever event's position (#69927) raises the gear (0)
    // but ignores DOWN (1); GEAR_SET/GEAR_DOWN lower it, but only after a replay raised it (GEAR_UP
    // doesn't raise it); after the pilot raised it only a left click on the lever does (the event
    // with PMDG's mouse flags, "#69927(536870912,131072)").
    bool pmdg777 = false;
    bool raisedByPilot = false;  // who last raised the gear: the pilot (setGearLever) or a replay
    bool pmdgLeverVars = false;  // and its lever's switch_295_a (0..100), and a sound flag while the legs move
    double gearRate = 0.25;     // the gear simvars' travel a second
    double legAnimRate = 0.0;   // the leg animations' (pmdgGear), if not the same
    double legAnim = 1.0;
    // Key events the sim sees while the setup watches (fire): the pilot's, and with keyNoise an
    // elevator axis that keeps firing.
    bool keyNoise = false;
    bool watchingKeys = false;
    std::vector<SeenKeyEvent> keyQueue;
    void fire(uint32_t id, double value) {
        if (watchingKeys) keyQueue.push_back({id, value});
    }
    static constexpr uint32_t kElevatorAxis = 0x10000 + 226;  // AXIS_ELEVATOR_SET
    // A gear method that's a toggle or steps: clicked the way ControlDriver does it.
    EventStepper stepper;
    GearCommandFollower gearFollow;
    int clicks = 0;
    // The gear's key events and input event lower it only after the pilot raised it (not after a
    // replay did): a replay that raises it leaves it up.
    bool replayLowersOnlyAfterPilot = false;
    // The setup's hover (the gear step on the ground): put up there and frozen. The sim counts it as
    // airborne up there unless hoverOnGround; with gearStuckFrozen its gear doesn't move while
    // frozen; with handleFollowsLever its gear handle simvar shows the lever at once.
    bool hoverable = true;
    bool hoverOnGround = false;
    bool gearStuckFrozen = false;
    bool handleFollowsLever = false;
    bool frozen = false;
    double lifted = 0.0;  // how far up it was put
    int hovers = 0, putBacks = 0, letGo = 0;
    double gearWhenPutBack = NAN;

    // what the setup did
    std::vector<ControlMethod> tried;
    int scans = 0, scanReads = 0, scanEnds = 0, ends = 0;
    bool driving = false;
    Control drivenControl = Control::Spoilers;
    ControlMethod driven;
    bool refuseInputValue = false;

    void setGearLever(double v) {  // the pilot's hand
        if (v < 0.5 && gearLever >= 0.5) raisedByPilot = true;
        moveGearLever(v);
    }
    void replayGearLever(double v) {  // what a replay's method did
        if (v < 0.5 && gearLever >= 0.5) raisedByPilot = false;
        moveGearLever(v);
    }
    void moveGearLever(double v) {
        if (v != gearLever) gearLeverChangedAt = t;
        gearLever = v;
    }
    double ieThrottle(int e) const { return 16.7 + 0.833 * thr[e]; }

    // The lever (TLA, degrees) where the axis puts it: detents at idle (0), CL (25), FLX/MCT (35)
    // and TOGA (45); reverse down to -20.
    double tla(int e) const {
        if (holdReverse) return -20.0;
        double a = axis[e];
        if (reverser || toggled) return -20.0 * a;
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
        if (t - gearLeverChangedAt >= 2.0 && !(frozen && gearStuckFrozen)) {
            gearPos = approach(gearPos, gearLever, gearRate, dt);
            legAnim = approach(legAnim, gearLever, legAnimRate > 0.0 ? legAnimRate : gearRate, dt);
        }
        if (gearPos >= 0.999) gearHandleSimvar = 1.0;  // the handle simvar lags, as on the A320
        if (gearPos <= 0.001) gearHandleSimvar = 0.0;
        if (handleFollowsLever) gearHandleSimvar = gearLever;
        if (rollWhenDriven && driving && drivenControl == Control::Throttle && (thr[0] > 50.0 || thr[1] > 50.0)) gs = 5.0;
        if (keyNoise) fire(kElevatorAxis, std::round(300.0 * std::sin(7.0 * t)));
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
            f.v[kReverserNozzle1 + e] = reverser || holdReverse ? 100.0 : toggled ? std::fmin(100.0, std::fmax(0.0, -5.0 * thr[e])) : 0.0;
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
        if (pmdgGear) vars.insert(vars.end(), {{"7X7XleftMLGExt", true}, {"7X7XrightMLGExt", true}, {"7X7XnoseMLGExt", true}});
        if (pmdgLeverVars) vars.insert(vars.end(), {{"7X7XleftMLGExtSnd", true}, {"switch_295_a", true}});
        return vars;
    }
    std::vector<double> scanValues() const {
        std::vector<double> v = {spoilerHandle * 100.0, ieThrottle(0), ieThrottle(1), gearLever, 1.0, t * 1000.0, 42.0,
                                 spoilerSurface};
        if (hideThrottleVars) v[1] = v[2] = 16.7;
        if (pmdgGear) v[3] = 1.0;
        if (airbus && hideLeverVars) v.insert(v.end(), {0.0, 0.0, 0.0, 0.0, 0.0});
        else if (airbus) v.insert(v.end(), {leverVar(0), leverVar(1), tla(0) >= 44.9 ? 1.0 : 0.0, out[0], out[1]});
        if (airbus && atRequest) v.insert(v.end(), {leverVar(0), leverVar(1)});
        if (pmdgGear) v.insert(v.end(), {legAnim * 1000.0, legAnim * 1000.0, legAnim * 1000.0});
        if (pmdgLeverVars) v.insert(v.end(), {gearPos < 0.999 ? 1.0 : 0.0, gearLever * 100.0});
        return v;
    }
    void scanBegin(std::vector<ScanVar>& vars) override {
        ++scans;
        vars = scanVars();
        keyQueue.clear();
        watchingKeys = true;
    }
    void scanRead(std::vector<double>& v) override {
        ++scanReads;
        v = scanValues();
    }
    void scanEnd() override {
        ++scanEnds;
        watchingKeys = false;
        keyQueue.clear();
    }
    void scanKeyEvents(std::vector<SeenKeyEvent>& events) override {
        events.clear();
        events.swap(keyQueue);
    }
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
    void toggleReverse() override {
        ++toggles;
        if (!reverseByToggle || toggles == ignoreToggle || (toggleStuck && toggled)) return;
        if (airbus ? axis[0] < 0.05 : std::fabs(thr[0]) < 1.0) toggled = !toggled;  // only at idle
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
        stepper = EventStepper();
        gearFollow = GearCommandFollower();
        return true;
    }
    void applyControl(Control c, const Frame& f, bool force) override {
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
                    thr[e] = reverser || toggled ? -25.0 * a : 100.0 * a;  // in reverse, the reverse amount
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
        } else if (c == Control::Gear && pmdg777) {
            const double command = eventTarget(c, f);
            EventMethod ev;
            const bool own = parseEventMethod(driven, ev) && ev.id == 69927;
            if (own && ev.mode == EventMethod::Mode::Position) {
                if (command < 0.5) replayGearLever(0.0);  // UP; DOWN does nothing
            } else if (own && ev.mode == EventMethod::Mode::Toggle) {
                // Clicked like ControlDriver: toward the command, once the legs show the last click.
                const int dir = stepper.step(command, gearFollow.update(gearPos), ev.scale, EventStepper::waitFrames(c), force);
                if (dir != 0) ++clicks;
                const bool leftClick = ev.click && ev.param == 0x20000000 && ev.release == 0x00020000;
                if (dir > 0 && leftClick && raisedByPilot) replayGearLever(1.0);
            } else if (driven.kind == MethodKind::KeyEvent && driven.var.empty()) {
                if (command >= 0.5 && !raisedByPilot) replayGearLever(1.0);
            }
        } else if (c == Control::Gear && driven == gearWorks) {
            EventMethod ev;
            if (parseEventMethod(driven, ev) && ev.mode != EventMethod::Mode::Position) {
                // A click toward the clip's command, once the legs show the last one.
                const double live = gearFollow.update(gearPos);
                const int dir = stepper.step(eventTarget(c, f), live, ev.scale, EventStepper::waitFrames(c), force);
                if (dir != 0) {
                    ++clicks;
                    replayGearLever(ev.mode == EventMethod::Mode::Toggle ? 1.0 - gearLever : (dir > 0 ? 1.0 : 0.0));
                }
                return;
            }
            // Key events (an event's position too) from the gear command, like ControlDriver.
            const double v = driven.kind == MethodKind::KeyEvent ? f.v[kGearHandle] : f.v[kInputGear];
            if (!std::isfinite(v) || (v >= 0.5 && gearLever < 0.5 && replayLowersOnlyAfterPilot && !raisedByPilot)) return;
            replayGearLever(v >= 0.5 ? 1.0 : 0.0);
        }
    }
    void endControl(Control) override {
        driving = false;
        ++ends;
    }
    bool beginHover(double liftFt, std::string& why) override {
        if (!hoverable) {
            why = "it can't here";
            return false;
        }
        frozen = true;
        lifted = liftFt;
        ++hovers;
        onGround = hoverOnGround;
        gs = 0.0;
        return true;
    }
    void endHover(bool restore) override {
        if (!frozen) return;
        frozen = false;
        if (restore) {
            ++putBacks;
            onGround = true;
            gearWhenPutBack = gearPos;
        } else {
            ++letGo;
        }
    }
    bool hovering() const override { return frozen; }
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
    std::string key;  // the aircraft's profile key
    double t = 0.0;
    int asks = 0;
    double firstReplayAt = -1.0;  // harness time when the first replay started
    std::string lastPhase;
    std::vector<std::string> prompts;  // every main text shown, in order
    bool sawTaskDone = false;          // the last instruction was shown as done
    // When the setup asks to move the control away once more ("away"): the pilot raises the gear,
    // or deploys the spoilers, unless the test says otherwise.
    std::function<void()> away;
    // What the setup tried bringing the control back with on its own ("back"); aircraft.tried
    // keeps the replays of the whole movement.
    std::vector<ControlMethod> backTried;

    SetupHarness() {
        g_setupLog.clear();
        flow.setLogger(&testLog, nullptr);
        flow.setSim(&aircraft);
    }

    bool start() {
        std::string err;
        AircraftProfile profile;
        profile.key = key;
        const bool ok = flow.start(profile, aircraft.live(), facts, err);
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
            if (before == "away") moveAway();
            const size_t tries = aircraft.tried.size();
            aircraft.step(kDt);
            flow.onFrame(kDt, aircraft.live(), facts);
            t += kDt;
            const JsonValue st = status();
            const std::string p = flow.active() ? st.getString("phase", "") : "idle";
            if (p == "back" && aircraft.tried.size() > tries) {
                backTried.insert(backTried.end(), aircraft.tried.begin() + static_cast<std::ptrdiff_t>(tries), aircraft.tried.end());
                aircraft.tried.resize(tries);
            }
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

    void moveAway() {
        if (away) {
            away();
            return;
        }
        const std::string step = status().getString("step", "");
        if (step == "gear") aircraft.setGearLever(0.0);
        if (step == "spoilers") aircraft.spoilerHandle = 1.0;
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
    CHECK_EQ(h.aircraft.ends, static_cast<int>(h.aircraft.tried.size() + h.backTried.size()));
    // Before those, the way back from where the pilot left them: both preset scalings didn't
    // retract them, the key event did.
    REQUIRE(h.backTried.size() == 3);
    CHECK(h.backTried[2] == h.aircraft.spoilersWork);
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
    CHECK(notice.getString("detail", "").find("40 seconds") != std::string::npos);
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
        CHECK_EQ(h.aircraft.toggles, 2);        // toggle reverse: tried, and toggled back
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
        CHECK_EQ(h.aircraft.toggles, hold ? 2 : 0);
        CHECK(hold ? h.aircraft.decreases > 0 : h.aircraft.decreases == 0);
        CHECK(!h.aircraft.reverser && !h.aircraft.holdReverse);
        CHECK_NEAR(h.aircraft.thr[0], 0.0, 1e-6);  // left at idle
        CHECK(describeControl(Control::Throttle, tp).find(hold ? "holding THROTTLE_DECR" : "REVERSE_THRUST_ON") !=
              std::string::npos);
    }
}

namespace {

void captureLog(void* ctx, int level, const char* msg) {
    static_cast<std::vector<std::string>*>(ctx)->push_back(msg);
    testLog(nullptr, level, msg);
}

bool logged(const std::vector<std::string>& lines, const char* text) {
    for (const std::string& l : lines) {
        if (l.find(text) != std::string::npos) return true;
    }
    return false;
}

}  // namespace

// The A350: neither the reverse events nor "decrease throttle" reverse it, but "toggle reverse" at
// idle does, the axis setting the amount (its nozzles and simvar show only the amount). The sweep
// toggles it back out at idle and checks with a little forward thrust that it's out: when the
// first toggle back doesn't take, it toggles again. Either way the aircraft is left out of
// reverse. With a lever calibration and without.
TEST(Setup_ThrottleReverseByToggle) {
    for (const bool airbus : {true, false}) {
        for (const int ignore : {0, 2}) {
            SetupHarness h;
            std::vector<std::string> logs;
            h.flow.setLogger(&captureLog, &logs);
            throttleOnly(h);
            h.aircraft.airbus = airbus;
            h.aircraft.reversers = false;
            h.aircraft.reverseByToggle = true;
            h.aircraft.ignoreToggle = ignore;
            h.facts.hasReverse = true;
            REQUIRE(h.start());
            ThrottlePilot p = throttlePilot(h);
            confirmKeyEvents(h, p);
            REQUIRE(h.phase() == "notice");
            REQUIRE(h.answer("begin"));
            REQUIRE(h.run(150.0, nullptr, "done"));
            const ControlProfile& tp = h.flow.profile()[Control::Throttle];
            CHECK(tp.state == ControlState::Ok);
            CHECK(tp.reverseBy == ReverseBy::Toggle);
            CHECK(!h.aircraft.toggled);  // out of reverse
            CHECK_EQ(h.aircraft.toggles, ignore ? 3 : 2);
            CHECK_EQ(h.aircraft.reverserCalls, 2);  // the reverse events first: tried, and stowed
            CHECK_EQ(h.aircraft.decreases, 0);      // the toggle worked: no "decrease throttle"
            CHECK(!h.aircraft.driving);
            if (airbus) {
                CHECK(h.aircraft.axis[0] < 0.05);  // left at idle
                CHECK(tp.lever.valid());
            } else {
                CHECK_NEAR(h.aircraft.thr[0], 0.0, 1e-6);
                CHECK(tp.lever.var.empty());
            }
            CHECK(tp.lever.reverse.empty());
            CHECK(describeControl(Control::Throttle, tp).find("toggle reverse") != std::string::npos);
            CHECK(logged(logs, "control setup: reverse thrust: with toggle reverse"));
            CHECK(logged(logs, "control setup: out of reverse again"));
            CHECK(ignore ? logged(logs, "still in reverse after toggling back") : !logged(logs, "still in reverse"));
        }
    }
}

// An aircraft that stays in reverse whatever is toggled: the sweep stops, asking the pilot to
// toggle it out.
TEST(Setup_ThrottleToggleThatWontComeOutStopsTheSweep) {
    SetupHarness h;
    throttleOnly(h);
    h.aircraft.reversers = false;
    h.aircraft.reverseByToggle = true;
    h.facts.hasReverse = true;
    REQUIRE(h.start());
    ThrottlePilot p = throttlePilot(h);
    confirmKeyEvents(h, p);
    REQUIRE(h.phase() == "notice");
    h.aircraft.toggleStuck = true;
    REQUIRE(h.answer("begin"));
    REQUIRE(h.run(60.0, nullptr, "retry"));
    CHECK(h.status().getString("note", "").find("toggle reverse") != std::string::npos);
    CHECK(!h.aircraft.driving);
    CHECK(h.aircraft.toggles >= 4);  // in, back, twice more, and once more stopping
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

// ---------------------------------------------------------------------------- toggle reverse in replays

namespace {

// The A350 in a replay: "toggle reverse" switches it in or out of reverse with the levers at
// idle (or not at all, `ignore` times); in reverse the throttle simvar heads for -20% x the axis,
// forward for 100% x the axis, 1.6% at least (idle). Nothing shows at reverse idle.
struct ToggleAircraft {
    bool reverse = false;
    double axis = 0.0;
    double thr = 1.6;
    int toggles = 0;
    int ignore = 0;
    double lagSec = 0.4;  // the simvar behind the lever (the A350's: about 1.5 s)
    void toggle() {
        ++toggles;
        if (ignore > 0) {
            --ignore;
            return;
        }
        if (axis < 0.01) reverse = !reverse;
    }
    void step(double dt) {
        const double target = reverse ? -20.0 * axis : std::fmax(1.6, 100.0 * axis);
        thr += (target - thr) * std::fmin(1.0, dt / lagSec);
    }
    bool reversing() const { return thr < -1.0; }
};

// Plays a throttle simvar (one value a frame, 60 Hz) through ReverseToggle as ControlDriver does,
// and notes what the aircraft did.
struct TogglePlayback {
    ReverseToggle rt;
    ToggleAircraft ac;
    double deepest = 0.0;      // the aircraft's simvar, furthest into reverse
    double mostForward = 0.0;  // and furthest forward
    void frame(double clip) {
        const ReverseToggle::Step st = rt.step(clip < -1.0, ac.reversing(), ac.thr);
        double fraction = 0.0;
        if (st.drive == ReverseToggle::Drive::Reverse) fraction = clip < 0.0 ? std::fmin(clip / -20.0, st.cap) : 0.0;
        if (st.drive == ReverseToggle::Drive::Forward) fraction = clip > 0.0 ? clip / 100.0 : 0.0;
        ac.axis = fraction;
        if (st.toggle) ac.toggle();
        ac.step(1.0 / 60.0);
        deepest = std::fmin(deepest, ac.thr);
        mostForward = std::fmax(mostForward, ac.thr);
    }
    void play(double seconds, double clip) {
        for (int i = 0; i < static_cast<int>(seconds * 60.0); ++i) frame(clip);
    }
};

}  // namespace

// The clip's reverse amount is replayed, not full reverse: in at idle, the amount once the
// aircraft shows reverse, out at idle after. Half reverse and a shallow ("idle") one, on a simvar
// that follows the lever quickly, one as slow as the A350's, and one so slow that the wait for
// idle runs out (toggled out while the simvar still shows reverse, which isn't taken for it).
TEST(Playback_ToggleReverseReplaysTheAmount) {
    for (const double lag : {0.4, 1.5, 3.0}) {
        for (const double amount : {-10.0, -2.0}) {
            const double k = lag / 0.4;  // time to settle
            TogglePlayback p;
            p.ac.lagSec = lag;
            p.play(2.0, 1.6);
            CHECK(!p.rt.busy());
            p.play(4.0 * k, amount);
            CHECK(p.ac.reverse);
            CHECK(p.rt.on());
            CHECK_NEAR(p.ac.thr, amount, 0.5);
            CHECK(p.deepest > amount - 1.0);  // never full reverse
            CHECK(p.mostForward < 2.0);       // nor forward thrust
            p.play(3.0 * k, 1.6);
            CHECK(!p.ac.reverse);  // out again
            CHECK(!p.rt.busy());
            CHECK_EQ(p.ac.toggles, 2);
            CHECK_EQ(p.rt.corrections(), 0);
            CHECK_NEAR(p.ac.thr, 1.6, 0.2);
        }
    }
}

// A toggle the aircraft didn't take shows: going into reverse, as a little forward thrust from
// the small first amount; coming out, as reverse where the clip has forward thrust. Either way
// it's toggled again, and the aircraft ends up out of reverse.
TEST(Playback_ToggleReverseRetriesAToggleNotTaken) {
    {
        TogglePlayback p;
        p.ac.ignore = 1;
        p.play(1.0, 1.6);
        p.play(5.0, -10.0);
        CHECK(p.ac.reverse);
        CHECK_NEAR(p.ac.thr, -10.0, 0.5);
        CHECK(p.mostForward < 25.0);  // the probe's thrust, briefly
        CHECK(p.rt.corrections() >= 1);
        p.play(3.0, 1.6);
        CHECK(!p.ac.reverse);
        CHECK_EQ(p.ac.toggles, 3);
    }
    {
        TogglePlayback p;
        p.play(1.0, 1.6);
        p.play(4.0, -10.0);
        REQUIRE(p.ac.reverse);
        p.ac.ignore = 1;    // the toggle out
        p.play(6.0, 20.0);  // taxi thrust: in reverse that's reverse
        CHECK(!p.ac.reverse);
        CHECK(!p.rt.busy());
        CHECK_EQ(p.ac.toggles, 3);
        CHECK_EQ(p.rt.corrections(), 1);
        CHECK_NEAR(p.ac.thr, 20.0, 1.0);
    }
}

// A seek out of the middle of reverse (to forward thrust): idle and toggled out first. And an
// aircraft that won't take the toggle: three tries, then idle until the clip changes.
TEST(Playback_ToggleReverseSeekAndGiveUp) {
    {
        TogglePlayback p;
        p.play(1.0, 1.6);
        p.play(3.0, -15.0);
        REQUIRE(p.ac.reverse);
        p.mostForward = 0.0;
        p.play(0.2, 40.0);
        CHECK(p.mostForward < 2.0);  // no thrust either way until it's out
        p.play(5.0, 40.0);
        CHECK(!p.ac.reverse);
        CHECK_EQ(p.ac.toggles, 2);
        CHECK_NEAR(p.ac.thr, 40.0, 1.0);
    }
    {
        TogglePlayback p;
        p.ac.ignore = 100;
        p.play(1.0, 1.6);
        p.play(10.0, -10.0);
        CHECK(!p.ac.reverse);
        CHECK_EQ(p.ac.toggles, ReverseToggle::kTries);
        CHECK(!p.rt.on());
        CHECK(p.ac.thr < 5.0);  // idle meanwhile
    }
}

namespace {

// An A350-like lever: CL and FLX detents, the lever variable 0..100.
LeverCalibration airbusLever() {
    LeverCalibration cal;
    cal.var = "LEVER_#";
    cal.forward = leverTable({{0.0, 0.0}, {0.6, 58.4}, {0.65, 58.4}, {0.8, 78.0}, {0.85, 78.0}, {1.0, 100.0}});
    return cal;
}

double piecewise(const std::vector<std::pair<double, double>>& pts, double t) {
    if (t <= pts.front().first) return pts.front().second;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (t <= pts[i].first) {
            const double f = (t - pts[i - 1].first) / (pts[i].first - pts[i - 1].first);
            return pts[i - 1].second + f * (pts[i].second - pts[i - 1].second);
        }
    }
    return pts.back().second;
}

// A flight's lever and thrust demand at 30 Hz: a FLX takeoff (the demand creeping 3% with
// speed), the thrust reduction to CL, climb thrust, then the autothrust taking the demand down
// and up while the lever stays in CL, the retard to idle, and a manual hold part way up.
std::vector<Frame> autothrustFlight(double gapStart, double gapLen) {
    const std::vector<std::pair<double, double>> lever = {
        {0, 0}, {5, 78}, {55, 78}, {56, 58.4}, {200, 58.4}, {201, 0}, {210, 0}, {213, 30}, {240, 30}};
    const std::vector<std::pair<double, double>> demand = {
        {0, 1.6}, {5, 84}, {55, 87}, {56, 77}, {116, 77}, {163, 30}, {180, 45}, {200, 12}, {201, 1.6}, {210, 1.6},
        {215, 40}, {240, 40}};
    std::vector<Frame> frames;
    for (int i = 0; i <= 240 * 30; ++i) {
        Frame f;
        for (double& v : f.v) v = NAN;
        f.t = i / 30.0;
        const bool gap = f.t >= gapStart && f.t < gapStart + gapLen;
        f.v[kLVarThrottle1] = gap ? NAN : piecewise(lever, f.t);
        f.v[kThrottle1] = piecewise(demand, f.t);
        frames.push_back(f);
    }
    return frames;
}

// The A350's thrust demand as the aircraft shows it: heading for where the axis puts it with a
// lag, rate limited (slowly from ground idle), and seen a few frames late.
struct DemandAircraft {
    double scale = 1.0;
    double offset = 0.0;
    double tau = 0.8;
    size_t delay = 2;
    double d = 1.6;
    std::vector<double> history;
    double steady(double axis) const {
        static const std::vector<std::pair<double, double>> g = {{0, 1.6}, {0.258, 36}, {0.633, 77.5}, {0.8, 86.5}, {1, 95}};
        return offset + scale * piecewise(g, axis);
    }
    void step(double axis, double dt) {
        double rate = (steady(axis) - d) / tau;
        const double up = d < 4.0 ? 1.65 : 11.0;
        rate = rate > up ? up : (rate < -11.0 ? -11.0 : rate);
        d += rate * dt;
        history.push_back(d);
    }
    double live() const { return history.size() > delay ? history[history.size() - 1 - delay] : 1.6; }
};

}  // namespace

// Autothrust shows as the lever held while the demand keeps moving: the CL stretch counts, with
// the retard to idle after it; the FLX takeoff, whose demand only creeps, and a manual hold part
// way up don't. A short gap in the lever doesn't split a hold, a long one does.
TEST(Playback_AutothrustStretchesFromTheClip) {
    const LeverCalibration cal = airbusLever();
    std::vector<AutothrustStretch> a = autothrustStretches(cal, autothrustFlight(150.0, 0.5), 0);
    REQUIRE(a.size() == 1);
    CHECK_NEAR(a[0].startT, 56.0, 0.1);
    CHECK(a[0].endT > 200.5 && a[0].endT < 201.0);  // the retard, not idle after it
    CHECK(autothrustStretches(cal, autothrustFlight(150.0, 0.5), 1).empty());  // engine 2 wasn't recorded

    a = autothrustStretches(cal, autothrustFlight(150.0, 2.0), 0);
    REQUIRE(a.size() == 2);
    CHECK(a[0].endT < 150.0);
    CHECK(a[1].startT >= 152.0);

    CHECK(autothrustStretches(LeverCalibration(), autothrustFlight(0.0, 0.0), 0).empty());
}

// Following an approach's thrust demand on aircraft whose demand responds faster or slower,
// higher or lower than the first guess, and seen late: within a couple of percent, where the CL
// detent's own thrust is 30 to 70% off. Never beyond the clip's lever, nor below idle.
TEST(Playback_ThrustFollowerTracksAnApproach) {
    const std::vector<std::pair<double, double>> clip = {{0, 7}, {10, 12}, {30, 12}, {35, 40}, {45, 40}, {52, 18},
                                                         {60, 30}, {70, 25}, {80, 47}, {90, 30}, {100, 12}, {120, 20}};
    struct Variant {
        double scale, offset, tau;
        size_t delay;
    };
    for (const Variant v : {Variant{1.0, 0.0, 0.8, 2}, Variant{0.75, 5.0, 1.5, 6}, Variant{1.3, 0.0, 0.3, 3}}) {
        DemandAircraft ac;
        ac.scale = v.scale;
        ac.offset = v.offset;
        ac.tau = v.tau;
        ac.delay = v.delay;
        ThrustFollower follower;
        const double cl = 0.625;
        double sent = 0.0, lowest = 1.0, highest = 0.0, off = 0.0;
        int n = 0;
        for (int i = 0; i < 120 * 60; ++i) {
            const double t = i * kDt;
            const double target = piecewise(clip, t);
            const double axis = follower.step(target, ac.live(), 0.0, cl, kDt);
            lowest = std::fmin(lowest, axis);
            highest = std::fmax(highest, axis);
            if (std::fabs(axis - sent) >= 48.0 / 32768.0) sent = axis;  // as the key events go
            ac.step(sent, kDt);
            if (t < 8.0) continue;
            off += std::fabs(ac.d - target);
            ++n;
        }
        CHECK(off / n < 2.5);
        CHECK(lowest >= 0.0);
        CHECK(highest <= cl);
    }
    // The lever pulled back to idle: idle, whatever the demand.
    ThrustFollower follower;
    CHECK_NEAR(follower.step(30.0, 10.0, 0.0, 0.0, kDt), 0.0, 0);
    // An idle above the axis's bottom (a reverse zone below it): the guess starts there.
    follower.reset();
    CHECK_NEAR(follower.step(0.0, 0.0, 0.3, 1.0, 0.0), 0.3, 1e-12);
    CHECK_NEAR(follower.step(100.0, 100.0, 0.3, 1.0, 0.0), 1.0, 1e-12);
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

namespace {

// Lever up at 2 s (the legs follow from 4 s), down again at 12 s.
std::function<void(double)> cycleGear(FakeAircraft& a) {
    return [&a](double s) { a.setGearLever(s >= 2.0 && s < 12.0 ? 0.0 : 1.0); };
}

}  // namespace

// Where the aircraft can't be held above the runway, the gear step waits for the takeoff.
TEST(Setup_GearAfterTakeoffFindsTheLeverAheadOfTheLegs) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    h.aircraft.hoverable = false;
    REQUIRE(h.start());
    CHECK_EQ(h.status().getNumber("steps", 0), 2.0);
    REQUIRE(h.answer("skip"));  // throttle
    h.run(1.0);
    CHECK_EQ(h.phase(), std::string("takeoff"));
    CHECK(h.status().getString("note", "").find("couldn't hold the aircraft there (it can't here)") != std::string::npos);
    CHECK(!h.aircraft.frozen);

    h.aircraft.onGround = false;
    h.run(0.1);
    CHECK_EQ(h.phase(), std::string("user"));
    CHECK(h.run(120.0, cycleGear(h.aircraft), "ask"));
    REQUIRE(!h.aircraft.tried.empty());
    CHECK(h.aircraft.tried[0] == h.aircraft.gearWorks);
    CHECK_NEAR(h.aircraft.gearPos, 1.0, 1e-6);
    CHECK(inOrder(h.prompts, {"Take off", "Raise the gear", "Lower the gear", "Watch the gear"}));
    REQUIRE(h.answer("yes"));
    CHECK_EQ(h.phase(), std::string("done"));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Ok);
    CHECK_EQ(h.aircraft.hovers, 0);
}

// On the runway the gear needs no takeoff: once stopped, the aircraft is held frozen just above
// where it stood (its height above the ground there plus 30 ft), the sim shows it airborne, and the
// gear is taught as in flight. The step done, the aircraft goes back once the gear is down.
TEST(Setup_GearHeldAboveTheRunwayWithoutATakeoff) {
    Frame f;
    f.v[kAgl] = 12.5;
    CHECK_NEAR(SetupFlow::hoverLift(f), 42.5, 1e-9);
    f.v[kAgl] = NAN;
    CHECK_NEAR(SetupFlow::hoverLift(f), SetupFlow::kHoverLiftFt, 1e-9);
    f.v[kAgl] = 800.0;  // not on the ground
    CHECK_NEAR(SetupFlow::hoverLift(f), SetupFlow::kHoverLiftFt, 1e-9);

    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    REQUIRE(h.start());
    REQUIRE(h.answer("skip"));  // throttle
    CHECK_EQ(h.phase(), std::string("hover"));
    {
        const JsonValue st = h.status();
        CHECK(st.getString("detail", "").find("No takeoff needed") != std::string::npos);
        const JsonValue* buttons = st.get("buttons");
        REQUIRE(buttons && buttons->array.size() == 2);
        CHECK_EQ(buttons->array[0].string, std::string("skip"));
    }
    // Still rolling: it waits.
    h.aircraft.gs = 5.0;
    h.run(1.0);
    CHECK_EQ(h.status().getString("prompt", ""), std::string("Come to a stop"));
    CHECK_EQ(h.aircraft.hovers, 0);
    h.aircraft.gs = 0.0;
    h.run(0.1);
    CHECK_EQ(h.aircraft.hovers, 1);
    CHECK(h.aircraft.frozen);
    CHECK_NEAR(h.aircraft.lifted, SetupFlow::kHoverClearFt, 1e-9);  // its AGL on the ground is 0 here
    CHECK(h.run(1.0, nullptr, "user"));
    CHECK(logged("the aircraft counts as airborne above the runway"));
    CHECK(h.status().getString("detail", "").find("held frozen just above the runway") != std::string::npos);

    CHECK(h.run(120.0, cycleGear(h.aircraft), "ask"));
    CHECK(h.aircraft.frozen);
    REQUIRE(!h.aircraft.tried.empty());
    CHECK(h.aircraft.tried[0] == h.aircraft.gearWorks);
    REQUIRE(h.answer("yes"));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Ok);
    CHECK_EQ(h.phase(), std::string("land"));
    CHECK(h.flow.running());
    CHECK(h.run(5.0, nullptr, "done"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(!h.aircraft.frozen);
    CHECK(h.aircraft.onGround);
    CHECK_NEAR(h.aircraft.gearWhenPutBack, 1.0, 1e-6);
    CHECK(inOrder(h.prompts, {"Come to a stop", "Lifting the aircraft off the runway", "Raise the gear", "Lower the gear",
                              "Raise the gear", "Watch the gear", "Did the gear retract and extend again?",
                              "Putting the aircraft back", "Setup finished"}));
    CHECK(std::find(h.prompts.begin(), h.prompts.end(), "Take off") == h.prompts.end());
}

// The sim still counts the aircraft as on the ground up there: it goes back down and the step
// waits for the takeoff, saying why.
TEST(Setup_GearAfterTakeoffWhenTheAircraftCountsAsOnTheGroundAboveTheRunway) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    h.aircraft.hoverOnGround = true;
    REQUIRE(h.start());
    REQUIRE(h.answer("skip"));  // throttle
    CHECK(h.run(SetupFlow::kHoverAirborneSec - 1.0, nullptr, "user") == false);
    CHECK_EQ(h.phase(), std::string("hover"));
    CHECK(h.aircraft.frozen);
    CHECK(h.run(5.0, nullptr, "takeoff"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(!h.aircraft.frozen);
    CHECK(h.status().getString("note", "").find("the aircraft still counted as on the ground there. It's set up after takeoff "
                                                "instead") != std::string::npos);
    CHECK(logged("can't set up the gear above the runway"));
    h.run(1.0);
    CHECK_EQ(h.phase(), std::string("takeoff"));

    // The takeoff path as before.
    h.aircraft.onGround = false;
    h.run(0.1);
    CHECK_EQ(h.phase(), std::string("user"));
    CHECK(h.run(120.0, cycleGear(h.aircraft), "ask"));
    REQUIRE(h.answer("yes"));
    CHECK_EQ(h.phase(), std::string("done"));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Ok);
    CHECK_EQ(h.aircraft.hovers, 1);
}

// Frozen, this aircraft's gear doesn't move: the pilot's lever up and the gear still for as long as
// playback waits for it, the step waits for the takeoff instead. The lever was left up, so before
// the aircraft goes back the pilot is asked to lower it.
TEST(Setup_GearAfterTakeoffWhenItWontMoveAboveTheRunway) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    h.aircraft.gearStuckFrozen = true;
    h.aircraft.handleFollowsLever = true;
    REQUIRE(h.start());
    REQUIRE(h.answer("skip"));  // throttle
    CHECK(h.run(2.0, nullptr, "user"));
    auto pilot = [&](double s) { h.aircraft.setGearLever(s >= 2.0 ? 0.0 : 1.0); };  // and leaves it up
    CHECK(h.run(20.0, pilot, "land"));
    CHECK(h.status().getString("note", "").find("the gear didn't move with its lever up there") != std::string::npos);
    CHECK_NEAR(h.aircraft.gearPos, 1.0, 1e-6);
    CHECK(h.aircraft.frozen);
    CHECK(h.run(SetupFlow::kLandGearWaitSec + 1.0, nullptr, "lower"));
    CHECK_EQ(h.status().getString("prompt", ""), std::string("Lower the gear"));
    CHECK(logged("the gear isn't down after 30 s (at 1.00, its handle at 0.00)"));  // the legs down, the lever up
    CHECK(h.aircraft.frozen);
    h.aircraft.setGearLever(1.0);
    CHECK(h.run(5.0, nullptr, "takeoff"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(!h.aircraft.frozen);
    CHECK(logged("the gear is down"));
}

// The last method tried raised the gear and couldn't lower it. The step fails, the setup waits
// for the gear, then asks the pilot to lower it, and only then puts the aircraft back.
TEST(Setup_GearLeftUpByAFailedMethodAsksThePilotToLowerIt) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    h.aircraft.replayLowersOnlyAfterPilot = true;
    REQUIRE(h.start());
    REQUIRE(h.answer("skip"));  // throttle
    CHECK(h.run(2.0, nullptr, "user"));
    CHECK(h.run(900.0, cycleGear(h.aircraft), "land"));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Failed);
    CHECK(h.aircraft.gearPos < 0.5);  // left up
    CHECK(h.aircraft.frozen);
    CHECK_EQ(h.status().getString("prompt", ""), std::string("Waiting for the gear to come down"));
    {
        const JsonValue st = h.status();
        const JsonValue* buttons = st.get("buttons");
        CHECK(buttons && buttons->array.empty());
    }
    CHECK(h.run(SetupFlow::kLandGearWaitSec - 1.0, nullptr, "lower") == false);
    CHECK(h.run(2.0, nullptr, "lower"));
    CHECK(logged("the gear isn't down after 30 s (at 0.00, its handle at 0.00); asking you to lower it"));
    {
        const JsonValue st = h.status();
        CHECK(st.getString("detail", "").find("puts it back as it is") != std::string::npos);
        const JsonValue* buttons = st.get("buttons");
        REQUIRE(buttons && buttons->array.size() == 1);
        CHECK_EQ(buttons->array[0].string, std::string("cancel"));
    }
    h.run(60.0);  // it waits for the pilot
    CHECK_EQ(h.phase(), std::string("lower"));
    CHECK_EQ(h.aircraft.putBacks, 0);
    h.aircraft.setGearLever(1.0);
    CHECK(h.run(15.0, nullptr, "done"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(h.aircraft.gearWhenPutBack > 0.98);
    CHECK(!h.aircraft.frozen);
    CHECK(inOrder(h.prompts, {"Waiting for the gear to come down", "Lower the gear", "Putting the aircraft back"}));
}

// Cancelled above the runway, the aircraft goes back once the gear is down; cancelled again while
// the pilot is asked to lower it, it goes back as it is.
TEST(Setup_CancelAboveTheRunwayPutsTheAircraftBack) {
    SetupHarness h;
    h.facts.hasSpoilers = false;
    h.facts.engines = 1;
    h.aircraft.engines = 1;
    REQUIRE(h.start());
    REQUIRE(h.answer("skip"));  // throttle
    CHECK(h.run(2.0, nullptr, "user"));
    REQUIRE(h.answer("cancel"));
    CHECK(h.flow.active());
    CHECK_EQ(h.phase(), std::string("land"));
    CHECK(h.aircraft.frozen);
    CHECK(h.run(5.0, nullptr, "idle"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(!h.aircraft.frozen);
    CHECK(h.aircraft.onGround);
    CHECK(!h.aircraft.driving);
    CHECK(logged("control setup stopped (cancelled)"));

    SetupHarness g;
    g.facts.hasSpoilers = false;
    g.facts.engines = 1;
    g.aircraft.engines = 1;
    REQUIRE(g.start());
    REQUIRE(g.answer("skip"));  // throttle
    CHECK(g.run(2.0, nullptr, "user"));
    g.aircraft.setGearLever(0.0);
    g.run(8.0);
    REQUIRE(g.answer("later"));
    CHECK_EQ(g.phase(), std::string("land"));
    CHECK(g.run(SetupFlow::kLandGearWaitSec + 1.0, nullptr, "lower"));
    std::string err;
    CHECK(!g.flow.answer("skip", err));
    REQUIRE(g.answer("cancel"));
    CHECK_EQ(g.aircraft.putBacks, 1);
    CHECK(g.aircraft.gearWhenPutBack < 0.5);  // as it was
    CHECK(!g.aircraft.frozen);
    CHECK(g.run(2.0, nullptr, "idle"));
    CHECK(logged("control setup stopped (finish later)"));
}

namespace {

const ControlMethod kLever777{MethodKind::KeyEvent, "#69927", 1.0};                    // its position
const ControlMethod kClick777{MethodKind::KeyEvent, "#69927(536870912,131072)", 1.0};  // a left click
const ControlMethod kGearKeys{MethodKind::KeyEvent, "", 1.0};
constexpr int kDown = static_cast<int>(Direction::Extend);
constexpr int kUp = static_cast<int>(Direction::Retract);

}  // namespace

// The PMDG 777 (in the air, so only the gear), as it took the replays: its lever event's position
// raises the gear but doesn't lower it; the key events lower it, but only after a replay raised
// it; after the pilot raised it only a left click on the lever does. No variable the scan sees is
// its lever, and its legs' animations, which move with the gear, aren't tried at all (each swung
// only its leg). From its table entry the setup tries the event's position and a click on it.
// The way back from where the pilot left the gear finds the click; the whole movement, the event
// raising it; and from where that left it, the key events lowering it. The profile keeps both
// ways down, without anything the 777's own beyond the event's number.
TEST(Setup_Pmdg777GearLearnsEachWayOnItsOwn) {
    SetupHarness h;
    h.key = "simobjects/airplanes/pmdg 777";
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.pmdg777 = true;
    // Its gear simvars take ten seconds to travel, its own leg animations four: too unlike to tell
    // them by the legs' time, but seconds all the same.
    h.aircraft.gearRate = 0.1;
    h.aircraft.legAnimRate = 0.25;
    const std::vector<ControlMethod> own = ownGearWays(h.key);
    REQUIRE(own.size() == 2);
    CHECK(own[0] == kLever777);
    CHECK(own[1] == kClick777);
    REQUIRE(h.start());
    // Lever up at 2 s (the legs follow from 4 s), down again at 16 s.
    auto pilot = [&](double s) { h.aircraft.setGearLever(s >= 2.0 && s < 16.0 ? 0.0 : 1.0); };
    CHECK(h.run(400.0, pilot, "ask"));
    CHECK(inOrder(h.prompts, {"Raise the gear", "Lower the gear", "Raise the gear", "Watch the gear",
                              "Did the gear retract and extend again?"}));
    // From where the pilot left it: the event's DOWN did nothing, the click lowered it. Then the
    // whole movement: the event raised it but didn't lower it, and from there the click didn't,
    // the key events did.
    REQUIRE(h.backTried.size() == 4);
    CHECK(h.backTried[0] == kLever777);
    CHECK(h.backTried[1] == kClick777);
    REQUIRE(h.aircraft.tried.size() == 1);
    CHECK(h.aircraft.tried[0] == kLever777);
    CHECK(h.backTried[2] == kClick777);
    CHECK(h.backTried[3] == kGearKeys);
    CHECK_NEAR(h.aircraft.gearPos, 1.0, 1e-6);
    CHECK(logged("L:var 7X7XleftMLGExt took 3."));
    CHECK(logged("it follows them (their animation, not the lever); not trying it"));
    CHECK(logged("\"Raise the gear\" once more: the way back is tried from where you leave the gear"));
    CHECK(logged("the aircraft's own event #69927 didn't bring the gear back from where you left it (it didn't move)"));
    CHECK(logged("the aircraft's own event #69927 (536870912, then 131072) as a toggle brought the gear back from where you "
                 "left it"));
    CHECK(logged("the aircraft's own event #69927 moved the gear 100% as far as you did, but didn't bring it back"));
    CHECK(logged("(536870912, then 131072) as a toggle didn't bring the gear back from where the replay left it"));
    CHECK(logged("key events GEAR_SET/GEAR_UP/GEAR_DOWN brought the gear back from where the replay left it"));
    // Each command and where the gear went, in every replay.
    CHECK(logged("gear up at "));
    CHECK(logged("the gear went from 1.00 to 0.00"));
    CHECK(logged("gear down at 0.0 s: the gear went from 0.00 to 1.00"));
    REQUIRE(h.answer("yes"));
    const ControlProfile& gp = h.flow.profile()[Control::Gear];
    CHECK(gp.state == ControlState::Ok);
    CHECK(gp.method == kLever777);
    REQUIRE(gp.ways[kUp].size() == 1);
    CHECK(gp.ways[kUp][0] == kLever777);
    REQUIRE(gp.ways[kDown].size() == 2);
    CHECK(gp.ways[kDown][0] == kGearKeys);
    CHECK(gp.ways[kDown][1] == kClick777);
    CHECK_EQ(describeControl(Control::Gear, gp),
             std::string("down: key events GEAR_SET/GEAR_UP/GEAR_DOWN, then the aircraft's own event #69927 (536870912, "
                         "then 131072) as a toggle; up: the aircraft's own event #69927"));
}

// Playback with that profile, through the picker ControlDriver uses (as the fake's methods act as
// ControlDriver's would). The clip lowers the gear the pilot raised: the key events first, which
// don't, then the click, which does and goes first. It raises it with the event, and lowers it
// again: the click first now, which doesn't after a replay raised it, then the key events, which
// do and go first again. Each goes to the front of its list in the profile too.
TEST(Playback_GearTriesTheNextWayAndKeepsTheOneThatWorked) {
    FakeAircraft a;
    a.onGround = false;
    a.pmdgGear = true;
    a.pmdg777 = true;
    ControlProfile gp;
    gp.state = ControlState::Ok;
    gp.method = kLever777;
    gp.ways[kDown] = {kGearKeys, kClick777};
    gp.ways[kUp] = {kLever777};
    // The pilot raised the gear.
    a.setGearLever(0.0);
    for (int i = 0; i < 60 * 8; ++i) a.step(kDt);
    REQUIRE(a.gearPos < 1e-6);

    WayPicker picker;
    picker.begin(Control::Gear, gp);
    REQUIRE(picker.ways().size() == 3);
    int active = -1, gaveUp = 0;
    std::vector<std::pair<Direction, ControlMethod>> promoted;
    std::vector<double> at;  // the gear at 50 s, 105 s and 160 s
    for (double t = 0.0; t < 165.0; t += kDt) {
        // The clip's gear command: up (as the pilot left it), down from 5 s, up from 60 s, down from 115 s.
        Frame f;
        for (double& v : f.v) v = NAN;
        f.v[kGearHandle] = (t >= 5.0 && t < 60.0) || t >= 115.0 ? 1.0 : 0.0;
        const WayPicker::Pick p = picker.step(eventTarget(Control::Gear, f), a.live());
        if (p.way != active) {
            if (active >= 0) a.endControl(Control::Gear);
            std::string why;
            REQUIRE(a.beginControl(Control::Gear, picker.ways()[static_cast<size_t>(p.way)], why));
            active = p.way;
        }
        a.applyControl(Control::Gear, f, p.fresh);
        if (p.gaveUp >= 0) ++gaveUp;
        if (p.promoted) {
            const ControlMethod& m = picker.ways()[static_cast<size_t>(p.worked)];
            promoted.push_back({p.dir, m});
            CHECK(promoteWay(gp, p.dir, m));
        }
        a.step(kDt);
        for (const double check : {50.0, 105.0, 160.0}) {
            if (t < check && t + kDt >= check) at.push_back(a.gearPos);
        }
    }
    REQUIRE(at.size() == 3);
    CHECK_NEAR(at[0], 1.0, 1e-6);
    CHECK_NEAR(at[1], 0.0, 1e-6);
    CHECK_NEAR(at[2], 1.0, 1e-6);
    CHECK_EQ(gaveUp, 2);
    REQUIRE(promoted.size() == 2);
    CHECK(promoted[0].first == Direction::Extend && promoted[0].second == kClick777);
    CHECK(promoted[1].first == Direction::Extend && promoted[1].second == kGearKeys);
    // The ways in the order each was driven: the event (up, as the gear was), the key events,
    // the click; the event; the click (first by then), the key events.
    REQUIRE(a.tried.size() == 6);
    CHECK(a.tried[0] == kLever777);
    CHECK(a.tried[1] == kGearKeys);
    CHECK(a.tried[2] == kClick777);
    CHECK(a.tried[3] == kLever777);
    CHECK(a.tried[4] == kClick777);
    CHECK(a.tried[5] == kGearKeys);
    REQUIRE(gp.ways[kDown].size() == 2);
    CHECK(gp.ways[kDown][0] == kGearKeys);
    CHECK(gp.ways[kDown][1] == kClick777);
    CHECK(!promoteWay(gp, Direction::Extend, kGearKeys));  // first already
    CHECK(!promoteWay(gp, Direction::Retract, kGearKeys));  // not that way
}

// A replay that starts with the gear left up (the last method raised it but couldn't lower it):
// the gear goes back down first, and only movement from there counts. The key events can lower
// the 777's gear (after a replay raised it) but not raise it, so they don't pass; counted from
// where the gear was, they moved it all the way and back. (From where the pilot left it they
// didn't bring it back at all.)
TEST(Setup_ReplayCountsOnlyFromWhereThePilotStarted) {
    SetupHarness h;
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.pmdg777 = true;  // no table entry here: only the key events are tried
    REQUIRE(h.start());
    auto pilot = [&](double s) { h.aircraft.setGearLever(s >= 2.0 && s < 16.0 ? 0.0 : 1.0); };
    REQUIRE(h.run(150.0, pilot, "replay"));
    REQUIRE(h.backTried.size() == 1);
    CHECK(logged("none of the method(s) brought the gear back from where you left it"));
    // Up, as if the last method had raised it.
    h.aircraft.replayGearLever(1.0);
    h.aircraft.replayGearLever(0.0);
    h.aircraft.gearPos = 0.0;
    h.aircraft.gearHandleSimvar = 0.0;
    CHECK(h.run(300.0, nullptr, "done"));
    CHECK_EQ(h.asks, 0);
    REQUIRE(h.aircraft.tried.size() == 1);
    CHECK(h.aircraft.tried[0] == (ControlMethod{MethodKind::KeyEvent, std::string(), 1.0}));
    CHECK(logged("the gear was back where you started it after "));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Failed);
}

// The same gear on an aircraft without an own event known: only the key events are tried, and
// they fail; the legs' animations are left out.
TEST(Setup_GearLegAnimationsAreNotTried) {
    SetupHarness h;
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.gearWorks = ControlMethod{MethodKind::KeyEvent, "#69927", 1.0};  // never offered here
    REQUIRE(h.start());
    auto pilot = [&](double s) { h.aircraft.setGearLever(s >= 2.0 && s < 16.0 ? 0.0 : 1.0); };
    CHECK(h.run(300.0, pilot, "done"));
    REQUIRE(h.aircraft.tried.size() == 1);
    CHECK(h.aircraft.tried[0] == (ControlMethod{MethodKind::KeyEvent, std::string(), 1.0}));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Failed);
}

// Of the gear's L:vars that are left, the one that changed first is tried first: the lever's
// (switch_295_a, as you click it) before a sound flag that follows the legs (2 s later).
TEST(Setup_GearTriesTheVariableThatLedFirst) {
    SetupHarness h;
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.pmdgLeverVars = true;
    h.aircraft.gearWorks = ControlMethod{MethodKind::KeyEvent, "#70003", 1.0};  // nothing works
    REQUIRE(h.start());
    auto pilot = [&](double s) { h.aircraft.setGearLever(s >= 2.0 && s < 16.0 ? 0.0 : 1.0); };
    CHECK(h.run(400.0, pilot, "done"));
    REQUIRE(h.aircraft.tried.size() == 3);
    CHECK(h.aircraft.tried[0] == (ControlMethod{MethodKind::KeyEvent, std::string(), 1.0}));
    CHECK(h.aircraft.tried[1] == (ControlMethod{MethodKind::LVar, "switch_295_a", 1.0}));
    CHECK(h.aircraft.tried[2] == (ControlMethod{MethodKind::LVar, "7X7XleftMLGExtSnd", 1.0}));
}

TEST(Setup_OwnGearLeverOnlyForAircraftWithOne) {
    CHECK_EQ(ownGearWays("simobjects/airplanes/pmdg 777").size(), size_t(2));
    CHECK_EQ(ownGearWays("simobjects/airplanes/pmdg 777f").size(), size_t(2));
    CHECK(ownGearWays("simobjects/airplanes/pmdg 737").empty());
    CHECK(ownGearWays("simobjects/airplanes/a350").empty());
    CHECK(ownGearWays("").empty());
    for (const ControlMethod& m : ownGearWays("simobjects/airplanes/pmdg 777")) CHECK(isOwnEvent(m));
    CHECK(!isOwnEvent(ControlMethod{MethodKind::KeyEvent, kKeyFlapsNotches, 1.0}));
    CHECK(!isOwnEvent(ControlMethod{MethodKind::LVar, "#1", 1.0}));
}

TEST(Setup_TrackerTravelTellsALeverFromWhatFollowsIt) {
    CandidateTracker t;
    t.begin({{"LEVER", true}, {"LEG_ANIMATION", true}});
    std::vector<double> v(2);
    for (int s = 0; s <= 300; ++s) {
        const double time = s * 0.1;
        const double lever = time >= 2.0 && time < 16.0 ? 0.0 : 1.0;
        // The legs from 4 s, at a quarter of their travel a second; back down from 18 s.
        double leg = 1.0;
        if (time >= 4.0 && time < 18.0) leg = std::fmax(0.0, 1.0 - 0.25 * (time - 4.0));
        if (time >= 18.0) leg = std::fmin(1.0, 0.25 * (time - 18.0));
        v[0] = lever;
        v[1] = leg * 1000.0;
        t.sample(time, v, leg, leg);
    }
    CHECK_NEAR(t.travelSec(0, 0.0, 30.0), 0.0, 1e-9);
    CHECK_NEAR(t.travelSec(1, 0.0, 30.0), t.referenceTravelSec(0.0, 30.0), 0.11);  // (a sample either way)
    CHECK_NEAR(t.referenceTravelSec(0.0, 30.0), 3.4, 0.11);  // 5% to 90% of the way
}

// The pilot's gear lever in the cockpit, on an aircraft whose lever answers only its own event (a
// third-party number nothing knows in advance), which fires as the lever moves, with its position.
// No variable the scan sees is the lever, and an elevator axis keeps firing all along. The event is
// learned as a position and tried first, replaying the parameters it fired with.
TEST(Setup_LearnsTheGearLeversOwnEventFromThePilotsMovement) {
    SetupHarness h;
    h.key = "simobjects/airplanes/some airliner";
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.keyNoise = true;
    h.aircraft.gearWorks = ControlMethod{MethodKind::KeyEvent, "#70001", 1.0};
    REQUIRE(h.start());
    // Lever up at 2 s (the legs follow from 4 s), down again at 16 s.
    auto pilot = [&](double s) {
        const double lever = s >= 2.0 && s < 16.0 ? 0.0 : 1.0;
        if (lever != h.aircraft.gearLever) h.aircraft.fire(70001, lever);
        h.aircraft.setGearLever(lever);
    };
    CHECK(h.run(150.0, pilot, "ask"));
    REQUIRE(h.aircraft.tried.size() == 1);
    CHECK(h.aircraft.tried[0] == h.aircraft.gearWorks);
    CHECK_NEAR(h.aircraft.gearPos, 1.0, 1e-6);
    // What fired, for the log: the lever's event with both positions, the axis mostly before.
    CHECK(logged("key event(s) of 2 kind(s) while you moved the gear"));
    CHECK(logged("#70001 (third-party +369): 2 in all, 2 with it at -2."));  // the legs follow the lever 2 s on
    CHECK(logged(" s, parameters 0 (1), 1 (1)"));
    CHECK(logged("AXIS_ELEVATOR_SET (65762): "));
    CHECK(logged(" of them before it moved: steadily, parameters -300 .. 300 (more than 4 values)"));
    CHECK(logged("#70001 (third-party +369) fired with it 2 time(s): a position"));
    REQUIRE(h.answer("yes"));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Ok);
    CHECK(h.flow.profile()[Control::Gear].method == h.aircraft.gearWorks);
}

// The same lever clicked instead: each click sends the same event with the same parameter (a
// mouse button), and each flips the lever. Learned as a toggle and replayed by clicking until the
// legs show the clip's command. A switch clicked meanwhile, which moved nothing, isn't tried.
TEST(Setup_LearnsAToggleGearEventFromCockpitClicks) {
    SetupHarness h;
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.keyNoise = true;
    const double kLeftClick = 536870912.0;  // PMDG's MOUSE_FLAG_LEFTSINGLE
    h.aircraft.gearWorks = ControlMethod{MethodKind::KeyEvent, "#70002(536870912)", 1.0};
    REQUIRE(h.start());
    bool lights = false;
    auto pilot = [&](double s) {
        const double lever = s >= 2.0 && s < 16.0 ? 0.0 : 1.0;
        if (lever != h.aircraft.gearLever) h.aircraft.fire(70002, kLeftClick);
        h.aircraft.setGearLever(lever);
        if (s >= 9.0 && !lights) {
            lights = true;
            h.aircraft.fire(70500, kLeftClick);
        }
    };
    CHECK(h.run(150.0, pilot, "ask"));
    REQUIRE(h.aircraft.tried.size() == 1);
    CHECK(h.aircraft.tried[0] == h.aircraft.gearWorks);
    REQUIRE(h.backTried.size() == 1);
    CHECK(h.backTried[0] == h.aircraft.gearWorks);
    CHECK_EQ(h.aircraft.clicks, 3);  // down from where the pilot left it; then up and down again
    CHECK_NEAR(h.aircraft.gearPos, 1.0, 1e-6);
    CHECK(logged("#70002 (third-party +370) (536870912) fired with it 2 time(s): a toggle"));
    CHECK(logged("#70500 (third-party +868) (536870912) fired 1 time(s); the gear command didn't move after 1"));
    REQUIRE(h.answer("yes"));
    // It did it all, from where the pilot left the gear too: one method, both ways.
    CHECK(h.flow.profile()[Control::Gear].method == h.aircraft.gearWorks);
    CHECK(!h.flow.profile()[Control::Gear].perDirection());
    CHECK_EQ(describeMethod(Control::Gear, h.aircraft.gearWorks),
             std::string("the aircraft's own event #70002 (536870912) as a toggle"));
}

// The PMDG 777 with its lever's event seen firing: tried once, as seen, not again from the table;
// the table's click is still tried.
TEST(Setup_GearEventSeenIsNotTriedAgainFromTheTable) {
    SetupHarness h;
    h.key = "simobjects/airplanes/pmdg 777";
    h.aircraft.onGround = false;
    h.aircraft.pmdgGear = true;
    h.aircraft.gearWorks = ControlMethod{MethodKind::KeyEvent, "#70003", 1.0};  // nothing works
    REQUIRE(h.start());
    auto pilot = [&](double s) {
        const double lever = s >= 2.0 && s < 16.0 ? 0.0 : 1.0;
        if (lever != h.aircraft.gearLever) h.aircraft.fire(69927, lever);
        h.aircraft.setGearLever(lever);
    };
    CHECK(h.run(400.0, pilot, "done"));
    REQUIRE(h.aircraft.tried.size() == 3);
    CHECK(h.aircraft.tried[0] == kLever777);
    CHECK(h.aircraft.tried[1] == kClick777);
    CHECK(h.aircraft.tried[2] == kGearKeys);
    CHECK_EQ(h.backTried.size(), size_t(3));
    CHECK(h.flow.profile()[Control::Gear].state == ControlState::Failed);
}

// An event that fired steadily before the movement's window isn't kept from then on (only
// counted), so it can't crowd out the others; the others are.
TEST(Setup_KeyEventTrackerKeepsOnlyWhatDidntFireSteadily) {
    KeyEventTracker k;
    for (int i = 0; i < 100; ++i) k.add(i * 0.1, 65762, i % 7);
    k.add(9.5, 70001, 0.0);
    k.dropBefore(4.95);
    for (int i = 100; i < 200; ++i) k.add(i * 0.1, 65762, i % 7);
    k.add(15.0, 70001, 1.0);
    REQUIRE(k.events().size() == 2);
    const KeyEventTracker::Event& axis = k.events()[0];
    CHECK_EQ(axis.count, size_t(200));
    CHECK_EQ(axis.before, size_t(50));
    CHECK(axis.steady());
    CHECK_EQ(axis.firings.size(), size_t(50));  // those it fired with the movement before it was known to be steady
    CHECK(axis.many);
    CHECK_EQ(axis.lo, 0.0);
    CHECK_EQ(axis.hi, 6.0);
    const KeyEventTracker::Event& lever = k.events()[1];
    CHECK(!lever.steady());
    REQUIRE(lever.firings.size() == 2);
    CHECK_EQ(lever.firings[1].t, 15.0);
    CHECK_EQ(lever.values.size(), size_t(2));
    CHECK_EQ(k.stored(), size_t(52));
}

TEST(Setup_EventMethodVars) {
    const char* const vars[] = {"#69927",
                                "#70010@-16383",
                                "#66587(29501)",
                                "#69940(536870912)/#69940(-2147483648)",
                                "#70011(1)/#70012(0)",
                                "#69927(536870912,131072)",
                                "#69940(536870912,131072)/#69940(-2147483648,524288)"};
    const EventMethod::Mode modes[] = {EventMethod::Mode::Position, EventMethod::Mode::Position, EventMethod::Mode::Toggle,
                                       EventMethod::Mode::Steps,    EventMethod::Mode::Steps,    EventMethod::Mode::Toggle,
                                       EventMethod::Mode::Steps};
    for (size_t i = 0; i < 7; ++i) {
        EventMethod e;
        REQUIRE(parseEventMethod(ControlMethod{MethodKind::KeyEvent, vars[i], 2.0}, e));
        CHECK(e.mode == modes[i]);
        CHECK_EQ(e.scale, 2.0);
        const ControlMethod back = eventMethod(e);
        CHECK_EQ(back.var, std::string(vars[i]));
        CHECK_EQ(back.scale, 2.0);
    }
    EventMethod e;
    REQUIRE(parseEventMethod(ControlMethod{MethodKind::KeyEvent, "#69940(536870912)/#69940(-2147483648)", 1.0}, e));
    CHECK_EQ(e.id, 69940u);
    CHECK_EQ(e.param, 536870912);
    CHECK_EQ(e.lowerId, 69940u);
    CHECK_EQ(e.lowerParam, INT32_MIN);
    // A position: offset + value x scale, the axis's ends and the lever's positions.
    REQUIRE(parseEventMethod(ControlMethod{MethodKind::KeyEvent, "#70010@-16383", 32766.0}, e));
    CHECK_EQ(eventParam(e, 0.0), -16383);
    CHECK_EQ(eventParam(e, 1.0), 16383);
    REQUIRE(parseEventMethod(ownGearWays("simobjects/airplanes/pmdg 777")[0], e));
    CHECK_EQ(eventParam(e, 0.0), 0);
    CHECK_EQ(eventParam(e, 1.0), 1);
    // A click: the button, then its release, on the same event (and either step's).
    REQUIRE(parseEventMethod(ownGearWays("simobjects/airplanes/pmdg 777")[1], e));
    CHECK(e.mode == EventMethod::Mode::Toggle && e.click && e.param == 0x20000000 && e.release == 0x00020000);
    REQUIRE(parseEventMethod(ControlMethod{MethodKind::KeyEvent, "#69940(536870912,131072)/#69940(-2147483648,524288)", 1.0}, e));
    CHECK(e.click && e.release == 131072 && e.lowerClick && e.lowerParam == INT32_MIN && e.lowerRelease == 524288);
    REQUIRE(parseEventMethod(ControlMethod{MethodKind::KeyEvent, "#69940(1)/#69940(2,3)", 1.0}, e));
    CHECK(!e.click && e.lowerClick && e.lowerRelease == 3);
    for (const char* bad : {"#", "#0", "#12(", "#12(3", "#12(3)/", "#12(3)/#13", "#12@", "#12@x", "#12x", "FLAPS_INCR",
                            "#12(3,)", "#12(3,x)", "#12(3,4", "#69927+keys"}) {
        CHECK(!parseEventMethod(ControlMethod{MethodKind::KeyEvent, bad, 1.0}, e));
    }
    CHECK(!parseEventMethod(ControlMethod{MethodKind::LVar, "#12", 1.0}, e));
}

// Clicking toward the clip's value: one click, then nothing until the aircraft shows it (the gear
// legs take a while to start), unless it never does; then the next.
TEST(Setup_EventStepperWaitsForTheAircraft) {
    EventStepper s;
    CHECK_EQ(s.step(0.0, 1.0, 1.0, EventStepper::kGearWaitFrames, true), -1);
    int sent = 0;
    for (int i = 0; i < 120; ++i) sent += s.step(0.0, 1.0, 1.0, EventStepper::kGearWaitFrames, true) != 0;
    CHECK_EQ(sent, 0);
    CHECK_EQ(s.step(0.0, 0.0, 1.0, EventStepper::kGearWaitFrames, false), 0);  // there
    CHECK_EQ(s.step(1.0, 0.0, 1.0, EventStepper::kGearWaitFrames, false), 1);
    // Flap notches: a step at a time, each once the handle shows the last.
    EventStepper f;
    CHECK_EQ(f.step(3.0, 0.0, 1.0, EventStepper::kWaitFrames, false), 1);
    int quiet = 0;
    while (f.step(3.0, 1.0, 1.0, EventStepper::kWaitFrames, false) == 0 && quiet < 1000) ++quiet;
    CHECK_EQ(quiet, EventStepper::kMinFrames - 1);  // the handle showed it: the next notch
    quiet = 0;
    while (f.step(3.0, 1.0, 1.0, EventStepper::kWaitFrames, false) == 0 && quiet < 1000) ++quiet;
    CHECK_EQ(quiet, EventStepper::kWaitFrames - 1);  // it didn't show that one: again, a second on
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

TEST(Profile_AircraftFolder) {
    const std::string a350 = "simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin_ulr/config";
    CHECK_EQ(aircraftFolder("SimObjects\\Airplanes\\a350\\presets\\inibuilds\\A350-900_Default_Cabin_ULR\\config\\aircraft.CFG"),
             a350);
    // The event's full path, a streamed aircraft's included, comes out the same as the state's.
    CHECK_EQ(aircraftFolder(".\\LocalState\\StreamedPackages\\inibuilds-aircraft-a350\\Content\\SimObjects\\Airplanes\\"
                            "a350\\presets\\inibuilds\\A350-900_Default_Cabin_ULR\\config\\aircraft.cfg"),
             a350);
    CHECK_EQ(aircraftFolder("C:\\Community\\pmdg-aircraft-738\\SimObjects\\Airplanes\\PMDG 737-800\\aircraft.cfg"),
             std::string("simobjects/airplanes/pmdg 737-800"));
    CHECK_EQ(aircraftFolder(""), std::string());
    CHECK(isTitleKey("title:A380-800"));
    CHECK(!isTitleKey(a350));
}

TEST(Profile_IdentityIgnoresStaleState) {
    const std::string a350 = "simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin_ulr/config";
    const std::string a380 = "simobjects/airplanes/a380/presets/x/a380-800/config";

    // Startup: the state answers before the first title; the title then goes with it.
    AircraftIdentity id;
    CHECK(id.wantState());
    CHECK(id.onState(a350));
    CHECK(!id.onTitle("A350-900 ULR"));
    CHECK_EQ(id.key(), a350);
    CHECK(!id.wantState());

    // Switched to the A380 (as logged): the title changes, the state still names the A350.
    CHECK(id.onTitle("A380-800 RR Cabin"));
    CHECK_EQ(id.key(), std::string("title:A380-800 RR Cabin"));
    CHECK(id.wantState());
    CHECK(!id.onState(a350));
    CHECK(id.stale());
    CHECK_EQ(id.key(), std::string("title:A380-800 RR Cabin"));
    // Its path when the sim catches up, or from the event.
    CHECK(id.onState(a380));
    CHECK(!id.stale());
    CHECK_EQ(id.key(), a380);

    // Back to the A350, the event first: the title then goes with it.
    CHECK(id.onEvent(a350));
    CHECK_EQ(id.key(), a350);
    CHECK(!id.onTitle("A350-900 ULR"));
    CHECK_EQ(id.key(), a350);
    // A lagging state answer doesn't undo the event.
    CHECK(!id.onState(a380));
    CHECK_EQ(id.key(), a350);

    // The title first, then the event.
    CHECK(id.onTitle("A380-800 RR Cabin"));
    CHECK(id.onEvent(a380));
    CHECK_EQ(id.key(), a380);
    CHECK(!id.onTitle("A380-800 RR Cabin"));

    // Another livery in the same aircraft.cfg (one title per livery): the event says it's the same.
    CHECK(id.onTitle("A380-800 Other Livery"));
    CHECK(id.onEvent(a380));
    CHECK_EQ(id.key(), a380);
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
    {
        // "Toggle reverse" (the A350).
        ProfileStore s;
        AircraftProfile& a350 = s.upsert("a350");
        a350[Control::Throttle] = {ControlState::Ok, ControlMethod{MethodKind::KeyEvent, "", 1.0}, true};
        a350[Control::Throttle].reverseBy = ReverseBy::Toggle;
        std::string j;
        s.toJson(j);
        CHECK(j.find("\"reverseBy\":\"toggle\"") != std::string::npos);
        ProfileStore t;
        REQUIRE(t.fromJson(j.data(), j.size(), &err));
        REQUIRE(t.find("a350"));
        CHECK((*t.find("a350"))[Control::Throttle].reverseBy == ReverseBy::Toggle);
        CHECK(describeControl(Control::Throttle, (*t.find("a350"))[Control::Throttle]).find("toggle reverse") !=
              std::string::npos);
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

// Profiles from before the per-direction ways: one method both ways, read and written as they
// were, and played back with that one method whatever the aircraft does. The gear's "+keys" of
// the builds just before becomes its ways: up by the event, down by the key events, then the click.
TEST(Profile_OldProfilesKeepOneMethodBothWays) {
    const char* old =
        "{\"version\":1,\"aircraft\":[{\"key\":\"simobjects/airplanes/a320neo\",\"controls\":{"
        "\"gear\":{\"state\":\"ok\",\"method\":\"input_set\",\"var\":\"LANDING_GEAR_GEAR\",\"scale\":1,\"confirmed\":true},"
        "\"spoilers\":{\"state\":\"ok\",\"method\":\"key_event\",\"var\":\"\",\"scale\":1}}},"
        "{\"key\":\"simobjects/airplanes/pmdg 777\",\"controls\":{"
        "\"gear\":{\"state\":\"ok\",\"method\":\"key_event\",\"var\":\"#69927+keys\",\"scale\":1,\"confirmed\":true}}}]}";
    ProfileStore s;
    std::string err;
    REQUIRE(s.fromJson(old, std::strlen(old), &err));
    REQUIRE(s.find("simobjects/airplanes/a320neo"));
    const AircraftProfile& a = *s.find("simobjects/airplanes/a320neo");
    const ControlMethod gearSet{MethodKind::InputSet, "LANDING_GEAR_GEAR", 1.0};
    for (const Control c : {Control::Gear, Control::Spoilers}) {
        CHECK(!a[c].perDirection());
        for (const Direction d : {Direction::Extend, Direction::Retract}) {
            const std::vector<ControlMethod> ways = a[c].waysTo(d);
            REQUIRE(ways.size() == 1);
            CHECK(ways[0] == a[c].method);
        }
    }
    CHECK(a[Control::Gear].method == gearSet);
    CHECK_EQ(describeControl(Control::Gear, a[Control::Gear]), describeMethod(Control::Gear, gearSet));
    // Written back as it was: no lists.
    ProfileStore justA;
    justA.upsert(a.key) = a;
    std::string json;
    justA.toJson(json);
    CHECK(json.find("\"down\"") == std::string::npos && json.find("\"up\"") == std::string::npos);
    CHECK(json.find("\"extend\"") == std::string::npos && json.find("\"retract\"") == std::string::npos);
    CHECK(!promoteWay(justA.upsert(a.key)[Control::Gear], Direction::Extend, gearSet));

    // Playback: always that one method, sent as before, though the gear doesn't follow it.
    WayPicker p;
    p.begin(Control::Gear, a[Control::Gear]);
    CHECK(!p.multi());
    REQUIRE(p.ways().size() == 1);
    FakeAircraft fake;
    for (int i = 0; i < 60 * 30; ++i) {
        Frame f;
        for (double& v : f.v) v = NAN;
        f.v[kGearHandle] = i < 60 * 15 ? 0.0 : 1.0;
        const WayPicker::Pick pick = p.step(eventTarget(Control::Gear, f), fake.live());
        CHECK(pick.way == 0 && !pick.fresh && pick.gaveUp < 0 && pick.worked < 0 && !pick.promoted);
        fake.step(kDt);
    }

    // The 777's "+keys".
    REQUIRE(s.find("simobjects/airplanes/pmdg 777"));
    const ControlProfile& g = (*s.find("simobjects/airplanes/pmdg 777"))[Control::Gear];
    CHECK(g.state == ControlState::Ok);
    CHECK(g.method == kLever777);
    REQUIRE(g.ways[kUp].size() == 1 && g.ways[kDown].size() == 2);
    CHECK(g.ways[kUp][0] == kLever777);
    CHECK(g.ways[kDown][0] == kGearKeys);
    CHECK(g.ways[kDown][1] == kClick777);

    // The lists round-trip, in their order, under each control's names.
    ProfileStore lists;
    AircraftProfile& l = lists.upsert("l");
    l[Control::Gear] = g;
    l[Control::Spoilers] = {ControlState::Ok, ControlMethod{MethodKind::KeyEvent, "", 1.0}, true};
    l[Control::Spoilers].ways[kUp] = {ControlMethod{MethodKind::KeyEvent, "", 1.0}, ControlMethod{MethodKind::SimVar, "", 1.0}};
    lists.toJson(json);
    CHECK(json.find("\"down\":[{\"method\":\"key_event\",\"var\":\"\",\"scale\":1}") != std::string::npos);
    CHECK(json.find("\"retract\":[") != std::string::npos);
    CHECK(json.find("\"extend\"") == std::string::npos);
    ProfileStore back;
    REQUIRE(back.fromJson(json.data(), json.size(), &err));
    REQUIRE(back.find("l"));
    const AircraftProfile& b = *back.find("l");
    CHECK(b[Control::Gear].ways[kDown] == g.ways[kDown]);
    CHECK(b[Control::Gear].ways[kUp] == g.ways[kUp]);
    CHECK(b[Control::Spoilers].ways[kUp] == l[Control::Spoilers].ways[kUp]);
    CHECK(b[Control::Spoilers].ways[kDown].empty());
    REQUIRE(b[Control::Spoilers].waysTo(Direction::Extend).size() == 1);  // that way: its method
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
    // Stops early once the setup reaches `until` (a phase name, "idle" once it's inactive), if given.
    bool run(double seconds, const std::function<void(double)>& pilot = nullptr, const char* until = nullptr) {
        for (double s = 0.0; s < seconds; s += kDt) {
            const JsonValue st = setupStatus();
            const std::string phase = st.getString("phase", "");
            if (pilot && (phase == "prepare" || phase == "user")) pilot(s);  // hands off during replays
            if (phase == "away") {  // once more, to try the way back from there
                if (st.getString("step", "") == "spoilers") aircraft.spoilerHandle = 1.0;
                if (st.getString("step", "") == "gear") aircraft.setGearLever(0.0);
            }
            aircraft.step(kDt);
            FrameInput in;
            in.raw = aircraft.live();
            in.simTimeSec = simTime;
            in.facts = facts;
            simTime += kDt;
            engine.onFrame(in, 1792247525000 + static_cast<int64_t>(simTime * 1000.0));
            const std::string now = engine.setupActive() ? setupStatus().getString("phase", "") : "idle";
            if (until && now == until) return true;
        }
        return !until;
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

// Set up / Redo on an aircraft that has a profile: the same offer as a new one, the checklist
// first; in the air, the gear. Started, "later" or a new flight, it's gone again.
TEST(EngineSetup_RedoGoesThroughTheSameChecklist) {
    EngineSetupHarness h;
    h.facts.gearRetractable = false;
    h.run(0.2);
    std::string err;
    REQUIRE(h.engine.setupStart(err));
    REQUIRE(h.engine.setupAnswer("skip", err));  // spoilers
    REQUIRE(h.engine.setupAnswer("skip", err));  // throttle
    h.run(0.2);
    if (h.engine.setupActive()) REQUIRE(h.engine.setupAnswer("close", err));  // the results screen
    h.run(0.2);
    REQUIRE(!h.engine.setupActive());
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));  // it has a profile: not offered

    h.facts.engineRunning = false;  // at the gate, engines off
    h.run(0.1);
    REQUIRE(h.engine.setupRequest(err));
    h.run(0.1);
    CHECK(!h.engine.setupActive());  // not started: the checklist first
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("wait"));
    CHECK(h.setupStatus().getBool("requested", false));
    h.facts.engineRunning = true;
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("new"));
    h.aircraft.onGround = false;
    h.facts.gearRetractable = true;
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string("gear"));
    h.aircraft.onGround = true;
    h.run(0.1);
    REQUIRE(h.engine.setupStart(err));
    CHECK(!h.setupStatus().getBool("requested", true));
    REQUIRE(h.engine.setupLater(err));

    REQUIRE(h.engine.setupRequest(err));
    REQUIRE(h.engine.setupLater(err));
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
    REQUIRE(h.engine.setupRequest(err));
    h.engine.onFlightLoaded("test");
    h.run(0.1);
    CHECK_EQ(h.setupStatus().getString("offer", "?"), std::string(""));
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

TEST(EngineSetup_RecordingStopsWhileItRunsAndStartsAfresh) {
    EngineSetupHarness h;
    h.run(5.0);
    CHECK(h.engine.recording());
    CHECK(h.engine.bufferedFrames() > 100);
    std::string err;
    REQUIRE(h.engine.setupStart(err));
    h.run(0.1);
    CHECK(!h.engine.recording());
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("control setup"));
    const size_t before = h.engine.bufferedFrames();
    h.run(30.0, deploySpoilers(h.aircraft, 2.0));
    CHECK_EQ(h.engine.bufferedFrames(), before);  // the setup's tests aren't recorded
    REQUIRE(h.engine.setupAnswer("yes", err));
    REQUIRE(h.engine.setupAnswer("skip", err));  // throttle
    // Held above the runway for the gear: that isn't recorded either.
    CHECK_EQ(h.setupStatus().getString("phase", ""), std::string("hover"));
    h.run(1.0);
    CHECK(h.aircraft.frozen);
    CHECK(!h.engine.recording());
    CHECK_EQ(h.engine.bufferedFrames(), before);

    // Finished (here: the gear left for later), recording starts from scratch once the aircraft
    // is back on the runway, and detection arms again as after a teleport.
    REQUIRE(h.engine.setupAnswer("later", err));
    CHECK(h.engine.setupActive());
    CHECK(h.run(5.0, nullptr, "idle"));
    CHECK(!h.aircraft.frozen);
    CHECK(h.engine.bufferedFrames() <= 1);  // afresh, from the runway
    h.run(2.0);
    CHECK(h.engine.recording());
    CHECK(h.engine.bufferedSeconds() < 2.1);
    CHECK(!h.engine.detector().armed());

    // On the results screen it's finished: recording doesn't wait for "close".
    EngineSetupHarness g;
    g.facts.gearRetractable = false;
    g.run(1.0);
    REQUIRE(g.engine.setupStart(err));
    g.run(30.0, deploySpoilers(g.aircraft, 2.0));
    REQUIRE(g.engine.setupAnswer("yes", err));
    REQUIRE(g.engine.setupAnswer("skip", err));
    CHECK_EQ(g.setupStatus().getString("phase", ""), std::string("done"));
    CHECK(!g.setupStatus().getString("detail", "").empty());
    g.run(1.0);
    CHECK(g.engine.setupActive());
    CHECK(g.engine.recording());
    CHECK(g.engine.bufferedSeconds() < 1.1);
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

// A way that worked in a replay where the one before it didn't goes first in the aircraft's
// profile, logged and saved, so later replays (and a restart) try it first.
TEST(EngineSetup_AWayThatWorkedInAReplayGoesFirstInTheProfile) {
    g_setupLog.clear();
    MemStore storage;
    {
        ProfileStore store;
        ControlProfile& gp = store.upsert("simobjects/airplanes/pmdg 777")[Control::Gear];
        gp = {ControlState::Ok, kLever777, true};
        gp.ways[kDown] = {kGearKeys, kClick777};
        gp.ways[kUp] = {kLever777};
        std::string json;
        store.toJson(json);
        storage.files["aircraft.json"].assign(json.begin(), json.end());
    }
    ProfileSpy spy;
    Engine engine(storage);
    engine.setLogger(&testLog, nullptr);
    engine.setSimOutput(&spy);
    engine.init();
    engine.setAircraftKey("simobjects/airplanes/pmdg 777-300er/presets/pmdg/pmdg 777-300er/config");
    const int applied = spy.applied;
    CHECK(engine.promoteWay(Control::Gear, Direction::Extend, kClick777));
    CHECK(logged("control profile: the gear down with the aircraft's own event #69927 (536870912, then 131072) as a "
                 "toggle first from now on"));
    CHECK_EQ(spy.applied, applied + 1);
    REQUIRE(spy.last[Control::Gear].ways[kDown].size() == 2);
    CHECK(spy.last[Control::Gear].ways[kDown][0] == kClick777);
    CHECK(!engine.promoteWay(Control::Gear, Direction::Extend, kClick777));  // first already
    CHECK(!engine.promoteWay(Control::Gear, Direction::Retract, kGearKeys));  // not that way
    CHECK(!engine.promoteWay(Control::Flaps, Direction::Extend, kGearKeys));   // no list
    CHECK_EQ(spy.applied, applied + 1);

    Engine again(storage);
    again.init();
    again.setAircraftKey("simobjects/airplanes/pmdg 777-200lr/presets/pmdg/pmdg 777-200lr/config");
    REQUIRE(again.currentProfile());
    const ControlProfile& back = (*again.currentProfile())[Control::Gear];
    REQUIRE(back.ways[kDown].size() == 2);
    CHECK(back.ways[kDown][0] == kClick777);
    CHECK(back.ways[kDown][1] == kGearKeys);
}

namespace {

// A setup on the runway, the throttle skipped: the gear step, the aircraft held above the runway.
bool heldAboveTheRunway(EngineSetupHarness& h) {
    h.facts.hasSpoilers = false;
    h.run(0.2);
    std::string err;
    return h.engine.setupStart(err) && h.engine.setupAnswer("skip", err) && h.run(2.0, nullptr, "user") &&
           h.aircraft.frozen;
}

}  // namespace

// The whole gear step on the runway, through the Engine: held above it, taught, saved, put back,
// and recording starts again from there.
TEST(EngineSetup_GearSetUpAboveTheRunwayAndPutBack) {
    g_setupLog.clear();
    EngineSetupHarness h;
    REQUIRE(heldAboveTheRunway(h));
    CHECK(!h.engine.recording());
    CHECK_NEAR(h.aircraft.lifted, SetupFlow::kHoverClearFt, 1e-9);
    std::string err;
    CHECK(!h.engine.play("x.rpl", err));
    CHECK(h.run(150.0, cycleGear(h.aircraft), "ask"));
    REQUIRE(h.engine.setupAnswer("yes", err));
    CHECK(h.spy.last[Control::Gear].state == ControlState::Ok);  // saved at once
    CHECK(h.spy.last[Control::Gear].method == h.aircraft.gearWorks);
    CHECK(h.aircraft.frozen);
    CHECK(h.run(5.0, nullptr, "done"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(!h.aircraft.frozen);
    CHECK(h.aircraft.onGround);
    h.run(1.0);
    CHECK(h.engine.recording());
    CHECK(h.engine.bufferedSeconds() < 1.1);
    const JsonValue s = h.setupStatus();
    const JsonValue* controls = s.get("controls");
    REQUIRE(controls && controls->array.size() == 4);
    CHECK_EQ(controls->array[3].getString("state", ""), std::string("ok"));  // the gear
}

// A flight loaded meanwhile put the aircraft somewhere else: it's let go there, not put back.
TEST(EngineSetup_FlightLoadedAboveTheRunwayLetsTheAircraftGo) {
    g_setupLog.clear();
    EngineSetupHarness h;
    REQUIRE(heldAboveTheRunway(h));
    h.aircraft.setGearLever(0.0);
    h.run(8.0);
    h.engine.onFlightLoaded("test");
    CHECK(!h.engine.setupActive());
    CHECK(!h.aircraft.frozen);
    CHECK_EQ(h.aircraft.letGo, 1);
    CHECK_EQ(h.aircraft.putBacks, 0);
    CHECK(logged("letting the aircraft go where it is (flight loaded)"));

    EngineSetupHarness m;
    REQUIRE(heldAboveTheRunway(m));
    m.engine.setAtMainMenu(true);
    CHECK(!m.engine.setupActive());
    CHECK(!m.aircraft.frozen);
    CHECK_EQ(m.aircraft.letGo, 1);
}

// Another aircraft key mid-step (or the same aircraft identified by its path at last): the setup
// stops, and the aircraft goes back once its gear is down, the pilot asked to lower it.
TEST(EngineSetup_AircraftChangedAboveTheRunwayPutsItBackOnceTheGearIsDown) {
    g_setupLog.clear();
    EngineSetupHarness h;
    REQUIRE(heldAboveTheRunway(h));
    h.aircraft.setGearLever(0.0);
    h.run(8.0);
    REQUIRE(h.aircraft.gearPos < 0.5);
    h.engine.setAircraftKey("simobjects/airplanes/other_airliner");
    CHECK(h.engine.setupActive());
    CHECK(h.aircraft.frozen);
    CHECK(h.run(SetupFlow::kLandGearWaitSec + 1.0, nullptr, "lower"));
    CHECK(!h.engine.recording());
    CHECK_EQ(h.aircraft.putBacks, 0);
    h.aircraft.setGearLever(1.0);
    CHECK(h.run(15.0, nullptr, "idle"));
    CHECK_EQ(h.aircraft.putBacks, 1);
    CHECK(h.aircraft.gearWhenPutBack > 0.98);
    CHECK(!h.aircraft.frozen);
    CHECK(logged("control setup stopped (aircraft changed)"));
}
