// Guided control setup: teaches BetterReplay how to drive the spoilers, flaps, throttle and gear
// on a new aircraft.
//
// For each control the pilot moves it once (spoilers, flaps and throttle on the runway with the
// engines running; the gear after takeoff). Meanwhile every input event and L:var the aircraft
// has is watched, to find the ones that moved with the control whatever they're called. Then the
// movement is replayed with each candidate method, through the same code playback uses
// (IControlSim), and checked: did the spoilers / flaps / gear / engines do what they did when the
// pilot moved them? A method that passes is put to the pilot ("did the engines spool up and back
// down?"); a "no" tries the next. The result is the aircraft's ControlProfile.
//
// The throttle's lever simvar isn't the lever on every aircraft (on the A320neo V2 it's the
// autothrust's thrust demand). So the pilot's throttle movement holds the lever at about halfway
// and at full before going back to idle. If the simvar kept moving during those holds and the key
// events drive the throttle, the step goes on: the variable that held still with the lever is the
// lever's own, and the setup steps the throttle axis through its range and records where the
// lever went (LeverCalibration). Aircraft whose simvar held still skip all that.
#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "AircraftProfile.h"
#include "Clip.h"

namespace br {

class JsonWriter;

// Live aircraft state the setup needs besides the channels.
struct SetupFacts {
    bool onRunway = false;
    bool parkingBrake = false;
    bool engineRunning = false;  // any engine combusting
    int engineType = 1;          // ENGINE TYPE: 0 piston, 1 jet, 2 none, 3 helo turbine, 5 turboprop
    int engines = 1;
    bool gearRetractable = true;
    bool hasSpoilers = true;
    int flapPositions = 0;  // FLAPS NUM HANDLE POSITIONS (including up)
    bool hasReverse = false;  // THROTTLE LOWER LIMIT below zero: thrust reversers
};

// A variable the setup watches while the pilot moves a control.
struct ScanVar {
    std::string name;
    bool lvar = false;  // false: input event
};

// What the setup needs from the sim (the module implements it).
class IControlSim {
public:
    virtual ~IControlSim() = default;
    // Lists every numeric input event and L:var the aircraft has; scanRead then reads their
    // values in that order (NaN where unreadable).
    virtual void scanBegin(std::vector<ScanVar>& vars) = 0;
    virtual void scanRead(std::vector<double>& values) = 0;
    virtual void scanEnd() = 0;
    // Drives a control with a method, from a frame's channels (methodSourceChannel): the same
    // code playback uses for an aircraft's profile. `force` resends unchanged values.
    virtual bool beginControl(Control c, const ControlMethod& m, std::string& why) = 0;
    virtual void applyControl(Control c, const Frame& f, bool force) = 0;
    virtual void endControl(Control c) = 0;
    // For the throttle lever's calibration: one variable's value now (NaN if unreadable), and the
    // thrust reversers (the reverse thrust on/off key events).
    virtual double readVar(const ScanVar& v) = 0;
    virtual void setReverser(bool on) = 0;
};

// Watches every scanned variable while the pilot moves a control and finds the ones that moved
// with it: steady before the movement, changing during it, back where they were at the end, and
// correlated with the control's own simvar. Only variables that change get a stored series.
class CandidateTracker {
public:
    static constexpr size_t kMaxStored = 400;

    void begin(std::vector<ScanVar> vars);
    void clear();
    // One scan at time t (seconds), with the control's two reference signals at that moment:
    // its handle (simvar) and its outcome (surface position, N1...).
    void sample(double t, const std::vector<double>& values, double handle, double outcome);
    // Marks variables that changed before time t as noise (timers, systems settling) and drops
    // their series.
    void dropChangedBefore(double t);

    struct Found {
        size_t index;
        double score;  // |correlation with the reference|, plus a bonus for a fitting name
        double peak;   // largest |value| between from and to
    };
    // Variables (input events, or L:vars) that moved with the reference (the handle, or the
    // outcome if `byOutcome`) between from and to, best first. `words` (null-terminated) are
    // name fragments that earn a bonus.
    std::vector<Found> find(double from, double to, bool lvars, bool byOutcome, const char* const* words,
                            size_t max) const;
    // Value of variable `index` at time t (interpolated; its value before it first changed).
    double valueAt(size_t index, double t) const;
    bool lookup(const std::string& name, bool lvar, size_t& index) const;
    // It changed during the scan and isn't noise.
    bool changed(size_t index) const;
    double firstChange(size_t index) const;  // time of its first change, NaN if none

    // How the handle (the control's simvar) behaved while the pilot's hand was still, between
    // from and to. A lever-like variable (one of `candidates` that took many values) holding
    // still for `holdSec` is the hand being still; `creep` is the most the handle moved during
    // such a hold (its first `settleSec` left out), and `steady` whether the handle itself held
    // within `tol` for `holdSec` at least `away` from `rest`.
    struct HoldCheck {
        double creep = 0.0;
        size_t by = 0;  // the variable that held still while the handle crept
        bool steady = false;
    };
    HoldCheck holdCheck(double from, double to, const std::vector<size_t>& candidates, double rest, double away,
                        double tol, double holdSec, double settleSec) const;
    // The variable's holds between from and to: stretches of at least `holdSec` within
    // `stillFraction` of its range. `levels` counts the distinct values held, `stillSec` their
    // total length.
    struct Holds {
        int levels = 0;
        double stillSec = 0.0;
        double range = 0.0;  // how far it moved
        int values = 0;      // distinct values it took (a lever passes through many; an index jumps)
    };
    Holds holds(size_t index, double from, double to, double holdSec, double stillFraction) const;

    const ScanVar& var(size_t index) const { return vars_[index]; }
    size_t size() const { return vars_.size(); }
    size_t stored() const { return stored_; }
    size_t samples() const { return times_.size(); }

private:
    struct Track {
        double base = NAN;          // value at the first readable sample
        long first = -1;            // sample index of the first change (-1: none)
        bool noise = false;
        std::vector<float> series;  // values from `first` on
    };
    double sampleValue(const Track& tr, size_t s) const;

    std::vector<ScanVar> vars_;
    std::vector<Track> tracks_;
    std::vector<double> times_;
    std::vector<double> refHandle_;
    std::vector<double> refOutcome_;
    std::unordered_map<std::string, size_t> byName_;
    size_t stored_ = 0;
};

// Whether a clip carries what a profile's method needs: the channel it reads, recorded from
// the same aircraft variable. Clips recorded before a setup, or on another aircraft, may not.
bool clipSupports(Control c, const ControlMethod& m, const Clip& clip, int engines, std::string* why);
// Whether a clip recorded the throttle lever's variable (lv_thr_N) that the calibration is for.
bool clipHasLever(const LeverCalibration& cal, const Clip& clip, int engines, std::string* why);

class SetupFlow {
public:
    using LogFn = void (*)(void* ctx, int level, const char* message);  // 0 info, 1 warn, 2 error

    void setLogger(LogFn fn, void* ctx);
    void setSim(IControlSim* sim) { sim_ = sim; }

    // Starts on a copy of `profile`. On the ground it teaches every control the aircraft has
    // (the gear after takeoff); in the air only the gear.
    bool start(const AircraftProfile& profile, const Frame& live, const SetupFacts& facts, std::string& error);
    void cancel(const char* why);
    // The pilot's answer: yes, no, again, skip, retry, ready (continue despite a precondition),
    // begin (the throttle sweep), later, cancel, close.
    bool answer(const std::string& a, std::string& error);
    bool active() const { return phase_ != Phase::Idle; }

    // Every sim frame while active. dt = 0 while the sim is paused.
    void onFrame(double dt, const Frame& live, const SetupFacts& facts);

    const AircraftProfile& profile() const { return profile_; }
    bool takeChanged();  // true once after a step changed the profile (time to save it)

    // Writes the active setup's fields into the current JSON object: active, step, stepNo,
    // steps, phase, title, prompt (one short instruction or question), detail (what it's for,
    // in the throttle's holds and lever phases), task / tasks / taskDone (the pilot's instructions
    // in this step), holdTask / hold (the instruction is a throttle hold; how far it is, 0..1),
    // note, method, methods, attempts (how each method before the current one ended),
    // progress (the throttle sweep, 0..1), plan (each step's control and its result, "current"
    // or "next"), buttons.
    void statusJson(JsonWriter& w) const;

    // Tuning, per control (exposed for tests).
    struct Tuning {
        double handleStart;  // the control's simvar moved this far: the pilot started
        double handleNeed;   // and this far: a full enough movement
        double outcomeStart;
        double outcomeNeed;
        double holdMaxSec;   // longest wait after a replay for things to settle
        double waitMaxSec;   // longest wait for the pilot
    };
    Tuning tuning(Control c) const;

private:
    // After Done: the throttle lever's phases, for aircraft whose lever simvar isn't the lever.
    // Query asks whether it is when the pilot's movement didn't tell; Positions repeats the
    // movement's holds when they didn't show the lever's variable; Notice warns before the Sweep
    // steps the axis through its range.
    enum class Phase { Idle, Prepare, Takeoff, User, Retry, Replay, Hold, Ask, Done, Query, Positions, Notice, Sweep };
    enum class LeverVerdict { Steady, Creeps, Unclear };  // the lever simvar while the hand was still

    Control control() const { return plan_[step_]; }
    void enterStep();
    void beginUser();
    void userFrame(double dt, const Frame& live);
    void scanFrame(double h, double o);  // a capture-clock scan, every kScanInterval
    void resetHolds();
    void trackHolds(double h, double dh, bool enough);  // the throttle's holds (task_ 0 and 1)
    void analyze();
    LeverVerdict checkLever();
    void confirmMethod();  // the pilot said yes to the method being tried
    void leverFromMovement();
    void beginPositions(const char* note);
    void positionsFrame(double dt, const Frame& live);
    bool findLever();
    void enterNotice();
    void beginSweep();
    void sweepFrame(double dt, const Frame& live);
    void sweepTo(double axis, bool force);
    void finishSweep();
    void stopSweep(const std::string& why);
    void parkThrottle();  // the sweep's throttle back to idle, reverser stowed
    std::string leverPattern(const std::string& name) const;
    double buildMethods();  // returns when the variables they use first changed
    std::string enginePattern(const std::string& name, bool lvar) const;
    void startMethod(size_t i);
    void replayFrame(double dt, const Frame& live);
    void holdFrame(const Frame& live);
    void track(const Frame& live);
    bool rolling(const Frame& live) const;
    void evaluate(const Frame& live);
    void nextMethod();
    void finishStep(ControlState state, const ControlMethod& method, bool confirmed);
    void endScan();
    void endDrive();
    const char* precondition(const Frame& live) const;  // what's missing, or null
    double handleSignal(const Frame& f) const;
    double outcomeSignal(const Frame& f) const;
    std::string prompt() const;
    void logf(int level, const char* fmt, ...) const;

    IControlSim* sim_ = nullptr;
    LogFn log_ = nullptr;
    void* logCtx_ = nullptr;

    AircraftProfile profile_;
    bool changed_ = false;
    SetupFacts facts_;
    bool onGround_ = true;
    std::vector<Control> plan_;
    size_t step_ = 0;
    Phase phase_ = Phase::Idle;
    double phaseT_ = 0.0;  // time in the current phase
    std::string need_;     // Prepare: what's missing ("Set the parking brake")
    std::string note_;     // outcome of the previous method or step
    bool scanning_ = false;
    bool driving_ = false;

    // the pilot's movement
    double t_ = 0.0;  // capture clock
    double nextCapture_ = 0.0;
    double nextScan_ = 0.0;
    std::vector<Frame> capture_;  // 30 Hz, t on the capture clock
    CandidateTracker tracker_;
    std::vector<ScanVar> scanVars_;
    std::vector<double> scanValues_;
    double handleRest_ = NAN;
    double outcomeRest_ = NAN;
    double handlePeak_ = 0.0;
    double outcomePeak_ = 0.0;
    double handleAnchor_ = NAN;
    double outcomeAnchor_ = NAN;
    double stableSince_ = 0.0;
    double handleStill_ = NAN;  // where the handle / outcome last stopped, and since when
    double outcomeStill_ = NAN;
    double handleStillSince_ = 0.0;
    double outcomeStillSince_ = 0.0;
    size_t task_ = 0;        // the pilot's instruction showing ("Deploy the spoilers", "Retract...")
    bool taskDone_ = false;  // the last one is done; waiting for things to settle
    static constexpr size_t kHoldPast = 10;
    double holdPast_[kHoldPast] = {};  // the throttle's holds: the simvar over the last second (a ring)
    size_t holdSamples_ = 0;
    double holdNext_ = 0.0;
    double holdSince_ = 0.0;    // since when it's been moving slowly enough to be held
    double holdLevel_ = NAN;    // where the last hold was, and when it was done
    double holdDoneAt_ = 0.0;
    double hold_ = 0.0;         // how far the hold showing is (0..1)
    bool moved_ = false;
    double movedAt_ = 0.0;
    double windowFrom_ = 0.0;
    bool judgeOutcome_ = true;  // check replays on the outcome (surfaces, N1), else on the handle
    double userPeak_ = 0.0;     // the pilot's movement on that signal

    // replay
    Clip mini_;  // the pilot's movement, t from 0
    std::vector<ControlMethod> methods_;
    size_t method_ = 0;
    std::vector<std::string> attempts_;  // how each method before method_ ended (statusJson)
    double replayT_ = 0.0;
    size_t hint_ = 0;
    Frame frame_;
    bool firstApply_ = false;
    double replayPeak_ = 0.0;
    double replayAnchor_ = NAN;
    double replayStable_ = 0.0;
    bool stoppedEarly_ = false;

    // the throttle lever (aircraft whose lever simvar isn't the lever)
    AircraftProfile before_;  // the profile when the run started: a skipped step keeps its result
    LeverVerdict leverVerdict_ = LeverVerdict::Steady;
    bool leverStage_ = false;  // the throttle's method is confirmed; now its lever
    ControlMethod chosen_;     // that method
    LeverCalibration lever_;   // what the lever stage found, stored with the step
    double leverIdle_ = NAN;   // the lever variable at idle (the end of the holds)
    double leverSpan_ = 0.0;   // and how far it moved there
    struct SweepMove {
        double axis;  // 0..1
        int record;   // 0: just get there, 1: a forward reading, 2: a reverse one
        int reverser; // -1 as it is, 0 stow, 1 engage (then a longer wait)
    };
    std::vector<SweepMove> sweep_;
    size_t sweepAt_ = 0;
    bool sweepExtended_ = false;  // the reverse pass and the way back are planned
    double sweepHeld_ = 0.0;      // time at this move
    double sweepLast_ = NAN;      // the lever's last reading, and how long it's been unchanged
    double sweepSteady_ = 0.0;
    Frame sweepFrame_;
    bool reverserOn_ = false;
    std::vector<LeverPoint> sweepForward_, sweepReverse_;
};

}  // namespace br
