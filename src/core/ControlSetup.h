// Guided control setup: teaches BetterReplay how to drive the spoilers, flaps, throttle and gear
// on a new aircraft.
//
// For each control the pilot moves it once (spoilers, flaps and throttle on the runway with the
// engines running; the gear in the air). Meanwhile every input event and L:var the aircraft
// has is watched, to find the ones that moved with the control whatever they're called, and so is
// every key event that fires (a cockpit click, a binding: the aircraft's own events, which no
// variable shows). Then the movement is replayed with each candidate method, through the same
// code playback uses (IControlSim), and checked: did the spoilers / flaps / gear / engines do what
// they did when the pilot moved them? A method that passes is put to the pilot ("did the engines
// spool up and back down?"); a "no" tries the next. The result is the aircraft's ControlProfile.
//
// The gear needs no takeoff. On the runway the setup holds the aircraft frozen a little above
// where it stands (IControlSim::beginHover), checks the sim counts it as airborne there, and
// teaches the gear as in flight. When the step ends, however it ends, the gear has to be down
// before the aircraft is put back: the setup waits for it, then asks the pilot to lower it. Where
// the aircraft still counts as on the ground up there, or its gear doesn't move, the step waits
// for a takeoff instead.
//
// How a control comes back can depend on how it got there: the PMDG 777's gear key events lowered
// its gear after its lever event had raised it, but after the pilot had, only a click on the lever
// did. So for the gear, flaps and spoilers each way is tested on its own as well. After the
// pilot's movement the pilot moves the control away once more ("Raise the gear"), and the way back
// is replayed from there with each candidate until one brings it back. Then the whole movement
// is replayed with each candidate as above; one that moves the control away but doesn't bring it
// back gets the way back replayed from where it left it, with each of the others. The profile
// keeps a list of ways for each direction where one method doesn't do it all (ControlProfile::ways).
//
// The throttle's lever simvar isn't the lever on every aircraft (on the A320neo V2 it's the
// autothrust's thrust demand). So the pilot's throttle movement holds the lever at about halfway
// and at full before going back to idle. If the simvar kept moving during those holds and the key
// events drive the throttle, the step goes on: the variable that held still with the lever is the
// lever's own, and the setup steps the throttle axis through its range and records where the
// lever went (LeverCalibration). With reversers it then finds how to command reverse thrust: the
// reverse events (then the axis sets the amount), "toggle reverse" (likewise; the A350), or
// "decrease throttle" held at idle (the PMDG 737), checking that the reversers deployed.
// Aircraft whose simvar held still skip the calibration, but not the reverse test.
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

// A key event the sim saw while the pilot moved a control: a cockpit click or one of the pilot's
// bindings (a sim key event, or from kThirdPartyEventFirst an aircraft's own).
struct SeenKeyEvent {
    uint32_t id = 0;
    double value = NAN;  // its first parameter (a signed integer), NaN without one
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
    // The key events the sim saw since scanBegin or the last call, oldest first, without those
    // BetterReplay sent itself (the setup's replays, playback).
    virtual void scanKeyEvents(std::vector<SeenKeyEvent>& events) = 0;
    // Drives a control with a method, from a frame's channels (methodSourceChannel): the same
    // code playback uses for an aircraft's profile. `force` resends unchanged values.
    virtual bool beginControl(Control c, const ControlMethod& m, std::string& why) = 0;
    virtual void applyControl(Control c, const Frame& f, bool force) = 0;
    virtual void endControl(Control c) = 0;
    // For the throttle lever's calibration: one variable's value now (NaN if unreadable), the
    // thrust reversers (the reverse thrust on/off key events), one THROTTLE_DECR, what a held
    // "decrease throttle" key sends every frame (the PMDG 737 goes into reverse while it's held),
    // and THROTTLE_REVERSE_THRUST_TOGGLE ("toggle reverse"), which switches every engine in or out
    // of reverse (the A350, which ignores the reverse events).
    virtual double readVar(const ScanVar& v) = 0;
    virtual void setReverser(bool on) = 0;
    virtual void throttleDecrease() = 0;
    virtual void toggleReverse() = 0;
    // The gear step on the ground: beginHover snapshots the aircraft, puts it `liftFt` straight up
    // from where it stands (heading and attitude kept, velocities zero) and freezes it, holding it
    // there until endHover. endHover puts the snapshot back and unfreezes it; with `restore` false
    // (the sim reset the aircraft: a flight loaded) it only unfreezes it. beginHover is false, with
    // the reason, if it can't (a replay is playing).
    virtual bool beginHover(double liftFt, std::string& why) = 0;
    virtual void endHover(bool restore) = 0;
    virtual bool hovering() const = 0;
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
    // within `tol` for `holdSec` at least `away` from `rest`. A variable that held still while the
    // handle moved more than `maxCreep` wasn't following the hand at all, so that doesn't count.
    struct HoldCheck {
        double creep = 0.0;
        size_t by = 0;  // the variable that held still while the handle crept
        bool steady = false;
    };
    HoldCheck holdCheck(double from, double to, const std::vector<size_t>& candidates, double rest, double away,
                        double tol, double holdSec, double settleSec, double maxCreep = INFINITY) const;
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
    // How long variable `index` took, between from and to, from leaving where it was to (nearly)
    // as far as it went: a lever moves in a moment, what follows the control (the gear legs'
    // animations) as long as the control does. referenceTravelSec: the same for the outcome
    // reference. NaN if it didn't move.
    double travelSec(size_t index, double from, double to) const;
    double referenceTravelSec(double from, double to) const;

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

// The key events the sim saw while the pilot moved a control, by event. Firings before the
// movement's window are only counted: an event that fired kSteadyBefore times there fires steadily
// (noise, like a variable that changed there), and from then on its firings aren't kept either, so
// an aircraft that keeps firing its own events can't crowd out the lever's. The rest are kept with
// their time and parameter.
class KeyEventTracker {
public:
    static constexpr size_t kMaxStored = 6000;
    static constexpr size_t kSteadyBefore = 2;
    static constexpr size_t kValues = 4;  // parameter values listed for the log

    struct Firing {
        double t;
        double value;
    };
    struct Event {
        uint32_t id = 0;
        size_t count = 0;   // every firing seen
        size_t before = 0;  // dropped by dropBefore
        std::vector<Firing> firings;
        bool steady() const { return before >= kSteadyBefore; }
        // Every firing's parameter, for the log: the first kValues values with their counts, and
        // the range (`many`: more values than that).
        std::vector<std::pair<double, size_t>> values;
        bool many = false;
        double lo = INFINITY, hi = -INFINITY;
    };
    void clear();
    void add(double t, uint32_t id, double value);
    void dropBefore(double t);
    const std::vector<Event>& events() const { return events_; }
    size_t stored() const { return stored_; }
    size_t lost() const { return lost_; }  // firings not kept for want of room

private:
    std::vector<Event> events_;
    size_t stored_ = 0;
    size_t lost_ = 0;
};

// Whether a clip carries what a profile's method needs: the channel it reads, recorded from
// the same aircraft variable. Clips recorded before a setup, or on another aircraft, may not.
bool clipSupports(Control c, const ControlMethod& m, const Clip& clip, int engines, std::string* why);
// The profile as a clip can replay it: each direction's ways without those the clip doesn't
// support (`left` counts them). False, with the reason, if a direction has none left, or (without
// ways) the method isn't supported.
bool clipWays(Control c, const ControlProfile& cp, const Clip& clip, int engines, ControlProfile& out, size_t& left,
              std::string* why);
// Whether a clip recorded the throttle lever's variable (lv_thr_N) that the calibration is for.
bool clipHasLever(const LeverCalibration& cal, const Clip& clip, int engines, std::string* why);

class SetupFlow {
public:
    using LogFn = void (*)(void* ctx, int level, const char* message);  // 0 info, 1 warn, 2 error

    void setLogger(LogFn fn, void* ctx);
    void setSim(IControlSim* sim) { sim_ = sim; }

    // Starts on a copy of `profile`. On the ground it teaches every control the aircraft has
    // (the gear last, held above the runway); in the air only the gear.
    bool start(const AircraftProfile& profile, const Frame& live, const SetupFacts& facts, std::string& error);
    // Stops. An aircraft held above the runway is put back first, once its gear is down (the
    // pilot is asked to lower it if it isn't within kLandGearWaitSec), so the setup stays active
    // until then. Cancelled while the pilot is asked, it's put back as it is.
    void cancel(const char* why);
    // Stops at once: the sim reset the aircraft (a flight loaded, the main menu). An aircraft held
    // above the runway is let go where the sim has it now, not put back.
    void abandon(const char* why);
    // The pilot's answer: yes, no, again, skip, retry, ready (continue despite a precondition),
    // begin (the throttle sweep), later, cancel, close.
    bool answer(const std::string& a, std::string& error);
    bool active() const { return phase_ != Phase::Idle; }
    // Active and not yet on its results screen: it's teaching or testing controls (holding the
    // aircraft above the runway, or waiting for a takeoff, for the gear). Recording stops meanwhile.
    bool running() const { return phase_ != Phase::Idle && phase_ != Phase::Done; }

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

    // The gear step on the ground (exposed for tests). The aircraft goes up by its height above the
    // ground where it stands (about as long as its gear) plus kHoverClearFt, or by kHoverLiftFt where
    // that isn't known (hoverLift), and must count as airborne there within kHoverAirborneSec. When
    // the step ends the gear gets kLandGearWaitSec to be down before the pilot is asked to lower it.
    static constexpr double kHoverClearFt = 30.0;
    static constexpr double kHoverLiftFt = 50.0;
    static constexpr double kHoverAirborneSec = 5.0;
    static constexpr double kLandGearWaitSec = 30.0;
    static double hoverLift(const Frame& live);

private:
    // After Done: the throttle lever's phases, for aircraft whose lever simvar isn't the lever.
    // Query asks whether it is when the pilot's movement didn't tell; Positions repeats the
    // movement's holds when they didn't show the lever's variable; Notice warns before the Sweep
    // steps the axis through its range. Away: the pilot moves the control away once more; Back:
    // the way back replayed from there (or from where a method's replay left it). Hover: the gear
    // step on the ground, lifting the aircraft above the runway; Land: waiting for its gear to be
    // down, putting it back and waiting for the sim to show it there; Lower: the pilot is asked to
    // lower the gear.
    enum class Phase {
        Idle, Prepare, Takeoff, User, Retry, Replay, Hold, Ask, Done, Query, Positions, Notice, Sweep, Away, Back,
        Hover, Land, Lower
    };
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
    void reverseOnly();
    void leverFromMovement();
    void beginPositions(const char* note);
    void positionsFrame(double dt, const Frame& live);
    bool findLever();
    void enterNotice();
    void beginSweep();
    void planReverse();
    void planToggle();
    void sweepFrame(double dt, const Frame& live);
    void sweepTo(double axis, bool force);
    void finishSweep();
    void stopSweep(const std::string& why);
    void parkThrottle();  // the sweep's throttle back to idle, reverser stowed
    std::string leverPattern(const std::string& name) const;
    double buildMethods();  // returns when the variables they use first changed
    void logKeyEvents() const;  // what fired while the pilot moved the control
    // KeyEvent methods with the events that fired with the movement; `first` gets the earliest
    // firing they replay.
    std::vector<ControlMethod> learnEvents(double& first);
    std::vector<double> eventSeries() const;  // eventTarget over capture_ (the gear from its legs)
    std::string enginePattern(const std::string& name, bool lvar) const;
    void startMethod(size_t i);
    void prepareMethod(const ControlMethod& m);  // fills in the channels it reads in mini_
    // Each way on its own (not the throttle). Away: the pilot moves the control away once more;
    // Back: the way back replayed from there, or from where method_'s replay left it, with each
    // method in turn until one brings it back.
    enum class BackFrom : uint8_t { Pilot, Replay };
    void beginAway();
    void awayFrame(const Frame& live);
    void startBack(BackFrom from);
    void tryBack();
    void backFrame(double dt, const Frame& live);
    void endBack(bool ok, const char* why);
    void startReplays();  // the whole movement, method by method
    void replayFrame(double dt, const Frame& live);
    void holdFrame(const Frame& live);
    void track(const Frame& live);
    double restTol() const;  // how near where the pilot started the control counts as there
    bool atRest(const Frame& live) const;
    // The gear's replay, for the log: each command (0 up, 1 down) and where the gear went until the next.
    void noteGearCommand(double command, const Frame& live);
    void logGearCommand() const;
    bool rolling(const Frame& live) const;
    void evaluate(const Frame& live);
    void nextMethod();
    void finishStep(ControlState state, const ControlMethod& method, bool confirmed);
    // The gear step on the ground. LandThen: what follows putting the aircraft back: the next
    // step, waiting for a takeoff (the hover didn't work), or stopping.
    enum class LandThen : uint8_t { Next, Takeoff, Stop };
    bool hovering() const;
    void beginHover();
    void hoverFrame(double dt, const Frame& live);
    void leaveHover(const std::string& why);  // the gear after takeoff instead
    void beginLand(LandThen then);
    void landFrame(double dt, const Frame& live);
    void putBack();
    void afterLand();
    void enterTakeoff();
    bool gearDown(const Frame& live) const;  // at least as far down as where the aircraft stood
    void clearStep();  // the step's movement, methods and scan
    void stop(const std::string& why);
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
    KeyEventTracker keys_;
    std::vector<SeenKeyEvent> keysSeen_;
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
    std::vector<double> keyColumn_;  // its KeyEvent channel as built (a learned position overwrites it)
    struct LearnedEvent {
        ControlMethod method;
        std::vector<KeyEventTracker::Firing> firings;  // a position's: what its replay sends, and when
    };
    std::vector<LearnedEvent> learned_;
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
    bool started_ = false;   // the control was back where the pilot started it (or the wait ran out)
    bool seenRest_ = false;  // it has been there during this replay: movement counts from then on
    double gearCommand_ = NAN, gearCommandAt_ = 0.0, gearFrom_ = NAN, gearTo_ = NAN;

    // each way on its own
    BackFrom backFrom_ = BackFrom::Pilot;
    std::vector<size_t> backTries_;  // the methods to try the way back with, in order
    size_t backAt_ = 0;
    double backFromDist_ = NAN;  // how far from where the pilot started it the control was then
    bool backMoved_ = false;     // it has come some way back since
    int pilotBack_ = -1;   // the method that brought the control back from where the pilot left it
    int replayBack_ = -1;  // the one that brought it back from where method_'s replay left it, when that didn't
    std::vector<ControlMethod> ways_[kDirections];  // what confirmMethod found, for finishStep

    // the gear step on the ground
    LandThen landThen_ = LandThen::Next;
    std::string landWhy_;      // LandThen::Stop: why the setup stops (logged once it's back)
    double hoverGear_[3] = {NAN, NAN, NAN};  // the gear where the aircraft stood, and its handle simvar
    double hoverHandle_ = NAN;
    double hoverAirSec_ = 0.0;      // Hover: how long the sim has shown it off the ground
    double gearHandleRest_ = NAN;   // User, held up there: the gear handle simvar at the start, and
    double gearHandleUpAt_ = NAN;   // when it went up with the gear still down
    double landDownSec_ = 0.0;      // Land: how long the gear has been down (and where it was last)
    double landGear_ = NAN;
    bool landPut_ = false;          // and it's been put back:
    double landGroundSec_ = 0.0;    // how long the sim has shown it on the ground since
    bool groundSeen_ = true;        // Takeoff: the sim has shown the aircraft on the ground (not at once after a hover)

    // the throttle lever (aircraft whose lever simvar isn't the lever)
    AircraftProfile before_;  // the profile when the run started: a skipped step keeps its result
    LeverVerdict leverVerdict_ = LeverVerdict::Steady;
    bool leverStage_ = false;  // the throttle's method is confirmed; now its lever
    bool reverseOnly_ = false; // and it needs no lever calibration: only reverse thrust is tried
    ControlMethod chosen_;     // that method
    LeverCalibration lever_;   // what the lever stage found, stored with the step
    ReverseBy reverseBy_ = ReverseBy::Unknown;  // and how reverse thrust is commanded
    double leverIdle_ = NAN;   // the lever variable at idle (the end of the holds)
    double leverSpan_ = 0.0;   // and how far it moved there
    struct SweepMove {
        double axis;  // 0..1
        int record;   // 0: just get there, 1: a forward reading, 2: a reverse one, 3: hold
                      // "decrease throttle", 4: let go and wait for the reversers to stow, 5: a
                      // reading after toggling reverse, 6: check it's out of reverse again (a
                      // little forward thrust), 7: at idle until the throttle simvar is back there
                      // too (before a toggle)
        int reverser; // -1 as it is, 0 stow, 1 engage, 2 toggle reverse (each then a longer wait)
    };
    std::vector<SweepMove> sweep_;
    size_t sweepAt_ = 0;
    bool sweepExtended_ = false;  // the reverse test and the way back are planned
    bool reverseZone_ = false;    // the axis has its own reverse zone
    bool reverseSeen_ = false;    // the reverse events deployed the reversers
    bool holdTried_ = false;      // "decrease throttle" held at idle: tried, and it deployed them
    bool holdSeen_ = false;
    bool toggleTried_ = false;    // "toggle reverse": tried, it deployed them, and the axis set the amount
    bool toggleSeen_ = false;
    bool toggleFollows_ = false;
    bool toggledOn_ = false;      // our toggles have the aircraft in reverse
    int toggleAgain_ = 0;         // extra toggles to get it out of reverse
    double toggleIdle_ = NAN;     // the throttle simvar at reverse idle, and the furthest into reverse
    double toggleDeep_ = NAN;
    double sweepHeld_ = 0.0;      // time at this move
    double sweepLast_ = NAN;      // the lever's last reading, and how long it's been unchanged
    double sweepSteady_ = 0.0;
    Frame sweepFrame_;
    bool reverserOn_ = false;
    std::vector<LeverPoint> sweepForward_, sweepReverse_;
};

}  // namespace br
