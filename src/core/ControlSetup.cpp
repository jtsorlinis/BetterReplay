#include "ControlSetup.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <iterator>

#include "Interp.h"
#include "Json.h"
#include "PlaybackController.h"

namespace br {

namespace {

constexpr double kCaptureInterval = 1.0 / 30.0;
constexpr double kScanInterval = 0.1;
constexpr double kStableSec = 1.5;       // the controls must sit still this long to count as done
constexpr double kLeadInSec = 0.5;  // the replay starts this long before the first change
// A control's own variables can change a few seconds before its simvar shows the movement (the
// gear legs only start moving once the doors are open), so variables count as noise only if they
// changed more than this long before the movement was seen.
constexpr double kSearchLeadSec = 4.0;
constexpr double kPreMotionKeepSec = 5.0;
constexpr size_t kMaxMethods = 8;
constexpr double kPassPeak = 0.6;    // a replay must move the control at least 60% as far as the pilot did
constexpr double kPassReturn = 0.15; // and bring it back to within 15% of that
// One that passed but went less than 90% as far is asked about only once the others have been
// tried, if none went further: the A380's SPOILERS_SET went 76%, its spoilers half out.
constexpr double kPassFull = 0.9;
// The spoiler calibration (SetupFlow::Phase::Calibrate): the event's value from none to full in
// kCalSteps steps, each held until the handle has been still kCalStillSec (kCalMinSec at least,
// kCalMaxSec at most), then back to where the pilot started them (kCalBackMaxSec at most). One
// that puts the handle within kCalSameTol of where it's sent wouldn't change the replay.
constexpr int kCalSteps = 20;
constexpr double kCalMinSec = 0.3;
constexpr double kCalStillSec = 0.3;
constexpr double kCalMaxSec = 3.0;
constexpr double kCalStillTol = 0.002;
constexpr double kCalBackMaxSec = 6.0;
constexpr double kCalSameTol = 0.03;
constexpr double kMaxRollKt = 3.0;   // the throttle replay stops if the aircraft starts to roll
constexpr double kStopKt = 2.0;      // stopped, for the steps on the runway
constexpr double kTaskStillSec = 0.75;  // a control held this long has finished that instruction
// The gear: a variable that took this long to travel, or this share of the time the legs took
// (where they took that long), follows them (a leg's animation, not the lever).
constexpr double kLegFollowShare = 0.7;
constexpr double kLegTravelMinSec = 2.0;
// Before each replay the control goes back to where the pilot's movement started it (the last
// method may have left it elsewhere: the PMDG 777's lever event raised the gear but didn't lower
// it), for up to this long; only movement from there counts.
constexpr double kRestWaitSec = 30.0;
// Key events seen while the pilot moved the control (those that fired steadily before it are
// noise: KeyEventTracker::kSteadyBefore). A position's value is where the control was
// kKeySettleSec after it fired (or when it fired again); a click's effect is the control's first
// move within kKeyFollowSec. A toggle or a step that fired more often than kKeyMaxSteps isn't one,
// and a position's parameter must follow the control's value (|correlation| kKeyPositionFit). At
// most kMaxLearned of them are tried.
constexpr double kKeySettleSec = 5.0;
constexpr double kKeyFollowSec = 5.0;
constexpr size_t kKeyMaxSteps = 16;
constexpr double kKeyPositionFit = 0.9;
constexpr size_t kMaxLearned = 2;
constexpr size_t kKeyLogKinds = 12;  // the log lists the most frequent ones

// The throttle lever. A lever-like variable (one that took this many values while the pilot
// moved the throttle) holding within 0.5% of its range for 2 s is the pilot's hand being still;
// what it drives gets the first 0.75 s to catch up. If the lever simvar still moves 0.5% after
// that, it isn't the lever.
constexpr int kLeverMinValues = 8;
constexpr double kLeverStillFraction = 0.005;
constexpr double kLeverHoldSec = 2.0;
constexpr double kLeverSettleSec = 0.75;
constexpr double kLeverCreepPct = 0.5;
constexpr double kLeverMaxCreep = 0.5;  // of the movement: more, and the still variable wasn't the lever
constexpr double kLeverFullPct = 95.0;  // the simvar at full; a thrust demand reads less (TOGA ~85)
constexpr int kLeverLevels = 3;           // the lever variable holds at idle, halfway and full
constexpr double kLeverPositionSec = 1.5; // the lever variable holding this long is a position
// The pilot's throttle holds ("about halfway", then "full"), seen on the lever simvar: moving less
// than 2% a second for 2 s. A hand moves it much faster; where the simvar isn't the lever it lags
// the lever and may creep, more slowly. Either way the lever itself has held at least a second
// longer than that. Or the same on the throttle's input events, the cockpit lever in percent (the
// A350's 0..100): the A380's simvar held at halfway but never at full, a minute of it.
constexpr double kHoldSec = 2.0;
constexpr double kHoldTolPct = 2.0;     // per second
constexpr double kHoldSampleSec = 0.1;  // the simvar's last second (SetupFlow::kHoldPast samples)
constexpr double kHoldLogSec = 15.0;    // waiting this long for a hold, the log says why
// The sweep: the axis in 40 steps (2.5% each), the reverse amount in 10; each held until the
// lever has stopped (at least 0.2 s, at most 1 s; 1.5 s after the reverser is switched).
constexpr int kSweepSteps = 40;
constexpr int kSweepReverseSteps = 10;
constexpr double kSweepMinHoldSec = 0.2;
constexpr double kSweepSteadySec = 0.1;
constexpr double kSweepMaxHoldSec = 1.0;
constexpr double kSweepReverserSec = 1.5;
constexpr double kSweepMaxSec = 150.0;
// Reverse: the reverse events, the axis then stepping the amount; an aircraft they don't reverse
// (the PMDG 737 moves its forward levers instead) is let off at half the axis. If they didn't
// work, "toggle reverse" the same way, once the throttle simvar is back at idle (at most 6 s: the
// A350's lags its lever by seconds), each step at least 0.5 s, until the reversers are out and
// the simvar has gone 5% further into reverse than at reverse idle. Then back to idle until the
// reverse amount is gone, toggled back, and checked with a little forward thrust (at most 3 s):
// still in reverse, it's toggled again, twice at most. If the toggle didn't work either,
// "decrease throttle" held at idle until the reversers deploy (at most 4 s), and let go: 6 s for
// them to stow.
constexpr double kSweepGiveUpAxis = 0.5;
constexpr double kSweepToggleStepSec = 0.5;
constexpr double kSweepToggleFollowPct = 5.0;
constexpr double kSweepToggleIdleSec = 6.0;
constexpr double kSweepStowCheckAxis = 0.15;
constexpr double kSweepStowCheckSec = 3.0;
constexpr int kSweepToggleAgain = 2;
constexpr double kSweepHoldSec = 4.0;
constexpr double kSweepStowSec = 6.0;

// The gear step on the ground (SetupFlow::kHover*). Above the runway the sim has to show the
// aircraft off the ground for kHoverSettleSec before the pilot is told to raise the gear; a height
// above the ground on the runway over kHoverMaxAglFt isn't one. The pilot's gear lever (its handle
// simvar) up without the gear moving for as long as playback waits for it means the gear doesn't
// move up there. When the step ends, the gear (each leg within kGearDownTol of where it was on the
// runway) must be down for kLandDownSec before the aircraft goes back, and then the sim must show
// it on the ground for kLandSettleSec (kLandSettleMaxSec at most) before the setup goes on.
constexpr double kHoverSettleSec = 0.5;
constexpr double kHoverMaxAglFt = 60.0;
constexpr double kGearDownTol = 0.02;
constexpr double kLandDownSec = 1.0;
constexpr double kLandSettleSec = 0.5;
constexpr double kLandSettleMaxSec = 5.0;

double meanOf(const Frame& f, int first, int count) {
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < count; ++i) {
        const double v = f.v[first + i];
        if (std::isfinite(v)) {
            sum += v;
            ++n;
        }
    }
    return n > 0 ? sum / n : NAN;
}

double absDiff(double a, double b) { return std::isfinite(a) && std::isfinite(b) ? std::fabs(a - b) : 0.0; }

// Moved more than `tol` from the anchor (or the anchor isn't set yet).
bool movedFrom(double v, double anchor, double tol) {
    if (std::isnan(anchor)) return std::isfinite(v);
    return std::isfinite(v) && std::fabs(v - anchor) > tol;
}

bool jetLike(int engineType) { return engineType == 1 || engineType == 3 || engineType == 5; }

int engineCount(const SetupFacts& f) { return f.engines < 1 ? 1 : (f.engines > 4 ? 4 : f.engines); }

std::string upper(const std::string& s) {
    std::string out = s;
    for (char& ch : out) ch = static_cast<char>(ch >= 'a' && ch <= 'z' ? ch - 32 : ch);
    return out;
}

bool hasWord(const std::string& name, const char* const* words) {
    const std::string u = upper(name);
    for (const char* const* w = words; w && *w; ++w) {
        if (u.find(*w) != std::string::npos) return true;
    }
    return false;
}

const char* const kSpoilerWords[] = {"SPOILER", "SPEED_BRAKE", "SPEEDBRAKE", "SPD_BRK", "SPDBRK", nullptr};
const char* const kFlapWords[] = {"FLAP", nullptr};
const char* const kThrottleWords[] = {"THROTTLE", "THRUST_LEVER", "TLA", "LEVER", nullptr};
const char* const kGearWords[] = {"GEAR", nullptr};
// Among variables that held as still as each other, the throttle lever's own position over what
// the autothrust asks for or the thrust it makes: the A350's INI_AUTOTHROTTLE_ENGINE1_AT_REQUEST
// holds with the lever on the ground, but in reverse it reads part throttle (and in flight it's
// the autothrust's); its INI_ENGINE1_TLA_N1 is an N1, its INI_3D_THROTTLE_LEVER_POSITION_1 the lever.
const char* const kLeverPositionWords[] = {"LEVER", "TLA", nullptr};
const char* const kLeverDemandWords[] = {"REQUEST", "COMMAND", "CMD", "TARGET", "TGT", "DEMAND", "_N1", nullptr};

const char* const* wordsFor(Control c) {
    switch (c) {
        case Control::Spoilers: return kSpoilerWords;
        case Control::Flaps: return kFlapWords;
        case Control::Throttle: return kThrottleWords;
        case Control::Gear: return kGearWords;
    }
    return nullptr;
}

// What the pilot does for each control, one instruction at a time (the panel ticks each off).
const char* const kSpoilerTasks[] = {"Deploy the spoilers", "Retract the spoilers"};
const char* const kFlapTasks[] = {"Lower the flaps one notch", "Wait for the flaps to extend",
                                  "Retract the flaps one notch"};
// The throttle's first two are holds: they show whether the lever simvar is the lever and, where
// it isn't, which variable is (Phase::Positions repeats them when that wasn't found).
const char* const kThrottleTasks[] = {"Move the throttle to about halfway and hold it", "Now to full and hold it",
                                      "Back to idle", "Wait for the engines to spool down"};
const char* const kGearTasks[] = {"Raise the gear", "Lower the gear"};
constexpr size_t kThrottleHolds = 2;
constexpr size_t kPositionTasks = 3;  // the throttle's first three

struct TaskList {
    const char* const* items;
    size_t count;
};

TaskList tasksFor(Control c) {
    switch (c) {
        case Control::Spoilers: return {kSpoilerTasks, 2};
        case Control::Flaps: return {kFlapTasks, 3};
        case Control::Throttle: return {kThrottleTasks, 4};
        case Control::Gear: return {kGearTasks, 2};
    }
    return {kSpoilerTasks, 0};
}

const char* const kHoldDetail =
    "Hold each position until it's ticked. On a lever with detents, the detent nearest halfway will do.";
// Phase::Away: the first instruction once more.
const char* const kAwayDetail =
    "Once more, and leave it there: BetterReplay then tries bringing it back from where you left it.";
// Phase::Back from there: a pass of its own, before the replays of the whole movement.
const char* const kBringBackDetail =
    "From where you left it, with each method in turn until one does. Then your whole movement is replayed with "
    "each method.";

// The throttle lever's phases (SetupFlow::Phase::Query and on).
const char* const kQueryPrompt = "Do the levers stay put under autothrust?";
const char* const kQueryDetail =
    "On an Airbus the thrust levers sit in a detent (CL, FLX/MCT, TOGA) while the autothrust sets the thrust, "
    "so the thrust this aircraft reports isn't where its levers are. Your movement didn't show which kind this is.";
const char* const kPositionsDetail =
    "The thrust this aircraft reports isn't where its levers are, so BetterReplay looks for the lever's own "
    "position. Hold each position until it's ticked. On a lever with detents, the detent nearest halfway will do.";
const char* const kPositionsAgain = "Couldn't find the lever's own position in that movement. Once more, please.";
const char* const kNoticeDetail =
    "Once it's set, BetterReplay moves the throttle itself, from idle to full and back, to learn where the lever's "
    "positions are, then tries reverse thrust if the aircraft has it. The engines spool up against the parking brake, "
    "with the brakes held as well, for 20 to 40 seconds.";
const char* const kSweepDetail =
    "Keep your hands off the throttle: BetterReplay moves it itself, from idle to full and back, to learn where the "
    "lever's positions are, then tries reverse thrust if the aircraft has it. The engines spool up against the "
    "parking brake, with the brakes held as well, for 20 to 40 seconds.";
// The aircraft started to move though the setup held the brakes (an aircraft that ignores BRAKES).
const char* const kRolledReplay =
    "Stopped early: the aircraft started to move. If the parking brake is set, hold the brakes as well, then Watch "
    "again.";
const char* const kRolledSweep =
    "Stopped: the aircraft started to move. If the parking brake is set, hold the brakes as well, then Try again.";
// Phase::Calibrate.
const char* const kCalibrateDetail =
    "That only took them part way. BetterReplay now moves the speed brake lever through its range to see where each "
    "setting puts it, then replays your movement through that.";
const char* const kReverseNoticeDetail =
    "Once it's set, BetterReplay tries reverse thrust itself, to learn how to replay it on this aircraft. The engines "
    "spool up against the parking brake, with the brakes held as well, for up to half a minute.";
const char* const kReverseSweepDetail =
    "Keep your hands off the throttle while BetterReplay tries reverse thrust. The engines spool up against the "
    "parking brake, with the brakes held as well, for up to half a minute.";

// The gear step on the ground: held above the runway, then put back.
const char* const kHoverPrompt = "Lifting the aircraft off the runway";
const char* const kHoverDetail =
    "No takeoff needed: BetterReplay holds the aircraft frozen just above the runway while you raise and lower the "
    "gear, then puts it back.";
const char* const kHeldDetail = "The aircraft is held frozen just above the runway until the gear step is done.";
const char* const kLandDetail = "Once the gear is down, BetterReplay puts the aircraft back on the runway.";
const char* const kLowerDetail =
    "The gear is still up. Once it's down, BetterReplay puts the aircraft back on the runway. Cancel setup puts it "
    "back as it is.";

// Recording stops while the setup runs (SetupFlow::running), so its tests stay out of clips.
const char* const kTakeoffDetail = "Recording is paused until the setup is done, so this takeoff isn't saved.";
const char* const kDoneDetail = "Recording starts again now, with these controls.";

const char* question(Control c) {
    switch (c) {
        case Control::Spoilers: return "Did the spoilers come up and go back down?";
        case Control::Flaps: return "Did the flaps extend and retract?";
        case Control::Throttle: return "Did the engines spool up and back down?";
        case Control::Gear: return "Did the gear retract and extend again?";
    }
    return "";
}

const char* watchWhat(Control c) {
    switch (c) {
        case Control::Spoilers: return "Watch the spoilers";
        case Control::Flaps: return "Watch the flaps";
        case Control::Throttle: return "Listen to the engines";
        case Control::Gear: return "Watch the gear";
    }
    return "Watch";
}

// Bringing the control back from where the pilot left it, which the setup does on its own.
const char* bringBackWhat(Control c) {
    switch (c) {
        case Control::Spoilers: return "Retracting the spoilers";
        case Control::Flaps: return "Retracting the flaps";
        case Control::Throttle: return "Bringing the throttle back";
        case Control::Gear: return "Lowering the gear";
    }
    return "Bringing it back";
}

// The pilot's movement takes the gear up and back down, the rest out and back in.
Direction awayDirection(Control c) { return c == Control::Gear ? Direction::Retract : Direction::Extend; }
Direction backDirection(Control c) { return c == Control::Gear ? Direction::Extend : Direction::Retract; }

// A method that hasn't started moving the control within this long, more than the aircraft took to
// show the pilot's movement (SetupFlow::startWaitSec), hasn't worked: the way back on its own, and
// a replay of the whole movement (the gear: as long as playback waits for it, EventStepper's 360
// frames at the frame rate the setup sees, 6 s at least, since its legs start seconds after the
// lever: the A380's about 6 s after GEAR_DOWN, its doors first).
double moveStartSec(Control c, double frameDt) {
    if (c != Control::Gear) return 3.0;
    return std::max(EventStepper::kGearWaitFrames / 60.0, EventStepper::kGearWaitFrames * frameDt);
}

std::string title(Control c) {
    std::string s = controlSpec(c).label;
    if (!s.empty()) s[0] = static_cast<char>(s[0] - 32);
    return s;
}

const char* stateLabel(ControlState s) {
    switch (s) {
        case ControlState::Ok: return "set up";
        case ControlState::Failed: return "not replayable";
        case ControlState::Skipped: return "skipped";
        case ControlState::Absent: return "not on this aircraft";
        case ControlState::Pending: return "not set up";
    }
    return "";
}

bool isDigit(char ch) { return ch >= '0' && ch <= '9'; }

// What the control's own KeyEvent methods send already (KEY_ID_MIN + n): seen fire, they aren't
// learned again.
bool sentByKeyMethod(Control c, uint32_t id) {
    static const uint32_t kSpoilers[] = {250, 846};           // SPOILERS_SET, AXIS_SPOILER_SET
    static const uint32_t kFlaps[] = {162, 222, 223, 998};    // FLAPS_SET, FLAPS_INCR, FLAPS_DECR, AXIS_FLAPS_SET
    static const uint32_t kGear[] = {164, 543, 544};          // GEAR_SET, GEAR_UP, GEAR_DOWN
    auto in = [id](const uint32_t* first, const uint32_t* last) {
        return std::find(first, last, id >= 0x10000 ? id - 0x10000 : 0xFFFFFFFFu) != last;
    };
    switch (c) {
        case Control::Spoilers: return in(std::begin(kSpoilers), std::end(kSpoilers));
        case Control::Flaps: return in(std::begin(kFlaps), std::end(kFlaps));
        case Control::Gear: return in(std::begin(kGear), std::end(kGear));
        case Control::Throttle: return true;
    }
    return false;
}

const char* eventValueName(Control c) {
    switch (c) {
        case Control::Spoilers: return "spoiler handle";
        case Control::Flaps: return "flap handle index";
        case Control::Gear: return "gear command";
        case Control::Throttle: return "throttle";
    }
    return "";
}

// CandidateTracker::travelSec over samples k0..k1: from the first sample 5% of the way to the
// furthest point to the first within 10% of it.
template <typename At>
double travelOf(const std::vector<double>& times, long k0, long k1, At at) {
    if (k1 < k0 + 2) return NAN;
    const double base = at(k0);
    if (!std::isfinite(base)) return NAN;
    double extreme = base;
    for (long s = k0; s <= k1; ++s) {
        const double x = at(s);
        if (std::isfinite(x) && std::fabs(x - base) > std::fabs(extreme - base)) extreme = x;
    }
    const double span = std::fabs(extreme - base);
    if (!(span > 1e-9)) return NAN;
    double left = NAN;
    for (long s = k0; s <= k1; ++s) {
        const double x = at(s);
        if (!std::isfinite(x)) continue;
        if (std::isnan(left) && std::fabs(x - base) > 0.05 * span) left = times[static_cast<size_t>(s)];
        if (std::isfinite(left) && std::fabs(x - extreme) <= 0.1 * span) return times[static_cast<size_t>(s)] - left;
    }
    return NAN;
}

}  // namespace

// ---------------------------------------------------------------------------- candidates

void CandidateTracker::begin(std::vector<ScanVar> vars) {
    clear();
    vars_ = std::move(vars);
    tracks_.resize(vars_.size());
    byName_.reserve(vars_.size());
    for (size_t i = 0; i < vars_.size(); ++i) byName_.emplace((vars_[i].lvar ? "L:" : "B:") + vars_[i].name, i);
}

void CandidateTracker::clear() {
    std::vector<ScanVar>().swap(vars_);
    std::vector<Track>().swap(tracks_);
    std::vector<double>().swap(times_);
    std::vector<double>().swap(refHandle_);
    std::vector<double>().swap(refOutcome_);
    byName_.clear();
    stored_ = 0;
}

void CandidateTracker::sample(double t, const std::vector<double>& values, double handle, double outcome) {
    const long s = static_cast<long>(times_.size());
    times_.push_back(t);
    refHandle_.push_back(handle);
    refOutcome_.push_back(outcome);
    for (size_t i = 0; i < tracks_.size(); ++i) {
        Track& tr = tracks_[i];
        if (tr.noise) continue;
        const double v = i < values.size() ? values[i] : NAN;
        if (tr.first >= 0) {
            tr.series.push_back(static_cast<float>(std::isfinite(v) ? v : tr.series.back()));
            continue;
        }
        if (!std::isfinite(v)) continue;
        if (std::isnan(tr.base)) {
            tr.base = v;
            continue;
        }
        if (v == tr.base) continue;
        if (stored_ >= kMaxStored) {
            tr.noise = true;  // too much is changing; this one won't be considered
            continue;
        }
        tr.first = s;
        tr.series.push_back(static_cast<float>(v));
        ++stored_;
    }
}

void CandidateTracker::dropChangedBefore(double t) {
    const long k = static_cast<long>(std::lower_bound(times_.begin(), times_.end(), t) - times_.begin());
    for (Track& tr : tracks_) {
        if (tr.noise || tr.first < 0 || tr.first >= k) continue;
        tr.noise = true;
        std::vector<float>().swap(tr.series);
        --stored_;
    }
}

double CandidateTracker::sampleValue(const Track& tr, size_t s) const {
    if (tr.first < 0 || static_cast<long>(s) < tr.first) return tr.base;
    const size_t i = s - static_cast<size_t>(tr.first);
    if (i < tr.series.size()) return tr.series[i];
    return tr.series.empty() ? tr.base : tr.series.back();
}

double CandidateTracker::valueAt(size_t index, double t) const {
    if (index >= tracks_.size() || times_.empty()) return NAN;
    const Track& tr = tracks_[index];
    const size_t hi = static_cast<size_t>(std::upper_bound(times_.begin(), times_.end(), t) - times_.begin());
    if (hi == 0) return sampleValue(tr, 0);
    if (hi >= times_.size()) return sampleValue(tr, times_.size() - 1);
    const size_t lo = hi - 1;
    const double span = times_[hi] - times_[lo];
    const double f = span > 0.0 ? (t - times_[lo]) / span : 0.0;
    return interpolate(Interp::Linear, sampleValue(tr, lo), sampleValue(tr, hi), f);
}

bool CandidateTracker::lookup(const std::string& name, bool lvar, size_t& index) const {
    const auto it = byName_.find((lvar ? "L:" : "B:") + name);
    if (it == byName_.end()) return false;
    index = it->second;
    return true;
}

bool CandidateTracker::changed(size_t index) const {
    return index < tracks_.size() && !tracks_[index].noise && tracks_[index].first >= 0;
}

double CandidateTracker::firstChange(size_t index) const {
    if (!changed(index) || static_cast<size_t>(tracks_[index].first) >= times_.size()) return NAN;
    return times_[static_cast<size_t>(tracks_[index].first)];
}

CandidateTracker::HoldCheck CandidateTracker::holdCheck(double from, double to, const std::vector<size_t>& candidates,
                                                        double rest, double away, double tol, double holdSec,
                                                        double settleSec, double maxCreep) const {
    HoldCheck out;
    const long k0 = static_cast<long>(std::lower_bound(times_.begin(), times_.end(), from) - times_.begin());
    const long k1 = static_cast<long>(std::upper_bound(times_.begin(), times_.end(), to) - times_.begin()) - 1;
    if (k1 < k0 + 2) return out;
    // The handle holding still by itself, away from where it rests (an idle hold doesn't tell:
    // a thrust demand sits still at idle too).
    for (long s = k0; s <= k1 && !out.steady; ++s) {
        const double h0 = refHandle_[static_cast<size_t>(s)];
        if (!std::isfinite(h0) || !(std::fabs(h0 - rest) > away)) continue;
        double lo = h0, hi = h0;
        for (long e = s + 1; e <= k1; ++e) {
            const double h = refHandle_[static_cast<size_t>(e)];
            if (!std::isfinite(h) || !(std::fabs(h - rest) > away)) break;
            lo = h < lo ? h : lo;
            hi = h > hi ? h : hi;
            if (hi - lo > tol) break;
            if (times_[static_cast<size_t>(e)] - times_[static_cast<size_t>(s)] >= holdSec) {
                out.steady = true;
                break;
            }
        }
    }
    // The handle while a lever-like variable held still.
    std::vector<float> values;
    for (const size_t idx : candidates) {
        if (!changed(idx)) continue;
        const Track& tr = tracks_[idx];
        values.clear();
        for (long s = k0; s <= k1; ++s) values.push_back(static_cast<float>(sampleValue(tr, static_cast<size_t>(s))));
        std::vector<float> sorted = values;
        std::sort(sorted.begin(), sorted.end());
        const size_t distinct = static_cast<size_t>(std::unique(sorted.begin(), sorted.end()) - sorted.begin());
        if (distinct < static_cast<size_t>(kLeverMinValues) || !std::isfinite(sorted.front()) ||
            !std::isfinite(sorted[distinct - 1])) {
            continue;  // a flag or a counter, not a lever
        }
        const double still = std::max(kLeverStillFraction * (sorted[distinct - 1] - sorted.front()), 1e-9);
        for (size_t s = 0; s < values.size();) {
            double lo = values[s], hi = values[s];
            size_t e = s;
            while (e + 1 < values.size()) {
                const double x = values[e + 1];
                if ((x > hi ? x : hi) - (x < lo ? x : lo) > still) break;
                lo = x < lo ? x : lo;
                hi = x > hi ? x : hi;
                ++e;
            }
            const double t0 = times_[static_cast<size_t>(k0) + s];
            const double t1 = times_[static_cast<size_t>(k0) + e];
            if (t1 - t0 >= holdSec) {
                double hlo = INFINITY, hhi = -INFINITY;
                for (size_t k = s; k <= e; ++k) {
                    const size_t i = static_cast<size_t>(k0) + k;
                    const double h = refHandle_[i];
                    if (times_[i] < t0 + settleSec || !std::isfinite(h)) continue;
                    hlo = h < hlo ? h : hlo;
                    hhi = h > hhi ? h : hhi;
                }
                if (hhi - hlo > out.creep && hhi - hlo <= maxCreep) {
                    out.creep = hhi - hlo;
                    out.by = idx;
                }
            }
            s = e + 1;
        }
    }
    return out;
}

CandidateTracker::Holds CandidateTracker::holds(size_t index, double from, double to, double holdSec,
                                                double stillFraction) const {
    Holds out;
    if (!changed(index)) return out;
    const Track& tr = tracks_[index];
    const long k0 = static_cast<long>(std::lower_bound(times_.begin(), times_.end(), from) - times_.begin());
    const long k1 = static_cast<long>(std::upper_bound(times_.begin(), times_.end(), to) - times_.begin()) - 1;
    if (k1 < k0 + 2) return out;
    double lo = INFINITY, hi = -INFINITY;
    std::vector<float> sorted;
    for (long s = k0; s <= k1; ++s) {
        const double x = sampleValue(tr, static_cast<size_t>(s));
        lo = x < lo ? x : lo;
        hi = x > hi ? x : hi;
        sorted.push_back(static_cast<float>(x));
    }
    if (!(hi > lo)) return out;
    out.range = hi - lo;
    std::sort(sorted.begin(), sorted.end());
    out.values = static_cast<int>(std::unique(sorted.begin(), sorted.end()) - sorted.begin());
    const double still = stillFraction * (hi - lo);
    std::vector<double> levels;
    for (long s = k0; s <= k1;) {
        double rlo = sampleValue(tr, static_cast<size_t>(s)), rhi = rlo;
        long e = s;
        while (e + 1 <= k1) {
            const double x = sampleValue(tr, static_cast<size_t>(e + 1));
            if ((x > rhi ? x : rhi) - (x < rlo ? x : rlo) > still) break;
            rlo = x < rlo ? x : rlo;
            rhi = x > rhi ? x : rhi;
            ++e;
        }
        const double span = times_[static_cast<size_t>(e)] - times_[static_cast<size_t>(s)];
        if (span >= holdSec) {
            out.stillSec += span;
            const double level = 0.5 * (rlo + rhi);
            bool known = false;
            for (const double l : levels) known |= std::fabs(l - level) <= 4.0 * still;
            if (!known) levels.push_back(level);
        }
        s = e + 1;
    }
    out.levels = static_cast<int>(levels.size());
    return out;
}

double CandidateTracker::travelSec(size_t index, double from, double to) const {
    if (!changed(index)) return NAN;
    const Track& tr = tracks_[index];
    const long k0 = static_cast<long>(std::lower_bound(times_.begin(), times_.end(), from) - times_.begin());
    const long k1 = static_cast<long>(std::upper_bound(times_.begin(), times_.end(), to) - times_.begin()) - 1;
    return travelOf(times_, k0, k1, [&](long s) { return sampleValue(tr, static_cast<size_t>(s)); });
}

double CandidateTracker::referenceTravelSec(double from, double to) const {
    const long k0 = static_cast<long>(std::lower_bound(times_.begin(), times_.end(), from) - times_.begin());
    const long k1 = static_cast<long>(std::upper_bound(times_.begin(), times_.end(), to) - times_.begin()) - 1;
    return travelOf(times_, k0, k1, [&](long s) { return refOutcome_[static_cast<size_t>(s)]; });
}

std::vector<CandidateTracker::Found> CandidateTracker::find(double from, double to, bool lvars, bool byOutcome,
                                                            const char* const* words, size_t max) const {
    std::vector<Found> out;
    const long k0 = static_cast<long>(std::lower_bound(times_.begin(), times_.end(), from) - times_.begin());
    const long k1 = static_cast<long>(std::upper_bound(times_.begin(), times_.end(), to) - times_.begin()) - 1;
    if (k1 < k0 + 2) return out;
    const std::vector<double>& ref = byOutcome ? refOutcome_ : refHandle_;
    for (size_t i = 0; i < tracks_.size(); ++i) {
        const Track& tr = tracks_[i];
        if (vars_[i].lvar != lvars || !changed(i) || tr.first > k1) continue;
        double lo = INFINITY, hi = -INFINITY;
        for (long s = k0; s <= k1; ++s) {
            const double x = sampleValue(tr, static_cast<size_t>(s));
            if (!std::isfinite(x)) continue;
            lo = x < lo ? x : lo;
            hi = x > hi ? x : hi;
        }
        const double range = hi - lo;
        const double base = tr.base;
        if (!(range > 1e-6 * (std::fabs(base) > 1.0 ? std::fabs(base) : 1.0))) continue;
        // Back where it was once the pilot put the control back: rules out counters and latches.
        const double end = sampleValue(tr, static_cast<size_t>(k1));
        if (std::fabs(end - base) > kPassReturn * range + 1e-9) continue;
        // The reference can trail the control's own variable by seconds (gear legs behind their
        // lever, N1 behind the throttle), so the best correlation over lags up to 4 s counts.
        double best = 0.0;
        for (long lag = 0; lag <= 40; lag += 5) {
            double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
            int n = 0;
            for (long s = k0; s <= k1; ++s) {
                const double x = sampleValue(tr, static_cast<size_t>(s - lag > 0 ? s - lag : 0));
                const double y = ref[static_cast<size_t>(s)];
                if (!std::isfinite(x) || !std::isfinite(y)) continue;
                sx += x;
                sy += y;
                sxx += x * x;
                syy += y * y;
                sxy += x * y;
                ++n;
            }
            if (n < 3) continue;
            const double mx = sx / n, my = sy / n;
            const double vx = sxx / n - mx * mx, vy = syy / n - my * my;
            if (!(vx > 0.0) || !(vy > 0.0)) continue;
            const double r = std::fabs((sxy / n - mx * my) / std::sqrt(vx * vy));
            best = r > best ? r : best;
        }
        if (!(best >= 0.5)) continue;
        const double peak = std::fabs(lo) > std::fabs(hi) ? std::fabs(lo) : std::fabs(hi);
        out.push_back({i, best + (hasWord(vars_[i].name, words) ? 0.25 : 0.0), peak});
    }
    std::sort(out.begin(), out.end(), [](const Found& a, const Found& b) { return a.score > b.score; });
    if (out.size() > max) out.resize(max);
    return out;
}

// ---------------------------------------------------------------------------- key events

void KeyEventTracker::clear() {
    std::vector<Event>().swap(events_);
    stored_ = 0;
    lost_ = 0;
}

void KeyEventTracker::add(double t, uint32_t id, double value) {
    Event* e = nullptr;
    for (Event& x : events_) {
        if (x.id == id) e = &x;
    }
    if (!e) {
        events_.emplace_back();
        e = &events_.back();
        e->id = id;
    }
    ++e->count;
    if (std::isfinite(value)) {
        e->lo = std::min(e->lo, value);
        e->hi = std::max(e->hi, value);
        auto it = std::find_if(e->values.begin(), e->values.end(), [value](const std::pair<double, size_t>& v) { return v.first == value; });
        if (it != e->values.end()) ++it->second;
        else if (e->values.size() < kValues) e->values.push_back({value, 1});
        else e->many = true;
    }
    if (e->steady()) return;
    if (stored_ >= kMaxStored) {
        ++lost_;
        return;
    }
    e->firings.push_back({t, value});
    ++stored_;
}

void KeyEventTracker::dropBefore(double t) {
    for (Event& e : events_) {
        size_t n = 0;
        while (n < e.firings.size() && e.firings[n].t < t) ++n;
        if (n == 0) continue;
        e.firings.erase(e.firings.begin(), e.firings.begin() + static_cast<std::ptrdiff_t>(n));
        e.before += n;
        stored_ -= n;
    }
}

// ---------------------------------------------------------------------------- clip check

bool clipSupports(Control c, const ControlMethod& m, const Clip& clip, int engines, std::string* why) {
    if (m.kind == MethodKind::None) return true;
    const ControlSpec& spec = controlSpec(c);
    const bool byEngine = m.kind == MethodKind::SimVar || m.kind == MethodKind::KeyEvent || m.var.find('#') != std::string::npos;
    const int n = spec.perEngine && byEngine ? (engines < 1 ? 1 : (engines > 4 ? 4 : engines)) : 1;
    for (int e = 0; e < n; ++e) {
        const int ch = methodSourceChannel(c, m.kind, e);
        if (ch < 0) {
            if (why) *why = "the method has no source channel";
            return false;
        }
        const ChannelDef& def = kChannels[ch];
        if (!clip.present.test(static_cast<size_t>(ch))) {
            if (why) *why = std::string("the clip has no ") + def.key;
            return false;
        }
        if (m.kind != MethodKind::InputSet && m.kind != MethodKind::InputValue && m.kind != MethodKind::LVar) continue;
        const std::string want = engineVar(m.var, e + 1);
        const std::string* recorded = nullptr;
        for (const auto& kv : clip.meta.vars) {
            if (kv.first == def.key) recorded = &kv.second;
        }
        if (recorded) {
            if (upper(*recorded) == upper(want)) continue;
            if (why) *why = std::string(def.key) + " was recorded from " + *recorded + ", not " + want;
            return false;
        }
        // Clips from before the setup don't say; input event channels then recorded the default name.
        bool known = false;
        if (m.kind != MethodKind::LVar && def.inputEvent) {
            const std::string u = upper(want);
            const char* alt = def.inputEvent;
            while (*alt && !known) {
                const char* end = std::strchr(alt, '|');
                const size_t len = end ? static_cast<size_t>(end - alt) : std::strlen(alt);
                known = u.size() == len && u.compare(0, len, alt, len) == 0;
                alt += len + (end ? 1 : 0);
            }
        }
        if (!known) {
            if (why) *why = std::string("the clip doesn't say what ") + def.key + " recorded";
            return false;
        }
    }
    return true;
}

bool clipWays(Control c, const ControlProfile& cp, const Clip& clip, int engines, ControlProfile& out, size_t& left,
              std::string* why) {
    out = cp;
    left = 0;
    if (!cp.perDirection()) return clipSupports(c, cp.method, clip, engines, why);
    std::string dropped;  // the ways left out, and why (when some are kept)
    for (int d = 0; d < kDirections; ++d) {
        std::vector<ControlMethod> kept;
        std::string reason;
        for (const ControlMethod& m : cp.waysTo(static_cast<Direction>(d))) {
            if (clipSupports(c, m, clip, engines, &reason)) {
                kept.push_back(m);
                continue;
            }
            ++left;
            dropped += (dropped.empty() ? "" : "; ") + describeMethod(c, m) + ": " + reason;
        }
        if (kept.empty()) {
            if (why) *why = std::string("nothing it has for ") + directionKey(c, static_cast<Direction>(d)) + " fits (" + reason + ")";
            return false;
        }
        out.ways[d] = kept;
    }
    if (why) *why = dropped;
    return true;
}

bool clipHasLever(const LeverCalibration& cal, const Clip& clip, int engines, std::string* why) {
    if (!cal.valid()) {
        if (why) *why = "the profile has no throttle lever";
        return false;
    }
    const int n = cal.var.find('#') != std::string::npos ? (engines < 1 ? 1 : (engines > 4 ? 4 : engines)) : 1;
    for (int e = 0; e < n; ++e) {
        const ChannelDef& def = kChannels[kLVarThrottle1 + e];
        if (!clip.present.test(static_cast<size_t>(kLVarThrottle1 + e))) {
            if (why) *why = std::string("the clip has no ") + def.key;
            return false;
        }
        const std::string want = upper(engineVar(cal.var, e + 1));
        bool same = false;
        for (const auto& kv : clip.meta.vars) {
            if (kv.first == def.key) same = upper(kv.second) == want;
        }
        if (!same) {
            if (why) *why = std::string(def.key) + " wasn't recorded from " + engineVar(cal.var, e + 1);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------- flow

void SetupFlow::setLogger(LogFn fn, void* ctx) {
    log_ = fn;
    logCtx_ = ctx;
}

void SetupFlow::logf(int level, const char* fmt, ...) const {
    if (!log_) return;
    char buf[512];  // as long as the module log's lines (a per-direction profile takes a few hundred)
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_(logCtx_, level, buf);
}

SetupFlow::Tuning SetupFlow::tuning(Control c) const {
    switch (c) {
        case Control::Spoilers: return {0.05, 0.25, 0.03, 0.10, 8.0, 90.0};
        case Control::Flaps:
            // By handle index where the aircraft has detents, else by the flap surface.
            if (facts_.flapPositions > 1) return {0.5, 0.9, 0.02, 0.05, 45.0, 120.0};
            return {0.02, 0.05, 0.02, 0.05, 45.0, 120.0};
        case Control::Throttle:
            // Lever in percent; outcome N1 (percent) on turbines, RPM on pistons.
            if (jetLike(facts_.engineType)) return {5.0, 40.0, 2.0, 8.0, 30.0, 120.0};
            return {5.0, 40.0, 80.0, 300.0, 30.0, 120.0};
        case Control::Gear: return {0.05, 0.5, 0.05, 0.5, 45.0, 300.0};
    }
    return {1, 1, 1, 1, 10, 60};
}

double SetupFlow::handleSignal(const Frame& f) const {
    switch (control()) {
        case Control::Spoilers: return f.v[kSpoilersHandle];
        case Control::Flaps: return facts_.flapPositions > 1 ? f.v[kFlapsHandleIndex] : f.v[kFlapsLeft];
        case Control::Throttle: return meanOf(f, kThrottle1, engineCount(facts_));
        case Control::Gear: return meanOf(f, kGearCenter, 3);
    }
    return NAN;
}

double SetupFlow::outcomeSignal(const Frame& f) const {
    switch (control()) {
        case Control::Spoilers: return std::isfinite(f.v[kSpoilersLeft]) ? f.v[kSpoilersLeft] : f.v[kSpoilersHandle];
        case Control::Flaps: {
            // The trailing edge and the slats together: the A220's notch 1 moves only its slats.
            const double te = f.v[kFlapsLeft], le = f.v[kSlatsLeft];
            if (!std::isfinite(te) && !std::isfinite(le)) return f.v[kFlapsHandleIndex];
            return (std::isfinite(te) ? te : 0.0) + (std::isfinite(le) ? le : 0.0);
        }
        case Control::Throttle:
            return meanOf(f, jetLike(facts_.engineType) ? kEngineN1_1 : kEngineRpm1, engineCount(facts_));
        case Control::Gear: return meanOf(f, kGearCenter, 3);
    }
    return NAN;
}

bool SetupFlow::start(const AircraftProfile& profile, const Frame& live, const SetupFacts& facts, std::string& error) {
    if (active()) {
        error = "the control setup is already running";
        return false;
    }
    if (!sim_) {
        error = "control setup unavailable";
        return false;
    }
    AircraftProfile p = profile;
    p.declined = false;
    const bool missing[kControlCount] = {!facts.hasSpoilers, facts.flapPositions < 2, facts.engines < 1,
                                         !facts.gearRetractable};
    for (int i = 0; i < kControlCount; ++i) {
        ControlProfile& cp = p.controls[i];
        if (missing[i]) {
            cp = ControlProfile();
            cp.state = ControlState::Absent;
        } else if (cp.state == ControlState::Absent) {
            cp.state = ControlState::Pending;
        }
    }
    const bool onGround = live.v[kOnGround] >= 0.5;
    std::vector<Control> plan;
    for (int i = 0; i < kControlCount; ++i) {
        const Control c = controlAt(i);
        if (p[c].state == ControlState::Absent) continue;
        if (!onGround && c != Control::Gear) continue;
        plan.push_back(c);
    }
    if (plan.empty()) {
        error = onGround ? "this aircraft has none of the controls the setup covers"
                         : "in the air only the gear can be set up; land and stop with the engines running for the rest";
        return false;
    }
    profile_ = p;
    before_ = profile;
    changed_ = true;
    facts_ = facts;
    onGround_ = onGround;
    plan_ = plan;
    step_ = 0;
    note_.clear();
    std::string list;
    for (Control c : plan_) list += std::string(list.empty() ? "" : ", ") + controlSpec(c).key;
    logf(0, "control setup started %s: %s (%d engine(s), engine type %d, %d flap position(s))",
         onGround ? "on the ground" : "in the air", list.c_str(), facts.engines, facts.engineType, facts.flapPositions);
    enterStep();
    return true;
}

void SetupFlow::enterStep() {
    need_.clear();
    phaseT_ = 0.0;
    brakesLogged_ = false;
    leverVerdict_ = LeverVerdict::Steady;
    leverStage_ = false;
    reverseOnly_ = false;
    chosen_ = ControlMethod();
    lever_ = LeverCalibration();
    reverseBy_ = ReverseBy::Unknown;
    leverIdle_ = NAN;
    leverSpan_ = 0.0;
    std::vector<SweepMove>().swap(sweep_);
    std::vector<LeverPoint>().swap(sweepForward_);
    std::vector<LeverPoint>().swap(sweepReverse_);
    for (std::vector<ControlMethod>& w : ways_) w.clear();
    pilotBack_ = replayBack_ = -1;
    if (step_ >= plan_.size()) {
        phase_ = Phase::Done;
        std::string summary;
        for (int i = 0; i < kControlCount; ++i) {
            const Control c = controlAt(i);
            if (profile_[c].state == ControlState::Absent) continue;
            summary += (summary.empty() ? "" : ", ") + title(c) + ": " + stateLabel(profile_[c].state);
        }
        logf(0, "control setup finished: %s", summary.c_str());
        return;
    }
    const Control c = control();
    logf(0, "control setup: step %zu of %zu, %s", step_ + 1, plan_.size(), controlSpec(c).key);
    if (c == Control::Gear && onGround_) {
        beginHover();
    } else {
        phase_ = Phase::Prepare;
    }
}

const char* SetupFlow::precondition(const Frame& live) const {
    const Control c = control();
    if (c == Control::Gear) return onGround_ ? "Take off" : nullptr;
    if (!onGround_) return "Land and stop on the runway";
    if (live.v[kGroundSpeed] > kStopKt) return "Come to a stop";
    if (c == Control::Throttle) {
        if (!facts_.engineRunning) return "Start the engines";
        if (!facts_.parkingBrake) return "Set the parking brake";
        return nullptr;
    }
    // Ground spoilers stow when the thrust levers are advanced. Some aircraft's lever simvar
    // doesn't read 0 at idle, so this is loose (and "Continue anyway" skips it).
    const double thr = meanOf(live, kThrottle1, engineCount(facts_));
    if (std::isfinite(thr) && thr > 25.0) return "Bring the throttles back to idle";
    return nullptr;
}

void SetupFlow::onFrame(double dt, const Frame& live, const SetupFacts& facts) {
    if (!active()) return;
    facts_ = facts;
    onGround_ = live.v[kOnGround] >= 0.5;
    phaseT_ += dt;
    if (dt > 0.0 && dt < 0.5) frameDt_ += 0.05 * (dt - frameDt_);
    switch (phase_) {
        case Phase::Prepare: {
            const char* need = precondition(live);
            need_ = need ? need : "";
            if (!need) beginUser();
            break;
        }
        case Phase::Takeoff:
            // (Right after a hover the sim may still show the aircraft up there for a frame or two.)
            if (onGround_) {
                groundSeen_ = true;
            } else if (groundSeen_) {
                beginUser();
            }
            break;
        case Phase::Hover: hoverFrame(dt, live); break;
        case Phase::Land:
        case Phase::Lower: landFrame(dt, live); break;
        case Phase::User: userFrame(dt, live); break;
        case Phase::Away: awayFrame(live); break;
        case Phase::Back: backFrame(dt, live); break;
        case Phase::Replay: replayFrame(dt, live); break;
        case Phase::Hold: holdFrame(live); break;
        case Phase::Positions: positionsFrame(dt, live); break;
        case Phase::Notice:
            if (facts_.parkingBrake) beginSweep();
            break;
        case Phase::Sweep: sweepFrame(dt, live); break;
        case Phase::Calibrate: calibrateFrame(dt, live); break;
        default: break;
    }
    if (!active()) return;
    if (brakesWanted()) {
        if (!braking_) {
            braking_ = true;
            brakeParked_ = facts_.parkingBrake;
            if (!brakesLogged_) {
                brakesLogged_ = true;
                logf(0, "control setup: holding the brakes (BRAKES, as the \".\" key) while the engines run up, %s",
                     brakeParked_ ? "as well as the parking brake" : "the parking brake not set");
            }
        }
        sim_->holdBrakes();
    } else {
        letGoBrakes(true);
    }
}

bool SetupFlow::brakesWanted() const {
    switch (phase_) {
        case Phase::User:
        case Phase::Positions:
        case Phase::Replay:
        case Phase::Hold:
        case Phase::Sweep: return control() == Control::Throttle && onGround_;
        default: return false;
    }
}

// Pressing the brakes lets go of the parking brake on some aircraft (as the pedals do on a real
// Boeing): set again, it holds the aircraft at idle and the sweep's wait for it doesn't stall.
void SetupFlow::letGoBrakes(bool parkAgain) {
    if (!braking_) return;
    braking_ = false;
    if (parkAgain && brakeParked_ && !facts_.parkingBrake) {
        logf(0, "control setup: the parking brake was let go while the brakes were held; setting it again");
        sim_->setParkingBrake(true);
    }
}

void SetupFlow::beginUser() {
    phase_ = Phase::User;
    phaseT_ = 0.0;
    t_ = 0.0;
    nextCapture_ = 0.0;
    nextScan_ = 0.0;
    capture_.clear();
    moved_ = false;
    movedAt_ = 0.0;
    handleRest_ = outcomeRest_ = NAN;
    handlePeak_ = outcomePeak_ = 0.0;
    handleAnchor_ = outcomeAnchor_ = NAN;
    stableSince_ = 0.0;
    handleStill_ = outcomeStill_ = NAN;
    handleStillSince_ = outcomeStillSince_ = 0.0;
    task_ = 0;
    taskDone_ = false;
    resetHolds();
    need_.clear();
    gearHandleRest_ = gearHandleUpAt_ = NAN;
    std::vector<ScanVar> vars;
    sim_->scanBegin(vars);
    scanning_ = true;
    size_t lvars = 0;
    for (const ScanVar& v : vars) lvars += v.lvar ? 1 : 0;
    logf(0, "control setup: watching %zu input event(s), %zu L:var(s) and the key events while you move the %s",
         vars.size() - lvars, lvars, controlSpec(control()).label);
    tracker_.begin(std::move(vars));
    scanValues_.assign(tracker_.size(), NAN);
    keys_.clear();
}

void SetupFlow::userFrame(double dt, const Frame& live) {
    t_ += dt;
    const Tuning tu = tuning(control());
    const double h = handleSignal(live);
    const double o = outcomeSignal(live);
    if (std::isnan(handleRest_)) handleRest_ = h;
    if (std::isnan(outcomeRest_)) outcomeRest_ = o;
    // Until the pilot moves the throttle, engines still spooling down are settling: they started
    // where they come to (the A220's idle, not the 29% N1 it was still coming down from).
    if (!moved_ && control() == Control::Throttle && o < outcomeRest_) outcomeRest_ = o;
    const double dh = fromRest(h, handleRest_);
    const double dout = fromRest(o, outcomeRest_);

    if (t_ + 1e-9 >= nextCapture_) {
        capture_.push_back(live);
        capture_.back().t = t_;
        nextCapture_ += kCaptureInterval;
        if (nextCapture_ < t_) nextCapture_ = t_ + kCaptureInterval;
        if (!moved_ && capture_.size() > 300) {
            // Before the pilot moves only the last few seconds matter (the replay's lead-in).
            size_t drop = 0;
            while (drop < capture_.size() && capture_[drop].t < t_ - kPreMotionKeepSec) ++drop;
            capture_.erase(capture_.begin(), capture_.begin() + static_cast<std::ptrdiff_t>(drop));
        }
    }
    scanFrame(h, o);
    // Every frame, at the setup's time: a cockpit click is a single event.
    sim_->scanKeyEvents(keysSeen_);
    for (const SeenKeyEvent& k : keysSeen_) keys_.add(t_, k.id, k.value);
    if (!moved_) keys_.dropBefore(t_ - kSearchLeadSec - 1.0);

    if (!moved_ && (dh > tu.handleStart || dout > tu.outcomeStart)) {
        moved_ = true;
        movedAt_ = t_;
        tracker_.dropChangedBefore(t_ - kSearchLeadSec);
        keys_.dropBefore(t_ - kSearchLeadSec);
        logf(0, "control setup: the %s started moving (%.3g -> %.3g, outcome %.3g -> %.3g); %zu variable(s) changing",
             controlSpec(control()).label, handleRest_, h, outcomeRest_, o, tracker_.stored());
    }
    if (moved_) {
        handlePeak_ = dh > handlePeak_ ? dh : handlePeak_;
        outcomePeak_ = dout > outcomePeak_ ? dout : outcomePeak_;
    }
    if (!moved_ && control() == Control::Gear && hovering()) {
        // Held above the runway: the pilot's lever up (where its handle simvar shows it) and the
        // gear not moving for as long as playback waits for it, the gear doesn't move up there.
        const double lever = live.v[kGearHandle];
        if (std::isnan(gearHandleRest_)) gearHandleRest_ = lever;
        if (!(gearHandleRest_ >= 0.5 && lever < 0.5)) {
            gearHandleUpAt_ = NAN;
        } else if (std::isnan(gearHandleUpAt_)) {
            gearHandleUpAt_ = t_;
        } else if (t_ - gearHandleUpAt_ > moveStartSec(Control::Gear, frameDt_)) {
            leaveHover("the gear didn't move with its lever up there");
            return;
        }
    }
    if (movedFrom(h, handleAnchor_, 0.5 * tu.handleStart) || movedFrom(o, outcomeAnchor_, 0.5 * tu.outcomeStart)) {
        handleAnchor_ = h;
        outcomeAnchor_ = o;
        stableSince_ = t_;
    }
    const bool enough = handlePeak_ >= tu.handleNeed || outcomePeak_ >= tu.outcomeNeed;
    const bool handleBack = dh <= std::max(tu.handleStart, 0.1 * handlePeak_);
    const bool back = handleBack && dout <= std::max(tu.outcomeStart, 0.1 * outcomePeak_);
    const bool stable = t_ - stableSince_ >= kStableSec;

    // Which instruction the pilot is on. The first is done once the control has gone far enough
    // and stopped (or is coming back); then, for the flaps, the surfaces stopping (extended); the
    // last once everything is back where it started. The throttle's first two are holds, then the
    // lever back at idle, and its movement only counts with both holds in it.
    if (movedFrom(h, handleStill_, 0.5 * tu.handleStart)) {
        handleStill_ = h;
        handleStillSince_ = t_;
    }
    if (movedFrom(o, outcomeStill_, 0.5 * tu.outcomeStart)) {
        outcomeStill_ = o;
        outcomeStillSince_ = t_;
    }
    const size_t tasks = tasksFor(control()).count;
    const bool throttle = control() == Control::Throttle;
    if (throttle) {
        trackHolds(live, h, enough);
        if (task_ == kThrottleHolds && handleBack) task_ = kThrottleHolds + 1;
    } else {
        if (task_ == 0 && moved_ && enough &&
            (t_ - handleStillSince_ >= kTaskStillSec || dh < handlePeak_ - tu.handleStart)) {
            task_ = 1;
        }
        if (task_ == 1 && tasks == 3) {
            const bool extended = t_ - outcomeStillSince_ >= kTaskStillSec && dout > tu.outcomeStart;
            if (handleBack || (control() == Control::Flaps && extended)) task_ = 2;
        }
    }
    taskDone_ = task_ + 1 == tasks && back;

    if (moved_ && enough && back && stable && (!throttle || task_ + 1 == tasks)) {
        analyze();
        return;
    }
    if (phaseT_ > tu.waitMaxSec) {
        if (!moved_ && hovering()) {
            leaveHover("the gear didn't move there");
            return;
        }
        endScan();
        tracker_.clear();
        keys_.clear();
        capture_.clear();
        phase_ = Phase::Retry;
        phaseT_ = 0.0;
        const std::string label = controlSpec(control()).label;
        if (!moved_) note_ = "Didn't see the " + label + " move.";
        else if (throttle && task_ < kThrottleHolds) note_ = "Didn't see the throttle held at halfway and at full.";
        else note_ = "The " + label + " didn't go all the way and come back.";
        logf(1, "control setup: %s", note_.c_str());
    }
}

void SetupFlow::resetHolds() {
    holdSim_ = HoldSignal();
    holdInput_ = HoldSignal();
    for (HoldSignal* s : {&holdSim_, &holdInput_}) {
        for (double& v : s->past) v = NAN;
    }
    holdDoneAt_ = 0.0;
    hold_ = 0.0;
    holdLogAt_ = kHoldLogSec;
}

double SetupFlow::HoldSignal::lastSecond(double v) const {
    return samples > 0 ? absDiff(v, past[samples % kHoldPast]) : 0.0;
}

// One signal's part in the holds: how far through kHoldSec it has moved less than kHoldTolPct a
// second, away from where it started; for the second ("full"), after moving again, at another
// level than the first and with the movement far enough to count.
double SetupFlow::holdProgress(HoldSignal& s, double v, bool enough) {
    if (std::isnan(s.rest)) s.rest = v;
    // A jump since the last sample (some aircraft move the simvar in steps) is movement at once.
    const double last = s.samples > 0 ? s.past[(s.samples - 1) % kHoldPast] : NAN;
    if (!std::isfinite(last) || !std::isfinite(v) || std::fabs(v - last) > kHoldTolPct) s.since = t_;
    if (t_ + 1e-9 >= s.next) {
        s.next = t_ + kHoldSampleSec;
        double& past = s.past[s.samples++ % kHoldPast];  // a second ago
        if (!std::isfinite(past) || !std::isfinite(v) || std::fabs(v - past) > kHoldTolPct) s.since = t_;
        past = v;
    }
    const bool away = absDiff(v, s.rest) > 2.0 * tuning(Control::Throttle).handleStart;
    s.moved |= away;
    const bool another = task_ == 0 || (enough && s.since > holdDoneAt_ &&
                                        (std::isnan(s.level) || absDiff(v, s.level) > 2.0 * kHoldTolPct));
    if (task_ >= kThrottleHolds || !away || !another) return 0.0;
    return std::min(1.0, (t_ - s.since) / kHoldSec);
}

// The throttle's holds (the first kThrottleHolds instructions): on the throttle's input events once
// they've moved with it (the hand), else on the lever simvar. Not on the simvar then: a thrust
// demand still catching up when the first hold ticked (the A380's at 27% of its 69% at halfway)
// held at another level a few seconds later, and ticked "full" with the lever still at halfway.
// Going to full first still works: the second hold is then halfway.
void SetupFlow::trackHolds(const Frame& live, double h, bool enough) {
    const double ie = meanOf(live, kInputThrottle1, engineCount(facts_));
    holdSim_.rest = handleRest_;
    const double byInput = holdProgress(holdInput_, ie, enough);
    const double bySim = holdProgress(holdSim_, h, enough) * (holdInput_.moved ? 0.0 : 1.0);
    hold_ = std::max(bySim, byInput);
    if (task_ >= kThrottleHolds) return;
    if (hold_ < 1.0) {
        const double waiting = t_ - std::max(movedAt_, holdDoneAt_);
        if (moved_ && waiting >= holdLogAt_) {
            holdLogAt_ += kHoldLogSec;
            char input[96] = "no throttle input events";
            if (std::isfinite(ie)) {
                std::snprintf(input, sizeof(input), "its input events at %.3g (moved %.1f, %.1f from the last hold)", ie,
                              holdInput_.lastSecond(ie), absDiff(ie, holdInput_.level));
            }
            logf(0, "control setup: no throttle hold yet after %.0f s (hold %zu of %zu): the lever simvar at %.3g%% "
                 "(moved %.1f%% in the last second, %.1f%% from the last hold), %s",
                 waiting, task_ + 1, kThrottleHolds, h, holdSim_.lastSecond(h), absDiff(h, holdSim_.level), input);
        }
        return;
    }
    if (bySim >= 1.0 && std::isfinite(ie)) {
        logf(0, "control setup: throttle held at %.3g%% (hold %zu of %zu; its input events, which haven't moved, at %.3g)",
             h, task_ + 1, kThrottleHolds, ie);
    } else if (bySim >= 1.0) {
        logf(0, "control setup: throttle held at %.3g%% (hold %zu of %zu)", h, task_ + 1, kThrottleHolds);
    } else {
        logf(0, "control setup: throttle held, by its input events at %.3g (hold %zu of %zu); the lever simvar at "
             "%.3g%%, moved %.1f%% in the last second and %.1f%% from the last hold",
             ie, task_ + 1, kThrottleHolds, h, holdSim_.lastSecond(h), absDiff(h, holdSim_.level));
    }
    holdSim_.level = h;
    holdInput_.level = ie;
    holdDoneAt_ = t_;
    holdLogAt_ = kHoldLogSec;
    hold_ = 0.0;
    ++task_;
}

void SetupFlow::scanFrame(double h, double o) {
    if (!scanning_ || t_ + 1e-9 < nextScan_) return;
    sim_->scanRead(scanValues_);
    tracker_.sample(t_, scanValues_, h, o);
    nextScan_ += kScanInterval;
    if (nextScan_ < t_) nextScan_ = t_ + kScanInterval;
    if (!moved_) tracker_.dropChangedBefore(t_ - kSearchLeadSec - 1.0);
}

void SetupFlow::analyze() {
    endScan();
    const Control c = control();
    const Tuning tu = tuning(c);
    attempts_.clear();
    note_.clear();  // a note about the previous step has been seen by now
    // The replay starts just before whatever changed first: the control's simvar, or one of the
    // variables (or key events) the methods use.
    windowFrom_ = std::max(0.0, movedAt_ - kSearchLeadSec);
    logKeyEvents();
    const double firstChange = buildMethods();
    if (c == Control::Throttle) leverVerdict_ = checkLever();
    windowFrom_ = std::max(0.0, std::min(movedAt_, firstChange) - kLeadInSec);
    mini_ = Clip();
    mini_.present.set();
    miniAt_.clear();
    for (const Frame& f : capture_) {
        if (f.t < windowFrom_ - 1e-9) continue;
        mini_.frames.push_back(f);
        mini_.frames.back().t = f.t - windowFrom_;
        miniAt_.push_back(f.t);
    }
    std::vector<Frame>().swap(capture_);
    if (c == Control::Gear) deriveGearCommand(mini_);
    for (int i = 0; i < 3; ++i) {  // the gear legs that moved (legsMidway)
        legMoved_[i] = false;
        for (const Frame& f : mini_.frames) {
            if (absDiff(f.v[kGearCenter + i], mini_.frames.front().v[kGearCenter + i]) > 0.5) legMoved_[i] = true;
        }
    }
    keyColumn_.clear();
    if (c != Control::Throttle) {
        const int ch = methodSourceChannel(c, MethodKind::KeyEvent, 0);
        for (const Frame& f : mini_.frames) keyColumn_.push_back(f.v[ch]);
    }
    judgeOutcome_ = outcomePeak_ >= tu.outcomeNeed;
    userPeak_ = judgeOutcome_ ? outcomePeak_ : handlePeak_;
    clipMovedAt_ = NAN;
    for (const Frame& f : mini_.frames) {
        const double v = judgeOutcome_ ? outcomeSignal(f) : handleSignal(f);
        if (fromRest(v, judgeOutcome_ ? outcomeRest_ : handleRest_) > replayStartTol()) {
            clipMovedAt_ = f.t;
            break;
        }
    }
    // The clip starts kLeadInSec before the movement's first sign (the control's simvar, or a
    // variable or key event the methods use).
    pilotLag_ = std::isfinite(clipMovedAt_) ? std::max(0.0, clipMovedAt_ - kLeadInSec) : 0.0;
    replayLag_ = NAN;
    logf(0, "control setup: you moved the %s over %.1f s (handle moved %.3g, outcome %.3g; checking replays on the %s); "
         "trying %zu method(s)",
         controlSpec(c).label, mini_.duration(), handlePeak_, outcomePeak_, judgeOutcome_ ? "outcome" : "handle",
         methods_.size());
    logf(0, "control setup: the %s moved %.1f s after the first sign of your movement; a method gets %.1f s to start moving it",
         judgeOutcome_ ? "outcome" : "handle", pilotLag_, startWaitSec());
    trimStill();
    if (methods_.empty() || mini_.frames.size() < 2) {
        note_ = "Couldn't replay the " + std::string(controlSpec(c).label) + ".";
        finishStep(ControlState::Failed, ControlMethod(), false);
        return;
    }
    pilotBack_ = replayBack_ = -1;
    shortBest_ = -1;
    shortPct_ = 0;
    shortRerun_ = askAnyway_ = false;
    spoilerCal_ = LeverCalibration();
    calTried_ = 0;
    if (c == Control::Throttle) {
        startMethod(0);
        return;
    }
    beginAway();
}

// The pilot's movement can sit still a long while between its moves (the 777's gear up for 20 s
// until "Lower the gear", in a 48 s movement), which every replay waited out again. Of such a
// stretch, after the control first moved and before the movement's end, replays keep as long as
// a method gets to start moving it, and kStableSec more, after it stopped (a replay may get there
// that much later than the pilot did), and the pilot's lag, and a second more, before it moved
// again (the command that moved it comes that much earlier). Not the throttle, whose holds are
// part of its check.
void SetupFlow::trimStill() {
    const Control c = control();
    if (c == Control::Throttle || !std::isfinite(clipMovedAt_)) return;
    const Tuning tu = tuning(c);
    const double keepBefore = startWaitSec() + kStableSec;
    const double keepAfter = pilotLag_ + 1.0;
    const std::vector<Frame>& in = mini_.frames;
    std::vector<Frame> out;
    std::vector<double> at, keys;
    double cut = 0.0;
    const auto keep = [&](size_t k, double shift) {
        out.push_back(in[k]);
        out.back().t -= shift;
        at.push_back(miniAt_[k]);
        if (k < keyColumn_.size()) keys.push_back(keyColumn_[k]);
    };
    for (size_t i = 0; i < in.size();) {
        // The frames from i on within half the start thresholds of frame i's handle and outcome.
        size_t j = i;
        if (in[i].t > clipMovedAt_) {
            const double h = handleSignal(in[i]), o = outcomeSignal(in[i]);
            while (j + 1 < in.size() && absDiff(handleSignal(in[j + 1]), h) <= 0.5 * tu.handleStart &&
                   absDiff(outcomeSignal(in[j + 1]), o) <= 0.5 * tu.outcomeStart) {
                ++j;
            }
        }
        const double from = in[i].t + keepBefore, to = in[j].t - keepAfter;
        const bool trim = j + 1 < in.size() && to - from > 1.0;
        for (size_t k = i; k <= j; ++k) {
            if (trim && in[k].t >= from && in[k].t < to) continue;
            keep(k, trim && in[k].t >= to ? cut + (to - from) : cut);
        }
        if (trim) cut += to - from;
        i = j + 1;
    }
    if (!(cut > 0.0)) return;
    logf(0, "control setup: replays leave out %.1f s where the %s sat still: %.1f s of your %.1f s", cut, controlSpec(c).label,
         mini_.duration() - cut, mini_.duration());
    mini_.frames.swap(out);
    miniAt_.swap(at);
    if (!keyColumn_.empty()) keyColumn_.swap(keys);
}

std::string SetupFlow::enginePattern(const std::string& name, bool lvar) const {
    const int engines = engineCount(facts_);
    for (size_t p = name.size(); p-- > 0;) {
        if (name[p] != '1') continue;
        if ((p > 0 && isDigit(name[p - 1])) || (p + 1 < name.size() && isDigit(name[p + 1]))) continue;
        std::string pattern = name;
        pattern[p] = '#';
        bool all = true;
        for (int e = 2; e <= engines && all; ++e) {
            size_t idx = 0;
            all = tracker_.lookup(engineVar(pattern, e), lvar, idx) && tracker_.changed(idx);
        }
        if (all) return pattern;
    }
    return std::string();
}

double SetupFlow::buildMethods() {
    methods_.clear();
    const Control c = control();
    const bool byOutcome = handlePeak_ < tuning(c).handleStart;
    const bool gear = c == Control::Gear;
    // More for the gear: its legs' animations rank high and are dropped below.
    const size_t most = gear ? 8 : 4;
    std::vector<CandidateTracker::Found> inputs = tracker_.find(windowFrom_, t_, false, byOutcome, wordsFor(c), most);
    std::vector<CandidateTracker::Found> lvars = tracker_.find(windowFrom_, t_, true, byOutcome, wordsFor(c), most);
    double firstChange = INFINITY;
    for (const auto& f : inputs) {
        logf(0, "control setup:   input event %s moved with it (score %.2f, peak %.3g)", tracker_.var(f.index).name.c_str(),
             f.score, f.peak);
    }
    for (const auto& f : lvars) {
        logf(0, "control setup:   L:var %s moved with it (score %.2f, peak %.3g)", tracker_.var(f.index).name.c_str(), f.score,
             f.peak);
    }
    // A gear leg's own animation (the PMDG 777's 7X7XleftMLGExt) moves with the gear, but written
    // it only swings that leg on screen. The lever gets where it's going in a moment; a variable
    // that took seconds, or most of the time the legs took, follows them. (On the 777 the gear
    // simvars and its own animations didn't travel alike, so the time alone has to do.)
    const double legsSec = gear ? tracker_.referenceTravelSec(windowFrom_, t_) : NAN;
    auto followsLegs = [&](const CandidateTracker::Found& f) {
        if (!gear) return false;
        const double sec = tracker_.travelSec(f.index, windowFrom_, t_);
        const bool follows =
            sec >= kLegTravelMinSec || (legsSec >= kLegTravelMinSec && sec >= kLegFollowShare * legsSec);
        logf(0, "control setup:   %s %s took %.1f s to travel, the gear legs %.1f s: %s",
             tracker_.var(f.index).lvar ? "L:var" : "input event", tracker_.var(f.index).name.c_str(), sec, legsSec,
             follows ? "it follows them (their animation, not the lever); not trying it" : "kept");
        return follows;
    };
    inputs.erase(std::remove_if(inputs.begin(), inputs.end(), followsLegs), inputs.end());
    lvars.erase(std::remove_if(lvars.begin(), lvars.end(), followsLegs), lvars.end());
    if (gear) {
        // The lever leads: what changed first comes first (the PMDG 777's lever, switch_295_a,
        // before the sound and airflow flags that follow the legs).
        auto first = [&](const CandidateTracker::Found& f) {
            const double t = tracker_.firstChange(f.index);
            return std::isfinite(t) ? t : INFINITY;
        };
        auto earlier = [&](const CandidateTracker::Found& a, const CandidateTracker::Found& b) { return first(a) < first(b); };
        std::stable_sort(inputs.begin(), inputs.end(), earlier);
        std::stable_sort(lvars.begin(), lvars.end(), earlier);
    }
    const bool perEngine = c == Control::Throttle && engineCount(facts_) > 1;
    std::vector<std::string> seen;
    auto varOf = [&](const CandidateTracker::Found& f, bool lvar) {
        const std::string& name = tracker_.var(f.index).name;
        if (!perEngine) return name;
        const std::string pattern = enginePattern(name, lvar);
        return pattern.empty() ? name : pattern;
    };
    auto fresh = [&](const std::string& key) {
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) return false;
        seen.push_back(key);
        return true;
    };

    std::string firstInput;
    size_t taken = 0;
    for (const auto& f : inputs) {
        if (taken == 2) break;
        const std::string var = varOf(f, false);
        if (!fresh("B:" + var)) continue;
        ++taken;
        firstChange = std::min(firstChange, tracker_.firstChange(f.index));
        if (firstInput.empty()) firstInput = var;
        // Presets differ: the A320neo V2's throttle reads 0..100 but its _SET takes 0..1, its
        // flaps read 0/25/50 and take that. Values in 0..1 need no scaling either way.
        if (f.peak > 1.0001) {
            const bool fractionFirst = c == Control::Spoilers || c == Control::Throttle;
            methods_.push_back({MethodKind::InputSet, var, fractionFirst ? 0.01 : 1.0});
            methods_.push_back({MethodKind::InputSet, var, fractionFirst ? 1.0 : 0.01});
        } else {
            methods_.push_back({MethodKind::InputSet, var, 1.0});
        }
    }
    // The key events that fired with the movement (the cockpit's lever, the pilot's binding):
    // aircraft like the PMDG 777 ignore the generic ones below, and no variable shows their own.
    double firstFiring = INFINITY;
    std::vector<ControlMethod> learned = learnEvents(firstFiring);
    firstChange = std::min(firstChange, firstFiring);
    methods_.insert(methods_.end(), learned.begin(), learned.end());
    // The ways the aircraft's own gear lever event moves it, where its SDK documents one, those
    // not seen above: for when the lever's event didn't show (the PMDG 777's lever sends none the
    // setup sees). Which of them works which way, the replays below find out.
    if (gear) {
        for (const ControlMethod& own : ownGearWays(profile_.key)) {
            if (std::find(methods_.begin(), methods_.end(), own) == methods_.end()) methods_.push_back(own);
        }
    }
    if (c != Control::Flaps || facts_.flapPositions > 1) methods_.push_back({MethodKind::KeyEvent, std::string(), 1.0});
    // What hardware bindings send, for aircraft that ignore the events above (the PMDG 737).
    // Before the simvars: writing those moves the outcome the check reads even where the aircraft
    // animates its own surfaces, which only the pilot can then turn down.
    if (c == Control::Flaps) {
        methods_.push_back({MethodKind::KeyEvent, kKeyFlapsNotches, 1.0});
        if (facts_.flapPositions > 1) methods_.push_back({MethodKind::KeyEvent, kKeyFlapsAxis, 1.0});
    } else if (c == Control::Spoilers) {
        methods_.push_back({MethodKind::KeyEvent, kKeySpoilersAxis, 1.0});
    }
    if (controlSpec(c).simvarFirst >= 0) methods_.push_back({MethodKind::SimVar, std::string(), 1.0});
    taken = 0;
    for (const auto& f : lvars) {
        if (taken == 2) break;
        const std::string var = varOf(f, true);
        if (!fresh("L:" + var)) continue;
        ++taken;
        firstChange = std::min(firstChange, tracker_.firstChange(f.index));
        methods_.push_back({MethodKind::LVar, var, 1.0});
    }
    if (!firstInput.empty()) methods_.push_back({MethodKind::InputValue, firstInput, 1.0});
    if (methods_.size() > kMaxMethods) methods_.resize(kMaxMethods);
    return firstChange;
}

// One line per kind of key event (the most frequent with the movement first): how often it fired
// with the movement and before it, when (from the moment the control moved) and with what. Enough
// to see in one run what an aircraft's cockpit lever and the pilot's bindings send.
void SetupFlow::logKeyEvents() const {
    const char* label = controlSpec(control()).label;
    const std::vector<KeyEventTracker::Event>& evs = keys_.events();
    if (evs.empty()) {
        logf(0, "control setup: no key events while you moved the %s", label);
        return;
    }
    size_t all = 0, with = 0;
    for (const auto& e : evs) {
        all += e.count;
        with += e.firings.size();
    }
    logf(0, "control setup: %zu key event(s) of %zu kind(s) while you moved the %s, %zu kept from %.0f s before it "
         "moved (times from when it moved):",
         all, evs.size(), label, with, kSearchLeadSec);
    std::vector<size_t> order(evs.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        if (evs[a].firings.size() != evs[b].firings.size()) return evs[a].firings.size() > evs[b].firings.size();
        return evs[a].count > evs[b].count;
    });
    char buf[200];
    for (size_t n = 0; n < order.size() && n < kKeyLogKinds; ++n) {
        const KeyEventTracker::Event& e = evs[order[n]];
        std::snprintf(buf, sizeof(buf), ": %zu in all", e.count);
        std::string line = keyEventName(e.id) + buf;
        if (e.steady()) {
            std::snprintf(buf, sizeof(buf), ", %zu of them before it moved: steadily", e.before);
            line += buf;
        } else {
            if (e.before > 0) {
                std::snprintf(buf, sizeof(buf), ", %zu before it moved", e.before);
                line += buf;
            }
            if (!e.firings.empty()) {
                const double t0 = e.firings.front().t - movedAt_, t1 = e.firings.back().t - movedAt_;
                if (e.firings.size() == 1) std::snprintf(buf, sizeof(buf), ", %zu with it at %.1f s", e.firings.size(), t0);
                else std::snprintf(buf, sizeof(buf), ", %zu with it at %.1f .. %.1f s", e.firings.size(), t0, t1);
                line += buf;
            }
        }
        if (e.values.empty()) {
            line += ", no parameter";
        } else if (e.many) {
            std::snprintf(buf, sizeof(buf), ", parameters %.10g .. %.10g (more than %zu values)", e.lo, e.hi,
                          KeyEventTracker::kValues);
            line += buf;
        } else {
            std::vector<std::pair<double, size_t>> values = e.values;
            std::sort(values.begin(), values.end());
            line += values.size() == 1 ? ", parameter" : ", parameters";
            for (size_t i = 0; i < values.size(); ++i) {
                std::snprintf(buf, sizeof(buf), "%s %.10g (%zu)", i == 0 ? "" : ",", values[i].first, values[i].second);
                line += buf;
            }
        }
        logf(0, "control setup:   %s", line.c_str());
    }
    if (order.size() > kKeyLogKinds) logf(0, "control setup:   and %zu more kind(s)", order.size() - kKeyLogKinds);
    if (keys_.lost() > 0) logf(1, "control setup: %zu key event(s) weren't kept: too many", keys_.lost());
}

std::vector<double> SetupFlow::eventSeries() const {
    const Control c = control();
    std::vector<double> out;
    out.reserve(capture_.size());
    GearCommandFollower gear;  // the gear handle simvar lags the lever (the A320's), the legs don't
    for (const Frame& f : capture_) out.push_back(c == Control::Gear ? gear.update(meanOf(f, kGearCenter, 3)) : eventTarget(c, f));
    return out;
}

// Key events that fired with the movement, as methods: each event the control's value (eventTarget)
// followed, whatever it's called. One whose parameter followed the value is a position (a lever's
// position, an axis; on the gear two values will do, elsewhere it takes three); otherwise each of
// its parameters is a click: a toggle if the control moved one way after one and back after the
// next, or a step if it always moved the same way, paired with one that moved it back.
std::vector<ControlMethod> SetupFlow::learnEvents(double& first) {
    learned_.clear();
    first = INFINITY;
    std::vector<ControlMethod> out;
    const Control c = control();
    const std::vector<KeyEventTracker::Event>& evs = keys_.events();
    if (evs.empty() || capture_.empty()) return out;
    if (c == Control::Throttle) {
        logf(0, "control setup:   (the throttle's key events aren't tried on their own: its key-event method sends the axis)");
        return out;
    }
    const std::vector<double> kv = eventSeries();
    // The capture frame before time t (the aircraft as it was when an event fired), or at it.
    auto before = [&](double t) {
        const auto it = std::lower_bound(capture_.begin(), capture_.end(), t, [](const Frame& f, double x) { return f.t < x; });
        return it == capture_.begin() ? size_t(0) : static_cast<size_t>(it - capture_.begin()) - 1;
    };
    double lo = INFINITY, hi = -INFINITY;
    for (size_t k = before(windowFrom_); k < kv.size(); ++k) {
        if (!std::isfinite(kv[k])) continue;
        lo = std::min(lo, kv[k]);
        hi = std::max(hi, kv[k]);
    }
    const double range = hi - lo;
    const char* what = eventValueName(c);
    if (!(range > 0.0)) {
        logf(0, "control setup:   the %s didn't move, so no key event can be matched to it", what);
        return out;
    }
    const double moveMin = c == Control::Gear ? 0.5 : tuning(c).handleStart;
    // Which way the control first moved within kKeyFollowSec after time t: +1, -1, or 0 (it didn't).
    auto firstMove = [&](double t) {
        size_t k = before(t);
        const double base = kv[k];
        if (!std::isfinite(base)) return 0;
        for (++k; k < kv.size() && capture_[k].t <= t + kKeyFollowSec; ++k) {
            if (std::isfinite(kv[k]) && std::fabs(kv[k] - base) >= moveMin) return kv[k] > base ? 1 : -1;
        }
        return 0;
    };

    struct Candidate {
        EventMethod ev;
        int rank;        // 0 a position, 1 a toggle, 2 steps
        double quality;  // within a rank: higher first
        double first;    // its first firing
        const KeyEventTracker::Event* event;
    };
    std::vector<Candidate> found;
    struct Half {
        uint32_t id;
        int32_t param;
        size_t moves;
        double first;
    };
    std::vector<Half> raises, lowers;
    size_t noise = 0;
    for (const KeyEventTracker::Event& e : evs) {
        if (e.firings.empty()) continue;
        const std::string name = keyEventName(e.id);
        if (e.steady()) {
            ++noise;
            continue;
        }
        if (sentByKeyMethod(c, e.id)) {
            logf(0, "control setup:   %s is what the %s's key-event method sends: not tried on its own", name.c_str(),
                 controlSpec(c).label);
            continue;
        }
        // A position: its parameter against where the control went, kKeySettleSec on (or until it
        // fired again).
        double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0, sxy = 0.0;
        size_t n = 0;
        std::vector<double> params, values;
        for (size_t i = 0; i < e.firings.size(); ++i) {
            const KeyEventTracker::Firing& f = e.firings[i];
            if (!std::isfinite(f.value)) continue;
            double settle = std::min(f.t + kKeySettleSec, t_);
            if (i + 1 < e.firings.size()) settle = std::min(settle, e.firings[i + 1].t);
            const double x = kv[before(settle)];
            if (!std::isfinite(x)) continue;
            sx += x;
            sy += f.value;
            sxx += x * x;
            syy += f.value * f.value;
            sxy += x * f.value;
            ++n;
            params.push_back(f.value);
            values.push_back(x);
        }
        auto distinct = [](std::vector<double> v) {
            std::sort(v.begin(), v.end());
            return static_cast<size_t>(std::unique(v.begin(), v.end()) - v.begin());
        };
        if (n >= 2 && distinct(params) >= (c == Control::Gear ? 2u : 3u) && distinct(values) >= 2) {
            const double mx = sx / n, my = sy / n;
            const double vx = sxx / n - mx * mx, vy = syy / n - my * my, cxy = sxy / n - mx * my;
            const double r = vx > 0.0 && vy > 0.0 ? cxy / std::sqrt(vx * vy) : 0.0;
            const double b = vx > 0.0 ? std::round(cxy / vx * 1000.0) / 1000.0 : 0.0;
            if (std::fabs(r) >= kKeyPositionFit && b != 0.0) {
                EventMethod ev;
                ev.mode = EventMethod::Mode::Position;
                ev.id = e.id;
                ev.scale = b;
                ev.offset = std::round(my - b * mx);
                logf(0, "control setup:   %s fired with it %zu time(s): a position (parameter %.10g + %.10g x the %s, fit "
                     "%.2f)",
                     name.c_str(), e.firings.size(), ev.offset, ev.scale, what, std::fabs(r));
                found.push_back({ev, 0, std::fabs(r), e.firings.front().t, &e});
                continue;
            }
        }
        // Clicks: each parameter on its own (none: 0). A click has one or two (a mouse button, the
        // wheel either way); more that don't follow the control are something else's axis.
        if (distinct(params) > 4) {
            logf(0, "control setup:   %s fired %zu time(s) with %zu parameters that didn't follow the %s", name.c_str(),
                 e.firings.size(), distinct(params), what);
            continue;
        }
        auto paramOf = [](double v) {
            if (!std::isfinite(v)) return int32_t(0);
            return static_cast<int32_t>(std::max(-2147483648.0, std::min(2147483647.0, std::round(v))));
        };
        std::vector<int32_t> seen;
        for (const KeyEventTracker::Firing& f0 : e.firings) {
            const int32_t p = paramOf(f0.value);
            if (std::find(seen.begin(), seen.end(), p) != seen.end()) continue;
            seen.push_back(p);
            std::vector<int> dirs;
            size_t fired = 0, still = 0;
            double t0 = INFINITY;
            for (const KeyEventTracker::Firing& f : e.firings) {
                if (paramOf(f.value) != p) continue;
                ++fired;
                t0 = std::min(t0, f.t);
                const int d = firstMove(f.t);
                if (d == 0) ++still;
                else dirs.push_back(d);
            }
            const std::string label = name + " (" + std::to_string(p) + ")";
            if (fired > kKeyMaxSteps) {
                logf(0, "control setup:   %s fired %zu times: too often for a click", label.c_str(), fired);
                continue;
            }
            if (dirs.empty() || (still > 0 && (fired <= 4 || 4 * still > fired))) {
                logf(0, "control setup:   %s fired %zu time(s); the %s didn't move after %zu", label.c_str(), fired, what,
                     still);
                continue;
            }
            bool up = false, down = false, alternate = true;
            for (size_t i = 0; i < dirs.size(); ++i) {
                up |= dirs[i] > 0;
                down |= dirs[i] < 0;
                if (i > 0 && dirs[i] == dirs[i - 1]) alternate = false;
            }
            if (up && down && alternate) {
                EventMethod ev;
                ev.mode = EventMethod::Mode::Toggle;
                ev.id = e.id;
                ev.param = p;
                ev.scale = std::round(range * 1000.0) / 1000.0;
                logf(0, "control setup:   %s fired with it %zu time(s): a toggle", label.c_str(), fired);
                found.push_back({ev, 1, static_cast<double>(dirs.size()), t0, &e});
            } else if (up != down) {
                (up ? raises : lowers).push_back({e.id, p, dirs.size(), t0});
            } else {
                logf(0, "control setup:   %s moved the %s both ways, not one after the other", label.c_str(), what);
            }
        }
    }
    if (noise > 0) {
        logf(0, "control setup:   %zu kind(s) of key event fired steadily (%zu times or more before the %s moved): not its own",
             noise, KeyEventTracker::kSteadyBefore, controlSpec(c).label);
    }
    // A step each way: those that moved it up, paired with those that moved it back (the same
    // event's other parameter first: a lever's left and right click).
    std::vector<bool> pairedLower(lowers.size(), false);
    for (const Half& r : raises) {
        size_t best = lowers.size();
        for (size_t i = 0; i < lowers.size(); ++i) {
            if (pairedLower[i]) continue;
            if (best == lowers.size()) best = i;
            const bool same = lowers[i].id == r.id, bestSame = lowers[best].id == r.id;
            if ((same && !bestSame) || (same == bestSame && lowers[i].moves > lowers[best].moves)) best = i;
        }
        if (best == lowers.size()) {
            logf(0, "control setup:   %s (%d) only ever moved the %s up", keyEventName(r.id).c_str(), r.param, what);
            continue;
        }
        pairedLower[best] = true;
        const Half& l = lowers[best];
        EventMethod ev;
        ev.mode = EventMethod::Mode::Steps;
        ev.id = r.id;
        ev.param = r.param;
        ev.lowerId = l.id;
        ev.lowerParam = l.param;
        ev.scale = std::round(range / static_cast<double>(std::max(r.moves, l.moves)) * 1000.0) / 1000.0;
        logf(0, "control setup:   %s (%d) and %s (%d) fired with it: a step each way", keyEventName(r.id).c_str(), r.param,
             keyEventName(l.id).c_str(), l.param);
        found.push_back({ev, 2, static_cast<double>(r.moves + l.moves), std::min(r.first, l.first), nullptr});
    }
    for (size_t i = 0; i < lowers.size(); ++i) {
        if (!pairedLower[i]) {
            logf(0, "control setup:   %s (%d) only ever moved the %s down", keyEventName(lowers[i].id).c_str(),
                 lowers[i].param, what);
        }
    }
    std::stable_sort(found.begin(), found.end(), [](const Candidate& a, const Candidate& b) {
        return a.rank != b.rank ? a.rank < b.rank : a.quality > b.quality;
    });
    for (const Candidate& k : found) {
        if (out.size() == kMaxLearned) break;
        if (!(k.ev.scale != 0.0)) continue;
        const ControlMethod m = eventMethod(k.ev);
        out.push_back(m);
        first = std::min(first, k.first);
        if (k.rank == 0 && k.event) {
            LearnedEvent le;
            le.method = m;
            for (const KeyEventTracker::Firing& f : k.event->firings) {
                if (std::isfinite(f.value)) le.firings.push_back(f);
            }
            learned_.push_back(std::move(le));
        }
    }
    return out;
}

void SetupFlow::startMethod(size_t i) {
    const Control c = control();
    method_ = i;
    const ControlMethod& m = methods_[i];
    std::string why;
    if (!sim_->beginControl(c, m, why)) {
        logf(1, "control setup: can't try %s: %s", describeMethod(c, m).c_str(), why.c_str());
        attempts_.push_back("unavailable");
        nextMethod();
        return;
    }
    driving_ = true;
    prepareMethod(m);
    replayT_ = 0.0;
    hint_ = 0;
    firstApply_ = true;
    replayPeak_ = 0.0;
    replayMovedAt_ = NAN;
    stoppedEarly_ = false;
    started_ = seenRest_ = false;
    restFromDist_ = NAN;
    restMoved_ = false;
    gearCommand_ = gearFrom_ = gearTo_ = NAN;
    gearCommandAt_ = 0.0;
    replayBack_ = -1;
    phase_ = Phase::Replay;
    phaseT_ = 0.0;
    logf(0, "control setup: replaying the %s with %s (method %zu of %zu)", controlSpec(c).label,
         describeMethod(c, m).c_str(), i + 1, methods_.size());
}

void SetupFlow::prepareMethod(const ControlMethod& m) {
    const Control c = control();
    // The method reads a channel the pilot's movement didn't record under this name: fill it in
    // from the scanned variable.
    if (m.kind == MethodKind::InputSet || m.kind == MethodKind::InputValue || m.kind == MethodKind::LVar) {
        const bool lvar = m.kind == MethodKind::LVar;
        const int n = c == Control::Throttle && m.var.find('#') != std::string::npos ? engineCount(facts_) : 1;
        for (int e = 0; e < n; ++e) {
            const int ch = methodSourceChannel(c, m.kind, e);
            size_t idx = 0;
            const bool have = tracker_.lookup(engineVar(m.var, e + 1), lvar, idx);
            for (size_t k = 0; k < mini_.frames.size(); ++k) {
                mini_.frames[k].v[ch] = have ? tracker_.valueAt(idx, miniAt_[k]) : NAN;
            }
        }
    }
    // A key event that's a position replays the parameters it fired with, when it fired: its
    // channel then holds the values they stand for. Other key-event methods read it as built.
    if (m.kind == MethodKind::KeyEvent && !keyColumn_.empty()) {
        const int ch = methodSourceChannel(c, m.kind, 0);
        for (size_t k = 0; k < mini_.frames.size() && k < keyColumn_.size(); ++k) mini_.frames[k].v[ch] = keyColumn_[k];
        EventMethod ev;
        for (const LearnedEvent& le : learned_) {
            if (!(le.method == m) || !parseEventMethod(m, ev) || ev.mode != EventMethod::Mode::Position || ev.scale == 0.0) {
                continue;
            }
            size_t j = 0;
            double v = NAN;
            for (size_t k = 0; k < mini_.frames.size(); ++k) {
                for (; j < le.firings.size() && le.firings[j].t <= miniAt_[k] + 1e-9; ++j) {
                    if (std::isfinite(le.firings[j].value)) v = (le.firings[j].value - ev.offset) / ev.scale;
                }
                if (std::isfinite(v)) mini_.frames[k].v[ch] = v;
            }
        }
    }
}

// ---------------------------------------------------------------------------- each way

// After the pilot's movement (at rest again): "Raise the gear" once more, so the way back can be
// tried from where the pilot leaves it, as replays meet it (the PMDG 777's gear came down after the
// pilot had raised it only with a click on its lever).
void SetupFlow::beginAway() {
    phase_ = Phase::Away;
    phaseT_ = 0.0;
    taskDone_ = false;
    replayAnchor_ = NAN;
    replayStable_ = 0.0;
    logf(0, "control setup: \"%s\" once more: the way back is tried from where you leave the %s", tasksFor(control()).items[0],
         controlSpec(control()).label);
}

void SetupFlow::awayFrame(const Frame& live) {
    const Control c = control();
    const Tuning tu = tuning(c);
    const char* label = controlSpec(c).label;
    const double v = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double d = fromRest(v, judgeOutcome_ ? outcomeRest_ : handleRest_);
    if (movedFrom(v, replayAnchor_, 0.5 * (judgeOutcome_ ? tu.outcomeStart : tu.handleStart))) {
        replayAnchor_ = v;
        replayStable_ = phaseT_;
    }
    // As far as a replay has to take it (kPassPeak), and still.
    const bool away = std::isfinite(v) && d >= kPassPeak * userPeak_;
    taskDone_ = away;
    if (away && phaseT_ - replayStable_ >= kStableSec && !legsMidway(live)) {
        logf(0, "control setup: the %s is away (%.3g from where you started it); bringing it back with each method in turn "
             "until one does",
             label, d);
        startBack(BackFrom::Pilot);
        return;
    }
    if (phaseT_ > tu.waitMaxSec) {
        logf(1, "control setup: didn't see the %s moved away again; the way back is only tried from where the replays leave it",
             label);
        startReplays();
    }
}

void SetupFlow::startBack(BackFrom from) {
    backFrom_ = from;
    backTries_.clear();
    if (from == BackFrom::Pilot) {
        // Every method, but not the simvars: written, they move what the check reads (only the
        // pilot could tell, and isn't asked about these).
        for (size_t i = 0; i < methods_.size(); ++i) {
            if (methods_[i].kind != MethodKind::SimVar) backTries_.push_back(i);
        }
    } else {
        // The one that did it from where the pilot left it first, then the others in their order.
        if (pilotBack_ >= 0 && static_cast<size_t>(pilotBack_) != method_) backTries_.push_back(static_cast<size_t>(pilotBack_));
        for (size_t i = 0; i < methods_.size(); ++i) {
            if (i != method_ && static_cast<int>(i) != pilotBack_) backTries_.push_back(i);
        }
    }
    backAt_ = 0;
    tryBack();
}

void SetupFlow::tryBack() {
    const Control c = control();
    const char* label = controlSpec(c).label;
    for (; backAt_ < backTries_.size(); ++backAt_) {
        const ControlMethod& m = methods_[backTries_[backAt_]];
        std::string why;
        if (sim_->beginControl(c, m, why)) break;
        logf(1, "control setup: can't try %s: %s", describeMethod(c, m).c_str(), why.c_str());
        if (backFrom_ == BackFrom::Pilot) attempts_.push_back("unavailable");
    }
    if (backAt_ >= backTries_.size()) {
        if (backFrom_ == BackFrom::Pilot) {
            logf(1, "control setup: none of the method(s) brought the %s back from where you left it", label);
            startReplays();
        } else {
            logf(0, "control setup: %s moved the %s away, but nothing brought it back from there",
                 describeMethod(c, methods_[method_]).c_str(), label);
            attempts_.push_back("failed");
            nextMethod();
        }
        return;
    }
    driving_ = true;
    const size_t i = backTries_[backAt_];
    prepareMethod(methods_[i]);
    replayT_ = 0.0;
    firstApply_ = true;
    backFromDist_ = NAN;
    backMoved_ = false;
    replayAnchor_ = NAN;
    replayStable_ = 0.0;
    gearCommand_ = gearFrom_ = gearTo_ = NAN;
    gearCommandAt_ = 0.0;
    phase_ = Phase::Back;
    phaseT_ = 0.0;
    logf(0, "control setup: bringing the %s back from where %s left it with %s (%zu of %zu)", label,
         backFrom_ == BackFrom::Pilot ? "you" : "the replay", describeMethod(c, methods_[i]).c_str(), backAt_ + 1,
         backTries_.size());
}

// The way back: where the pilot's movement started the control, held until it's there and still,
// it stops short, or it hasn't started back within moveStartSec.
void SetupFlow::backFrame(double dt, const Frame& live) {
    const Control c = control();
    const Tuning tu = tuning(c);
    const Frame& rest = mini_.frames.front();
    replayT_ += dt;
    sim_->applyControl(c, drivenFrame(rest, methods_[backTries_[backAt_]]), firstApply_);
    firstApply_ = false;
    if (c == Control::Gear) noteGearCommand(eventTarget(c, rest), live);
    const double v = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double d = fromRest(v, judgeOutcome_ ? outcomeRest_ : handleRest_);
    if (std::isnan(backFromDist_)) backFromDist_ = d;
    // On its way as soon as it has moved as far as counts as moving: the A380's gear legs were
    // 0.12 down when the wait ran out (its doors open first), short of the 0.15 that's back.
    if (d <= backFromDist_ - replayStartTol()) backMoved_ = true;
    if (movedFrom(v, replayAnchor_, 0.5 * (judgeOutcome_ ? tu.outcomeStart : tu.handleStart))) {
        replayAnchor_ = v;
        replayStable_ = phaseT_;
    }
    const bool settled = phaseT_ >= 1.0 && phaseT_ - replayStable_ >= kStableSec && !legsMidway(live);
    const bool there = atRest(live);
    if (there && settled) {
        endBack(true, "");
    } else if (!backMoved_ && phaseT_ >= giveUpSec(methods_[backTries_[backAt_]])) {
        endBack(false, "it didn't move");
    } else if (backMoved_ && settled) {
        endBack(false, "it stopped short");
    } else if (phaseT_ >= tu.holdMaxSec) {
        endBack(there, "it hadn't got there");
    }
}

void SetupFlow::endBack(bool ok, const char* why) {
    const Control c = control();
    const char* label = controlSpec(c).label;
    endDrive();
    if (c == Control::Gear) logGearCommand();
    const size_t i = backTries_[backAt_];
    const std::string desc = describeMethod(c, methods_[i]);
    const char* who = backFrom_ == BackFrom::Pilot ? "you" : "the replay";
    if (!ok) {
        logf(0, "control setup: %s didn't bring the %s back from where %s left it (%s)", desc.c_str(), label, who, why);
        if (backFrom_ == BackFrom::Pilot) attempts_.push_back("failed");
        ++backAt_;
        tryBack();
        return;
    }
    logf(0, "control setup: %s brought the %s back from where %s left it", desc.c_str(), label, who);
    if (backFrom_ == BackFrom::Pilot) {
        pilotBack_ = static_cast<int>(i);
        startReplays();
        return;
    }
    // Out with method_, back with this: put to the pilot like a method that did both.
    replayBack_ = static_cast<int>(i);
    logf(0, "control setup: %s %s the %s and %s %s it; asking", describeMethod(c, methods_[method_]).c_str(),
         c == Control::Gear ? "raised" : "moved out", label, desc.c_str(), c == Control::Gear ? "lowered" : "brought back");
    note_.clear();
    phase_ = Phase::Ask;
    phaseT_ = 0.0;
}

void SetupFlow::startReplays() {
    attempts_.clear();
    startMethod(0);
}

double SetupFlow::restTol() const {
    return std::max(replayStartTol(), kPassReturn * userPeak_);
}

double SetupFlow::replayStartTol() const {
    const Tuning tu = tuning(control());
    return judgeOutcome_ ? tu.outcomeStart : tu.handleStart;
}

// As long as the aircraft took to show the pilot's movement on the signal the check reads, and
// moveStartSec more: the PMDG 777's flaps start about 3 s after the command, then take another
// to move as far as counts.
double SetupFlow::startWaitSec() const { return moveStartSec(control(), frameDt_) + pilotLag_; }

double SetupFlow::giveUpSec(const ControlMethod& m) const {
    EventMethod ev;
    const bool steps = parseEventMethod(m, ev) && ev.mode != EventMethod::Mode::Position;
    return startWaitSec() + (steps ? EventStepper::waitFrames(control()) * frameDt_ : 0.0);
}

double SetupFlow::attemptLeft() const {
    if (phase_ == Phase::Back) {
        return backMoved_ ? -1.0 : std::max(0.0, giveUpSec(methods_[backTries_[backAt_]]) - phaseT_);
    }
    if (phase_ != Phase::Replay || method_ >= methods_.size()) return -1.0;
    const ControlMethod& m = methods_[method_];
    if (!started_) return restMoved_ || control() == Control::Throttle ? -1.0 : std::max(0.0, giveUpSec(m) - phaseT_);
    double end = mini_.duration();
    if (seenRest_ && replayPeak_ < replayStartTol() && std::isfinite(clipMovedAt_)) {
        end = std::min(end, clipMovedAt_ + giveUpSec(m));  // replayFrame's stall
    }
    return std::max(0.0, end - replayT_);
}

// How far `v` is from `rest`, where the pilot's movement started the control. The spoilers only
// count going out: back past where they started is back, not a movement of its own (the A380's
// lever armed at 0.06, then deployed and retracted to 0, waited for 0.06 on "Retract the spoilers").
// So does the throttle: engines below where they started have spooled down (the A220's were still
// coming down from the taxi at 29% N1, and its "Wait for the engines to spool down" waited for that
// at its 21% idle).
double SetupFlow::fromRest(double v, double rest) const {
    const Control c = control();
    if ((c == Control::Spoilers || c == Control::Throttle) && std::isfinite(v) && std::isfinite(rest)) {
        return std::max(0.0, v - rest);
    }
    return absDiff(v, rest);
}

bool SetupFlow::atRest(const Frame& live) const {
    const double v = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    return std::isfinite(v) && fromRest(v, judgeOutcome_ ? outcomeRest_ : handleRest_) <= restTol();
}

// Only from where the pilot started the control: a replay that starts elsewhere (the last method
// left the gear up) doesn't get its way back there counted as the movement.
void SetupFlow::track(const Frame& live) {
    if (!seenRest_) seenRest_ = atRest(live);
    if (!seenRest_) return;
    const double v = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double d = fromRest(v, judgeOutcome_ ? outcomeRest_ : handleRest_);
    if (d > replayPeak_) replayPeak_ = d;
    if (std::isnan(replayMovedAt_) && phase_ == Phase::Replay && d > replayStartTol()) replayMovedAt_ = replayT_;
}

void SetupFlow::noteGearCommand(double command, const Frame& live) {
    gearTo_ = outcomeSignal(live);
    if (!std::isfinite(command) || command == gearCommand_) return;
    logGearCommand();
    gearCommand_ = command;
    gearCommandAt_ = replayT_;
    gearFrom_ = gearTo_;
}

void SetupFlow::logGearCommand() const {
    if (!std::isfinite(gearCommand_)) return;
    logf(0, "control setup:   gear %s at %.1f s: the gear went from %.2f to %.2f", gearCommand_ > 0.5 ? "down" : "up",
         gearCommandAt_, gearFrom_, gearTo_);
}

bool SetupFlow::rolling(const Frame& live) const {
    return control() == Control::Throttle && onGround_ && live.v[kGroundSpeed] > kMaxRollKt;
}

void SetupFlow::replayFrame(double dt, const Frame& live) {
    const Control c = control();
    const char* label = controlSpec(c).label;
    if (!started_) {
        // First back to where the pilot's movement started (the movement's first frame), held
        // until the control is there.
        sim_->applyControl(c, drivenFrame(mini_.frames.front(), methods_[method_]), firstApply_);
        firstApply_ = false;
        // The throttle's also waits for the aircraft to stop (the brakes held meanwhile): the A220
        // was still rolling at 7.5 kt from the pilot's run-up, and the roll check stopped the
        // replay at once.
        const bool rest = atRest(live);
        const bool still = c != Control::Throttle || !onGround_ || live.v[kGroundSpeed] <= kStopKt;
        const bool there = rest && still;
        // Not on its way there as soon as a method has to start moving it (giveUpSec): it can't
        // be replayed from there, as the check only counts movement from there. Left up by the
        // way back, the PMDG 777's gear waited 30 s for its lever event, then replayed all 48 s.
        const double d = fromRest(judgeOutcome_ ? outcomeSignal(live) : handleSignal(live),
                                  judgeOutcome_ ? outcomeRest_ : handleRest_);
        if (std::isnan(restFromDist_)) restFromDist_ = d;
        if (d <= restFromDist_ - replayStartTol()) restMoved_ = true;  // (as backFrame)
        if (!there && !restMoved_ && c != Control::Throttle && phaseT_ >= giveUpSec(methods_[method_])) {
            logf(0, "control setup: %s didn't bring the %s back to where you started it (it didn't move); not replaying it",
                 describeMethod(c, methods_[method_]).c_str(), label);
            endDrive();
            if (c == Control::Gear) logGearCommand();
            attempts_.push_back("failed");
            nextMethod();
            return;
        }
        if (!there && phaseT_ < kRestWaitSec) return;
        started_ = true;
        if (!rest) {
            logf(1, "control setup: the %s didn't go back to where you started it within %.0f s; replaying anyway", label,
                 kRestWaitSec);
        } else if (!still) {
            logf(1, "control setup: the aircraft is still moving (%.1f kt) after %.0f s; replaying anyway",
                 live.v[kGroundSpeed], kRestWaitSec);
        } else if (phaseT_ > 0.5) {
            logf(0, "control setup: the %s was back where you started it after %.1f s", label, phaseT_);
        }
    }
    replayT_ += dt;
    const double dur = mini_.duration();
    sampleFrames(mini_.frames.data(), mini_.frames.size(), replayT_ < dur ? replayT_ : dur, frame_, &hint_);
    sim_->applyControl(c, drivenFrame(frame_, methods_[method_]), firstApply_);
    firstApply_ = false;
    track(live);
    if (c == Control::Gear) noteGearCommand(eventTarget(c, frame_), live);
    if (rolling(live)) {
        stoppedEarly_ = true;
        sim_->applyControl(c, mini_.frames.front(), true);
        logf(1, "control setup: the aircraft started to roll (%.1f kt); stopping the throttle replay", live.v[kGroundSpeed]);
        endDrive();
        evaluate(live);
        return;
    }
    // Not moving yet, a while after the pilot's movement had moved it (as on the way back): the
    // method doesn't, so the rest of the movement isn't replayed. Its command may only come when
    // the clip's control did (the 777's flap handle follows its flaps), then the aircraft takes as
    // long as it did for the pilot. The hold puts the control back where the movement ends and the
    // check fails it as usual.
    const double wait = giveUpSec(methods_[method_]);
    const bool stalled = seenRest_ && replayPeak_ < replayStartTol() && std::isfinite(clipMovedAt_) &&
                         replayT_ >= clipMovedAt_ + wait;
    if (stalled && replayT_ < dur) {
        logf(0, "control setup: no movement of the %s %.1f s after yours; not replaying the rest", label, wait);
    }
    if (replayT_ >= dur || stalled) {
        phase_ = Phase::Hold;
        phaseT_ = 0.0;
        replayAnchor_ = NAN;
        replayStable_ = 0.0;
    }
}

void SetupFlow::holdFrame(const Frame& live) {
    const Control c = control();
    const Tuning tu = tuning(c);
    // Keep asserting the movement's end (the control at rest) while things settle.
    sim_->applyControl(c, drivenFrame(mini_.frames.back(), methods_[method_]), false);
    track(live);
    if (c == Control::Gear) noteGearCommand(eventTarget(c, mini_.frames.back()), live);
    const double v = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double tol = 0.5 * (judgeOutcome_ ? tu.outcomeStart : tu.handleStart);
    if (movedFrom(v, replayAnchor_, tol)) {
        replayAnchor_ = v;
        replayStable_ = phaseT_;
    }
    if (rolling(live)) {
        stoppedEarly_ = true;
        endDrive();
        evaluate(live);
        return;
    }
    if ((phaseT_ >= 1.0 && phaseT_ - replayStable_ >= kStableSec && !legsMidway(live)) || phaseT_ >= tu.holdMaxSec) {
        endDrive();
        evaluate(live);
    }
}

// The gear's legs can go in turn, with a pause between: the A380 lowers its nose leg all the way,
// then its main legs about 10 s later (their doors first), and the way back read the pause, the
// legs' mean at 0.34, as stopping short. Of the legs that moved in the pilot's movement, some
// where it started them and others not: under way, not still.
bool SetupFlow::legsMidway(const Frame& live) const {
    if (control() != Control::Gear || mini_.frames.empty()) return false;
    const Frame& rest = mini_.frames.front();
    bool atStart = false, elsewhere = false;
    for (int i = 0; i < 3; ++i) {
        const double v = live.v[kGearCenter + i];
        if (!legMoved_[i] || !std::isfinite(v)) continue;
        (absDiff(v, rest.v[kGearCenter + i]) <= replayStartTol() ? atStart : elsewhere) = true;
    }
    return atStart && elsewhere;
}

void SetupFlow::evaluate(const Frame& live) {
    const Control c = control();
    if (c == Control::Gear) logGearCommand();
    const double end = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double rest = judgeOutcome_ ? outcomeRest_ : handleRest_;
    const bool reached = replayPeak_ >= kPassPeak * userPeak_;
    const bool returned = fromRest(end, rest) <= restTol();
    const int pct = userPeak_ > 0.0 ? static_cast<int>(std::lround(100.0 * replayPeak_ / userPeak_)) : 0;
    const std::string desc = describeMethod(c, methods_[method_]);
    // How much later than in the pilot's movement it moved (ControlProfile::leadSec).
    replayLag_ = reached ? replayMovedAt_ - clipMovedAt_ : NAN;
    if (reached && std::isfinite(replayLag_)) {
        logf(0, "control setup: %s moved the %s %.1f s %s than you did", desc.c_str(), controlSpec(c).label,
             std::fabs(replayLag_), replayLag_ >= 0.0 ? "later" : "earlier");
    }
    const bool passed = stoppedEarly_ || (reached && returned);
    // Part way with a spoiler event that takes a position, and back: where does each value put
    // them? (The A380's SPOILERS_SET and AXIS_SPOILER_SET both went 76%.)
    const bool partWay = !stoppedEarly_ && returned && replayPeak_ > 2.0 * replayStartTol() &&
                         replayPeak_ < kPassFull * userPeak_;
    if (partWay && !askAnyway_ && canCalibrate(methods_[method_])) {
        logf(0, "control setup: %s moved the %s only %d%% as far as you did; stepping it through its range to see where it "
             "puts them",
             desc.c_str(), controlSpec(c).label, pct);
        calPassed_ = passed;
        calReached_ = reached;
        calPct_ = pct;
        if (beginCalibrate()) return;
    }
    judge(passed, reached, returned, pct);
}

void SetupFlow::judge(bool passed, bool reached, bool returned, int pct) {
    const Control c = control();
    const std::string desc = describeMethod(c, methods_[method_]);
    if (passed) {
        // Short of kPassFull: the others first, unless this is the last and went furthest.
        if (!stoppedEarly_ && !askAnyway_ && replayPeak_ < kPassFull * userPeak_) {
            if (shortBest_ < 0 || pct > shortPct_) {
                shortBest_ = static_cast<int>(method_);
                shortPct_ = pct;
            }
            if (method_ + 1 < methods_.size() || shortBest_ != static_cast<int>(method_)) {
                logf(0, "control setup: %s moved the %s only %d%% as far as you did; trying the other method(s) first",
                     desc.c_str(), controlSpec(c).label, pct);
                attempts_.push_back("short");
                nextMethod();
                return;
            }
        }
        logf(0, "control setup: %s moved the %s %d%% as far as you did%s; asking", desc.c_str(), controlSpec(c).label, pct,
             returned ? "" : " (it didn't come all the way back)");
        note_ = stoppedEarly_ ? kRolledReplay : "";
        if (!stoppedEarly_ && replayPeak_ < kPassFull * userPeak_ && shortBest_ == static_cast<int>(method_)) {
            note_ = "This one went " + std::to_string(pct) + "% as far as you did. No other method went further.";
        }
        phase_ = Phase::Ask;
        phaseT_ = 0.0;
        return;
    }
    logf(0, "control setup: %s moved the %s %d%% as far as you did%s", desc.c_str(), controlSpec(c).label, pct,
         reached && !returned ? ", but didn't bring it back" : "");
    if (reached && !returned && c != Control::Throttle && methods_.size() > 1) {
        // It goes one way: the way back from there with the others (the PMDG 777's lever event
        // raised its gear but didn't lower it; then the gear key events did).
        startBack(BackFrom::Replay);
        return;
    }
    attempts_.push_back("failed");  // the panel lists it as a row
    nextMethod();
}

void SetupFlow::nextMethod() {
    askAnyway_ = false;
    if (!shortRerun_ && method_ + 1 < methods_.size()) {
        startMethod(method_ + 1);
        return;
    }
    const Control c = control();
    // None went further than the one that passed short of kPassFull (or the pilot said no to
    // those that did): it once more, to ask about.
    if (!shortRerun_ && shortBest_ >= 0) {
        shortRerun_ = askAnyway_ = true;
        logf(0, "control setup: back to %s, which went furthest (%d%% as far as you did); replaying it once more to ask",
             describeMethod(c, methods_[static_cast<size_t>(shortBest_)]).c_str(), shortPct_);
        startMethod(static_cast<size_t>(shortBest_));
        return;
    }
    logf(1, "control setup: none of the %zu method(s) replayed the %s; playback will leave it to the aircraft",
         methods_.size(), controlSpec(c).label);
    note_ = "Couldn't replay the " + std::string(controlSpec(c).label) + " on this aircraft; replays will leave them alone.";
    finishStep(ControlState::Failed, ControlMethod(), false);
}

// ---------------------------------------------------------------------------- spoiler calibration

namespace {
// The spoilers' events that take a position (spoilerPositionEvent), each stepped once per step.
unsigned calBit(const ControlMethod& m) { return m.var.empty() ? 1u : 2u; }  // SPOILERS_SET, AXIS_SPOILER_SET
}  // namespace

bool SetupFlow::canCalibrate(const ControlMethod& m) const {
    return control() == Control::Spoilers && spoilerPositionEvent(m) && !(calTried_ & calBit(m));
}

bool SetupFlow::beginCalibrate() {
    const Control c = control();
    const ControlMethod& m = methods_[method_];
    calTried_ |= calBit(m);
    std::string why;
    if (!sim_->beginControl(c, m, why)) {
        logf(1, "control setup: can't step %s: %s", describeMethod(c, m).c_str(), why.c_str());
        return false;
    }
    driving_ = true;
    calPoints_.clear();
    calStep_ = 0;
    calStepT_ = calStill_ = 0.0;
    calAnchor_ = NAN;
    calFresh_ = true;
    calBack_ = false;
    calFrame_ = mini_.frames.front();
    phase_ = Phase::Calibrate;
    phaseT_ = 0.0;
    return true;
}

void SetupFlow::calibrateFrame(double dt, const Frame& live) {
    const Control c = control();
    calFrame_.v[kSpoilersHandle] = calBack_ ? mini_.frames.front().v[kSpoilersHandle] : static_cast<double>(calStep_) / kCalSteps;
    sim_->applyControl(c, calFrame_, calFresh_);
    calFresh_ = false;
    calStepT_ += dt;
    const double h = live.v[kSpoilersHandle];
    if (movedFrom(h, calAnchor_, kCalStillTol)) {
        calAnchor_ = h;
        calStill_ = calStepT_;
    }
    const bool still = calStepT_ >= kCalMinSec && calStepT_ - calStill_ >= kCalStillSec;
    if (calBack_) {
        if ((still && atRest(live)) || calStepT_ >= kCalBackMaxSec) {
            endDrive();
            finishCalibrate();
        }
        return;
    }
    if (!still && calStepT_ < kCalMaxSec) return;
    calPoints_.push_back({static_cast<double>(calStep_) / kCalSteps, h});
    if (++calStep_ > kCalSteps) calBack_ = true;
    calStepT_ = calStill_ = 0.0;
    calAnchor_ = NAN;
    calFresh_ = true;
}

// Where the event put the handle, against where the pilot took it: a calibration that gets there
// and differs from the event's own values is replayed through; else the replay before it stands.
void SetupFlow::finishCalibrate() {
    const Control c = control();
    const ControlMethod& m = methods_[method_];
    const char* event = spoilerPositionEvent(m);
    LeverCalibration cal;
    cal.var = event;
    cal.forward = leverTable(calPoints_);
    double hi = -INFINITY;
    for (const LeverPoint& p : cal.forward) hi = std::max(hi, p.lever);
    const double want = handleRest_ + handlePeak_;  // where the pilot deployed them to
    logf(0, "control setup: %s stepped from none to full put the handle from %.3g to %.3g (yours went to %.3g)", event,
         cal.forward.empty() ? NAN : cal.forward.front().lever, hi, want);
    if (!cal.valid() || !(hi >= handleRest_ + kPassFull * handlePeak_)) {
        logf(0, "control setup: %s doesn't take the %s as far as you did at any value; not calibrating it", event,
             controlSpec(c).label);
        judge(calPassed_, calReached_, true, calPct_);
        return;
    }
    const double at = invertLever(cal.forward, want);
    if (std::fabs(at - want) < kCalSameTol) {
        logf(0, "control setup: %s puts the handle where it's sent; calibrating it wouldn't change its replay", event);
        judge(calPassed_, calReached_, true, calPct_);
        return;
    }
    spoilerCal_ = cal;
    logf(0, "control setup: %s calibrated (%zu points): your %.3g is %s at %.3g; replaying it through that", event,
         cal.forward.size(), want, event, at);
    startMethod(method_);
}

// The frame `m` is driven from: the clip's spoiler handle through the calibration, where it was
// made with that event.
const Frame& SetupFlow::drivenFrame(const Frame& f, const ControlMethod& m) {
    if (control() != Control::Spoilers || !spoilerCal_.valid()) return f;
    driveFrame_ = f;
    driveFrame_.v[kSpoilersHandle] = spoilerCommand(spoilerCal_, m, f.v[kSpoilersHandle]);
    return driveFrame_;
}

// ---------------------------------------------------------------------------- throttle lever

// Whether the throttle's lever simvar is the lever, from the pilot's movement: while a lever-like
// variable held still (the hand still), did the simvar keep moving?
SetupFlow::LeverVerdict SetupFlow::checkLever() {
    const Tuning tu = tuning(Control::Throttle);
    std::vector<size_t> candidates;
    for (const bool lvars : {false, true}) {
        for (const auto& f : tracker_.find(windowFrom_, t_, lvars, false, kThrottleWords, lvars ? 40 : 8)) {
            candidates.push_back(f.index);
        }
    }
    // Held still by itself: within twice the creep, for a hardware lever that jitters a little. A
    // creep is the simvar catching up or drifting after the lever stopped; one that held still
    // while the simvar moved half the movement or more isn't the lever (a switch nearby).
    const CandidateTracker::HoldCheck hc =
        tracker_.holdCheck(windowFrom_, t_, candidates, handleRest_, 0.5 * tu.handleNeed, 2.0 * kLeverCreepPct,
                           kLeverHoldSec, kLeverSettleSec, kLeverMaxCreep * handlePeak_);
    if (hc.creep > kLeverCreepPct) {
        const ScanVar& v = tracker_.var(hc.by);
        logf(0, "control setup: the throttle lever simvar moved %.2g%% while %s %s held still: it isn't the lever here",
             hc.creep, v.lvar ? "L:var" : "input event", v.name.c_str());
        return LeverVerdict::Creeps;
    }
    // Held still, and at full where a lever at full reads 100%. A thrust demand that settles
    // quickly (no wind, no speed) holds still too, but short of 100%: that's worth asking about.
    const double full = handleRest_ + handlePeak_;
    if (hc.steady && full >= kLeverFullPct) {
        logf(0, "control setup: the throttle lever simvar held still while you held the throttle (it moved %.2g%% at "
             "most while the lever-like variables held): it's the lever",
             hc.creep);
        return LeverVerdict::Steady;
    }
    if (hc.steady) {
        logf(0, "control setup: the throttle lever simvar held still, but read %.3g%% at most with the throttle at full",
             full);
        return LeverVerdict::Unclear;
    }
    logf(0, "control setup: couldn't tell whether the throttle lever simvar is the lever (no %.0f s hold away from idle)",
         kLeverHoldSec);
    return LeverVerdict::Unclear;
}

void SetupFlow::confirmMethod() {
    const ControlMethod m = methods_[method_];
    if (control() != Control::Throttle) {
        // Out with this method; back with it or the one that brought it back after it, then the one
        // that brought it back from where the pilot left it, if another. All one method: as before.
        const Control c = control();
        std::vector<ControlMethod> back{replayBack_ >= 0 ? methods_[static_cast<size_t>(replayBack_)] : m};
        if (pilotBack_ >= 0 && !(methods_[static_cast<size_t>(pilotBack_)] == back[0])) {
            back.push_back(methods_[static_cast<size_t>(pilotBack_)]);
        }
        if (back.size() > 1 || !(back[0] == m)) {
            ways_[static_cast<int>(awayDirection(c))] = {m};
            ways_[static_cast<int>(backDirection(c))] = back;
        }
        finishStep(ControlState::Ok, m, true);
        return;
    }
    const bool keyEvents = m.kind == MethodKind::KeyEvent;
    if (!keyEvents) {
        if (control() == Control::Throttle && leverVerdict_ != LeverVerdict::Steady) {
            logf(0, "control setup: %s replays the aircraft's own variable, so the throttle needs no lever calibration",
                 describeMethod(control(), m).c_str());
        }
        finishStep(ControlState::Ok, m, true);
        return;
    }
    // The key events move the lever, but replayed from a simvar that isn't it (or maybe not: the
    // pilot is asked): find the lever. Where the simvar is the lever, only reverse thrust is left.
    leverStage_ = true;
    chosen_ = m;
    mini_ = Clip();
    methods_.clear();
    attempts_.clear();
    method_ = 0;
    if (leverVerdict_ == LeverVerdict::Steady) {
        reverseOnly();
    } else if (leverVerdict_ == LeverVerdict::Unclear) {
        phase_ = Phase::Query;  // the tracker keeps the movement until the answer
        phaseT_ = 0.0;
    } else {
        leverFromMovement();
    }
}

// The throttle needs no lever calibration: with reversers, the sweep only tries reverse thrust
// (the PMDG 737, whose lever simvar is the lever, takes "decrease throttle" held at idle).
void SetupFlow::reverseOnly() {
    tracker_.clear();
    lever_ = LeverCalibration();
    if (!facts_.hasReverse) {
        finishStep(ControlState::Ok, chosen_, true);
        return;
    }
    reverseOnly_ = true;
    logf(0, "control setup: the throttle needs no lever calibration; trying reverse thrust");
    enterNotice();
}

// The lever's own variable from the holds in the pilot's movement (still in the tracker), then
// its calibration; or, if they didn't show it, the holds once more.
void SetupFlow::leverFromMovement() {
    const bool found = findLever();
    tracker_.clear();
    if (found) {
        enterNotice();
    } else {
        beginPositions(kPositionsAgain);
    }
}

// The throttle's holds once more, for the lever's own variable alone (the movement didn't show it).
void SetupFlow::beginPositions(const char* note) {
    phase_ = Phase::Positions;
    phaseT_ = 0.0;
    t_ = 0.0;
    nextScan_ = 0.0;
    moved_ = false;
    movedAt_ = 0.0;
    handleRest_ = outcomeRest_ = NAN;
    handlePeak_ = outcomePeak_ = 0.0;
    task_ = 0;
    taskDone_ = false;
    resetHolds();
    note_ = note ? note : "";
    need_.clear();
    std::vector<ScanVar> vars;
    sim_->scanBegin(vars);
    scanning_ = true;
    tracker_.begin(std::move(vars));
    scanValues_.assign(tracker_.size(), NAN);
    logf(0, "control setup: hold the throttle at halfway and at full again (looking for the lever's own variable)");
}

void SetupFlow::positionsFrame(double dt, const Frame& live) {
    t_ += dt;
    const Tuning tu = tuning(Control::Throttle);
    const double h = handleSignal(live);
    const double o = outcomeSignal(live);
    if (std::isnan(handleRest_)) handleRest_ = h;
    if (std::isnan(outcomeRest_)) outcomeRest_ = o;
    scanFrame(h, o);
    const double dh = absDiff(h, handleRest_);
    if (!moved_ && dh > tu.handleStart) {
        moved_ = true;
        movedAt_ = t_;
        tracker_.dropChangedBefore(t_ - kSearchLeadSec);
    }
    if (moved_) {
        handlePeak_ = dh > handlePeak_ ? dh : handlePeak_;
        outcomePeak_ = std::max(outcomePeak_, absDiff(o, outcomeRest_));
    }
    trackHolds(live, h, handlePeak_ >= tu.handleNeed || outcomePeak_ >= tu.outcomeNeed);
    const bool back = moved_ && dh <= tu.handleStart;
    taskDone_ = task_ + 1 == kPositionTasks && back;
    if (taskDone_ && t_ - holdSim_.since >= kStableSec) {
        endScan();
        const bool found = findLever();
        tracker_.clear();
        if (found) {
            enterNotice();
            return;
        }
        phase_ = Phase::Retry;
        phaseT_ = 0.0;
        note_ = "Couldn't find the throttle lever's own position. Hold each position still until it's ticked.";
        return;
    }
    if (phaseT_ > tu.waitMaxSec + 60.0) {
        endScan();
        tracker_.clear();
        phase_ = Phase::Retry;
        phaseT_ = 0.0;
        note_ = task_ < kThrottleHolds ? "Didn't see the throttle held at halfway and at full."
                                       : "Didn't see the throttle brought back to idle.";
        logf(1, "control setup: %s", note_.c_str());
    }
}

// The per-engine pattern ('#' for the engine number) when every engine's variable changed, from
// any engine's name ("INI_ENG2_THROTTLE_POS" -> "INI_ENG#_THROTTLE_POS"); else the name.
std::string SetupFlow::leverPattern(const std::string& name) const {
    const int engines = engineCount(facts_);
    for (size_t p = name.size(); p-- > 0;) {
        if (name[p] < '1' || name[p] > '0' + engines) continue;
        if ((p > 0 && isDigit(name[p - 1])) || (p + 1 < name.size() && isDigit(name[p + 1]))) continue;
        std::string pattern = name;
        pattern[p] = '#';
        bool all = true;
        for (int e = 1; e <= engines && all; ++e) {
            size_t idx = 0;
            all = tracker_.lookup(engineVar(pattern, e), true, idx) && tracker_.changed(idx);
        }
        if (all) return pattern;
    }
    return name;
}

// The lever's own variable, from the pilot's holds (the throttle's movement, or Positions): an
// L:var that moved with the throttle, held a value at idle and at each position, and was still
// for the longest (the thrust demand keeps creeping after each move). Sets lever_.
bool SetupFlow::findLever() {
    lever_ = LeverCalibration();
    const double from = std::max(0.0, movedAt_ - kSearchLeadSec);
    // An airliner has dozens of throttle L:vars (thrust demands, requests, animations, flags).
    const auto found = tracker_.find(from, t_, true, false, kThrottleWords, 40);
    bool have = false;
    size_t best = 0;
    CandidateTracker::Holds bestHolds;
    double bestScore = 0.0;
    for (const auto& f : found) {
        const CandidateTracker::Holds hd = tracker_.holds(f.index, from, t_, kLeverPositionSec, kLeverStillFraction);
        const std::string& name = tracker_.var(f.index).name;
        const double score = f.score + (hasWord(name, kLeverPositionWords) ? 0.1 : 0.0) -
                             (hasWord(name, kLeverDemandWords) ? 0.1 : 0.0);
        logf(0, "control setup:   L:var %s held %d position(s), still %.1f s of %.1f, %d values (score %.2f)", name.c_str(),
             hd.levels, hd.stillSec, t_ - from, hd.values, score);
        // Flags and detent indexes hold still too, but don't say where between detents the lever is.
        if (hd.levels < kLeverLevels || hd.values < kLeverMinValues) continue;
        const bool better = !have || hd.stillSec > bestHolds.stillSec + 0.5 ||
                            (hd.stillSec > bestHolds.stillSec - 0.5 && score > bestScore);
        if (!better) continue;
        have = true;
        best = f.index;
        bestHolds = hd;
        bestScore = score;
    }
    if (!have) {
        logf(1, "control setup: no L:var held still at the throttle positions (%zu moved with it)", found.size());
        return false;
    }
    lever_.var = leverPattern(tracker_.var(best).name);
    size_t idx = best;
    tracker_.lookup(engineVar(lever_.var, 1), true, idx);
    leverIdle_ = tracker_.valueAt(idx, t_);
    leverSpan_ = bestHolds.range;
    logf(0, "control setup: the throttle lever is L:var %s (%d positions, idle %.4g, range %.4g)", lever_.var.c_str(),
         bestHolds.levels, leverIdle_, leverSpan_);
    return true;
}

// The sweep starts by itself, as the spoilers' calibration does; first the parking brake, as the
// engines spool up against it (Continue anyway carries on without).
void SetupFlow::enterNotice() {
    note_.clear();
    if (facts_.parkingBrake) {
        beginSweep();
        return;
    }
    phase_ = Phase::Notice;
    phaseT_ = 0.0;
    need_ = "Set the parking brake";
}

void SetupFlow::beginSweep() {
    std::string why;
    if (!sim_->beginControl(Control::Throttle, chosen_, why)) {
        logf(1, "control setup: can't move the throttle to calibrate its lever: %s", why.c_str());
        note_ = "Couldn't move the throttle (" + why + ").";
        phase_ = Phase::Retry;
        phaseT_ = 0.0;
        return;
    }
    driving_ = true;
    sweep_.clear();
    sweepAt_ = 0;
    sweepExtended_ = false;
    reverseZone_ = reverseSeen_ = holdTried_ = holdSeen_ = false;
    toggleTried_ = toggleSeen_ = toggleFollows_ = toggledOn_ = false;
    toggleAgain_ = 0;
    toggleIdle_ = toggleDeep_ = NAN;
    sweepForward_.clear();
    sweepReverse_.clear();
    reverserOn_ = false;
    for (double& v : sweepFrame_.v) v = NAN;
    phase_ = Phase::Sweep;
    phaseT_ = 0.0;
    if (reverseOnly_) {
        // No lever to calibrate: straight to the reverse test, from idle (axis 0, where the key
        // events put this throttle at idle), and back there.
        sweepExtended_ = true;
        planReverse();
        sweep_.push_back({0.0, 0, -1});
        logf(0, "control setup: trying reverse thrust");
    } else {
        for (int i = 0; i <= kSweepSteps; ++i) sweep_.push_back({static_cast<double>(i) / kSweepSteps, 1, -1});
        logf(0, "control setup: stepping the throttle axis from bottom to full to calibrate L:var %s", lever_.var.c_str());
    }
    sweepTo(sweep_[0].axis, true);
}

// The reverse test's moves: the reverse events and the amount stepped up the axis, then back to
// idle and stowed ("toggle reverse" is added after that if they didn't work).
void SetupFlow::planReverse() {
    sweep_.push_back({0.0, 0, -1});
    sweep_.push_back({0.0, 2, 1});
    for (int i = 1; i <= kSweepReverseSteps; ++i) sweep_.push_back({static_cast<double>(i) / kSweepReverseSteps, 2, -1});
    sweep_.push_back({0.0, 0, -1});
    sweep_.push_back({0.0, 0, 0});
}

// The toggle test's moves, next: toggled at idle and the amount stepped up the axis, then back to
// idle, toggled back and checked ("decrease throttle" held is added after that if it didn't work).
void SetupFlow::planToggle() {
    std::vector<SweepMove> moves = {{0.0, 7, -1}, {0.0, 5, 2}};
    for (int i = 1; i <= kSweepReverseSteps; ++i) moves.push_back({static_cast<double>(i) / kSweepReverseSteps, 5, -1});
    moves.insert(moves.end(), {{0.0, 7, -1}, {0.0, 0, 2}, {kSweepStowCheckAxis, 6, -1}, {0.0, 0, -1}});
    sweep_.insert(sweep_.begin() + static_cast<std::ptrdiff_t>(sweepAt_), moves.begin(), moves.end());
}

void SetupFlow::sweepTo(double axis, bool force) {
    for (int e = 0; e < 4; ++e) sweepFrame_.v[kThrottleAxis1 + e] = axis;
    sim_->applyControl(Control::Throttle, sweepFrame_, force);
    sweepHeld_ = 0.0;
    sweepSteady_ = 0.0;
    sweepLast_ = NAN;
}

void SetupFlow::sweepFrame(double dt, const Frame& live) {
    if (rolling(live)) {
        logf(1, "control setup: the aircraft started to roll (%.1f kt); stopping the throttle calibration", live.v[kGroundSpeed]);
        stopSweep(kRolledSweep);
        return;
    }
    if (phaseT_ > kSweepMaxSec) {
        stopSweep("The throttle calibration took too long.");
        return;
    }
    sweepHeld_ += dt;
    // The lever's reading (none when only reverse is tried: then nothing to wait for).
    const double v = lever_.var.empty() ? NAN : sim_->readVar({engineVar(lever_.var, 1), true});
    const bool same = std::isfinite(v) && std::isfinite(sweepLast_) && std::fabs(v - sweepLast_) <= 1e-6 * std::max(1.0, std::fabs(v));
    sweepSteady_ = same || lever_.var.empty() ? sweepSteady_ + dt : 0.0;
    sweepLast_ = v;
    const SweepMove m = sweep_[sweepAt_];  // (a copy: the plan can grow below)
    bool reversingNow = false;
    for (int e = 0; e < engineCount(facts_); ++e) reversingNow |= reversing(live, e);
    const double thrNow = meanOf(live, kThrottle1, engineCount(facts_));
    if (m.record == 2 && reversingNow) reverseSeen_ = true;
    if (m.record == 5 && reversingNow) toggleSeen_ = true;
    if (m.record == 6) {
        // Out of reverse again? A little forward thrust shows it; in reverse it's a reverse amount.
        const bool forward = std::isfinite(thrNow) && thrNow > ReverseToggle::kForwardPct;
        if (sweepHeld_ < 0.5 || (!reversingNow && !forward && sweepHeld_ < kSweepStowCheckSec)) return;
        if (reversingNow) {
            toggledOn_ = true;
            if (toggleAgain_ >= kSweepToggleAgain) {
                logf(2, "control setup: still in reverse after toggling reverse %d more time(s)", toggleAgain_);
                stopSweep("Couldn't get out of reverse thrust. Press your \"toggle reverse\" key or button.");
                return;
            }
            ++toggleAgain_;
            logf(1, "control setup: still in reverse after toggling back (throttle %.1f%%); toggling again", thrNow);
            const SweepMove again[] = {{0.0, 7, -1}, {0.0, 0, 2}, {kSweepStowCheckAxis, 6, -1}};
            sweep_.insert(sweep_.begin() + static_cast<std::ptrdiff_t>(sweepAt_ + 1), again, again + 3);
        } else {
            logf(0, "control setup: out of reverse again (throttle %.1f%% at axis %.2f)", thrNow, m.axis);
            if (!(toggleSeen_ && toggleFollows_) && !holdTried_) {
                // The toggle didn't work: try "decrease throttle" held at idle, after the idle move.
                holdTried_ = true;
                const SweepMove hold[] = {{0.0, 3, -1}, {0.0, 4, -1}};
                sweep_.insert(sweep_.begin() + static_cast<std::ptrdiff_t>(std::min(sweepAt_ + 2, sweep_.size())), hold,
                              hold + 2);
            }
        }
    } else if (m.record == 7) {
        // At idle before a toggle: until the throttle simvar is back there, out of reverse or not.
        const bool atIdle = !reversingNow && !(thrNow > ReverseToggle::kForwardPct);
        if (sweepHeld_ < kSweepMinHoldSec || (!atIdle && sweepHeld_ < kSweepToggleIdleSec)) return;
        if (!atIdle) logf(1, "control setup: the throttle simvar didn't come back to idle (%.1f%%); toggling anyway", thrNow);
    } else if (m.record == 3) {
        // "Decrease throttle" held at idle: a held key sends it every frame.
        holdSeen_ |= reversingNow;
        if (!holdSeen_ && sweepHeld_ < kSweepHoldSec) {
            sim_->throttleDecrease();
            return;
        }
        logf(0, "control setup: holding THROTTLE_DECR at idle %s the reversers (%.1f s)",
             holdSeen_ ? "deployed" : "didn't deploy", sweepHeld_);
    } else if (m.record == 4) {
        // Let go: they should come back to idle by themselves.
        if (reversingNow && sweepHeld_ < kSweepStowSec) return;
        if (reversingNow) logf(1, "control setup: the reversers were still out %.0f s after letting go", sweepHeld_);
    } else {
        const double minHold = m.reverser >= 0 ? kSweepReverserSec : m.record == 5 ? kSweepToggleStepSec : kSweepMinHoldSec;
        const double maxHold = m.reverser >= 0 ? kSweepReverserSec + 1.5 : std::max(kSweepMaxHoldSec, minHold);
        if (!(sweepHeld_ >= minHold && sweepSteady_ >= kSweepSteadySec) && sweepHeld_ < maxHold) return;
        if (m.record == 1) sweepForward_.push_back({m.axis, v});
        if (m.record == 2) sweepReverse_.push_back({m.axis, v});
        if (m.record == 5 && std::isfinite(thrNow)) {
            // The reverse amount, on the throttle simvar (what replays set it from).
            if (m.axis <= 0.0) toggleIdle_ = thrNow;
            else toggleDeep_ = std::isfinite(toggleDeep_) ? std::min(toggleDeep_, thrNow) : thrNow;
            toggleFollows_ = std::isfinite(toggleIdle_) && toggleDeep_ <= toggleIdle_ - kSweepToggleFollowPct;
        }
    }
    ++sweepAt_;
    if (m.record == 2 && m.axis >= kSweepGiveUpAxis - 1e-9 && !reverseSeen_) {
        // The reverse events and half the axis haven't deployed anything: they don't reverse this
        // aircraft (the PMDG 737 moves its forward levers instead). Straight on to stowing.
        while (sweepAt_ < sweep_.size() && sweep_[sweepAt_].record == 2) ++sweepAt_;
        logf(0, "control setup: the reverse thrust events didn't deploy the reversers (axis up to %.2f)", m.axis);
    }
    if (m.record == 5 && m.axis > 0.0) {
        // Enough once the reversers are out and the amount follows the axis; given up at half the
        // axis if they aren't out. Then back to idle and toggled back.
        if ((toggleSeen_ && toggleFollows_) || (m.axis >= kSweepGiveUpAxis - 1e-9 && !toggleSeen_)) {
            while (sweepAt_ < sweep_.size() && sweep_[sweepAt_].record == 5) ++sweepAt_;
        }
        if (sweepAt_ >= sweep_.size() || sweep_[sweepAt_].record != 5) {
            logf(toggleSeen_ && !toggleFollows_ ? 1 : 0,
                 "control setup: toggle reverse %s (axis up to %.2f; throttle %.1f%% at reverse idle, %.1f%% at most)%s",
                 toggleSeen_ ? "deployed the reversers" : "didn't deploy the reversers", m.axis, toggleIdle_, toggleDeep_,
                 toggleSeen_ && !toggleFollows_ ? ", but the axis didn't set the reverse amount" : "");
        }
    }
    if (m.reverser == 0 && !reverseSeen_ && !toggleTried_) {
        // Stowed after the reverse events, which didn't work: try "toggle reverse".
        toggleTried_ = true;
        planToggle();
    }
    if (sweepAt_ >= sweep_.size() && !sweepExtended_) {
        // The forward pass is done. Unless the axis has its own reverse zone (the lever went
        // beyond idle, the other way from full), the reversers get a test of their own; then back
        // to idle.
        sweepExtended_ = true;
        const std::vector<LeverPoint> fwd = leverTable(sweepForward_);
        double lo = INFINITY, hi = -INFINITY;
        for (const LeverPoint& p : fwd) {
            lo = p.lever < lo ? p.lever : lo;
            hi = p.lever > hi ? p.lever : hi;
        }
        const double idle = std::isfinite(leverIdle_) ? leverIdle_ : (fwd.empty() ? NAN : fwd.front().lever);
        const double toFull = fwd.size() >= 2 && fwd.back().lever < fwd.front().lever ? -1.0 : 1.0;
        double beyond = 0.0;
        for (const LeverPoint& p : fwd) beyond = std::max(beyond, (idle - p.lever) * toFull);
        reverseZone_ = std::isfinite(idle) && beyond > std::max(0.01 * (hi - lo), 1e-9);
        if (facts_.hasReverse && !reverseZone_) planReverse();
        sweep_.push_back({fwd.size() >= 2 && std::isfinite(idle) ? invertLever(fwd, idle) : 0.0, 0, -1});
        logf(0, "control setup: forward pass done (L:var %s %.4g..%.4g)%s", lever_.var.c_str(), lo, hi,
             facts_.hasReverse ? (reverseZone_ ? "; the axis has its own reverse zone" : "; now the reversers") : "");
    }
    if (sweepAt_ >= sweep_.size()) {
        finishSweep();
        return;
    }
    const SweepMove& next = sweep_[sweepAt_];
    if (next.reverser == 2) {
        toggledOn_ = !toggledOn_;
        sim_->toggleReverse();
        logf(0, "control setup: toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE) %s", toggledOn_ ? "in" : "back out");
    } else if (next.reverser >= 0) {
        reverserOn_ = next.reverser == 1;
        sim_->setReverser(reverserOn_);
    }
    sweepTo(next.axis, true);
}

void SetupFlow::finishSweep() {
    endDrive();
    lever_.forward = leverTable(sweepForward_);
    std::vector<LeverPoint> rev = leverTable(sweepReverse_);
    double lo = 0.0, hi = 0.0;
    for (size_t i = 0; i < lever_.forward.size(); ++i) {
        const double l = lever_.forward[i].lever;
        lo = i == 0 || l < lo ? l : lo;
        hi = i == 0 || l > hi ? l : hi;
    }
    if (!reverseOnly_ && (lever_.forward.size() < 2 || !(hi - lo > 0.25 * leverSpan_))) {
        logf(1, "control setup: L:var %s moved %.4g over the axis sweep (%.4g when you held the positions)",
             lever_.var.c_str(), hi - lo, leverSpan_);
        lever_ = LeverCalibration();  // a retry looks for another variable
        stopSweep("The variable found for the throttle lever didn't follow the throttle.");
        return;
    }
    // How reverse thrust is commanded here: the axis itself, the reverse events (with a table of
    // the lever under them, if it moved), "toggle reverse", or "decrease throttle" held at idle
    // (which beats a toggle whose amount didn't follow the axis: both would replay all or nothing).
    ReverseBy by = ReverseBy::None;
    if (facts_.hasReverse) {
        by = reverseZone_                     ? ReverseBy::Axis
             : reverseSeen_                   ? ReverseBy::Events
             : toggleSeen_ && toggleFollows_  ? ReverseBy::Toggle
             : holdSeen_                      ? ReverseBy::Hold
             : toggleSeen_                    ? ReverseBy::Toggle
                                              : ReverseBy::None;
    }
    double rlo = 0.0, rhi = 0.0;
    for (size_t i = 0; i < rev.size(); ++i) {
        rlo = i == 0 || rev[i].lever < rlo ? rev[i].lever : rlo;
        rhi = i == 0 || rev[i].lever > rhi ? rev[i].lever : rhi;
    }
    if (by != ReverseBy::Events) {
        rev.clear();
    } else if (!rev.empty() && !(rhi - rlo > 0.02 * (hi - lo))) {
        logf(0, "control setup: the reverse thrust events didn't move L:var %s; reverse amounts from the simvar",
             lever_.var.c_str());
        rev.clear();
    }
    lever_.reverse = rev;
    reverseBy_ = by;
    if (facts_.hasReverse) {
        logf(by == ReverseBy::None ? 1 : 0, "control setup: reverse thrust: %s",
             by == ReverseBy::Axis     ? "on the throttle axis"
             : by == ReverseBy::Events ? "with the reverse thrust events, the axis setting the amount"
             : by == ReverseBy::Toggle ? (toggleFollows_ ? "with toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE), the axis "
                                                           "setting the amount"
                                                         : "with toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE), though "
                                                           "the axis didn't set the amount")
             : by == ReverseBy::Hold   ? "by holding THROTTLE_DECR (\"decrease throttle\") at idle"
                                       : "nothing tried deployed the reversers; replays leave reverse out");
    }
    if (reverseOnly_) {
        lever_ = LeverCalibration();
        std::vector<SweepMove>().swap(sweep_);
        finishStep(ControlState::Ok, chosen_, true);
        return;
    }
    // The detents: where the lever held one value over several steps.
    std::string detents;
    const double tol = 0.002 * (hi - lo);
    for (size_t i = 0; i < lever_.forward.size();) {
        size_t j = i;
        while (j + 1 < lever_.forward.size() && std::fabs(lever_.forward[j + 1].lever - lever_.forward[i].lever) <= tol) ++j;
        if (j > i) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%s%.4g (axis %.2f-%.2f)", detents.empty() ? "" : ", ", lever_.forward[i].lever,
                          lever_.forward[i].axis, lever_.forward[j].axis);
            detents += buf;
        }
        i = j + 1;
    }
    logf(0, "control setup: throttle lever calibrated: %zu forward point(s), %.4g..%.4g, detents %s; %zu reverse point(s)",
         lever_.forward.size(), lo, hi, detents.empty() ? "none" : detents.c_str(), lever_.reverse.size());
    std::vector<SweepMove>().swap(sweep_);
    finishStep(ControlState::Ok, chosen_, true);
}

void SetupFlow::stopSweep(const std::string& why) {
    parkThrottle();
    endDrive();
    std::vector<SweepMove>().swap(sweep_);
    note_ = why;
    phase_ = Phase::Retry;
    phaseT_ = 0.0;
}

void SetupFlow::parkThrottle() {
    if (!driving_) return;
    if (reverserOn_) {
        reverserOn_ = false;
        sim_->setReverser(false);
    }
    if (toggledOn_) {
        // Toggled into reverse: back out, with the levers at idle (axis 0 in reverse).
        toggledOn_ = false;
        sweepTo(0.0, true);
        sim_->toggleReverse();
        logf(1, "control setup: toggled reverse back out while stopping; check that the reversers are stowed");
    }
    const std::vector<LeverPoint> fwd = leverTable(sweepForward_);
    sweepTo(fwd.size() >= 2 && std::isfinite(leverIdle_) ? invertLever(fwd, leverIdle_) : 0.0, true);
}

void SetupFlow::finishStep(ControlState state, const ControlMethod& method, bool confirmed) {
    endDrive();
    endScan();
    const Control c = control();
    ControlProfile& cp = profile_[c];
    if (state == ControlState::Skipped && before_[c].state == ControlState::Ok) {
        cp = before_[c];  // skipped on a later run: it keeps what it had
        logf(0, "control setup: %s skipped; keeping its earlier setup (%s)", controlSpec(c).key, describeControl(c, cp).c_str());
    } else {
        cp.state = state;
        cp.method = state == ControlState::Ok ? method : ControlMethod();
        cp.confirmed = confirmed;
        const bool keyThrottle = state == ControlState::Ok && c == Control::Throttle && method.kind == MethodKind::KeyEvent;
        cp.lever = keyThrottle ? lever_ : LeverCalibration();
        cp.reverseBy = keyThrottle ? reverseBy_ : ReverseBy::Unknown;
        for (int d = 0; d < kDirections; ++d) cp.ways[d] = state == ControlState::Ok ? ways_[d] : std::vector<ControlMethod>();
        // The spoilers keep their calibration where a way of theirs is the event it was made with.
        if (state == ControlState::Ok && c == Control::Spoilers && spoilerCal_.valid()) {
            bool uses = false;
            for (int d = 0; d < kDirections; ++d) {
                for (const ControlMethod& m : cp.waysTo(static_cast<Direction>(d))) {
                    const char* event = spoilerPositionEvent(m);
                    uses |= event && spoilerCal_.var == event;
                }
            }
            if (uses) cp.lever = spoilerCal_;
        }
        // As much later as the confirmed replay moved it, for replays to make up (the throttle's
        // N1 follows its lever as it did for the pilot).
        const bool lead = state == ControlState::Ok && c != Control::Throttle && replayLag_ >= kMinLeadSec;
        cp.leadSec = lead ? std::min(replayLag_, kMaxLeadSec) : 0.0;
        logf(0, "control setup: %s %s%s%s", controlSpec(c).key, stateKey(state), state == ControlState::Ok ? " via " : "",
             state == ControlState::Ok ? describeControl(c, cp).c_str() : "");
        if (lead) logf(0, "control setup: replays drive the %s %.1f s ahead of the clip", controlSpec(c).label, cp.leadSec);
    }
    changed_ = true;
    clearStep();
    if (hovering()) {
        beginLand(LandThen::Next);
        return;
    }
    ++step_;
    enterStep();
}

void SetupFlow::clearStep() {
    endDrive();
    endScan();
    tracker_.clear();
    keys_.clear();
    mini_ = Clip();
    std::vector<double>().swap(keyColumn_);
    learned_.clear();
    methods_.clear();
    attempts_.clear();
    for (std::vector<ControlMethod>& w : ways_) w.clear();
    std::vector<Frame>().swap(capture_);
}

// ---------------------------------------------------------------------------- gear on the ground

bool SetupFlow::hovering() const { return sim_ && sim_->hovering(); }

double SetupFlow::hoverLift(const Frame& live) {
    const double agl = live.v[kAgl];
    return std::isfinite(agl) && agl >= 0.0 && agl <= kHoverMaxAglFt ? agl + kHoverClearFt : kHoverLiftFt;
}

// The gear step on the ground: the aircraft goes up once it has stopped (hoverFrame).
void SetupFlow::beginHover() {
    phase_ = Phase::Hover;
    phaseT_ = 0.0;
    hoverAirSec_ = 0.0;
    need_.clear();
}

void SetupFlow::hoverFrame(double dt, const Frame& live) {
    if (!hovering()) {
        if (!onGround_) {
            logf(0, "control setup: airborne; the gear is set up in the air");
            beginUser();
            return;
        }
        if (live.v[kGroundSpeed] > kStopKt) {
            need_ = "Come to a stop";
            phaseT_ = 0.0;
            return;
        }
        need_.clear();
        static const int kLegs[3] = {kGearCenter, kGearLeft, kGearRight};
        for (int i = 0; i < 3; ++i) hoverGear_[i] = live.v[kLegs[i]];
        hoverHandle_ = live.v[kGearHandle];
        const double lift = hoverLift(live);
        std::string why;
        if (!sim_->beginHover(lift, why)) {
            leaveHover("BetterReplay couldn't hold the aircraft there (" + why + ")");
            return;
        }
        logf(0, "control setup: holding the aircraft %.0f ft above where it stood (%.1f ft above the ground there), frozen, "
             "for the gear",
             lift, live.v[kAgl]);
        phaseT_ = 0.0;
        hoverAirSec_ = 0.0;
        return;
    }
    // The pilot is told to raise the gear once the sim shows the aircraft airborne up there.
    hoverAirSec_ = onGround_ ? 0.0 : hoverAirSec_ + dt;
    if (hoverAirSec_ >= kHoverSettleSec) {
        logf(0, "control setup: the aircraft counts as airborne above the runway (after %.1f s)", phaseT_);
        beginUser();
        return;
    }
    if (phaseT_ >= kHoverAirborneSec) leaveHover("the aircraft still counted as on the ground there");
}

// The gear can't be set up above the runway on this aircraft: it goes back onto the runway (once
// its gear is down) and the step waits for a takeoff instead.
void SetupFlow::leaveHover(const std::string& why) {
    clearStep();
    note_ = "Couldn't set up the gear above the runway: " + why + ". It's set up after takeoff instead.";
    logf(1, "control setup: can't set up the gear above the runway: %s; it's set up after takeoff instead", why.c_str());
    if (hovering()) {
        beginLand(LandThen::Takeoff);
    } else {
        enterTakeoff();
    }
}

void SetupFlow::enterTakeoff() {
    phase_ = Phase::Takeoff;
    phaseT_ = 0.0;
    groundSeen_ = false;
}

// The gear step on the ground has ended (or the setup is stopping): the aircraft goes back onto
// the runway once its gear is down, then `then`.
void SetupFlow::beginLand(LandThen then) {
    endDrive();
    endScan();
    landThen_ = then;
    phase_ = Phase::Land;
    phaseT_ = 0.0;
    taskDone_ = false;
    landDownSec_ = 0.0;
    landPut_ = false;
    landGroundSec_ = 0.0;
    logf(0, "control setup: putting the aircraft back on the runway once the gear is down");
}

bool SetupFlow::gearDown(const Frame& live) const {
    static const int kLegs[3] = {kGearCenter, kGearLeft, kGearRight};
    for (int i = 0; i < 3; ++i) {
        const double want = hoverGear_[i];
        const double v = live.v[kLegs[i]];
        if (std::isfinite(want) && std::isfinite(v) && v < std::min(want, 1.0) - kGearDownTol) return false;
    }
    // And its lever, where the handle simvar showed it down on the runway: one left up while the
    // gear didn't move might raise it once the aircraft is let go.
    return !(hoverHandle_ >= 0.5) || live.v[kGearHandle] >= 0.5;
}

// Land and Lower: until the gear is down (the pilot is asked after kLandGearWaitSec), then the
// aircraft goes back, until the sim shows it on the ground.
void SetupFlow::landFrame(double dt, const Frame& live) {
    if (landPut_) {
        landGroundSec_ = onGround_ ? landGroundSec_ + dt : 0.0;
        if (landGroundSec_ < kLandSettleSec && phaseT_ < kLandSettleMaxSec) return;
        if (landGroundSec_ < kLandSettleSec) {
            logf(1, "control setup: the sim doesn't show the aircraft on the ground %.0f s after putting it back; going on",
                 kLandSettleMaxSec);
        }
        afterLand();
        return;
    }
    if (!hovering()) {  // (let go meanwhile)
        afterLand();
        return;
    }
    landGear_ = meanOf(live, kGearCenter, 3);
    landDownSec_ = gearDown(live) ? landDownSec_ + dt : 0.0;
    if (landDownSec_ >= kLandDownSec) {
        if (phase_ == Phase::Lower) logf(0, "control setup: the gear is down");
        putBack();
        return;
    }
    if (phase_ == Phase::Land && phaseT_ >= kLandGearWaitSec) {
        logf(1, "control setup: the gear isn't down after %.0f s (at %.2f, its handle at %.2f); asking you to lower it",
             kLandGearWaitSec, landGear_, live.v[kGearHandle]);
        phase_ = Phase::Lower;
        phaseT_ = 0.0;
    }
}

void SetupFlow::putBack() {
    logf(0, "control setup: putting the aircraft back where it stood on the runway (gear at %.2f)", landGear_);
    sim_->endHover(true);
    landPut_ = true;
    landGroundSec_ = 0.0;
    phase_ = Phase::Land;
    phaseT_ = 0.0;
}

void SetupFlow::afterLand() {
    landPut_ = false;
    switch (landThen_) {
        case LandThen::Next:
            ++step_;
            enterStep();
            break;
        case LandThen::Takeoff: enterTakeoff(); break;
        case LandThen::Stop: stop(landWhy_); break;
    }
}

void SetupFlow::endScan() {
    if (!scanning_) return;
    scanning_ = false;
    if (sim_) sim_->scanEnd();
}

void SetupFlow::endDrive() {
    if (!driving_) return;
    driving_ = false;
    if (sim_ && step_ < plan_.size()) sim_->endControl(control());
}

void SetupFlow::cancel(const char* why) {
    if (!active()) return;
    if (phase_ == Phase::Land || phase_ == Phase::Lower) {
        // Putting the aircraft back already: it stops once that's done. Asked to lower the gear,
        // the pilot cancelled: it goes back as it is.
        if (landThen_ != LandThen::Stop) {
            landThen_ = LandThen::Stop;
            landWhy_ = why;
        }
        if (phase_ == Phase::Lower) {
            logf(1, "control setup: cancelled with the gear still up");
            putBack();
        }
        return;
    }
    if (phase_ == Phase::Sweep) parkThrottle();
    if (phase_ == Phase::Calibrate) sim_->applyControl(control(), mini_.frames.front(), true);  // spoilers back in
    if (hovering()) {
        clearStep();
        landWhy_ = why;
        logf(0, "control setup stopping (%s)", why);
        beginLand(LandThen::Stop);
        return;
    }
    stop(why);
}

void SetupFlow::abandon(const char* why) {
    if (!active()) return;
    letGoBrakes(false);  // the sim reset the aircraft: its parking brake is as the flight has it
    if (phase_ == Phase::Sweep) parkThrottle();
    if (hovering()) {
        logf(0, "control setup: letting the aircraft go where it is (%s)", why);
        sim_->endHover(false);
    }
    stop(why);
}

void SetupFlow::stop(const std::string& why) {
    letGoBrakes(true);
    clearStep();
    phase_ = Phase::Idle;
    logf(0, "control setup stopped (%s)", why.c_str());
}

bool SetupFlow::answer(const std::string& a, std::string& error) {
    if (!active()) {
        error = "the control setup isn't running";
        return false;
    }
    if (a == "cancel" || a == "later") {
        cancel(a == "later" ? "finish later" : "cancelled");
        return true;
    }
    switch (phase_) {
        case Phase::Ask:
            if (a == "yes") {
                note_.clear();  // the step tracker shows it
                confirmMethod();
                return true;
            }
            if (a == "no") {
                attempts_.push_back("rejected");
                note_.clear();
                if (static_cast<int>(method_) == shortBest_) shortBest_ = -1;  // not to come back to
                nextMethod();
                return true;
            }
            if (a == "again") {
                askAnyway_ = true;  // asked about again however far it goes this time
                startMethod(method_);
                return true;
            }
            break;
        case Phase::Retry:
            if (a == "retry") {
                note_.clear();
                if (!leverStage_) {
                    beginUser();
                } else if (lever_.var.empty() && !reverseOnly_) {
                    beginPositions(nullptr);
                } else {
                    enterNotice();
                }
                return true;
            }
            break;
        case Phase::Query:
            if (a == "yes" || a == "no") {
                logf(0, "control setup: you said the throttle levers %s under autothrust",
                     a == "yes" ? "stay put" : "move with the thrust");
                if (a == "yes") {
                    leverFromMovement();
                } else {
                    reverseOnly();
                }
                return true;
            }
            break;
        case Phase::Notice:
            if (a == "ready" || a == "begin") {  // without the parking brake ("begin": the panels before)
                logf(0, "control setup: continuing without \"%s\"", need_.c_str());
                beginSweep();
                return true;
            }
            break;
        case Phase::Prepare:
            // The pilot says it's ready (e.g. an aircraft whose parking brake isn't the simvar).
            // Being on the ground (or in the air for the gear) still counts.
            if (a == "ready" && (control() == Control::Gear ? !onGround_ : onGround_)) {
                logf(0, "control setup: continuing without \"%s\"", need_.c_str());
                beginUser();
                return true;
            }
            break;
        case Phase::Done:
            if (a == "close") {
                phase_ = Phase::Idle;
                return true;
            }
            break;
        default: break;
    }
    if (a == "skip" && phase_ != Phase::Done && phase_ != Phase::Replay && phase_ != Phase::Hold && phase_ != Phase::Sweep &&
        phase_ != Phase::Back && phase_ != Phase::Land && phase_ != Phase::Lower && phase_ != Phase::Calibrate) {
        note_.clear();
        if (leverStage_) {
            // The method stays; the lever keeps an earlier calibration made for the same method.
            const ControlProfile& was = before_[Control::Throttle];
            const bool same = was.state == ControlState::Ok && was.method == chosen_;
            lever_ = same ? was.lever : LeverCalibration();
            reverseBy_ = same ? was.reverseBy : ReverseBy::Unknown;
            logf(0, "control setup: %s skipped%s", reverseOnly_ ? "reverse thrust test" : "lever calibration",
                 same ? "; keeping the earlier setup" : "");
            finishStep(ControlState::Ok, chosen_, true);
        } else {
            finishStep(ControlState::Skipped, ControlMethod(), false);
        }
        return true;
    }
    error = "'" + a + "' doesn't fit this step";
    return false;
}

bool SetupFlow::takeChanged() {
    const bool c = changed_;
    changed_ = false;
    return c;
}

// One short instruction or question at a time.
std::string SetupFlow::prompt() const {
    if (phase_ == Phase::Done) return "Setup finished";
    const Control c = control();
    const TaskList tasks = tasksFor(c);
    switch (phase_) {
        case Phase::Prepare: return need_.empty() ? tasks.items[0] : need_;
        case Phase::User: return tasks.items[task_ < tasks.count ? task_ : tasks.count - 1];
        case Phase::Away: return tasks.items[0];
        case Phase::Takeoff: return "Take off";
        case Phase::Hover: return need_.empty() ? kHoverPrompt : need_;
        case Phase::Land:
            return landPut_ || landDownSec_ > 0.0 ? "Putting the aircraft back" : "Waiting for the gear to come down";
        case Phase::Lower: return "Lower the gear";
        case Phase::Retry: return "That didn't register";
        case Phase::Back:
            return backFrom_ == BackFrom::Pilot ? bringBackWhat(c) : watchWhat(c);
        case Phase::Replay:
        case Phase::Hold:
            return watchWhat(c);
        case Phase::Ask: return question(c);
        case Phase::Query: return kQueryPrompt;
        case Phase::Positions: return kThrottleTasks[task_ < kPositionTasks ? task_ : kPositionTasks - 1];
        case Phase::Notice: return need_.empty() ? "Hands off the throttle" : need_;
        case Phase::Sweep: return "Moving the throttle";
        case Phase::Calibrate: return watchWhat(c);
        default: return "";
    }
}

void SetupFlow::statusJson(JsonWriter& w) const {
    w.kv("active", active());
    if (!active()) return;
    static const char* const kPhases[] = {"idle",  "prepare", "takeoff", "user",  "retry",     "replay", "hold", "ask",
                                          "done",  "query",   "positions", "notice", "sweep", "away", "back",
                                          "hover", "land",    "lower", "calibrate"};
    w.kv("phase", kPhases[static_cast<int>(phase_)]);
    const bool done = phase_ == Phase::Done;
    w.kv("step", done ? "" : controlSpec(control()).key);
    w.kv("title", done ? std::string("Done") : title(control()));
    w.kv("stepNo", static_cast<int>(done ? plan_.size() : step_ + 1));
    w.kv("steps", static_cast<int>(plan_.size()));
    w.kv("prompt", prompt());
    // The pilot's instructions in this step: which one is showing, and whether it's done (the
    // last one, while things settle). The throttle's holds also say how long it's been held.
    // Away and Lower are one instruction each (the first again; "Lower the gear").
    const bool single = phase_ == Phase::Away || phase_ == Phase::Lower;
    const bool pilot = phase_ == Phase::User || phase_ == Phase::Positions || single;
    const bool holding = pilot && !single && control() == Control::Throttle && task_ < kThrottleHolds;
    const bool bringBack = phase_ == Phase::Back && backFrom_ == BackFrom::Pilot;
    const char* detail = "";
    switch (phase_) {
        case Phase::User: detail = holding ? kHoldDetail : hovering() ? kHeldDetail : ""; break;
        case Phase::Away: detail = kAwayDetail; break;
        case Phase::Back: detail = bringBack ? kBringBackDetail : ""; break;
        case Phase::Hover: detail = kHoverDetail; break;
        case Phase::Land: detail = kLandDetail; break;
        case Phase::Lower: detail = kLowerDetail; break;
        case Phase::Query: detail = kQueryDetail; break;
        case Phase::Positions: detail = kPositionsDetail; break;
        case Phase::Notice: detail = reverseOnly_ ? kReverseNoticeDetail : kNoticeDetail; break;
        case Phase::Sweep: detail = reverseOnly_ ? kReverseSweepDetail : kSweepDetail; break;
        case Phase::Calibrate: detail = kCalibrateDetail; break;
        case Phase::Takeoff: detail = kTakeoffDetail; break;
        case Phase::Done: detail = kDoneDetail; break;
        default: break;
    }
    w.kv("detail", detail);
    w.kv("task", static_cast<int>(pilot && !single ? task_ : 0));
    w.kv("tasks", static_cast<int>(done                         ? 0
                                   : single                     ? 1
                                   : phase_ == Phase::Positions ? kPositionTasks
                                                                : tasksFor(control()).count));
    w.kv("taskDone", pilot && taskDone_);
    w.kv("holdTask", holding);
    w.kv("hold", holding ? hold_ : 0.0);
    w.kv("note", note_);
    // Bringing the control back from where the pilot left it is a pass of its own, numbered by the
    // methods it tries (not the simvars); the replays of the whole movement then start at 1.
    w.kv("bringBack", bringBack);
    w.kv("method", static_cast<int>(bringBack ? backAt_ + 1 : method_ + 1));
    w.kv("methods", static_cast<int>(bringBack ? backTries_.size() : methods_.size()));
    w.kv("left", attemptLeft());
    // The sweep's planned moves aren't all known until the forward pass is done: estimate.
    const size_t rest = sweepExtended_ ? 0 : (facts_.hasReverse ? kSweepReverseSteps + 5 : 1);
    w.kv("progress", phase_ == Phase::Sweep && !sweep_.empty()
                         ? static_cast<double>(sweepAt_) / static_cast<double>(sweep_.size() + rest)
                     : phase_ == Phase::Calibrate ? static_cast<double>(calStep_) / (kCalSteps + 2.0)
                                                  : 0.0);
    // How each method before the current one ended: "failed" (the automatic check), "short" (it
    // passed, but went less than 90% as far as the pilot: the others are tried first), "rejected"
    // (the pilot said no) or "unavailable" (the aircraft hasn't got what it needs). Replaying the
    // one that went furthest once more after the others, "method" is one of these.
    w.key("attempts").beginArray();
    for (const std::string& a : attempts_) w.value(a);
    w.endArray();
    // Every step of this run: finished ones with their result, then "current" and "next".
    w.key("plan").beginArray();
    for (size_t i = 0; i < plan_.size(); ++i) {
        const Control c = plan_[i];
        w.beginObject();
        w.kv("control", controlSpec(c).key);
        w.kv("state", i < step_ ? stateKey(profile_[c].state) : i == step_ ? "current" : "next");
        w.endObject();
    }
    w.endArray();
    w.key("buttons").beginArray();
    switch (phase_) {
        case Phase::Prepare:
            if (!need_.empty()) w.value("ready");
            w.value("skip").value("cancel");
            break;
        case Phase::User:
        case Phase::Away:
        case Phase::Hover: w.value("skip").value("cancel"); break;
        case Phase::Takeoff: w.value("skip").value("later"); break;
        case Phase::Lower: w.value("cancel"); break;
        case Phase::Retry: w.value("retry").value("skip").value("cancel"); break;
        case Phase::Replay:
        case Phase::Hold:
        case Phase::Back: w.value("cancel"); break;
        case Phase::Ask: w.value("yes").value("no").value("again").value("skip"); break;
        case Phase::Query: w.value("yes").value("no"); break;
        case Phase::Positions: w.value("skip").value("cancel"); break;
        case Phase::Notice: w.value("ready").value("skip").value("cancel"); break;
        case Phase::Sweep:
        case Phase::Calibrate: w.value("cancel"); break;
        case Phase::Done: w.value("close"); break;
        default: break;
    }
    w.endArray();
}

}  // namespace br
