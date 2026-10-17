#include "ControlSetup.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

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
constexpr double kMaxRollKt = 3.0;   // the throttle replay stops if the aircraft starts to roll

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

const char* const* wordsFor(Control c) {
    switch (c) {
        case Control::Spoilers: return kSpoilerWords;
        case Control::Flaps: return kFlapWords;
        case Control::Throttle: return kThrottleWords;
        case Control::Gear: return kGearWords;
    }
    return nullptr;
}

const char* instruction(Control c) {
    switch (c) {
        case Control::Spoilers: return "Deploy the spoilers fully, then retract them.";
        case Control::Flaps: return "Lower the flaps one notch, wait for them to extend, then retract them.";
        case Control::Throttle: return "Advance the throttle to full, then back to idle.";
        case Control::Gear: return "Raise the gear, then lower it again.";
    }
    return "";
}

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

const char* backHint(Control c) {
    switch (c) {
        case Control::Spoilers:
        case Control::Flaps: return "Now retract them.";
        case Control::Throttle: return "Now back to idle.";
        case Control::Gear: return "Now lower it again.";
    }
    return "";
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

// ---------------------------------------------------------------------------- flow

void SetupFlow::setLogger(LogFn fn, void* ctx) {
    log_ = fn;
    logCtx_ = ctx;
}

void SetupFlow::logf(int level, const char* fmt, ...) const {
    if (!log_) return;
    char buf[400];
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
        case Control::Flaps: return std::isfinite(f.v[kFlapsLeft]) ? f.v[kFlapsLeft] : f.v[kFlapsHandleIndex];
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
    detail_.clear();
    phaseT_ = 0.0;
    if (step_ >= plan_.size()) {
        phase_ = Phase::Done;
        std::string summary;
        for (int i = 0; i < kControlCount; ++i) {
            const Control c = controlAt(i);
            if (profile_[c].state == ControlState::Absent) continue;
            summary += (summary.empty() ? "" : " · ") + title(c) + ": " + stateLabel(profile_[c].state);
        }
        detail_ = summary;
        logf(0, "control setup finished: %s", summary.c_str());
        return;
    }
    const Control c = control();
    phase_ = c == Control::Gear && onGround_ ? Phase::Takeoff : Phase::Prepare;
    logf(0, "control setup: step %zu of %zu, %s", step_ + 1, plan_.size(), controlSpec(c).key);
}

const char* SetupFlow::precondition(const Frame& live) const {
    const Control c = control();
    if (c == Control::Gear) return onGround_ ? "Take off first." : nullptr;
    if (!onGround_) return "Land and stop on the runway to continue.";
    if (live.v[kGroundSpeed] > 2.0) return "Come to a stop first.";
    if (c == Control::Throttle) {
        if (!facts_.engineRunning) return "Start the engines first.";
        if (!facts_.parkingBrake) return "Set the parking brake first.";
        return nullptr;
    }
    // Ground spoilers stow when the thrust levers are advanced. Some aircraft's lever simvar
    // doesn't read 0 at idle, so this is loose (and "Continue anyway" skips it).
    const double thr = meanOf(live, kThrottle1, engineCount(facts_));
    if (std::isfinite(thr) && thr > 25.0) return "Bring the throttles back to idle first.";
    return nullptr;
}

void SetupFlow::onFrame(double dt, const Frame& live, const SetupFacts& facts) {
    if (!active()) return;
    facts_ = facts;
    onGround_ = live.v[kOnGround] >= 0.5;
    phaseT_ += dt;
    switch (phase_) {
        case Phase::Prepare: {
            const char* need = precondition(live);
            detail_ = need ? need : "";
            if (!need) beginUser();
            break;
        }
        case Phase::Takeoff:
            if (!onGround_) beginUser();
            break;
        case Phase::User: userFrame(dt, live); break;
        case Phase::Replay: replayFrame(dt, live); break;
        case Phase::Hold: holdFrame(live); break;
        default: break;
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
    detail_.clear();
    std::vector<ScanVar> vars;
    sim_->scanBegin(vars);
    scanning_ = true;
    size_t lvars = 0;
    for (const ScanVar& v : vars) lvars += v.lvar ? 1 : 0;
    logf(0, "control setup: watching %zu input event(s) and %zu L:var(s) while you move the %s", vars.size() - lvars,
         lvars, controlSpec(control()).label);
    tracker_.begin(std::move(vars));
    scanValues_.assign(tracker_.size(), NAN);
}

void SetupFlow::userFrame(double dt, const Frame& live) {
    t_ += dt;
    const Tuning tu = tuning(control());
    const double h = handleSignal(live);
    const double o = outcomeSignal(live);
    if (std::isnan(handleRest_)) handleRest_ = h;
    if (std::isnan(outcomeRest_)) outcomeRest_ = o;
    const double dh = absDiff(h, handleRest_);
    const double dout = absDiff(o, outcomeRest_);

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
    if (scanning_ && t_ + 1e-9 >= nextScan_) {
        sim_->scanRead(scanValues_);
        tracker_.sample(t_, scanValues_, h, o);
        nextScan_ += kScanInterval;
        if (nextScan_ < t_) nextScan_ = t_ + kScanInterval;
        if (!moved_) tracker_.dropChangedBefore(t_ - kSearchLeadSec - 1.0);
    }

    if (!moved_ && (dh > tu.handleStart || dout > tu.outcomeStart)) {
        moved_ = true;
        movedAt_ = t_;
        tracker_.dropChangedBefore(t_ - kSearchLeadSec);
        logf(0, "control setup: the %s started moving (%.3g -> %.3g, outcome %.3g -> %.3g); %zu variable(s) changing",
             controlSpec(control()).label, handleRest_, h, outcomeRest_, o, tracker_.stored());
    }
    if (moved_) {
        handlePeak_ = dh > handlePeak_ ? dh : handlePeak_;
        outcomePeak_ = dout > outcomePeak_ ? dout : outcomePeak_;
    }
    if (movedFrom(h, handleAnchor_, 0.5 * tu.handleStart) || movedFrom(o, outcomeAnchor_, 0.5 * tu.outcomeStart)) {
        handleAnchor_ = h;
        outcomeAnchor_ = o;
        stableSince_ = t_;
    }
    const bool enough = handlePeak_ >= tu.handleNeed || outcomePeak_ >= tu.outcomeNeed;
    const bool back = dh <= std::max(tu.handleStart, 0.1 * handlePeak_) && dout <= std::max(tu.outcomeStart, 0.1 * outcomePeak_);
    const bool stable = t_ - stableSince_ >= kStableSec;
    if (!moved_ || !enough) {
        detail_.clear();
    } else if (!back) {
        detail_ = backHint(control());
    } else if (!stable) {
        detail_ = control() == Control::Throttle ? "Waiting for the engines to spool down..." : "Waiting for it to settle...";
    }
    if (moved_ && enough && back && stable) {
        analyze();
        return;
    }
    if (phaseT_ > tu.waitMaxSec) {
        endScan();
        tracker_.clear();
        capture_.clear();
        phase_ = Phase::Retry;
        phaseT_ = 0.0;
        detail_.clear();
        const std::string label = controlSpec(control()).label;
        note_ = moved_ ? "The " + label + " didn't go all the way and come back." : "Didn't see the " + label + " move.";
        logf(1, "control setup: %s", note_.c_str());
    }
}

void SetupFlow::analyze() {
    endScan();
    const Control c = control();
    const Tuning tu = tuning(c);
    // The replay starts just before whatever changed first: the control's simvar, or one of the
    // variables the methods use.
    windowFrom_ = std::max(0.0, movedAt_ - kSearchLeadSec);
    const double firstChange = buildMethods();
    windowFrom_ = std::max(0.0, std::min(movedAt_, firstChange) - kLeadInSec);
    mini_ = Clip();
    mini_.present.set();
    for (const Frame& f : capture_) {
        if (f.t < windowFrom_ - 1e-9) continue;
        mini_.frames.push_back(f);
        mini_.frames.back().t = f.t - windowFrom_;
    }
    std::vector<Frame>().swap(capture_);
    if (c == Control::Gear) deriveGearCommand(mini_);
    judgeOutcome_ = outcomePeak_ >= tu.outcomeNeed;
    userPeak_ = judgeOutcome_ ? outcomePeak_ : handlePeak_;
    logf(0, "control setup: you moved the %s over %.1f s (handle moved %.3g, outcome %.3g; checking replays on the %s); "
         "trying %zu method(s)",
         controlSpec(c).label, mini_.duration(), handlePeak_, outcomePeak_, judgeOutcome_ ? "outcome" : "handle",
         methods_.size());
    if (methods_.empty() || mini_.frames.size() < 2) {
        note_ = "Couldn't replay the " + std::string(controlSpec(c).label) + ".";
        finishStep(ControlState::Failed, ControlMethod(), false);
        return;
    }
    startMethod(0);
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
    const std::vector<CandidateTracker::Found> inputs = tracker_.find(windowFrom_, t_, false, byOutcome, wordsFor(c), 4);
    const std::vector<CandidateTracker::Found> lvars = tracker_.find(windowFrom_, t_, true, byOutcome, wordsFor(c), 4);
    double firstChange = INFINITY;
    for (const auto& f : inputs) {
        logf(0, "control setup:   input event %s moved with it (score %.2f, peak %.3g)", tracker_.var(f.index).name.c_str(),
             f.score, f.peak);
    }
    for (const auto& f : lvars) {
        logf(0, "control setup:   L:var %s moved with it (score %.2f, peak %.3g)", tracker_.var(f.index).name.c_str(), f.score,
             f.peak);
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
    if (c != Control::Flaps || facts_.flapPositions > 1) methods_.push_back({MethodKind::KeyEvent, std::string(), 1.0});
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

void SetupFlow::startMethod(size_t i) {
    const Control c = control();
    method_ = i;
    const ControlMethod& m = methods_[i];
    std::string why;
    if (!sim_->beginControl(c, m, why)) {
        logf(1, "control setup: can't try %s: %s", describeMethod(c, m).c_str(), why.c_str());
        nextMethod();
        return;
    }
    driving_ = true;
    // The method reads a channel the pilot's movement didn't record under this name: fill it in
    // from the scanned variable.
    if (m.kind == MethodKind::InputSet || m.kind == MethodKind::InputValue || m.kind == MethodKind::LVar) {
        const bool lvar = m.kind == MethodKind::LVar;
        const int n = c == Control::Throttle && m.var.find('#') != std::string::npos ? engineCount(facts_) : 1;
        for (int e = 0; e < n; ++e) {
            const int ch = methodSourceChannel(c, m.kind, e);
            size_t idx = 0;
            const bool have = tracker_.lookup(engineVar(m.var, e + 1), lvar, idx);
            for (Frame& f : mini_.frames) f.v[ch] = have ? tracker_.valueAt(idx, windowFrom_ + f.t) : NAN;
        }
    }
    replayT_ = 0.0;
    hint_ = 0;
    firstApply_ = true;
    replayPeak_ = 0.0;
    stoppedEarly_ = false;
    phase_ = Phase::Replay;
    phaseT_ = 0.0;
    detail_.clear();
    logf(0, "control setup: replaying the %s with %s (method %zu of %zu)", controlSpec(c).label,
         describeMethod(c, m).c_str(), i + 1, methods_.size());
}

void SetupFlow::track(const Frame& live) {
    const double v = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double rest = judgeOutcome_ ? outcomeRest_ : handleRest_;
    const double d = absDiff(v, rest);
    if (d > replayPeak_) replayPeak_ = d;
}

bool SetupFlow::rolling(const Frame& live) const {
    return control() == Control::Throttle && onGround_ && live.v[kGroundSpeed] > kMaxRollKt;
}

void SetupFlow::replayFrame(double dt, const Frame& live) {
    const Control c = control();
    replayT_ += dt;
    const double dur = mini_.duration();
    sampleFrames(mini_.frames.data(), mini_.frames.size(), replayT_ < dur ? replayT_ : dur, frame_, &hint_);
    sim_->applyControl(c, frame_, firstApply_);
    firstApply_ = false;
    track(live);
    if (rolling(live)) {
        stoppedEarly_ = true;
        sim_->applyControl(c, mini_.frames.front(), true);
        logf(1, "control setup: the aircraft started to roll (%.1f kt); stopping the throttle replay", live.v[kGroundSpeed]);
        endDrive();
        evaluate(live);
        return;
    }
    if (replayT_ >= dur) {
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
    sim_->applyControl(c, mini_.frames.back(), false);
    track(live);
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
    if ((phaseT_ >= 1.0 && phaseT_ - replayStable_ >= kStableSec) || phaseT_ >= tu.holdMaxSec) {
        endDrive();
        evaluate(live);
    }
}

void SetupFlow::evaluate(const Frame& live) {
    const Control c = control();
    const Tuning tu = tuning(c);
    const double end = judgeOutcome_ ? outcomeSignal(live) : handleSignal(live);
    const double rest = judgeOutcome_ ? outcomeRest_ : handleRest_;
    const double tol = std::max(judgeOutcome_ ? tu.outcomeStart : tu.handleStart, kPassReturn * userPeak_);
    const bool reached = replayPeak_ >= kPassPeak * userPeak_;
    const bool returned = absDiff(end, rest) <= tol;
    const int pct = userPeak_ > 0.0 ? static_cast<int>(std::lround(100.0 * replayPeak_ / userPeak_)) : 0;
    const std::string desc = describeMethod(c, methods_[method_]);
    if (stoppedEarly_ || (reached && returned)) {
        logf(0, "control setup: %s moved the %s %d%% as far as you did%s; asking", desc.c_str(), controlSpec(c).label, pct,
             returned ? "" : " (it didn't come all the way back)");
        note_ = stoppedEarly_ ? "Stopped early: the aircraft started to roll. Is the parking brake set?" : "";
        phase_ = Phase::Ask;
        phaseT_ = 0.0;
        detail_.clear();
        return;
    }
    logf(0, "control setup: %s moved the %s %d%% as far as you did%s", desc.c_str(), controlSpec(c).label, pct,
         reached && !returned ? ", but didn't bring it back" : "");
    note_ = "Method " + std::to_string(method_ + 1) + " didn't move the " + controlSpec(c).label + "; trying the next.";
    nextMethod();
}

void SetupFlow::nextMethod() {
    if (method_ + 1 < methods_.size()) {
        startMethod(method_ + 1);
        return;
    }
    const Control c = control();
    logf(1, "control setup: none of the %zu method(s) replayed the %s; playback will leave it to the aircraft",
         methods_.size(), controlSpec(c).label);
    note_ = "Couldn't replay the " + std::string(controlSpec(c).label) + " on this aircraft; replays will leave them alone.";
    finishStep(ControlState::Failed, ControlMethod(), false);
}

void SetupFlow::finishStep(ControlState state, const ControlMethod& method, bool confirmed) {
    endDrive();
    endScan();
    const Control c = control();
    ControlProfile& cp = profile_[c];
    cp.state = state;
    cp.method = state == ControlState::Ok ? method : ControlMethod();
    cp.confirmed = confirmed;
    changed_ = true;
    logf(0, "control setup: %s %s%s%s", controlSpec(c).key, stateKey(state), state == ControlState::Ok ? " via " : "",
         state == ControlState::Ok ? describeMethod(c, method).c_str() : "");
    tracker_.clear();
    mini_ = Clip();
    methods_.clear();
    std::vector<Frame>().swap(capture_);
    ++step_;
    enterStep();
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
    endDrive();
    endScan();
    tracker_.clear();
    mini_ = Clip();
    methods_.clear();
    std::vector<Frame>().swap(capture_);
    phase_ = Phase::Idle;
    logf(0, "control setup stopped (%s)", why);
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
                note_ = title(control()) + " set up.";
                finishStep(ControlState::Ok, methods_[method_], true);
                return true;
            }
            if (a == "no") {
                note_ = "Trying the next method.";
                nextMethod();
                return true;
            }
            if (a == "again") {
                startMethod(method_);
                return true;
            }
            break;
        case Phase::Retry:
            if (a == "retry") {
                note_.clear();
                beginUser();
                return true;
            }
            break;
        case Phase::Prepare:
            // The pilot says it's ready (e.g. an aircraft whose parking brake isn't the simvar).
            // Being on the ground (or in the air for the gear) still counts.
            if (a == "ready" && (control() == Control::Gear ? !onGround_ : onGround_)) {
                logf(0, "control setup: continuing without \"%s\"", detail_.c_str());
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
    if (a == "skip" && phase_ != Phase::Done && phase_ != Phase::Replay && phase_ != Phase::Hold) {
        note_ = title(control()) + " skipped.";
        finishStep(ControlState::Skipped, ControlMethod(), false);
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

std::string SetupFlow::prompt() const {
    if (phase_ == Phase::Done) return "Control setup finished.";
    const Control c = control();
    switch (phase_) {
        case Phase::Prepare:
        case Phase::User: return instruction(c);
        case Phase::Takeoff:
            return "Take off. Once you're climbing, raise the gear, then lower it again. Stay well below the gear speed limit.";
        case Phase::Retry: return "That didn't register. Try again?";
        case Phase::Replay:
        case Phase::Hold:
            return std::string(watchWhat(c)) + ": replaying your movement. Hands off the controls.";
        case Phase::Ask: return question(c);
        default: return "";
    }
}

void SetupFlow::statusJson(JsonWriter& w) const {
    w.kv("active", active());
    if (!active()) return;
    static const char* const kPhases[] = {"idle", "prepare", "takeoff", "user", "retry", "replay", "hold", "ask", "done"};
    w.kv("phase", kPhases[static_cast<int>(phase_)]);
    const bool done = phase_ == Phase::Done;
    w.kv("step", done ? "" : controlSpec(control()).key);
    w.kv("title", done ? std::string("Done") : title(control()));
    w.kv("stepNo", static_cast<int>(done ? plan_.size() : step_ + 1));
    w.kv("steps", static_cast<int>(plan_.size()));
    w.kv("prompt", prompt());
    std::string detail = detail_;
    if (phase_ == Phase::Replay || phase_ == Phase::Hold) {
        detail = "Method " + std::to_string(method_ + 1) + " of " + std::to_string(methods_.size()) + ": " +
                 describeMethod(control(), methods_[method_]) + ".";
    }
    w.kv("detail", detail);
    w.kv("note", note_);
    w.kv("method", static_cast<int>(method_ + 1));
    w.kv("methods", static_cast<int>(methods_.size()));
    w.key("buttons").beginArray();
    switch (phase_) {
        case Phase::Prepare:
            if (!detail_.empty()) w.value("ready");
            w.value("skip").value("cancel");
            break;
        case Phase::User: w.value("skip").value("cancel"); break;
        case Phase::Takeoff: w.value("skip").value("later"); break;
        case Phase::Retry: w.value("retry").value("skip").value("cancel"); break;
        case Phase::Replay:
        case Phase::Hold: w.value("cancel"); break;
        case Phase::Ask: w.value("yes").value("no").value("again").value("skip"); break;
        case Phase::Done: w.value("close"); break;
        default: break;
    }
    w.endArray();
}

}  // namespace br
