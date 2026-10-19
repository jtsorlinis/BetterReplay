#include "Engine.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "ClipFormat.h"
#include "Json.h"
#include "TimeUtil.h"

namespace br {

namespace {

constexpr const char* kIndexFile = "index.json";
constexpr const char* kIndexTmp = "index.json.tmp";
constexpr const char* kProfilesFile = "aircraft.json";
constexpr const char* kProfilesTmp = "aircraft.json.tmp";
constexpr const char* kReasonPaused = "paused";
constexpr const char* kReasonSetup = "control setup";

class RingSpan : public FrameSource {
public:
    RingSpan(const RingBuffer<Frame>& ring, size_t first, size_t count) : ring_(ring), first_(first), count_(count) {}
    size_t size() const override { return count_; }
    const Frame& at(size_t i) const override { return ring_.at(first_ + i); }

private:
    const RingBuffer<Frame>& ring_;
    size_t first_;
    size_t count_;
};

double frameTime(const Frame& f) { return f.t; }

// PlayMode::Event channels are on/off (lights, gear handle); during playback only a change of
// state is sent, so a handle recorded mid-travel at 0.3, 0.6, 0.9 produces one event, not three.
double eventState(double v) { return std::isnan(v) ? NAN : (v >= 0.5 ? 1.0 : 0.0); }

bool equalsIgnoringCase(const std::string& a, const std::string& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        const char x = a[i] >= 'A' && a[i] <= 'Z' ? static_cast<char>(a[i] + 32) : a[i];
        const char y = b[i] >= 'A' && b[i] <= 'Z' ? static_cast<char>(b[i] + 32) : b[i];
        if (x != y) return false;
    }
    return true;
}

// The engines' power for the takeoff's spool-up: the highest N1, or RPM on pistons.
double enginePower(const Frame& f, int engineType) {
    if (engineType == 2) return NAN;  // ENGINE TYPE none: a glider
    const int first = engineType == 0 ? kEngineRpm1 : kEngineN1_1;
    double p = NAN;
    for (int e = 0; e < 4; ++e) {
        const double v = f.v[first + e];
        if (std::isfinite(v) && (std::isnan(p) || v > p)) p = v;
    }
    return p;
}

// Within `limit` of zero, or not reported.
bool retracted(double v, double limit) { return !std::isfinite(v) || v < limit; }

DetectorSample toDetectorSample(const Frame& f, const FrameInput& in) {
    DetectorSample s;
    s.t = f.t;
    s.onGround = f.v[kOnGround] >= 0.5;
    s.aglFt = f.v[kAgl];
    s.groundSpeedKt = f.v[kGroundSpeed];
    s.verticalSpeedFpm = f.v[kVerticalSpeed];
    s.gForce = f.v[kGForce];
    s.latDeg = f.v[kLat];
    s.lonDeg = f.v[kLon];
    s.altFt = f.v[kAlt];
    s.touchdownNormalVelFps = in.touchdownNormalVelFps;
    s.enginePower = enginePower(f, in.facts.engineType);
    s.onRunway = in.facts.onRunway;
    s.flapsRetracted = retracted(f.v[kFlapsHandleIndex], 0.5) && retracted(f.v[kFlapsLeft], 0.02) &&
                       retracted(f.v[kFlapsRight], 0.02);
    return s;
}

}  // namespace

Engine::Engine(IStorage& storage, const EngineConfig& config)
    : storage_(storage), cfg_(config), detector_(config.detector) {
    const size_t capacity = static_cast<size_t>(std::ceil(cfg_.bufferSeconds * cfg_.sampleRateHz * 1.05)) + 16;
    ring_.reset(capacity);
    scratch_.reserve(capacity * 160 + 4096);
    detector_.setLogger(&Engine::detectorLog, this);
    setup_.setLogger(&Engine::setupLog, this);
}

void Engine::setLogger(LogFn fn, void* ctx) {
    log_ = fn;
    logCtx_ = ctx;
}

void Engine::logf(int level, const char* fmt, ...) const {
    if (!log_) return;
    char buf[512];  // as long as the module log's lines
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_(logCtx_, level, buf);
}

void Engine::detectorLog(void* ctx, const char* msg) { static_cast<Engine*>(ctx)->logf(0, "detector: %s", msg); }

void Engine::setupLog(void* ctx, int level, const char* msg) { static_cast<Engine*>(ctx)->logf(level, "%s", msg); }

// ---------------------------------------------------------------------------- index

void Engine::init() {
    loadIndex();
    reconcileWithDisk();
    // Also applies a lowered limit to an existing library.
    const std::vector<std::string> removed = index_.enforceRetention(cfg_.maxAutoClips);
    for (const std::string& f : removed) {
        storage_.removeFile(f);
        logf(0, "retention: deleted %s (only the newest %zu auto clips are kept)", f.c_str(), cfg_.maxAutoClips);
    }
    if (!removed.empty()) saveIndex();
    ++clipsRev_;
    logf(0, "library: %zu clip(s)", index_.size());
    loadProfiles();
}

void Engine::loadProfiles() {
    std::vector<uint8_t> buf;
    for (const char* name : {kProfilesFile, kProfilesTmp}) {
        if (!storage_.readFile(name, buf)) continue;
        std::string err;
        if (profiles_.fromJson(reinterpret_cast<const char*>(buf.data()), buf.size(), &err)) {
            logf(0, "loaded %s (%zu aircraft control profile(s))", name, profiles_.size());
            if (const size_t n = profiles_.groupVariants()) {
                logf(0, "%zu control profile(s) moved to their aircraft's family, shared by its variants", n);
                saveProfiles();
            }
            return;
        }
        logf(1, "%s is unreadable (%s)", name, err.c_str());
    }
}

void Engine::saveProfiles() {
    std::string json;
    profiles_.toJson(json);
    writeJsonFile(kProfilesFile, kProfilesTmp, json);
}

bool Engine::writeJsonFile(const char* name, const char* tmp, const std::string& json) {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(json.data());
    if (!storage_.writeFile(tmp, data, json.size())) {
        lastError_ = std::string("could not write ") + name;
        logf(2, "could not write %s", tmp);
        return false;
    }
    if (!storage_.replaceFile(tmp, name)) {
        logf(1, "renaming %s failed; writing %s directly", tmp, name);
        if (!storage_.writeFile(name, data, json.size())) {
            lastError_ = std::string("could not write ") + name;
            logf(2, "could not write %s", name);
            return false;
        }
    }
    return true;
}

void Engine::loadIndex() {
    std::vector<uint8_t> buf;
    for (const char* name : {kIndexFile, kIndexTmp}) {
        if (!storage_.readFile(name, buf)) continue;
        std::string err;
        if (index_.fromJson(reinterpret_cast<const char*>(buf.data()), buf.size(), &err)) {
            logf(0, "loaded %s (%zu clips)", name, index_.size());
            return;
        }
        logf(1, "%s is unreadable (%s)", name, err.c_str());
    }
    index_.clear();
    logf(0, "no index found; starting a new library");
}

void Engine::reconcileWithDisk() {
    std::vector<std::string> files;
    if (!storage_.listFiles(files)) {
        logf(1, "cannot list the work folder; skipping index reconciliation");
        return;
    }
    bool changed = false;

    std::vector<std::string> missing;
    for (const ClipEntry& e : index_.clips()) {
        if (std::find(files.begin(), files.end(), e.file) == files.end()) missing.push_back(e.file);
    }
    for (const std::string& f : missing) {
        index_.remove(f);
        changed = true;
        logf(1, "index entry %s has no file; dropped", f.c_str());
    }

    std::vector<uint8_t> buf;
    for (const std::string& f : files) {
        if (!isSafeClipFileName(f) || index_.find(f)) continue;
        ClipMeta meta;
        std::string err;
        if (storage_.readFile(f, buf) && readClipMeta(buf.data(), buf.size(), meta, &err)) {
            const double rate = meta.sampleRateHz > 0 ? meta.sampleRateHz : cfg_.sampleRateHz;
            index_.add(makeEntry(f, meta, meta.frames > 1 ? (meta.frames - 1) / rate : 0.0));
            changed = true;
            logf(0, "adopted %s from disk", f.c_str());
        } else {
            logf(1, "ignoring unreadable clip %s (%s)", f.c_str(), err.c_str());
        }
    }
    if (changed) saveIndex();
}

void Engine::saveIndex() {
    json_.clear();
    index_.toJson(json_);
    writeJsonFile(kIndexFile, kIndexTmp, json_);
}

// ---------------------------------------------------------------------------- sim state

void Engine::onFlightLoaded(const char* what) {
    // Note: the sim also loads a default flight when returning to the menu, so this must not
    // clear atMainMenu_ (the flow events own that flag).
    endPlayback(false, "flight loaded");
    setup_.abandon("flight loaded");  // (the flight put the aircraft where it is: not put back)
    setupSnoozed_ = false;
    setupRequested_ = false;
    finishEvents();
    ring_.clear();
    airports_.clear();  // the module asks for the new flight's
    prevSimTime_ = NAN;
    detector_.disarm(recTime_, what);
    logf(0, "flight loaded (%s): buffer cleared", what);
}

void Engine::onRelocated(const char* what) {
    if (playbackActive_) {
        // Our own position writes must not be mistaken for a user relocation.
        logf(0, "ignoring '%s' during playback", what);
        return;
    }
    finishEvents();
    detector_.disarm(recTime_, what);
}

void Engine::setSimRunning(bool running) {
    if (running != simRunning_) logf(0, "sim %s", running ? "running" : "stopped");
    simRunning_ = running;
}

void Engine::setPauseFlags(uint32_t flags) {
    if (flags != pauseFlags_) logf(0, "pause state 0x%x", static_cast<unsigned>(flags));
    pauseFlags_ = flags;
}

void Engine::setAtMainMenu(bool atMenu) {
    if (atMenu != atMainMenu_) logf(0, atMenu ? "back at the main menu" : "left the main menu");
    if (atMenu) {
        endPlayback(false, "main menu");
        setup_.abandon("main menu");
        finishEvents();
    }
    atMainMenu_ = atMenu;
}

void Engine::setSimReplayActive(bool active) {
    if (active != simReplay_) logf(0, "sim replay/cinematic %s", active ? "started" : "ended");
    simReplay_ = active;
}

void Engine::setRelocating(bool relocating) {
    if (relocating != relocating_) logf(0, "relocation %s", relocating ? "started" : "finished");
    relocating_ = relocating;
}

void Engine::setAircraftTitle(const std::string& title) {
    if (title != aircraft_) logf(0, "aircraft: %s", title.c_str());
    aircraft_ = title;
}

void Engine::setAircraftKey(const std::string& path) {
    aircraftPath_ = path;
    const std::string key = aircraftFamily(path);
    if (key == aircraftKey_) return;
    setup_.cancel("aircraft changed");  // (held above the runway, the aircraft is put back once its gear is down)
    aircraftKey_ = key;
    const AircraftProfile* p = currentProfile();
    if (!p) {
        logf(0, "aircraft key %s: no control profile yet", key.c_str());
    } else {
        std::string summary;
        for (int i = 0; i < kControlCount; ++i) {
            const Control c = controlAt(i);
            summary += std::string(i ? ", " : "") + controlSpec(c).key + " " + stateKey((*p)[c].state);
            if ((*p)[c].state == ControlState::Ok) summary += " (" + describeControl(c, (*p)[c]) + ")";
        }
        logf(0, "aircraft key %s: control profile %s%s", key.c_str(), p->declined ? "declined; " : "", summary.c_str());
    }
    if (out_) out_->applyProfile(p);
}

const AircraftProfile* Engine::currentProfile() const {
    return aircraftKey_.empty() ? nullptr : profiles_.find(aircraftKey_);
}

void Engine::setChannelVar(int channel, const std::string& var) {
    if (channel >= 0 && channel < kChannelCount) channelVar_[channel] = var;
}

// ---------------------------------------------------------------------------- recording

const char* Engine::computeSuspendReason(bool slewActive) const {
    if (playbackActive_) return "playback";
    if (restoringAircraft()) return "restoring aircraft";
    if (setup_.running()) return kReasonSetup;
    if (atMainMenu_) return "main menu";
    if (!simRunning_) return "sim not running";
    if (pauseFlags_ != 0) return kReasonPaused;
    if (slewActive) return "slew";
    if (relocating_) return "relocating";
    if (simReplay_) return "sim replay";
    return nullptr;
}

double Engine::restoreSecondsLeft() const {
    if (playbackActive_ || !out_ || !out_->restoring()) return 0.0;
    const double left = cfg_.restoreMaxSec - static_cast<double>(lastUtcMs_ - restoreStartMs_) / 1000.0;
    return left > 0.0 ? left : 0.0;
}

double Engine::bufferedSeconds() const { return ring_.size() < 2 ? 0.0 : ring_.back().t - ring_.front().t; }

void Engine::onFrame(const FrameInput& in, int64_t utcNowMs) {
    slewActive_ = in.slewActive;

    double dt = 0.0;
    if (std::isfinite(in.simTimeSec)) {
        if (std::isfinite(prevSimTime_)) dt = in.simTimeSec - prevSimTime_;
        prevSimTime_ = in.simTimeSec;
    } else {
        dt = in.fallbackDtSec;
    }
    if (!(dt > 0.0)) dt = 0.0;
    if (dt > cfg_.maxFrameDtSec) dt = cfg_.maxFrameDtSec;
    // Replays step from the sim's per-frame update (onUpdate). Without it they move by the real
    // time between frames, as they're shown: the sim's time steps unevenly from frame to frame (on
    // the A350 at 36 fps by 18, 27 or 37 ms, though every frame took 27 to 28 ms), and stepped by
    // that a replay at 140 kt moved the aircraft about a metre unevenly every frame, and the
    // engine sounds around it (heard in the cabin) fluttered. It holds while the sim is paused, or
    // its time has stood still for longer than a frame or two.
    const int64_t wallMs = lastUtcMs_ > 0 && utcNowMs > lastUtcMs_ ? utcNowMs - lastUtcMs_ : 0;
    simStillMs_ = dt > 0.0 ? 0 : simStillMs_ + wallMs;
    double playDt = dt;
    const double wallDt = static_cast<double>(wallMs) / 1000.0;
    if (lastUtcMs_ > 0 && wallDt <= cfg_.maxFrameDtSec) playDt = wallDt;
    if (playbackHeld()) playDt = 0.0;
    // Data frames with no update before them: one more within the same frame is normal (the sim
    // sends one about once a second), more in a row means the updates stopped.
    if (++framesWithoutUpdate_ >= kUpdateLostFrames) updateDriven_ = false;
    lastWallDt_ = wallDt;
    if (!updateDriven_) lastPlayDt_ = playDt;
    facts_ = in.facts;
    lastUtcMs_ = utcNowMs;
    if (!playbackActive_ && restoreSecondsLeft() <= 0.0 && out_ && out_->restoring()) out_->finishRestore();

    if (!playbackActive_ && !restoringAircraft()) {
        lastRaw_ = in.raw;
        haveLastRaw_ = true;
    }
    if (setup_.active()) {
        // The setup's clock stops while the sim is paused or elsewhere, like the recording's.
        const bool hold = atMainMenu_ || !simRunning_ || pauseFlags_ != 0 || in.slewActive || relocating_ || simReplay_;
        setup_.onFrame(hold ? 0.0 : dt, in.raw, in.facts);
        if (setup_.takeChanged()) storeSetupProfile();
    }

    const char* reason = computeSuspendReason(in.slewActive);
    if (settleFrames_ > 0) --settleFrames_;
    if (reason) {
        if (suspendReason_ == nullptr || std::strcmp(reason, suspendReason_) != 0) {
            logf(0, "recording suspended: %s", reason);
        }
        // Anything other than a plain pause may have moved the aircraft.
        const ResumeKind kind = std::strcmp(reason, kReasonSetup) == 0    ? ResumeKind::Restart
                                : std::strcmp(reason, kReasonPaused) == 0 ? ResumeKind::Pause
                                                                          : ResumeKind::Relocate;
        if (kind > pendingResume_) pendingResume_ = kind;
        suspendReason_ = reason;
        if (playbackActive_ && !updateDriven_) playbackFrame(playDt);
        return;
    }
    if (suspendReason_ != nullptr) {
        logf(0, "recording resumed (was: %s)", suspendReason_);
        if (pendingResume_ == ResumeKind::Restart) {
            // After the control setup nothing from before it belongs in a clip either (it ran
            // spool-ups on the runway, the takeoff wasn't recorded): an event in progress when it
            // started is saved up to then, and recording starts from scratch.
            finishEvents();
            ring_.clear();
            detector_.disarm(recTime_, kReasonSetup);
            logf(0, "recording starts afresh after the control setup");
        } else if (pendingResume_ == ResumeKind::Relocate) {
            finishEvents();
            detector_.disarm(recTime_, suspendReason_);
        } else if (pendingResume_ == ResumeKind::Pause) {
            detector_.notifyResumedAfterPause();
        }
        pendingResume_ = ResumeKind::None;
        suspendReason_ = nullptr;
        nextSampleT_ = recTime_;
    }

    recTime_ += dt;
    recUtcMs_ = utcNowMs;
    recordFrame(in, utcNowMs);
}

void Engine::recordFrame(const FrameInput& in, int64_t utcNowMs) {
    const double interval = 1.0 / cfg_.sampleRateHz;
    if (recTime_ + 0.25 * interval < nextSampleT_) return;  // decimate to the sample rate
    nextSampleT_ += interval;
    if (nextSampleT_ < recTime_) nextSampleT_ = recTime_ + interval;

    Frame& f = ring_.push();
    f = in.raw;
    f.t = recTime_;

    DetectedEvent events[2];
    const int n = detector_.update(toDetectorSample(f, in), events);
    for (int i = 0; i < n; ++i) {
        std::string err;
        saveRange(events[i].type, events[i].startT, events[i].endT, &events[i], utcNowMs, err, nullptr);
    }
}

void Engine::finishEvents() {
    DetectedEvent events[2];
    const int n = detector_.finishInProgress(ring_.empty() ? recTime_ : ring_.back().t, events);
    for (int i = 0; i < n; ++i) {
        std::string err;
        saveRange(events[i].type, events[i].startT, events[i].endT, &events[i], recUtcMs_, err, nullptr);
    }
}

bool Engine::saveRange(EventType type, double startT, double endT, const DetectedEvent* ev, int64_t utcNowMs,
                       std::string& error, std::string* savedName) {
    if (ring_.size() < 2) {
        error = "nothing recorded yet";
        return false;
    }
    const double oldest = ring_.front().t;
    const double newest = ring_.back().t;
    if (startT < oldest - 0.5) {
        logf(1, "%s clip wants %.1f s more than the buffer holds; trimming", eventName(type), oldest - startT);
    }
    const double s = std::max(startT, oldest);
    const double e = std::min(endT, newest);
    const size_t i0 = lowerBoundBy(ring_, s, frameTime);
    const size_t i1 = std::min(lowerBoundBy(ring_, e + 1e-6, frameTime), ring_.size());
    if (i1 <= i0 + 1) {
        error = "clip too short";
        logf(1, "%s clip too short to save", eventName(type));
        return false;
    }

    const Frame& first = ring_.at(i0);
    const double duration = ring_.at(i1 - 1).t - first.t;
    ClipMeta meta;
    meta.event = type;
    meta.sampleRateHz = static_cast<float>(cfg_.sampleRateHz);
    meta.utcStartMs = utcNowMs - static_cast<int64_t>((recTime_ - first.t) * 1000.0);
    meta.aircraft = aircraft_;
    meta.aircraftPath = aircraftPath_;
    for (int c = 0; c < kChannelCount; ++c) {
        if (!channelVar_[c].empty()) meta.vars.emplace_back(kChannels[c].key, channelVar_[c]);
    }
    if (ev) {
        meta.eventOffsetSec = static_cast<float>(ev->eventT - first.t);
        meta.touchdownVsFpm = ev->touchdownVsFpm;
        meta.peakG = ev->peakG;
        meta.bounces = ev->bounces;
        // Where it lifted off or touched down.
        const Frame& at = ring_.at(std::min(lowerBoundBy(ring_, ev->eventT, frameTime), ring_.size() - 1));
        meta.airport = airports_.at(at.v[kLat], at.v[kLon]);
    }
    writeClip(meta, RingSpan(ring_, i0, i1 - i0), scratch_);

    // Auto clips are named after the moment of liftoff/touchdown, manual ones after "now".
    const int64_t nameTime = ev ? meta.utcStartMs + static_cast<int64_t>(meta.eventOffsetSec * 1000.0) : utcNowMs;
    const std::string name = index_.uniqueFileName(nameTime, type);
    if (!storage_.writeFile(name, scratch_.data(), scratch_.size())) {
        error = "could not write " + name;
        lastError_ = error;
        logf(2, "could not write %s", name.c_str());
        return false;
    }

    index_.add(makeEntry(name, meta, duration));
    for (const std::string& old : index_.enforceRetention(cfg_.maxAutoClips)) {
        if (storage_.removeFile(old)) {
            logf(0, "retention: deleted %s", old.c_str());
        } else {
            logf(1, "retention: could not delete %s", old.c_str());
        }
    }
    saveIndex();
    ++clipsRev_;
    lastSaved_ = name;
    lastError_.clear();
    logf(0, "saved %s: %.1f s, %zu frames, %zu KB%s%s", name.c_str(), duration, i1 - i0, scratch_.size() / 1024,
         meta.airport.empty() ? "" : ", at ", meta.airport.c_str());
    if (savedName) *savedName = name;
    return true;
}

// ---------------------------------------------------------------------------- commands

bool Engine::saveLast(int64_t utcNowMs, std::string& error) {
    if (ring_.size() < 2) {
        error = "nothing recorded yet";
        return false;
    }
    return saveRange(EventType::Manual, ring_.front().t, ring_.back().t, nullptr, utcNowMs, error, nullptr);
}

bool Engine::deleteClip(const std::string& file, std::string& error) {
    if (!isSafeClipFileName(file) || !index_.find(file)) {
        error = "unknown clip";
        return false;
    }
    if (playbackActive_ && playback_.file() == file) endPlayback(true, "clip deleted");
    if (!storage_.removeFile(file)) logf(1, "could not delete %s from disk (dropping it from the index anyway)", file.c_str());
    index_.remove(file);
    saveIndex();
    ++clipsRev_;
    logf(0, "deleted %s", file.c_str());
    return true;
}

// ---------------------------------------------------------------------------- playback

bool Engine::play(const std::string& file, std::string& error) {
    if (!out_) {
        error = "playback unavailable";
        return false;
    }
    if (!isSafeClipFileName(file) || !index_.find(file)) {
        error = "unknown clip";
        return false;
    }
    if (setup_.active()) {
        error = "finish or cancel the control setup first";
        return false;
    }
    if (!playbackActive_) {
        if (atMainMenu_ || !simRunning_) {
            error = "not in a flight";
            return false;
        }
        if (!haveLastRaw_) {
            error = "no aircraft data yet";
            return false;
        }
    }
    std::vector<uint8_t> buf;
    if (!storage_.readFile(file, buf)) {
        error = "could not read " + file;
        return false;
    }
    Clip clip;
    std::string parseError;
    if (!readClip(buf.data(), buf.size(), clip, &parseError)) {
        error = file + ": " + parseError;
        return false;
    }
    if (clip.frames.size() < 2) {
        error = "clip is empty";
        return false;
    }
    if (!recordedOnThisAircraft(clip.meta, error)) {
        logf(0, "playback of %s refused: recorded in %s (%s), this is %s (%s)", file.c_str(), clip.meta.aircraft.c_str(),
             clip.meta.aircraftPath.c_str(), aircraft_.c_str(), aircraftPath_.c_str());
        return false;
    }

    const bool switching = playbackActive_;
    playback_.load(std::move(clip), file);
    if (!switching) {
        snapshot_ = lastRaw_;
        for (int c = 0; c < kChannelCount; ++c) lastEventValue_[c] = eventState(snapshot_.v[c]);
        playbackActive_ = true;
    }
    out_->beginPlayback(playback_.clip(), playback_.peak());
    playback_.play();
    jumpPending_ = true;
    logf(0, "playback %s %s (%.1f s, %zu frames, %zu/%d channels)%s", switching ? "switched to" : "of", file.c_str(),
         playback_.duration(), playback_.clip().frames.size(), playback_.present().count(),
         static_cast<int>(kChannelCount), switching ? "" : "; aircraft snapshot taken");
    return true;
}

bool Engine::recordedOnThisAircraft(const ClipMeta& meta, std::string& error) const {
    bool same = false;
    if (!meta.aircraftPath.empty() && aircraftKey_.empty()) {
        error = "the aircraft isn't identified yet";
        return false;
    }
    if (!meta.aircraftPath.empty() && !isTitleKey(meta.aircraftPath) && !isTitleKey(aircraftPath_)) {
        // The exact aircraft.cfg, not its family: variants share a control profile but not clips.
        // Their weights and thrust differ, and the physics fights a replay recorded on another.
        same = meta.aircraftPath == aircraftPath_;
    } else {
        // Older clips, and an aircraft whose path wasn't known then or isn't now: by title.
        same = meta.aircraft.empty() || equalsIgnoringCase(meta.aircraft, aircraft_);
    }
    if (same) return true;
    error = meta.aircraft.empty() ? "recorded in another aircraft: switch to it to replay this clip"
                                  : "recorded in the " + meta.aircraft + ": switch to that aircraft to replay it";
    return false;
}

bool Engine::pausePlayback(std::string& error) {
    if (!playbackActive_) {
        error = "nothing is playing";
        return false;
    }
    playback_.pause();
    return true;
}

bool Engine::resumePlayback(std::string& error) {
    if (!playbackActive_) {
        error = "nothing is playing";
        return false;
    }
    const double before = playback_.time();
    playback_.play();  // at the end this restarts from the beginning: a jump like seek
    if (playback_.time() != before) jumpPending_ = true;
    return true;
}

bool Engine::seek(double t, std::string& error) {
    if (!playbackActive_) {
        error = "nothing is playing";
        return false;
    }
    playback_.seek(t);
    jumpPending_ = true;
    return true;
}

bool Engine::setSpeed(double speed, std::string& error) {
    if (!playbackActive_) {
        error = "nothing is playing";
        return false;
    }
    playback_.setSpeed(speed);
    return true;
}

bool Engine::stopPlayback(std::string& error) {
    if (!playbackActive_) {
        error = "nothing is playing";
        return false;
    }
    endPlayback(true, "stopped");
    return true;
}

// The sim's own update, once a frame at the same point in it, with its frame time: on the A350
// at 36 fps exactly 1/36 s every frame, and a hitch as long as it was. A replay steps and writes
// the aircraft from here when the module has it. The data frames come at a different point in
// the sim's frame from one frame to the next (their data was a frame stale every other frame or
// so, and the physics ran 0 to 3 steps between a write and the next read), and replays written
// from them squeaked now and then in the A350's internal engine sound, which goes by how the
// aircraft moves from frame to frame (stepped unevenly, it fluttered).
void Engine::onUpdate(double frameDtSec) {
    framesWithoutUpdate_ = 0;
    updateDriven_ = true;
    if (!playbackActive_) return;
    double dt = playbackHeld() || !(frameDtSec > 0.0) ? 0.0 : frameDtSec;
    if (dt > cfg_.maxFrameDtSec) dt = cfg_.maxFrameDtSec;
    lastPlayDt_ = dt;
    playbackFrame(dt);
}

void Engine::playbackFrame(double dt) {
    if (jumpPending_ && out_) {
        playback_.current(playFrame_);  // where the jump goes, without moving the playhead
        settling_ = false;
        preparing_ = !out_->prepareJump(playFrame_);
        if (preparing_) return;
    }
    if (settling_) dt = 0.0;  // held where the jump went until the sim side is ready there
    playback_.advance(dt);
    playback_.current(playFrame_);
    // After play or seek the aircraft can be in any state (gear up from the end of the last run,
    // lights switched by hand...), so every event channel is resent, not just the changes.
    const bool jumped = jumpPending_;
    jumpPending_ = false;
    if (!out_) return;
    out_->writePlaybackFrame(playFrame_, jumped);
    const ChannelMask& present = playback_.present();
    for (int c = 0; c < kChannelCount; ++c) {
        if (kChannels[c].play != PlayMode::Event || !present.test(static_cast<size_t>(c))) continue;
        const double state = eventState(playFrame_.v[c]);
        if (std::isnan(state) || (state == lastEventValue_[c] && !jumped)) continue;
        out_->sendChannelEvent(c, state);
        lastEventValue_[c] = state;
    }
    if (jumped) settling_ = true;
    if (settling_) settling_ = !out_->settleJump(playFrame_);
}

void Engine::endPlayback(bool restore, const char* why) {
    if (!playbackActive_) return;
    if (out_) {
        if (restore) {
            const ChannelMask& present = playback_.present();
            for (int c = 0; c < kChannelCount; ++c) {
                if (kChannels[c].play != PlayMode::Event || !present.test(static_cast<size_t>(c))) continue;
                const double state = eventState(snapshot_.v[c]);
                if (!std::isnan(state)) out_->sendChannelEvent(c, state);
            }
        }
        out_->endPlayback(snapshot_, restore);
    }
    jumpPending_ = false;
    preparing_ = false;
    settling_ = false;
    playback_.unload();
    playbackActive_ = false;
    settleFrames_ = kSettleFrames;
    restoreStartMs_ = lastUtcMs_;
    logf(0, "playback ended (%s)%s", why, restore ? "; aircraft restored" : "");
}

// ---------------------------------------------------------------------------- control setup

bool Engine::setupRequest(std::string& error) {
    if (aircraftKey_.empty()) {
        error = "the aircraft isn't identified yet";
        return false;
    }
    if (playbackActive_) {
        error = "stop the replay first";
        return false;
    }
    if (atMainMenu_ || !simRunning_ || !haveLastRaw_) {
        error = "not in a flight";
        return false;
    }
    if (setup_.active()) return true;
    setupRequested_ = true;
    setupSnoozed_ = false;
    return true;
}

bool Engine::setupStart(std::string& error) {
    if (aircraftKey_.empty()) {
        error = "the aircraft isn't identified yet";
        return false;
    }
    if (playbackActive_) {
        error = "stop the replay first";
        return false;
    }
    if (atMainMenu_ || !simRunning_ || !haveLastRaw_) {
        error = "not in a flight";
        return false;
    }
    AircraftProfile base;
    if (const AircraftProfile* p = currentProfile()) base = *p;
    base.key = aircraftKey_;
    if (!setup_.start(base, lastRaw_, facts_, error)) return false;
    setupSnoozed_ = false;
    setupRequested_ = false;
    if (setup_.takeChanged()) storeSetupProfile();
    return true;
}

bool Engine::setupAnswer(const std::string& answer, std::string& error) {
    if (!setup_.answer(answer, error)) return false;
    if (setup_.takeChanged()) storeSetupProfile();
    if (answer == "later") setupSnoozed_ = true;
    return true;
}

bool Engine::setupLater(std::string& error) {
    (void)error;
    setup_.cancel("finish later");
    setupSnoozed_ = true;
    setupRequested_ = false;
    return true;
}

bool Engine::setupNever(std::string& error) {
    if (aircraftKey_.empty()) {
        error = "the aircraft isn't identified yet";
        return false;
    }
    setup_.cancel("declined");
    setupRequested_ = false;
    AircraftProfile& p = profiles_.upsert(aircraftKey_);
    p.declined = true;
    p.title = aircraft_;
    p.updatedUtcMs = lastUtcMs_;
    saveProfiles();
    logf(0, "control setup declined for %s", aircraftKey_.c_str());
    if (out_) out_->applyProfile(currentProfile());
    return true;
}

bool Engine::setupReset(std::string& error) {
    if (aircraftKey_.empty()) {
        error = "the aircraft isn't identified yet";
        return false;
    }
    setup_.cancel("profile reset");
    if (profiles_.remove(aircraftKey_)) {
        saveProfiles();
        logf(0, "control profile for %s removed", aircraftKey_.c_str());
    }
    setupSnoozed_ = false;
    if (out_) out_->applyProfile(nullptr);
    return true;
}

bool Engine::promoteWay(Control c, Direction d, const ControlMethod& m) {
    if (aircraftKey_.empty() || !profiles_.find(aircraftKey_)) return false;
    AircraftProfile& p = profiles_.upsert(aircraftKey_);
    if (!br::promoteWay(p[c], d, m)) return false;
    p.updatedUtcMs = lastUtcMs_;
    logf(0, "control profile: the %s %s with %s first from now on (it worked in a replay where the way before it didn't); "
         "now %s",
         controlSpec(c).label, directionKey(c, d), describeMethod(c, m).c_str(), describeControl(c, p[c]).c_str());
    saveProfiles();
    if (out_) out_->applyProfile(currentProfile());
    return true;
}

void Engine::storeSetupProfile() {
    if (aircraftKey_.empty()) return;
    AircraftProfile& p = profiles_.upsert(aircraftKey_);
    p = setup_.profile();
    p.key = aircraftKey_;
    p.title = aircraft_;
    p.updatedUtcMs = lastUtcMs_;
    saveProfiles();
    if (out_) out_->applyProfile(currentProfile());
}

const char* Engine::setupOffer() const {
    if (setup_.active() || setupSnoozed_ || playbackActive_ || aircraftKey_.empty() || !haveLastRaw_ || atMainMenu_ ||
        !simRunning_) {
        return "";
    }
    const AircraftProfile* p = currentProfile();
    const bool onGround = lastRaw_.v[kOnGround] >= 0.5;
    // Set up / Redo: the same offer as a new aircraft, whatever the profile says.
    if (setupRequested_) {
        if (!onGround) return facts_.gearRetractable ? "gear" : "wait";
        return readyForSetup() ? "new" : "wait";
    }
    if (p && p->declined) return "";
    bool groundPending = !p;
    for (int i = 0; p && i < kControlCount; ++i) {
        if (controlAt(i) != Control::Gear && p->controls[i].state == ControlState::Pending) groundPending = true;
    }
    // Until the aircraft is ready for it, the offer says what's missing ("wait").
    if (groundPending) return readyForSetup() ? "new" : "wait";
    const bool gearPending = (*p)[Control::Gear].state == ControlState::Pending && facts_.gearRetractable;
    return gearPending && !onGround ? "gear" : "";
}

bool Engine::readyForSetup() const {
    return facts_.parkingBrake && lastRaw_.v[kOnGround] >= 0.5 && facts_.onRunway && facts_.engineRunning;
}

// ---------------------------------------------------------------------------- status

void Engine::statusJson(std::string& out) const {
    out.clear();
    JsonWriter w(out);
    w.beginObject();
    w.kv("v", 1);
    w.kv("rec", recording());
    w.kv("why", suspendReason_ ? suspendReason_ : "");
    w.kv("restoreIn", restoreSecondsLeft());  // after Stop: the aircraft is unfrozen by then at the latest
    w.kv("restoreMax", cfg_.restoreMaxSec);
    w.kv("armed", detector_.armed());
    w.kv("armIn", detector_.armRemaining(recTime_));
    w.kv("phase", detector_.phase());
    w.kv("bufferSec", bufferedSeconds());
    w.kv("bufferMax", cfg_.bufferSeconds);
    w.kv("aircraft", aircraft_);
    w.kv("clipsRev", clipsRev_);
    w.kv("clipCount", static_cast<int>(index_.size()));
    w.kv("lastSaved", lastSaved_);
    w.kv("error", lastError_);
    w.key("play").beginObject();
    w.kv("active", playbackActive_);
    w.kv("file", playback_.file());
    w.kv("t", playback_.time());
    w.kv("dur", playback_.duration());
    w.kv("speed", playback_.speed());
    w.kv("playing", playback_.playing());
    w.kv("preparing", preparing_ || settling_);
    // What it's getting ready, for the panel's checklist: each step "", "active" or "done".
    const JumpPrep prep = out_ && (preparing_ || settling_) ? out_->jumpPrep() : JumpPrep();
    auto step = [](PrepStep s) { return s == PrepStep::Active ? "active" : s == PrepStep::Done ? "done" : ""; };
    w.key("prep").beginObject();
    w.kv("gear", step(prep.gear));
    w.kv("flaps", step(prep.flaps));
    w.kv("flapsIn", prep.flapsIn);
    w.kv("engines", step(prep.engines));
    w.kv("enginesDown", prep.enginesDown);
    w.endObject();
    w.kv("eventT", playback_.loaded() ? static_cast<double>(playback_.clip().meta.eventOffsetSec) : 0.0);
    w.endObject();
    w.key("setup").beginObject();
    w.kv("offer", setupOffer());
    w.kv("requested", setupRequested_);  // Set up / Redo: no "never for this aircraft"
    // What the setup needs before it can start, for the "wait" offer's checklist.
    w.key("ready").beginObject();
    w.kv("parkingBrake", facts_.parkingBrake);
    w.kv("onRunway", facts_.onRunway && lastRaw_.v[kOnGround] >= 0.5);
    w.kv("engines", facts_.engineRunning);
    w.endObject();
    const AircraftProfile* p = currentProfile();
    w.kv("have", !p ? "none" : p->declined ? "declined" : p->anyPending() ? "partial" : "complete");
    w.key("controls").beginArray();
    for (int i = 0; p && i < kControlCount; ++i) {
        const Control c = controlAt(i);
        const ControlProfile& cp = (*p)[c];
        w.beginObject();
        w.kv("control", controlSpec(c).key);
        w.kv("state", stateKey(cp.state));
        w.kv("method", cp.state == ControlState::Ok ? describeControl(c, cp) : std::string());
        w.endObject();
    }
    w.endArray();
    setup_.statusJson(w);
    w.endObject();
    w.endObject();
}

void Engine::clipsJson(size_t offset, size_t limit, std::string& out) const {
    out.clear();
    JsonWriter w(out);
    const std::vector<ClipEntry>& clips = index_.clips();
    w.beginObject();
    w.kv("rev", clipsRev_);
    w.kv("total", static_cast<int>(clips.size()));
    w.kv("offset", static_cast<int>(offset));
    w.key("clips").beginArray();
    for (size_t i = offset; i < clips.size() && i < offset + limit; ++i) writeClipEntryJson(w, clips[i]);
    w.endArray();
    w.endObject();
}

}  // namespace br
