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
constexpr const char* kReasonPaused = "paused";

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

DetectorSample toDetectorSample(const Frame& f, double touchdownNormalVelFps) {
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
    s.touchdownNormalVelFps = touchdownNormalVelFps;
    return s;
}

}  // namespace

Engine::Engine(IStorage& storage, const EngineConfig& config)
    : storage_(storage), cfg_(config), detector_(config.detector) {
    const size_t capacity = static_cast<size_t>(std::ceil(cfg_.bufferSeconds * cfg_.sampleRateHz * 1.05)) + 16;
    ring_.reset(capacity);
    scratch_.reserve(capacity * 160 + 4096);
    detector_.setLogger(&Engine::detectorLog, this);
}

void Engine::setLogger(LogFn fn, void* ctx) {
    log_ = fn;
    logCtx_ = ctx;
}

void Engine::logf(int level, const char* fmt, ...) const {
    if (!log_) return;
    char buf[320];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log_(logCtx_, level, buf);
}

void Engine::detectorLog(void* ctx, const char* msg) { static_cast<Engine*>(ctx)->logf(0, "detector: %s", msg); }

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
    const uint8_t* data = reinterpret_cast<const uint8_t*>(json_.data());
    if (!storage_.writeFile(kIndexTmp, data, json_.size())) {
        lastError_ = "could not write index";
        logf(2, "could not write %s", kIndexTmp);
        return;
    }
    if (!storage_.replaceFile(kIndexTmp, kIndexFile)) {
        logf(1, "renaming %s failed; writing %s directly", kIndexTmp, kIndexFile);
        if (!storage_.writeFile(kIndexFile, data, json_.size())) {
            lastError_ = "could not write index";
            logf(2, "could not write %s", kIndexFile);
        }
    }
}

// ---------------------------------------------------------------------------- sim state

void Engine::onFlightLoaded(const char* what) {
    // Note: the sim also loads a default flight when returning to the menu, so this must not
    // clear atMainMenu_ (the flow events own that flag).
    endPlayback(false, "flight loaded");
    ring_.clear();
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
    if (atMenu) endPlayback(false, "main menu");
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

// ---------------------------------------------------------------------------- recording

const char* Engine::computeSuspendReason(bool slewActive) const {
    if (playbackActive_) return "playback";
    if (restoringAircraft()) return "restoring aircraft";
    if (atMainMenu_) return "main menu";
    if (!simRunning_) return "sim not running";
    if (pauseFlags_ != 0) return kReasonPaused;
    if (slewActive) return "slew";
    if (relocating_) return "relocating";
    if (simReplay_) return "sim replay";
    return nullptr;
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

    if (!playbackActive_ && !restoringAircraft()) {
        lastRaw_ = in.raw;
        haveLastRaw_ = true;
    }

    const char* reason = computeSuspendReason(in.slewActive);
    if (settleFrames_ > 0) --settleFrames_;
    if (reason) {
        if (suspendReason_ == nullptr || std::strcmp(reason, suspendReason_) != 0) {
            logf(0, "recording suspended: %s", reason);
        }
        // Anything other than a plain pause may have moved the aircraft.
        const ResumeKind kind = std::strcmp(reason, kReasonPaused) == 0 ? ResumeKind::Pause : ResumeKind::Relocate;
        if (kind == ResumeKind::Relocate || pendingResume_ == ResumeKind::None) pendingResume_ = kind;
        suspendReason_ = reason;
        if (playbackActive_) playbackFrame(dt);
        return;
    }
    if (suspendReason_ != nullptr) {
        logf(0, "recording resumed (was: %s)", suspendReason_);
        if (pendingResume_ == ResumeKind::Relocate) {
            detector_.disarm(recTime_, suspendReason_);
        } else if (pendingResume_ == ResumeKind::Pause) {
            detector_.notifyResumedAfterPause();
        }
        pendingResume_ = ResumeKind::None;
        suspendReason_ = nullptr;
        nextSampleT_ = recTime_;
    }

    recTime_ += dt;
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
    const int n = detector_.update(toDetectorSample(f, in.touchdownNormalVelFps), events);
    for (int i = 0; i < n; ++i) {
        std::string err;
        saveRange(events[i].type, events[i].startT, events[i].endT, &events[i], utcNowMs, err, nullptr);
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
    if (ev) {
        meta.eventOffsetSec = static_cast<float>(ev->eventT - first.t);
        meta.touchdownVsFpm = ev->touchdownVsFpm;
        meta.peakG = ev->peakG;
        meta.bounces = ev->bounces;
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
    logf(0, "saved %s: %.1f s, %zu frames, %zu KB", name.c_str(), duration, i1 - i0, scratch_.size() / 1024);
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

    const bool switching = playbackActive_;
    playback_.load(std::move(clip), file);
    if (!switching) {
        snapshot_ = lastRaw_;
        for (int c = 0; c < kChannelCount; ++c) lastEventValue_[c] = eventState(snapshot_.v[c]);
        playbackActive_ = true;
    }
    out_->beginPlayback(playback_.present(), playback_.peak());
    playback_.play();
    jumpPending_ = true;
    logf(0, "playback %s %s (%.1f s, %zu frames, %zu/%d channels)%s", switching ? "switched to" : "of", file.c_str(),
         playback_.duration(), playback_.clip().frames.size(), playback_.present().count(),
         static_cast<int>(kChannelCount), switching ? "" : "; aircraft snapshot taken");
    return true;
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

void Engine::playbackFrame(double dt) {
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
    playback_.unload();
    playbackActive_ = false;
    settleFrames_ = kSettleFrames;
    logf(0, "playback ended (%s)%s", why, restore ? "; aircraft restored" : "");
}

// ---------------------------------------------------------------------------- status

void Engine::statusJson(std::string& out) const {
    out.clear();
    JsonWriter w(out);
    w.beginObject();
    w.kv("v", 1);
    w.kv("rec", recording());
    w.kv("why", suspendReason_ ? suspendReason_ : "");
    w.kv("armed", detector_.armed());
    w.kv("armIn", detector_.armRemaining(recTime_));
    w.kv("phase", detector_.phase());
    w.kv("bufferSec", bufferedSeconds());
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
    w.kv("eventT", playback_.loaded() ? static_cast<double>(playback_.clip().meta.eventOffsetSec) : 0.0);
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
