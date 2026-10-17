// The recorder: owns the ring buffer, event detection, clip library and playback state.
// Platform-independent; the WASM module feeds it sim frames/events and implements the
// storage and sim-output interfaces.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ClipIndex.h"
#include "EventDetector.h"
#include "PlaybackController.h"
#include "RingBuffer.h"

namespace br {

// What playback needs from the sim (the module implements it with SimConnect).
class ISimOutput {
public:
    virtual ~ISimOutput() = default;
    // Freeze lat/lon, altitude and attitude; `present` = channels the clip has.
    virtual void beginPlayback(const ChannelMask& present) = 0;
    // Write every PlayMode::Set channel in `present` (NaN values: keep the live value). `jumped` is
    // true on the first frame after play or seek: PlayMode::Snap channels are written then too.
    virtual void writePlaybackFrame(const Frame& frame, bool jumped) = 0;
    // A PlayMode::Event channel's state: sent when it changes, and for every such channel in the
    // clip after play, seek and stop, so the aircraft can't stay out of step with the clip.
    virtual void sendChannelEvent(int channel, double value) = 0;
    // Optionally restore the pre-playback snapshot (Set and Snap channels), then unfreeze.
    virtual void endPlayback(const Frame& snapshot, bool restore) = 0;
    // True while the sim side is still settling the restored aircraft (e.g. waiting for the gear
    // to come back down before unfreezing on the ground). Recording stays suspended meanwhile.
    virtual bool restoring() const { return false; }
};

// Persistent storage for clips and index.json (the module maps this onto \work).
class IStorage {
public:
    virtual ~IStorage() = default;
    virtual bool writeFile(const std::string& name, const uint8_t* data, size_t size) = 0;
    virtual bool readFile(const std::string& name, std::vector<uint8_t>& out) = 0;
    virtual bool removeFile(const std::string& name) = 0;
    virtual bool replaceFile(const std::string& from, const std::string& to) = 0;  // rename, overwriting
    virtual bool listFiles(std::vector<std::string>& names) = 0;                    // false if unsupported
};

struct EngineConfig {
    DetectorConfig detector;
    double sampleRateHz = 30.0;
    double bufferSeconds = 300.0;
    size_t maxAutoClips = 20;  // newest takeoff/landing clips kept; manual saves are never auto-deleted
    double maxFrameDtSec = 0.5;  // longer gaps (hitches, loading) are clamped
};

// Everything the module reads from the sim once per frame.
struct FrameInput {
    Frame raw;                            // channel values; raw.t is ignored
    double simTimeSec = NAN;              // SIMULATION TIME; drives the recording clock
    double fallbackDtSec = 0.0;           // used when simTimeSec is unavailable
    double touchdownNormalVelFps = NAN;   // PLANE TOUCHDOWN NORMAL VELOCITY
    bool slewActive = false;              // IS SLEW ACTIVE
};

class Engine {
public:
    using LogFn = void (*)(void* ctx, int level, const char* message);  // level: 0 info, 1 warn, 2 error

    Engine(IStorage& storage, const EngineConfig& config = EngineConfig());

    void setLogger(LogFn fn, void* ctx);
    void setSimOutput(ISimOutput* out) { out_ = out; }

    // Loads index.json (falling back to its .tmp copy, then to scanning *.rpl headers) and
    // reconciles it with the files actually present.
    void init();

    // ---- sim state (from SimConnect system/flow events)
    void onFlightLoaded(const char* what);
    void onRelocated(const char* what);  // teleport, skip, back-on-track, crash reset
    void setSimRunning(bool running);
    void setPauseFlags(uint32_t flags);
    void setAtMainMenu(bool atMenu);
    void setSimReplayActive(bool active);  // the sim's own replay / cinematics
    void setRelocating(bool relocating);   // teleport / skip / back-on-track in progress
    void setAircraftTitle(const std::string& title);

    // ---- per sim frame
    void onFrame(const FrameInput& in, int64_t utcNowMs);

    // ---- commands (return false and set `error` on failure)
    bool saveLast(int64_t utcNowMs, std::string& error);
    bool deleteClip(const std::string& file, std::string& error);

    // Playback. play() snapshots the aircraft, freezes it and starts from the beginning;
    // playing another clip while one is active keeps the original snapshot.
    bool play(const std::string& file, std::string& error);
    bool pausePlayback(std::string& error);
    bool resumePlayback(std::string& error);
    bool seek(double t, std::string& error);
    bool setSpeed(double speed, std::string& error);
    bool stopPlayback(std::string& error);  // restores the snapshot and unfreezes
    bool playbackActive() const { return playbackActive_; }
    const PlaybackController& playback() const { return playback_; }

    // ---- status
    bool recording() const { return suspendReason_ == nullptr; }
    const char* suspendReason() const { return suspendReason_; }
    double bufferedSeconds() const;
    size_t bufferedFrames() const { return ring_.size(); }
    double recordingClock() const { return recTime_; }
    int clipsRevision() const { return clipsRev_; }
    const ClipIndex& index() const { return index_; }
    const EventDetector& detector() const { return detector_; }

    void statusJson(std::string& out) const;
    void clipsJson(size_t offset, size_t limit, std::string& out) const;

private:
    enum class ResumeKind { None, Pause, Relocate };

    const char* computeSuspendReason(bool slewActive) const;
    bool restoringAircraft() const { return settleFrames_ > 0 || (out_ && out_->restoring()); }
    void recordFrame(const FrameInput& in, int64_t utcNowMs);
    void playbackFrame(double dt);
    void endPlayback(bool restore, const char* why);
    bool saveRange(EventType type, double startT, double endT, const DetectedEvent* ev, int64_t utcNowMs,
                   std::string& error, std::string* savedName);
    void saveIndex();
    void loadIndex();
    void reconcileWithDisk();
    void logf(int level, const char* fmt, ...) const;
    static void detectorLog(void* ctx, const char* msg);

    IStorage& storage_;
    EngineConfig cfg_;
    LogFn log_ = nullptr;
    void* logCtx_ = nullptr;

    RingBuffer<Frame> ring_;
    EventDetector detector_;
    ClipIndex index_;
    int clipsRev_ = 1;

    // sim state
    bool simRunning_ = false;
    uint32_t pauseFlags_ = 0;
    bool atMainMenu_ = false;
    bool simReplay_ = false;
    bool relocating_ = false;
    bool slewActive_ = false;
    std::string aircraft_;

    // playback
    ISimOutput* out_ = nullptr;
    PlaybackController playback_;
    bool playbackActive_ = false;
    bool haveLastRaw_ = false;
    Frame lastRaw_;   // latest live frame (snapshot source)
    Frame snapshot_;  // aircraft state when playback started
    double lastEventValue_[kChannelCount] = {};
    bool jumpPending_ = false;  // play/seek happened: resync event channels and snap the next frame
    Frame playFrame_;
    // After a restore the sim may report the old position for a frame or two; don't record those.
    static constexpr int kSettleFrames = 30;
    int settleFrames_ = 0;

    // recording clock
    double recTime_ = 0.0;
    double nextSampleT_ = 0.0;
    double prevSimTime_ = NAN;
    const char* suspendReason_ = "waiting for sim";
    ResumeKind pendingResume_ = ResumeKind::None;

    std::vector<uint8_t> scratch_;  // reused clip serialization buffer
    mutable std::string json_;      // reused index serialization buffer
    std::string lastSaved_;
    std::string lastError_;
};

}  // namespace br
