// The recorder: owns the ring buffer, event detection, clip library and playback state.
// Platform-independent; the WASM module feeds it sim frames/events and implements the
// storage and sim-output interfaces.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "AircraftProfile.h"
#include "Airports.h"
#include "ClipIndex.h"
#include "ControlSetup.h"
#include "EventDetector.h"
#include "PlaybackController.h"
#include "RingBuffer.h"

namespace br {

// What playback needs from the sim (the module implements it with SimConnect).
class ISimOutput {
public:
    virtual ~ISimOutput() = default;
    // Freeze lat/lon, altitude and attitude. `clip` is the loaded clip (its present channels and
    // the variables they recorded), `peak` each channel's largest |value| in it.
    virtual void beginPlayback(const Clip& clip, const Frame& peak) = 0;
    // Write every PlayMode::Set channel in `present` (NaN values: keep the live value). `jumped` is
    // true on the first frame after play or seek: PlayMode::Snap channels are written then too.
    virtual void writePlaybackFrame(const Frame& frame, bool jumped) = 0;
    // A PlayMode::Event channel's state: sent when it changes, and for every such channel in the
    // clip after play, seek and stop, so the aircraft can't stay out of step with the clip.
    virtual void sendChannelEvent(int channel, double value) = 0;
    // Called before playback jumps (play, seek, replay) to `target`, every frame until it returns
    // true; meanwhile the playhead holds and nothing is written, so the aircraft stays where it
    // is. Lets the sim side get ready first, e.g. lower the gear in the air before the jump puts
    // the aircraft on the runway (an aircraft that animates its own gear can't extend it there).
    virtual bool prepareJump(const Frame& target) {
        (void)target;
        return true;
    }
    // Optionally restore the pre-playback snapshot (Set and Snap channels), then unfreeze.
    virtual void endPlayback(const Frame& snapshot, bool restore) = 0;
    // True while the sim side is still settling the restored aircraft (e.g. waiting for the gear
    // to come back down before unfreezing on the ground). Recording stays suspended meanwhile.
    virtual bool restoring() const { return false; }
    // Stop settling and unfreeze now: EngineConfig::restoreMaxSec after Stop, if still restoring.
    virtual void finishRestore() {}
    // The current aircraft's control profile changed (null: it has none). Recording and playback
    // follow it from now on.
    virtual void applyProfile(const AircraftProfile* profile) { (void)profile; }
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
    double bufferSeconds = 600.0;  // about 12 MB of frames
    size_t maxAutoClips = 20;  // newest takeoff/landing clips kept; manual saves are never auto-deleted
    double maxFrameDtSec = 0.5;  // longer gaps (hitches, loading) are clamped
    double restoreMaxSec = 10.0;  // after Stop, the longest the aircraft stays frozen settling (the panel counts it down)
};

// Everything the module reads from the sim once per frame.
struct FrameInput {
    Frame raw;                            // channel values; raw.t is ignored
    double simTimeSec = NAN;              // SIMULATION TIME; drives the recording clock
    double fallbackDtSec = 0.0;           // used when simTimeSec is unavailable
    double touchdownNormalVelFps = NAN;   // PLANE TOUCHDOWN NORMAL VELOCITY
    bool slewActive = false;              // IS SLEW ACTIVE
    SetupFacts facts;                     // for the control setup
};

class Engine {
public:
    using LogFn = void (*)(void* ctx, int level, const char* message);  // level: 0 info, 1 warn, 2 error

    Engine(IStorage& storage, const EngineConfig& config = EngineConfig());

    void setLogger(LogFn fn, void* ctx);
    void setSimOutput(ISimOutput* out) { out_ = out; }
    void setControlSim(IControlSim* sim) { setup_.setSim(sim); }

    // Loads index.json (falling back to its .tmp copy, then to scanning *.rpl headers) and
    // reconciles it with the files actually present, then aircraft.json (control profiles).
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
    // Identifies the aircraft (the module passes its aircraft.cfg folder). Clips record it, and only
    // replay on the same aircraft.cfg, so not on another variant; the control profile is kept under
    // its aircraftFamily(), so the variants share one.
    void setAircraftKey(const std::string& path);
    const std::string& aircraftKey() const { return aircraftKey_; }  // the family
    const AircraftProfile* currentProfile() const;
    // An airport the sim reports around the aircraft. Takeoff and landing clips record the one
    // they were at.
    void addAirport(const std::string& ident, const std::string& region, double latDeg, double lonDeg) {
        airports_.add(ident, region, latDeg, lonDeg);
    }
    const AirportList& airports() const { return airports_; }
    // The aircraft variable an input event / L:var channel records from now on ("" = none);
    // saved clips list them.
    void setChannelVar(int channel, const std::string& var);

    // ---- per sim frame
    void onFrame(const FrameInput& in, int64_t utcNowMs);
    // The sim's per-frame update and its frame time (seconds). When the module calls it, replays
    // step and write the aircraft from here rather than from onFrame.
    void onUpdate(double frameDtSec);

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
    // After Stop, how long the aircraft may still stay frozen while it settles (0: it isn't).
    double restoreSecondsLeft() const;
    const PlaybackController& playback() const { return playback_; }
    // The last frame: the real time since the one before, and how far the replay clock moved on it
    // (before the playback speed; 0 while held). For the playback trace.
    double lastWallDt() const { return lastWallDt_; }
    double lastPlayDt() const { return lastPlayDt_; }

    // Control setup (ControlSetup.h). Offered for every aircraft without a profile: it can start
    // once on a runway with the engines running and the parking brake set. "later" hides the offer until the next
    // flight, "never" for this aircraft; "reset" forgets its profile.
    bool setupStart(std::string& error);
    bool setupAnswer(const std::string& answer, std::string& error);
    bool setupLater(std::string& error);
    bool setupNever(std::string& error);
    bool setupReset(std::string& error);
    bool setupActive() const { return setup_.active(); }
    // "new" (ready to start), "wait" (no profile yet, but not on a runway with the engines running
    // and the parking brake set), "gear" (only the gear left, and airborne) or "".
    const char* setupOffer() const;
    bool readyForSetup() const;
    const ProfileStore& profiles() const { return profiles_; }
    // A replay found way `m` moving a control where the one before it in the profile's list that
    // way didn't (WayPicker): it goes first in the aircraft's profile, logged and saved, so later
    // replays try it first. False if the profile has no such list or it's first already.
    bool promoteWay(Control c, Direction d, const ControlMethod& m);

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
    // What resuming recording has to do, weakest first: after a pause nothing, after a relocation
    // detection re-arms, after the control setup the buffer starts over too.
    enum class ResumeKind { None, Pause, Relocate, Restart };

    const char* computeSuspendReason(bool slewActive) const;
    bool restoringAircraft() const { return settleFrames_ > 0 || (out_ && out_->restoring()); }
    void recordFrame(const FrameInput& in, int64_t utcNowMs);
    // The flight ends or the aircraft is moved: a landing or takeoff in progress is saved up to
    // the last recorded moment rather than lost.
    void finishEvents();
    // A clip replays only on the aircraft.cfg it was recorded on, not on another variant that shares
    // its control profile; otherwise `error` asks to switch to it.
    bool recordedOnThisAircraft(const ClipMeta& meta, std::string& error) const;
    void playbackFrame(double dt);
    void endPlayback(bool restore, const char* why);
    bool saveRange(EventType type, double startT, double endT, const DetectedEvent* ev, int64_t utcNowMs,
                   std::string& error, std::string* savedName);
    void saveIndex();
    void loadIndex();
    void reconcileWithDisk();
    void loadProfiles();
    void saveProfiles();
    void storeSetupProfile();  // the setup's profile -> profiles_, aircraft.json and the sim
    bool writeJsonFile(const char* name, const char* tmp, const std::string& json);
    static void setupLog(void* ctx, int level, const char* msg);
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
    std::string aircraftPath_;
    std::string aircraftKey_;  // aircraftFamily(aircraftPath_)
    SetupFacts facts_;
    AirportList airports_;
    int64_t lastUtcMs_ = 0;

    // control profiles and setup
    ProfileStore profiles_;
    SetupFlow setup_;
    bool setupSnoozed_ = false;  // "later": no offer until the next flight
    std::string channelVar_[kChannelCount];

    // playback
    ISimOutput* out_ = nullptr;
    PlaybackController playback_;
    bool playbackActive_ = false;
    bool haveLastRaw_ = false;
    Frame lastRaw_;   // latest live frame (snapshot source)
    Frame snapshot_;  // aircraft state when playback started
    double lastEventValue_[kChannelCount] = {};
    bool jumpPending_ = false;  // play/seek happened: resync event channels and snap the next frame
    bool preparing_ = false;    // holding before a jump until ISimOutput::prepareJump is ready
    Frame playFrame_;
    // After a restore the sim may report the old position for a frame or two; don't record those.
    static constexpr int kSettleFrames = 30;
    int settleFrames_ = 0;
    int64_t restoreStartMs_ = 0;  // when playback last ended (wall clock), for restoreMaxSec
    // How long the sim's time has stood still (wall clock); past kSimStillMs a replay holds too.
    static constexpr int64_t kSimStillMs = 250;
    int64_t simStillMs_ = 0;
    // Paused, or the sim's time standing still: a replay holds.
    bool playbackHeld() const { return pauseFlags_ != 0 || simStillMs_ > kSimStillMs; }
    bool updateDriven_ = false;   // replays go by onUpdate
    int framesWithoutUpdate_ = 0;
    static constexpr int kUpdateLostFrames = 3;
    double lastWallDt_ = 0.0;
    double lastPlayDt_ = 0.0;

    // recording clock
    double recTime_ = 0.0;
    int64_t recUtcMs_ = 0;  // wall time at recTime_
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
