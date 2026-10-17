#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "ClipFormat.h"
#include "Engine.h"
#include "PlaybackController.h"
#include "TestHarness.h"

using namespace br;

namespace {

Clip makeClip(double seconds, double rate = 30.0) {
    Clip c;
    const size_t n = static_cast<size_t>(seconds * rate) + 1;
    c.frames.resize(n);
    for (size_t i = 0; i < n; ++i) {
        Frame& f = c.frames[i];
        for (double& v : f.v) v = 0.0;
        f.t = static_cast<double>(i) / rate;
        f.v[kLat] = 10.0 + f.t;  // 1 degree per second, easy to check
        f.v[kHeading] = std::fmod(350.0 + 10.0 * f.t, 360.0);
        f.v[kLightLanding] = f.t >= 2.0 ? 1.0 : 0.0;
        f.v[kLightNav] = 1.0;
    }
    c.present.set();
    c.meta.event = EventType::Landing;
    c.meta.eventOffsetSec = 3.0f;
    return c;
}

}  // namespace

TEST(Playback_AdvanceSeekSpeedAndEnd) {
    PlaybackController p;
    CHECK(!p.loaded());
    p.load(makeClip(10.0), "a.rpl");
    REQUIRE(p.loaded());
    CHECK_NEAR(p.duration(), 10.0, 1e-9);
    CHECK(!p.playing());
    p.advance(1.0);  // paused: no movement
    CHECK_NEAR(p.time(), 0.0, 0);

    p.play();
    p.advance(1.5);
    CHECK_NEAR(p.time(), 1.5, 1e-12);
    Frame f;
    p.current(f);
    CHECK_NEAR(f.v[kLat], 11.5, 1e-9);

    p.setSpeed(4.0);
    p.advance(0.5);
    CHECK_NEAR(p.time(), 3.5, 1e-12);
    p.setSpeed(100.0);
    CHECK_NEAR(p.speed(), 4.0, 0);
    p.setSpeed(0.01);
    CHECK_NEAR(p.speed(), 0.25, 0);
    p.advance(2.0);
    CHECK_NEAR(p.time(), 4.0, 1e-12);

    p.seek(-3.0);
    CHECK_NEAR(p.time(), 0.0, 0);
    p.seek(1e9);
    CHECK_NEAR(p.time(), 10.0, 0);
    p.seek(9.0);
    p.setSpeed(1.0);
    p.advance(5.0);  // runs off the end: clamps and pauses
    CHECK_NEAR(p.time(), 10.0, 0);
    CHECK(!p.playing());
    p.play();  // play at the end restarts
    CHECK_NEAR(p.time(), 0.0, 0);
    CHECK(p.playing());
}

namespace {

// Gear that starts retracting at `moveStart` and is up 8 s later; the recorded handle simvar
// only flips at `handleFlip` (the A320neo V2 pattern seen in a real clip).
Clip retractingGearClip(double moveStart, double handleFlip) {
    Clip c = makeClip(30.0);
    for (Frame& f : c.frames) {
        const double ext = f.t < moveStart ? 1.0 : (f.t > moveStart + 8.0 ? 0.0 : 1.0 - (f.t - moveStart) / 8.0);
        f.v[kGearCenter] = ext;
        f.v[kGearLeft] = ext;
        f.v[kGearRight] = ext;
        f.v[kGearHandle] = f.t < handleFlip ? 1.0 : 0.0;
    }
    return c;
}

double firstTimeHandleIs(const Clip& c, double state) {
    for (const Frame& f : c.frames) {
        if (f.v[kGearHandle] == state) return f.t;
    }
    return -1.0;
}

int handleChanges(const Clip& c) {
    int n = 0;
    for (size_t i = 1; i < c.frames.size(); ++i) n += c.frames[i].v[kGearHandle] != c.frames[i - 1].v[kGearHandle];
    return n;
}

}  // namespace

TEST(Playback_GearCommandFollowsPositionsNotLaggingHandle) {
    Clip c = retractingGearClip(10.0, 22.0);
    REQUIRE(deriveGearCommand(c));
    CHECK_NEAR(firstTimeHandleIs(c, 0.0), 10.0 + 0.03 * 8.0, 2.0 / 30.0);  // as soon as it moves
    CHECK_NEAR(c.frames.front().v[kGearHandle], 1.0, 0);
    CHECK_NEAR(c.frames.back().v[kGearHandle], 0.0, 0);
    CHECK_EQ(handleChanges(c), 1);  // no flip-flopping while the gear is in mid-travel
}

TEST(Playback_GearCommandFollowsReversal) {
    // Gear raised, then lowered again halfway up: up, then down, nothing in between.
    Clip c = makeClip(20.0);
    for (Frame& f : c.frames) {
        const double ext = f.t < 2.0 ? 1.0 : (f.t < 6.0 ? 1.0 - (f.t - 2.0) / 8.0 : 0.5 + (f.t - 6.0) / 8.0);
        f.v[kGearCenter] = f.v[kGearLeft] = f.v[kGearRight] = ext > 1.0 ? 1.0 : ext;
        f.v[kGearHandle] = 1.0;
    }
    REQUIRE(deriveGearCommand(c));
    CHECK_EQ(handleChanges(c), 2);
    CHECK_NEAR(firstTimeHandleIs(c, 0.0), 2.0 + 0.03 * 8.0, 2.0 / 30.0);
    CHECK_NEAR(c.frames.back().v[kGearHandle], 1.0, 0);
}

TEST(Playback_GearCommandForExtension) {
    Clip c = makeClip(20.0);
    for (Frame& f : c.frames) {
        const double ext = f.t < 5.0 ? 0.0 : (f.t > 13.0 ? 1.0 : (f.t - 5.0) / 8.0);
        f.v[kGearCenter] = f.v[kGearLeft] = f.v[kGearRight] = ext;
        f.v[kGearHandle] = 0.0;  // handle simvar never updated
    }
    REQUIRE(deriveGearCommand(c));
    CHECK_NEAR(firstTimeHandleIs(c, 1.0), 5.0 + 0.03 * 8.0, 2.0 / 30.0);
}

TEST(Playback_FixedGearKeepsRecordedHandle) {
    Clip c = makeClip(10.0);
    for (Frame& f : c.frames) {
        f.v[kGearCenter] = f.v[kGearLeft] = f.v[kGearRight] = 1.0;
        f.v[kGearHandle] = 1.0;
    }
    CHECK(!deriveGearCommand(c));
    Clip none = makeClip(10.0);
    none.present.reset(kGearCenter).reset(kGearLeft).reset(kGearRight);
    CHECK(!deriveGearCommand(none));
}

TEST(Playback_ChannelsWithoutValuesAreNotPlayed) {
    // e.g. an input event the recording aircraft didn't have: in the file, but all NaN.
    Clip c = makeClip(2.0);
    for (Frame& f : c.frames) f.v[kInputGear] = NAN;
    c.frames[5].v[kInputThrottle1] = NAN;  // a gap is fine
    PlaybackController p;
    p.load(std::move(c), "a.rpl");
    CHECK(!p.present().test(kInputGear));
    CHECK(p.present().test(kInputThrottle1));
    CHECK(p.present().test(kLat));
}

TEST(Playback_PeakPerChannel) {
    // The A320neo V2's throttle input event in percent: full reverse ~0, idle 16.7, TOGA 86.
    Clip c = makeClip(3.0);
    for (Frame& f : c.frames) {
        f.v[kInputThrottle1] = f.t < 1.0 ? 16.7 : (f.t < 2.0 ? 0.17 : 86.0);
        f.v[kInputGear] = 1.0;
    }
    c.frames[3].v[kInputThrottle1] = NAN;
    PlaybackController p;
    p.load(std::move(c), "a.rpl");
    CHECK_NEAR(p.peak().v[kInputThrottle1], 86.0, 1e-9);
    CHECK_NEAR(p.peak().v[kInputGear], 1.0, 0);
    CHECK_NEAR(p.peak().v[kLat], 13.0, 1e-9);  // 10 + t, t up to 3 s
}

TEST(Playback_InterpolatesHeadingThroughNorth) {
    PlaybackController p;
    p.load(makeClip(5.0), "a.rpl");
    p.seek(1.0);  // 350 + 10 = 0 (360)
    Frame f;
    p.current(f);
    CHECK_NEAR(f.v[kHeading], 0.0, 1e-6);
    p.seek(1.0 + 1.0 / 60.0);  // halfway between two samples just past north
    p.current(f);
    CHECK_NEAR(f.v[kHeading], 10.0 / 60.0, 1e-6);
}

// ------------------------------------------------------------------- engine-level playback

namespace {

struct MemStorage : IStorage {
    std::map<std::string, std::vector<uint8_t>> files;
    bool writeFile(const std::string& n, const uint8_t* d, size_t s) override {
        files[n].assign(d, d + s);
        return true;
    }
    bool readFile(const std::string& n, std::vector<uint8_t>& out) override {
        auto it = files.find(n);
        if (it == files.end()) return false;
        out = it->second;
        return true;
    }
    bool removeFile(const std::string& n) override { return files.erase(n) > 0; }
    bool replaceFile(const std::string& a, const std::string& b) override {
        auto it = files.find(a);
        if (it == files.end()) return false;
        files[b] = it->second;
        files.erase(a);
        return true;
    }
    bool listFiles(std::vector<std::string>& names) override {
        names.clear();
        for (auto& kv : files) names.push_back(kv.first);
        return true;
    }
};

struct FakeSim : ISimOutput {
    int begins = 0;
    int ends = 0;
    int jumps = 0;
    bool busyRestoring = false;
    bool lastRestore = false;
    Frame restored;
    std::vector<Frame> written;
    std::vector<std::pair<int, double>> events;
    ChannelMask present;
    Frame peak;

    void beginPlayback(const Clip& clip, const Frame& pk) override {
        ++begins;
        present = clip.present;
        peak = pk;
    }
    void writePlaybackFrame(const Frame& f, bool jumped) override {
        written.push_back(f);
        if (jumped) ++jumps;
    }
    void sendChannelEvent(int c, double v) override { events.push_back({c, v}); }
    void endPlayback(const Frame& snapshot, bool restore) override {
        ++ends;
        lastRestore = restore;
        restored = snapshot;
    }
    bool restoring() const override { return busyRestoring; }

    // Events sent for one channel, in order.
    std::vector<double> eventsFor(int channel) const {
        std::vector<double> out;
        for (const auto& e : events) {
            if (e.first == channel) out.push_back(e.second);
        }
        return out;
    }
};

// Event channels in makeClip() (it has every channel): gear handle, lights, reversers.
size_t eventChannelCount() {
    size_t n = 0;
    for (const ChannelDef& ch : kChannels) n += ch.play == PlayMode::Event ? 1 : 0;
    return n;
}
const size_t kClipEventChannels = eventChannelCount();

FrameInput liveFrame(double simTime, double lat) {
    FrameInput in;
    for (double& v : in.raw.v) v = 0.0;
    in.raw.v[kLat] = lat;
    in.raw.v[kAlt] = 3000.0;
    in.raw.v[kAgl] = 3000.0;
    in.raw.v[kGroundSpeed] = 150.0;
    in.raw.v[kLightLanding] = 0.0;
    in.raw.v[kLightNav] = 1.0;
    in.simTimeSec = simTime;
    return in;
}

struct PlaybackHarness {
    MemStorage storage;
    FakeSim sim;
    Engine engine{storage};
    double simTime = 0.0;
    std::string clipFile;

    PlaybackHarness() {
        engine.setSimOutput(&sim);
        engine.init();
        engine.setSimRunning(true);
        // Put a 10 s clip in the library the normal way (via the index), then fly on.
        const Clip clip = makeClip(10.0);
        std::vector<uint8_t> bytes;
        writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
        clipFile = "2026-10-17_1432_landing.rpl";
        storage.files[clipFile] = bytes;
        storage.files.erase("index.json");
        engine.init();  // adopts the file from disk
        frames(60, -33.0);
    }

    void frames(int n, double lat) {
        for (int i = 0; i < n; ++i) {
            simTime += 1.0 / 60.0;
            engine.onFrame(liveFrame(simTime, lat), 1792247525000);
        }
    }
};

}  // namespace

TEST(EnginePlayback_PlaysClipAndSuspendsRecording) {
    PlaybackHarness h;
    REQUIRE(h.engine.index().find(h.clipFile) != nullptr);
    const size_t buffered = h.engine.bufferedFrames();
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    CHECK_EQ(h.sim.begins, 1);
    CHECK(h.sim.present.test(kLat));
    CHECK(h.engine.playbackActive());

    h.frames(60, -33.0);  // 1 s of sim time
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("playback"));
    CHECK_EQ(h.engine.bufferedFrames(), buffered);  // nothing recorded during our playback
    REQUIRE(h.sim.written.size() == 60);
    CHECK_NEAR(h.sim.written.back().v[kLat], 11.0, 1e-6);  // clip lat = 10 + t
    CHECK_NEAR(h.engine.playback().time(), 1.0, 1e-6);
}

TEST(EnginePlayback_EventChannelsResyncOnPlayThenSendChanges) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(1, -33.0);
    // The first frame puts every event channel into the clip's state, whatever the aircraft had.
    CHECK_EQ(h.sim.jumps, 1);
    REQUIRE(h.sim.events.size() == kClipEventChannels);
    CHECK(h.sim.eventsFor(kLightLanding) == std::vector<double>{0.0});
    CHECK(h.sim.eventsFor(kLightNav) == std::vector<double>{1.0});

    h.frames(60 * 4, -33.0);  // landing light turns on at t = 2 s in the clip; nothing else changes
    CHECK_EQ(h.sim.jumps, 1);
    REQUIRE(h.sim.events.size() == kClipEventChannels + 1);
    CHECK_EQ(h.sim.events.back().first, static_cast<int>(kLightLanding));
    CHECK_NEAR(h.sim.events.back().second, 1.0, 0);

    // Stopping puts every event channel back to the live state: the landing light goes off.
    REQUIRE(h.engine.stopPlayback(err));
    REQUIRE(h.sim.events.size() == 2 * kClipEventChannels + 1);
    CHECK(h.sim.eventsFor(kLightLanding) == (std::vector<double>{0.0, 1.0, 0.0}));
    CHECK(h.sim.eventsFor(kLightNav) == (std::vector<double>{1.0, 1.0}));
}

TEST(EnginePlayback_SeekResyncsEventChannels) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(60 * 3, -33.0);  // past the landing light switching on at 2 s
    CHECK(h.sim.eventsFor(kLightLanding) == (std::vector<double>{0.0, 1.0}));

    REQUIRE(h.engine.seek(1.0, err));  // back to before it switched on
    h.frames(1, -33.0);
    CHECK_EQ(h.sim.jumps, 2);
    CHECK_EQ(h.sim.events.size(), 2 * kClipEventChannels + 1);
    CHECK(h.sim.eventsFor(kLightLanding) == (std::vector<double>{0.0, 1.0, 0.0}));

    h.frames(60 * 2, -33.0);  // and it switches on again when the playhead passes 2 s
    CHECK_EQ(h.sim.jumps, 2);
    CHECK(h.sim.eventsFor(kLightLanding) == (std::vector<double>{0.0, 1.0, 0.0, 1.0}));

    // Replay from the end restarts the clip: also a jump.
    h.frames(60 * 9, -33.0);
    CHECK(!h.engine.playback().playing());
    REQUIRE(h.engine.resumePlayback(err));
    h.frames(1, -33.0);
    CHECK_EQ(h.sim.jumps, 3);
    CHECK(h.sim.eventsFor(kLightLanding) == (std::vector<double>{0.0, 1.0, 0.0, 1.0, 0.0}));
    // Resuming from a pause mid-clip is not.
    REQUIRE(h.engine.pausePlayback(err));
    REQUIRE(h.engine.resumePlayback(err));
    h.frames(1, -33.0);
    CHECK_EQ(h.sim.jumps, 3);
}

TEST(EnginePlayback_SpeedSeekPause) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    REQUIRE(h.engine.setSpeed(2.0, err));
    h.frames(30, -33.0);  // 0.5 s at 2x
    CHECK_NEAR(h.engine.playback().time(), 1.0, 1e-6);
    REQUIRE(h.engine.pausePlayback(err));
    h.frames(30, -33.0);
    CHECK_NEAR(h.engine.playback().time(), 1.0, 1e-6);
    REQUIRE(h.engine.seek(7.5, err));
    h.frames(1, -33.0);
    CHECK_NEAR(h.sim.written.back().v[kLat], 17.5, 1e-6);  // paused frames still written after a seek
    REQUIRE(h.engine.resumePlayback(err));
    h.frames(60 * 3, -33.0);  // runs to the end and pauses there
    CHECK_NEAR(h.engine.playback().time(), 10.0, 1e-9);
    CHECK(!h.engine.playback().playing());
    CHECK(h.engine.playbackActive());  // still holding the last frame until stopped
}

TEST(EnginePlayback_StopRestoresSnapshotAndRearms) {
    PlaybackHarness h;
    std::string err;
    h.frames(1, -33.5);  // live state right before playback
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(120, -33.5);
    REQUIRE(h.engine.stopPlayback(err));
    CHECK_EQ(h.sim.ends, 1);
    CHECK(h.sim.lastRestore);
    CHECK_NEAR(h.sim.restored.v[kLat], -33.5, 0);
    CHECK(!h.engine.playbackActive());
    CHECK(!h.engine.stopPlayback(err));  // nothing to stop now

    const size_t before = h.engine.bufferedFrames();
    h.frames(30, -33.5);  // settling: not recorded yet
    CHECK_EQ(h.engine.bufferedFrames(), before);
    h.frames(60, -33.5);
    CHECK(h.engine.recording());
    CHECK(h.engine.bufferedFrames() > before);
    CHECK(!h.engine.detector().armed());  // playback counts as a relocation
}

TEST(EnginePlayback_SwitchingClipsKeepsOriginalSnapshot) {
    PlaybackHarness h;
    std::string err;
    h.frames(1, -40.0);
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(60, -40.0);
    REQUIRE(h.engine.play(h.clipFile, err));  // play again while active
    CHECK_EQ(h.sim.begins, 2);
    REQUIRE(h.engine.stopPlayback(err));
    CHECK_NEAR(h.sim.restored.v[kLat], -40.0, 0);
}

TEST(EnginePlayback_FlightLoadEndsWithoutRestore) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.engine.onFlightLoaded("test");
    CHECK_EQ(h.sim.ends, 1);
    CHECK(!h.sim.lastRestore);
    CHECK(!h.engine.playbackActive());
}

TEST(EnginePlayback_RejectsBadRequests) {
    PlaybackHarness h;
    std::string err;
    CHECK(!h.engine.play("missing.rpl", err));
    CHECK(!h.engine.play("../evil.rpl", err));
    CHECK(!h.engine.seek(1.0, err));
    CHECK(!h.engine.pausePlayback(err));
    h.engine.setAtMainMenu(true);
    CHECK(!h.engine.play(h.clipFile, err));
    CHECK_EQ(err, std::string("not in a flight"));

    // A corrupt file is reported, not played.
    h.engine.setAtMainMenu(false);
    h.storage.files[h.clipFile].resize(20);
    CHECK(!h.engine.play(h.clipFile, err));
    CHECK(!h.engine.playbackActive());
}

TEST(EnginePlayback_EventChannelsSendStateChangesOnly) {
    // A gear handle recorded mid-travel (0.3, 0.6, 0.9) is one state change: down.
    PlaybackHarness h;
    Clip clip = makeClip(3.0);
    for (Frame& f : clip.frames) {
        f.v[kLightLanding] = 0.0;
        f.v[kGearHandle] = f.t < 1.0 ? 0.0 : (f.t < 1.5 ? 0.3 : (f.t < 2.0 ? 0.6 : 0.9));
    }
    std::vector<uint8_t> bytes;
    writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
    h.storage.files[h.clipFile] = bytes;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(60 * 4, -33.0);
    // Initial sync (up), then a single change to down.
    CHECK(h.sim.eventsFor(kGearHandle) == (std::vector<double>{0.0, 1.0}));
    REQUIRE(h.engine.stopPlayback(err));  // live aircraft had the handle up: send it back up
    CHECK(h.sim.eventsFor(kGearHandle) == (std::vector<double>{0.0, 1.0, 0.0}));
}

TEST(EnginePlayback_ReversersFollowTheClip) {
    PlaybackHarness h;
    Clip clip = makeClip(6.0);
    for (Frame& f : clip.frames) {
        const bool rev = f.t >= 2.0 && f.t < 4.0;  // reverse on the rollout, then stowed
        f.v[kReverser1] = f.v[kReverser2] = rev ? 1.0 : 0.0;
        f.v[kThrottle1] = f.v[kThrottle2] = rev ? -20.0 : 0.0;
    }
    std::vector<uint8_t> bytes;
    writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
    h.storage.files[h.clipFile] = bytes;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(60 * 5, -33.0);
    CHECK(h.sim.eventsFor(kReverser1) == (std::vector<double>{0.0, 1.0, 0.0}));
    CHECK(h.sim.eventsFor(kReverser2) == (std::vector<double>{0.0, 1.0, 0.0}));
    // The reverse amount is the negative throttle lever, written as a Set channel.
    bool wroteReverse = false;
    for (const Frame& f : h.sim.written) wroteReverse |= f.v[kThrottle1] < -19.0;
    CHECK(wroteReverse);
}

TEST(EnginePlayback_GearRetractsWhenRecordedGearStartedMoving) {
    PlaybackHarness h;
    const Clip clip = retractingGearClip(10.0, 22.0);
    std::vector<uint8_t> bytes;
    writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
    h.storage.files[h.clipFile] = bytes;
    std::string err;
    h.frames(1, -33.0);
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(60 * 9, -33.0);  // 9 s: gear still down in the recording
    for (const auto& e : h.sim.events) CHECK(e.first != static_cast<int>(kGearHandle) || e.second == 1.0);
    h.frames(60 * 3, -33.0);  // 12 s: retraction began at 10 s
    bool sentUp = false;
    for (const auto& e : h.sim.events) sentUp |= e.first == static_cast<int>(kGearHandle) && e.second == 0.0;
    CHECK(sentUp);
}

TEST(EnginePlayback_RecordingWaitsWhileSimIsRestoring) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(30, -33.0);
    h.sim.busyRestoring = true;  // e.g. waiting for the gear to come back down
    REQUIRE(h.engine.stopPlayback(err));
    const size_t before = h.engine.bufferedFrames();
    h.frames(120, -33.0);  // well past the fixed settle frames
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("restoring aircraft"));
    CHECK_EQ(h.engine.bufferedFrames(), before);
    h.sim.busyRestoring = false;
    h.frames(30, -33.0);
    CHECK(h.engine.recording());
    CHECK(h.engine.bufferedFrames() > before);
}

TEST(EnginePlayback_DeletingThePlayingClipStopsIt) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    REQUIRE(h.engine.deleteClip(h.clipFile, err));
    CHECK(!h.engine.playbackActive());
    CHECK(h.sim.lastRestore);
}
