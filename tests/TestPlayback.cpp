#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "ClipFormat.h"
#include "Engine.h"
#include "Json.h"
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

namespace {

// Gear legs taking 8 s each way: up, down at 40 s, up again at 60 s, down at 80 s.
double downUpDown(double t) {
    auto travel = [](double t, double from) { return std::fmin(1.0, std::fmax(0.0, (t - from) / 8.0)); };
    if (t < 60.0) return travel(t, 40.0);
    if (t < 80.0) return 1.0 - travel(t, 60.0);
    return travel(t, 80.0);
}

Clip gearClip(EventType event, float eventT) {
    Clip c = makeClip(200.0);
    for (Frame& f : c.frames) {
        f.v[kGearCenter] = f.v[kGearLeft] = f.v[kGearRight] = downUpDown(f.t);
        f.v[kGearHandle] = 0.0;  // handle simvar never updated
    }
    c.meta.event = event;
    c.meta.eventOffsetSec = eventT;
    deriveGearCommand(c);
    return c;
}

}  // namespace

TEST(Playback_GearDownTimeForTheSeekBar) {
    const double moved = 0.03 * 8.0;  // the command turns once the legs have moved 3%
    // A landing: the last time before the touchdown, not one after it.
    CHECK_NEAR(gearDownTime(gearClip(EventType::Landing, 150.0f)), 80.0 + moved, 2.0 / 30.0);
    CHECK_NEAR(gearDownTime(gearClip(EventType::Landing, 70.0f)), 40.0 + moved, 2.0 / 30.0);
    // Anything else: the first.
    CHECK_NEAR(gearDownTime(gearClip(EventType::Manual, 0.0f)), 40.0 + moved, 2.0 / 30.0);
    // Never came down.
    CHECK(std::isnan(gearDownTime(makeClip(20.0))));

    PlaybackController p;
    p.load(gearClip(EventType::Landing, 150.0f), "a.rpl");
    CHECK_NEAR(p.gearDownT(), 80.0 + moved, 2.0 / 30.0);
    p.unload();
    CHECK(std::isnan(p.gearDownT()));
}

namespace {

// The 777 landing that started with its flaps still on their way from 25 to 30: 0.72 at the
// start, out at 15 s, the handle index following them (5 until 3.2 s, then 6); retracted from
// 40 s to 50 s after landing.
Clip flapsOnTheirWayClip() {
    Clip c = makeClip(60.0);
    for (Frame& f : c.frames) {
        const double out = f.t < 15.0   ? 0.7166 + (1.0 - 0.7166) * f.t / 15.0
                           : f.t < 40.0 ? 1.0
                           : f.t < 50.0 ? 1.0 - (f.t - 40.0) / 10.0
                                        : 0.0;
        f.v[kFlapsLeft] = out;
        f.v[kFlapsRight] = out;
        f.v[kFlapsHandleIndex] = f.t < 3.2 ? 5.0 : f.t < 40.0 ? 6.0 : f.t < 50.0 ? 3.0 : 0.0;
    }
    return c;
}

}  // namespace

TEST(Playback_FlapsDestinationIsWhereTheirMovementEnds) {
    const Clip c = flapsOnTheirWayClip();
    Frame d = flapsDestination(c, 0.0);
    CHECK_NEAR(d.v[kFlapsHandleIndex], 6.0, 0);
    CHECK_NEAR(d.v[kFlapsLeft], 1.0, 1e-9);
    CHECK_NEAR(d.t, 16.0, 0.1);  // once they've been still for kFlapsRestSec, not at the later retraction
    d = flapsDestination(c, 2.0);
    CHECK_NEAR(d.v[kFlapsHandleIndex], 6.0, 0);
    // Still there: the frame at that time, not where they go next.
    d = flapsDestination(c, 20.0);
    CHECK_NEAR(d.t, 20.0, 1e-9);
    CHECK_NEAR(d.v[kFlapsHandleIndex], 6.0, 0);
    d = flapsDestination(c, 38.5);
    CHECK_NEAR(d.v[kFlapsHandleIndex], 6.0, 0);  // the retraction starts more than kFlapsRestSec later
    d = flapsDestination(c, 45.0);
    CHECK_NEAR(d.v[kFlapsHandleIndex], 0.0, 0);
    CHECK_NEAR(d.v[kFlapsLeft], 0.0, 1e-9);
}

TEST(Playback_FlapsDestinationStopsWhereTheyTurnBack) {
    Clip c = makeClip(20.0);
    for (Frame& f : c.frames) {
        const double out = f.t < 5.0 ? 0.5 + 0.04 * f.t : 0.7 - 0.04 * (f.t - 5.0);
        f.v[kFlapsLeft] = out < 0.0 ? 0.0 : out;
        f.v[kFlapsRight] = f.v[kFlapsLeft];
        f.v[kFlapsHandleIndex] = f.t < 5.0 ? 4.0 : 2.0;
    }
    const Frame d = flapsDestination(c, 1.0);
    CHECK_NEAR(d.t, 5.0, 0.1);
    CHECK_NEAR(d.v[kFlapsLeft], 0.7, 0.01);
}

TEST(Playback_FlapsDestinationWithoutFlaps) {
    Clip c = makeClip(5.0);
    c.present.reset(static_cast<size_t>(kFlapsLeft));
    c.present.reset(static_cast<size_t>(kFlapsRight));
    CHECK_NEAR(flapsDestination(c, 2.0).t, 2.0, 1e-9);
    CHECK_NEAR(flapsDestination(Clip(), 2.0).t, 0.0, 0);
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
    double speed = NAN;  // playbackSpeed's
    void playbackSpeed(double s) override { speed = s; }
    void sendChannelEvent(int c, double v) override { events.push_back({c, v}); }
    void endPlayback(const Frame& snapshot, bool restore) override {
        ++ends;
        lastRestore = restore;
        restored = snapshot;
    }
    bool restoring() const override { return busyRestoring; }
    int finishes = 0;
    void finishRestore() override {
        ++finishes;
        busyRestoring = false;
    }
    // prepareJump: not ready for this many more calls.
    int prepareFrames = 0;
    int prepareCalls = 0;
    double prepareTarget = NAN;
    bool prepareJump(const Frame& target) override {
        ++prepareCalls;
        prepareTarget = target.t;
        if (prepareFrames > 0) {
            --prepareFrames;
            return false;
        }
        return true;
    }
    // settleJump: not ready for this many more calls; prep is what jumpPrep reports.
    int settleFrames = 0;
    int settleCalls = 0;
    double settleTarget = NAN;
    JumpPrep prep;
    bool settleJump(const Frame& target) override {
        ++settleCalls;
        settleTarget = target.t;
        if (settleFrames > 0) {
            --settleFrames;
            return false;
        }
        return true;
    }
    JumpPrep jumpPrep() const override { return prep; }

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
            engine.onFrame(liveFrame(simTime, lat), 1792247525000 + std::llround(simTime * 1000.0));  // a ms clock, as the sim's
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
    CHECK_EQ(h.sim.speed, 1.0);  // the sim side knows it (a control driven ahead of the clip)
    REQUIRE(h.engine.setSpeed(2.0, err));
    CHECK_EQ(h.sim.speed, 2.0);
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

TEST(EnginePlayback_OnlyOnTheAircraftItWasRecordedOn) {
    PlaybackHarness h;
    const std::string a350900 = "simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin/config";
    Clip clip = makeClip(3.0);
    clip.meta.aircraft = "A350-900 (Default Cabin)";
    clip.meta.aircraftPath = a350900;
    std::vector<uint8_t> bytes;
    writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
    h.storage.files[h.clipFile] = bytes;
    std::string err;

    h.engine.setAircraftTitle("737-800 PAX BW TC");
    h.engine.setAircraftKey("simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config");
    CHECK(!h.engine.play(h.clipFile, err));
    CHECK_EQ(err, std::string("recorded in the A350-900 (Default Cabin): switch to that aircraft to replay it"));
    CHECK(!h.engine.playbackActive());
    CHECK_EQ(h.sim.begins, 0);

    // The other variants share its control profile but not its clips: the -900ULR (another preset
    // in the same folder) and the -1000.
    h.engine.setAircraftTitle("A350-900 ULR");
    h.engine.setAircraftKey("simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin_ulr/config");
    const std::string family = h.engine.aircraftKey();
    CHECK(!h.engine.play(h.clipFile, err));
    CHECK_EQ(err, std::string("recorded in the A350-900 (Default Cabin): switch to that aircraft to replay it"));
    h.engine.setAircraftTitle("A350-1000");
    h.engine.setAircraftKey("simobjects/airplanes/a350-1000/presets/inibuilds/a350-1000/config");
    CHECK_EQ(h.engine.aircraftKey(), family);
    CHECK(!h.engine.play(h.clipFile, err));
    CHECK_EQ(h.sim.begins, 0);

    h.engine.setAircraftTitle("A350-900 (Default Cabin)");
    h.engine.setAircraftKey(a350900);
    CHECK_EQ(h.engine.aircraftKey(), family);
    REQUIRE(h.engine.play(h.clipFile, err));
    REQUIRE(h.engine.stopPlayback(err));

    // Clips from before the aircraft's path was recorded go by its title.
    clip.meta.aircraftPath.clear();
    writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
    h.storage.files[h.clipFile] = bytes;
    h.engine.setAircraftTitle("A350-900 ULR");
    CHECK(!h.engine.play(h.clipFile, err));
    h.engine.setAircraftTitle("a350-900 (default cabin)");
    CHECK(h.engine.play(h.clipFile, err));
}

namespace {

const char* const kA350900 = "simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin/config";
const char* const kA350Ulr = "simobjects/airplanes/a350/presets/inibuilds/a350-900_default_cabin_ulr/config";
const char* const kB738 = "simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config";

void putClip(MemStorage& storage, const std::string& file, const std::string& title, const std::string& path) {
    Clip clip = makeClip(3.0);
    clip.meta.aircraft = title;
    clip.meta.aircraftPath = path;
    std::vector<uint8_t> bytes;
    writeClip(clip.meta, VectorFrameSource(clip.frames), bytes);
    storage.files[file] = bytes;
}

// The clip list as the panel gets it: file -> "playable", and the page's "aircraftKnown".
std::map<std::string, bool> playableClips(const Engine& e, bool* aircraftKnown) {
    std::string json;
    e.clipsJson(0, 20, json);
    JsonValue v;
    std::string err;
    std::map<std::string, bool> out;
    if (!parseJson(json.data(), json.size(), v, &err) || !v.get("clips")) return out;
    if (aircraftKnown) *aircraftKnown = v.getBool("aircraftKnown", false);
    for (const JsonValue& c : v.get("clips")->array) out[c.getString("file", "")] = c.getBool("playable", false);
    return out;
}

}  // namespace

TEST(Engine_ClipListSaysWhichClipsReplayOnThisAircraft) {
    MemStorage storage;
    putClip(storage, "a350.rpl", "A350-900 (Default Cabin)", kA350900);
    putClip(storage, "b738.rpl", "737-800 PAX BW TC", kB738);
    putClip(storage, "old.rpl", "A350-900 (Default Cabin)", "");  // from before paths were recorded
    Engine e{storage};
    e.init();
    REQUIRE(e.index().size() == 3);

    // Not identified yet: only a clip going by its title could match, and there's no title yet.
    bool known = true;
    std::map<std::string, bool> p = playableClips(e, &known);
    CHECK(!known);
    CHECK(!p["a350.rpl"] && !p["b738.rpl"] && !p["old.rpl"]);

    // The -900ULR shares the -900's control profile, not its clips; the list comes again.
    int rev = e.clipsRevision();
    e.setAircraftTitle("A350-900 ULR");
    CHECK(e.clipsRevision() != rev);
    rev = e.clipsRevision();
    e.setAircraftKey(kA350Ulr);
    CHECK(e.clipsRevision() != rev);
    p = playableClips(e, &known);
    CHECK(known);
    CHECK(!p["a350.rpl"] && !p["b738.rpl"] && !p["old.rpl"]);

    // The -900: same family, but another aircraft.cfg, so the list comes again for it too.
    rev = e.clipsRevision();
    e.setAircraftKey(kA350900);
    CHECK(e.clipsRevision() != rev);
    e.setAircraftTitle("A350-900 (Default Cabin)");
    p = playableClips(e, nullptr);
    CHECK(p["a350.rpl"]);
    CHECK(!p["b738.rpl"]);
    CHECK(p["old.rpl"]);

    // The same aircraft again: nothing to refresh.
    rev = e.clipsRevision();
    e.setAircraftKey(kA350900);
    e.setAircraftTitle("A350-900 (Default Cabin)");
    CHECK_EQ(e.clipsRevision(), rev);
}

TEST(Engine_OldIndexGetsAircraftPathsFromItsClips) {
    MemStorage storage;
    putClip(storage, "a350.rpl", "A350-900 (Default Cabin)", kA350900);
    putClip(storage, "old.rpl", "A350-900 (Default Cabin)", "");
    const std::string v1 =
        "{\"version\":1,\"clips\":[{\"file\":\"a350.rpl\",\"event\":\"landing\",\"utcMs\":2,\"aircraft\":\"A350-900 (Default Cabin)\"},"
        "{\"file\":\"old.rpl\",\"event\":\"landing\",\"utcMs\":1,\"aircraft\":\"A350-900 (Default Cabin)\"}]}";
    storage.files["index.json"].assign(v1.begin(), v1.end());
    {
        Engine e{storage};
        e.init();
        REQUIRE(e.index().find("a350.rpl") && e.index().find("old.rpl"));
        CHECK_EQ(e.index().find("a350.rpl")->aircraftPath, std::string(kA350900));
        CHECK(e.index().find("old.rpl")->aircraftPath.empty());  // goes by its title

        // So the list agrees with Play: not on the -900ULR, whose title isn't the clip's either.
        e.setAircraftTitle("A350-900 ULR");
        e.setAircraftKey(kA350Ulr);
        std::map<std::string, bool> p = playableClips(e, nullptr);
        CHECK(!p["a350.rpl"]);
        CHECK(!p["old.rpl"]);
    }
    const std::vector<uint8_t>& saved = storage.files["index.json"];
    const std::string json(saved.begin(), saved.end());
    CHECK(json.find("\"version\":2") != std::string::npos);
    CHECK(json.find(kA350900) != std::string::npos);

    // Read once: the saved index has them, and a clip without one isn't read again.
    storage.files.erase("a350.rpl");
    storage.files.erase("old.rpl");
    putClip(storage, "a350.rpl", "A350-900 (Default Cabin)", kA350900);
    putClip(storage, "old.rpl", "A350-900 (Default Cabin)", kA350Ulr);  // (would be picked up if read)
    Engine again{storage};
    again.init();
    CHECK(again.index().pathsKnown());
    CHECK_EQ(again.index().find("a350.rpl")->aircraftPath, std::string(kA350900));
    CHECK(again.index().find("old.rpl")->aircraftPath.empty());
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

TEST(EnginePlayback_HoldsBeforeAJumpUntilTheSimIsReady) {
    PlaybackHarness h;
    std::string err;
    h.sim.prepareFrames = 30;  // e.g. half a second lowering the gear before going to the runway
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(30, -33.0);
    CHECK(h.sim.written.empty());  // nothing written: the aircraft stays where it was
    CHECK(h.sim.events.empty());
    CHECK_NEAR(h.engine.playback().time(), 0.0, 0);  // the playhead waits
    CHECK_NEAR(h.sim.prepareTarget, 0.0, 1e-9);
    std::string status;
    h.engine.statusJson(status);
    CHECK(status.find("\"preparing\":true") != std::string::npos);

    h.frames(1, -33.0);  // ready: the jump happens and playback runs on
    CHECK_EQ(h.sim.jumps, 1);
    REQUIRE(h.sim.written.size() == 1);
    h.engine.statusJson(status);
    CHECK(status.find("\"preparing\":false") != std::string::npos);
    h.frames(60, -33.0);
    CHECK_NEAR(h.engine.playback().time(), 1.0 + 1.0 / 60.0, 1e-3);  // the clock is in ms

    // A seek asks again, for the new target; frames in between don't.
    const int calls = h.sim.prepareCalls;
    REQUIRE(h.engine.seek(5.0, err));
    h.frames(1, -33.0);
    CHECK_EQ(h.sim.prepareCalls, calls + 1);
    CHECK_NEAR(h.sim.prepareTarget, 5.0, 1e-9);
}

TEST(EnginePlayback_HoldsAtTheStartAfterAJumpUntilTheSimIsReady) {
    PlaybackHarness h;
    std::string err;
    h.sim.prepareFrames = 10;  // the gear and engines getting ready where the aircraft is
    h.sim.settleFrames = 30;   // then half a second at the start, the flaps getting there
    h.sim.prep.gear = PrepStep::Done;
    h.sim.prep.engines = PrepStep::Active;
    h.sim.prep.enginesDown = true;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(10, -33.0);
    std::string status;
    h.engine.statusJson(status);
    CHECK(status.find("\"preparing\":true,\"prep\":{\"gear\":\"done\",\"flaps\":\"\",\"flapsIn\":false,\"engines\":"
                      "\"active\",\"enginesDown\":true}") != std::string::npos);
    CHECK(h.sim.written.empty());
    h.sim.prep.engines = PrepStep::Done;
    h.sim.prep.flaps = PrepStep::Active;
    h.sim.prep.flapsIn = true;

    h.frames(1, -33.0);  // the jump: settleJump is asked from its frame on
    CHECK_EQ(h.sim.jumps, 1);
    CHECK_EQ(h.sim.settleCalls, 1);
    h.frames(29, -33.0);
    CHECK_EQ(h.sim.settleCalls, 30);
    CHECK_EQ(h.sim.jumps, 1);
    // Held at the start (the jump's frame steps as ever): written every frame, the playhead
    // still, the events sent once (the jump's).
    REQUIRE(h.sim.written.size() == 30);
    const double held = h.sim.written[0].t;
    CHECK(held > 0.0 && held < 0.02);
    for (const Frame& f : h.sim.written) CHECK_NEAR(f.t, held, 1e-9);
    CHECK_NEAR(h.engine.playback().time(), held, 0);
    CHECK_NEAR(h.sim.settleTarget, held, 1e-9);
    CHECK_EQ(h.sim.events.size(), kClipEventChannels);
    h.engine.statusJson(status);
    CHECK(status.find("\"preparing\":true,\"prep\":{\"gear\":\"done\",\"flaps\":\"active\",\"flapsIn\":true,") !=
          std::string::npos);

    h.frames(1, -33.0);  // ready: the replay starts from there
    CHECK_EQ(h.sim.settleCalls, 31);
    h.engine.statusJson(status);
    CHECK(status.find("\"preparing\":false,\"prep\":{\"gear\":\"\",\"flaps\":\"\",\"flapsIn\":false,\"engines\":\"\","
                      "\"enginesDown\":false}") != std::string::npos);
    h.frames(60, -33.0);
    CHECK_NEAR(h.engine.playback().time(), held + 1.0, 1e-3);
    CHECK_EQ(h.sim.settleCalls, 31);  // only after a jump

    // A seek during the hold jumps again, and holds there instead.
    h.sim.settleFrames = 5;
    REQUIRE(h.engine.seek(5.0, err));
    h.frames(3, -33.0);
    CHECK_NEAR(h.engine.playback().time(), h.sim.settleTarget, 0);
    CHECK_NEAR(h.sim.settleTarget, 5.0 + 1.0 / 60.0, 1e-3);
    REQUIRE(h.engine.seek(7.0, err));
    h.frames(10, -33.0);  // two more frames held, ready on the third, then seven frames on
    CHECK_NEAR(h.sim.settleTarget, 7.0 + 1.0 / 60.0, 1e-3);
    CHECK_NEAR(h.engine.playback().time(), 7.0 + 8.0 / 60.0, 1e-3);
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

TEST(EnginePlayback_RestoreWaitEndsAfterTenSeconds) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(30, -33.0);
    CHECK_NEAR(h.engine.restoreSecondsLeft(), 0.0, 0);  // not while playing
    h.sim.busyRestoring = true;  // the gear never comes down
    REQUIRE(h.engine.stopPlayback(err));
    h.frames(60, -33.0);  // 1 s
    CHECK_NEAR(h.engine.restoreSecondsLeft(), 9.0, 0.05);
    std::string json;
    h.engine.statusJson(json);
    CHECK(json.find("\"restoreIn\":8.9") != std::string::npos || json.find("\"restoreIn\":9") != std::string::npos);
    CHECK(json.find("\"restoreMax\":10") != std::string::npos);
    h.frames(60 * 8, -33.0);  // 9 s
    CHECK_EQ(h.sim.finishes, 0);
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("restoring aircraft"));
    h.frames(60 + 2, -33.0);  // just past 10 s: let go
    CHECK_EQ(h.sim.finishes, 1);
    CHECK_NEAR(h.engine.restoreSecondsLeft(), 0.0, 0);
    h.frames(60, -33.0);
    CHECK_EQ(h.sim.finishes, 1);
    CHECK(h.engine.recording());
}

TEST(EnginePlayback_PlayingAgainEndsTheRestoreCountdown) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    h.frames(30, -33.0);
    h.sim.busyRestoring = true;
    REQUIRE(h.engine.stopPlayback(err));
    h.frames(60, -33.0);
    REQUIRE(h.engine.play(h.clipFile, err));
    h.sim.busyRestoring = false;  // SimBridge::beginPlayback drops the wait
    h.frames(60 * 6, -33.0);
    CHECK_NEAR(h.engine.restoreSecondsLeft(), 0.0, 0);
    CHECK_EQ(h.sim.finishes, 0);
}

// The sim's time can step unevenly while frames come evenly (on the A350: 18, 37, 28 ms a frame,
// sometimes 0, each frame 27 to 28 ms apart): the replay moves by the real time between frames.
// It holds while the sim is paused, or its time stands still for longer than a frame or two.
TEST(EnginePlayback_StepsByRealTimeBetweenFrames) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    int64_t utc = 1792247600000;
    h.engine.onFrame(liveFrame(h.simTime, -33.0), utc);
    const double steps[] = {0.018, 0.037, 0.0, 0.045, 0.037, 0.027};
    double last = h.engine.playback().time();
    for (int i = 0; i < 60; ++i) {
        h.simTime += steps[i % 6];
        utc += 27;
        h.engine.onFrame(liveFrame(h.simTime, -33.0), utc);
        const double t = h.engine.playback().time();
        CHECK_NEAR(t - last, 0.027, 1e-6);
        last = t;
    }
    h.engine.setPauseFlags(1);  // paused
    for (int i = 0; i < 30; ++i) {
        utc += 27;
        h.engine.onFrame(liveFrame(h.simTime, -33.0), utc);
    }
    CHECK_NEAR(h.engine.playback().time(), last, 1e-9);
    h.engine.setPauseFlags(0);
    for (int i = 0; i < 30; ++i) {  // the sim's time standing still without a pause: a little, then held
        utc += 27;
        h.engine.onFrame(liveFrame(h.simTime, -33.0), utc);
    }
    const double held = h.engine.playback().time();
    CHECK(held - last <= 0.3);
    utc += 27;
    h.engine.onFrame(liveFrame(h.simTime, -33.0), utc);
    CHECK_NEAR(h.engine.playback().time(), held, 1e-9);
    h.simTime += 0.027;  // and on again with it
    utc += 27;
    h.engine.onFrame(liveFrame(h.simTime, -33.0), utc);
    CHECK_NEAR(h.engine.playback().time(), held + 0.027, 1e-6);
}

// With the sim's per-frame update (onUpdate) the replay steps by its frame time and writes from
// there, once a frame; data frames in between don't move it. If the updates stop, the data
// frames drive it again.
TEST(EnginePlayback_StepsByTheSimsFrameUpdate) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    auto frame = [&](bool update, double frameDt, double simDt) {
        if (update) h.engine.onUpdate(frameDt);
        h.simTime += simDt;
        h.engine.onFrame(liveFrame(h.simTime, -33.0), 1792247525000 + std::llround(h.simTime * 1000.0));
    };
    frame(true, 1.0 / 36.0, 1.0 / 36.0);
    double last = h.engine.playback().time();
    size_t writes = h.sim.written.size();
    const double simSteps[] = {0.0185, 0.037, 0.0, 0.0556, 0.0278};
    for (int i = 0; i < 72; ++i) {
        frame(true, i == 40 ? 0.0633 : 1.0 / 36.0, simSteps[i % 5]);  // a 63 ms hitch at frame 40
        const double t = h.engine.playback().time();
        CHECK_NEAR(t - last, i == 40 ? 0.0633 : 1.0 / 36.0, 1e-9);
        CHECK_EQ(h.sim.written.size(), writes + 1);  // one write a frame
        last = t;
        writes = h.sim.written.size();
    }
    // Another data frame within the same frame (no update before it): nothing moves.
    h.engine.onFrame(liveFrame(h.simTime, -33.0), 1792247525000 + std::llround(h.simTime * 1000.0));
    CHECK_NEAR(h.engine.playback().time(), last, 0);
    CHECK_EQ(h.sim.written.size(), writes);

    h.engine.setPauseFlags(1);  // paused: held
    for (int i = 0; i < 10; ++i) frame(true, 1.0 / 36.0, 0.0);
    CHECK_NEAR(h.engine.playback().time(), last, 1e-12);
    h.engine.setPauseFlags(0);
    frame(true, 1.0 / 36.0, 1.0 / 36.0);  // the data frame after the pause lifts the hold
    last = h.engine.playback().time();
    frame(true, 1.0 / 36.0, 1.0 / 36.0);
    CHECK_NEAR(h.engine.playback().time() - last, 1.0 / 36.0, 1e-9);

    // The updates stop: three data frames later the data frames drive the replay again.
    last = h.engine.playback().time();
    for (int i = 0; i < 30; ++i) frame(false, 0.0, 1.0 / 36.0);
    CHECK(h.engine.playback().time() - last > 25.0 / 36.0);
}

TEST(EnginePlayback_DeletingThePlayingClipStopsIt) {
    PlaybackHarness h;
    std::string err;
    REQUIRE(h.engine.play(h.clipFile, err));
    REQUIRE(h.engine.deleteClip(h.clipFile, err));
    CHECK(!h.engine.playbackActive());
    CHECK(h.sim.lastRestore);
}
