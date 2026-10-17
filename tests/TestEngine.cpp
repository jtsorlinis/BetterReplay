#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "ClipFormat.h"
#include "Engine.h"
#include "Json.h"
#include "TestHarness.h"
#include "TraceGen.h"

using namespace br;
using namespace tracegen;

namespace {

constexpr int64_t kUtcBase = 1792247525000;  // 2026-10-17 14:32:05 UTC

struct MemoryStorage : IStorage {
    std::map<std::string, std::vector<uint8_t>> files;
    bool failRename = false;
    bool failList = false;

    bool writeFile(const std::string& name, const uint8_t* data, size_t size) override {
        files[name].assign(data, data + size);
        return true;
    }
    bool readFile(const std::string& name, std::vector<uint8_t>& out) override {
        const auto it = files.find(name);
        if (it == files.end()) return false;
        out = it->second;
        return true;
    }
    bool removeFile(const std::string& name) override { return files.erase(name) > 0; }
    bool replaceFile(const std::string& from, const std::string& to) override {
        if (failRename) return false;
        const auto it = files.find(from);
        if (it == files.end()) return false;
        files[to] = it->second;
        files.erase(from);
        return true;
    }
    bool listFiles(std::vector<std::string>& names) override {
        if (failList) return false;
        names.clear();
        for (const auto& kv : files) names.push_back(kv.first);
        return true;
    }
    size_t clipCount() const {
        size_t n = 0;
        for (const auto& kv : files) n += isSafeClipFileName(kv.first) ? 1 : 0;
        return n;
    }
};

bool g_verbose = false;
void testLog(void*, int level, const char* msg) {
    if (g_verbose || level >= 2) std::printf("         [engine %d] %s\n", level, msg);
}

FrameInput toInput(const DetectorSample& s) {
    FrameInput in;
    for (double& v : in.raw.v) v = 0.0;
    in.raw.v[kOnGround] = s.onGround ? 1.0 : 0.0;
    in.raw.v[kAgl] = s.aglFt;
    in.raw.v[kGroundSpeed] = s.groundSpeedKt;
    in.raw.v[kVerticalSpeed] = s.verticalSpeedFpm;
    in.raw.v[kGForce] = s.gForce;
    in.raw.v[kLat] = s.latDeg;
    in.raw.v[kLon] = s.lonDeg;
    in.raw.v[kAlt] = s.altFt;
    in.simTimeSec = s.t;
    in.touchdownNormalVelFps = s.touchdownNormalVelFps;
    return in;
}

struct Harness {
    MemoryStorage storage;
    Engine engine;
    int64_t utcOffsetMs = 0;

    explicit Harness(const EngineConfig& cfg = EngineConfig()) : engine(storage, cfg) {
        engine.setLogger(&testLog, nullptr);
        engine.init();
        engine.setSimRunning(true);
        engine.setAircraftTitle("Test Aircraft");
    }

    void feed(const std::vector<DetectorSample>& s, size_t from, size_t to) {
        for (size_t i = from; i < to && i < s.size(); ++i) {
            engine.onFrame(toInput(s[i]), kUtcBase + utcOffsetMs + static_cast<int64_t>(s[i].t * 1000.0));
        }
    }
    void feedAll(const std::vector<DetectorSample>& s) { feed(s, 0, s.size()); }

    std::vector<const ClipEntry*> clipsOf(EventType type) const {
        std::vector<const ClipEntry*> out;
        for (const ClipEntry& e : engine.index().clips()) {
            if (e.event == type) out.push_back(&e);
        }
        return out;
    }

    bool load(const std::string& file, Clip& clip) {
        const auto it = storage.files.find(file);
        if (it == storage.files.end()) return false;
        std::string err;
        return readClip(it->second.data(), it->second.size(), clip, &err);
    }
};

// Approach from 1500 ft and land; returns the touchdown sample index.
size_t landingTrace(TraceGen& g) {
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(25.0, 15.0, 0.0, true);
    g.hold(10.0, true);
    return td;
}

}  // namespace

TEST(Engine_FullFlightSavesTakeoffAndLanding) {
    TraceGen g(true, 0.0, 0.0);
    g.dt = 1.0 / 60.0;  // the sim runs faster than the recording rate
    g.hold(30.0, true);
    g.segment(30.0, 70.0, 0.0, true);
    g.segment(40.0, 90.0, 1200.0, false);
    g.segment(60.0, 140.0, 3000.0, false);
    g.hold(60.0, false);
    g.segment(120.0, 120.0, 1500.0, false);
    const size_t td = landingTrace(g);

    Harness h;
    h.feedAll(g.samples);

    REQUIRE(h.storage.clipCount() == 2);
    REQUIRE(h.storage.files.count("index.json") == 1);
    CHECK(h.storage.files.count("index.json.tmp") == 0);
    const auto takeoffs = h.clipsOf(EventType::Takeoff);
    const auto landings = h.clipsOf(EventType::Landing);
    REQUIRE(takeoffs.size() == 1);
    REQUIRE(landings.size() == 1);
    CHECK_EQ(h.engine.index().clips()[0].file, landings[0]->file);  // newest first
    CHECK(landings[0]->file.find("_landing.rpl") != std::string::npos);
    CHECK(landings[0]->file.compare(0, 11, "2026-10-17_") == 0);
    CHECK_EQ(landings[0]->aircraft, std::string("Test Aircraft"));
    CHECK(landings[0]->autoClip);
    CHECK_NEAR(landings[0]->touchdownVsFpm, -150.0, 2.0);
    CHECK_NEAR(landings[0]->durationSec, 120.0 + 21.25, 0.1);  // 115 -> 15 kt crosses 30 kt after 21.25 s

    Clip clip;
    REQUIRE(h.load(landings[0]->file, clip));
    CHECK(clip.meta.event == EventType::Landing);
    CHECK_NEAR(clip.meta.eventOffsetSec, 120.0, 0.1);
    CHECK_NEAR(clip.frames.size() / clip.duration(), 30.0, 0.5);  // decimated to 30 Hz
    for (size_t i = 1; i < clip.frames.size(); ++i) {
        if (!(clip.frames[i].t > clip.frames[i - 1].t)) {
            CHECK(clip.frames[i].t > clip.frames[i - 1].t);
            break;
        }
    }
    // The frame at eventOffsetSec is the first on-ground frame.
    for (size_t i = 1; i < clip.frames.size(); ++i) {
        if (clip.frames[i].v[kOnGround] >= 0.5 && clip.frames[i - 1].v[kOnGround] < 0.5) {
            CHECK_NEAR(clip.frames[i].t, clip.meta.eventOffsetSec, 1.0 / 30.0);
            break;
        }
    }
    // Clip start time is the touchdown wall time minus the pre-roll.
    const int64_t tdUtc = kUtcBase + static_cast<int64_t>(g.samples[td].t * 1000.0);
    CHECK_NEAR(static_cast<double>(clip.meta.utcStartMs), static_cast<double>(tdUtc - 120000), 100.0);

    Clip to;
    REQUIRE(h.load(takeoffs[0]->file, to));
    CHECK(to.meta.event == EventType::Takeoff);
    CHECK(std::isnan(to.meta.touchdownVsFpm));
}

TEST(Engine_PauseMidLandingIsSqueezedOut) {
    TraceGen base(false, 1500.0, 120.0);
    landingTrace(base);
    Harness h0;
    h0.feedAll(base.samples);
    REQUIRE(h0.clipsOf(EventType::Landing).size() == 1);
    const double baseDuration = h0.clipsOf(EventType::Landing)[0]->durationSec;

    TraceGen g(false, 1500.0, 120.0);
    const size_t td = landingTrace(g);
    const size_t pauseAt = td + 45;
    Harness h;
    h.feed(g.samples, 0, pauseAt);
    h.engine.setPauseFlags(1);
    for (int i = 0; i < 1800; ++i) h.feed(g.samples, pauseAt - 1, pauseAt);  // sim time frozen
    CHECK(!h.engine.recording());
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("paused"));
    h.engine.setPauseFlags(0);
    h.feed(g.samples, pauseAt, g.samples.size());

    const auto landings = h.clipsOf(EventType::Landing);
    REQUIRE(landings.size() == 1);
    CHECK_NEAR(landings[0]->durationSec, baseDuration, 0.05);
}

TEST(Engine_SlewOnFinalDisarmsDetection) {
    TraceGen g(false, 1500.0, 120.0);
    g.segment(150.0, 125.0, 200.0, false);
    const size_t slewFrom = g.index();
    g.hold(5.0, false);
    const size_t slewTo = g.index();
    g.segment(5.0, 115.0, 0.0, false);  // touches down 5 s after leaving slew
    g.segment(25.0, 15.0, 0.0, true);

    Harness h;
    h.feed(g.samples, 0, slewFrom);
    for (size_t i = slewFrom; i < slewTo; ++i) {
        FrameInput in = toInput(g.samples[i]);
        in.slewActive = true;
        h.engine.onFrame(in, kUtcBase);
    }
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("slew"));
    h.feed(g.samples, slewTo, g.samples.size());
    CHECK(h.clipsOf(EventType::Landing).empty());
    CHECK(h.engine.detector().armed());  // re-armed by the end of the rollout
}

TEST(Engine_ManualSaveOfLastMinutes) {
    Harness empty;
    std::string err;
    CHECK(!empty.engine.saveLast(kUtcBase, err));
    CHECK_EQ(err, std::string("nothing recorded yet"));

    TraceGen g(false, 3000.0, 150.0);
    g.hold(20.0, false);
    Harness h;
    h.feedAll(g.samples);
    REQUIRE(h.engine.saveLast(kUtcBase + 20000, err));
    REQUIRE(h.engine.index().size() == 1);
    const ClipEntry& e = h.engine.index().clips()[0];
    CHECK(e.event == EventType::Manual);
    CHECK(!e.autoClip);
    CHECK_EQ(e.file, std::string("2026-10-17_1432_manual.rpl"));
    CHECK_NEAR(e.durationSec, 20.0, 0.1);
}

TEST(Engine_RetentionKeepsNewestAutoClips) {
    EngineConfig cfg;
    cfg.maxAutoClips = 2;
    Harness h(cfg);
    std::vector<std::string> saved;
    for (int flight = 0; flight < 4; ++flight) {
        h.utcOffsetMs = flight * 3600000LL;
        h.engine.onFlightLoaded("test flight");
        TraceGen g(false, 1500.0, 120.0);
        landingTrace(g);
        // Each new flight restarts sim time from 0.
        h.feedAll(g.samples);
        saved.push_back(h.engine.index().clips()[0].file);
        if (flight == 0) {
            std::string err;
            REQUIRE(h.engine.setKeep(saved[0], true, err));
        }
    }
    // Flight 0 is kept; of flights 1..3 only the newest two survive.
    CHECK_EQ(h.engine.index().size(), 3u);
    CHECK(h.engine.index().find(saved[0]) != nullptr);
    CHECK(h.engine.index().find(saved[1]) == nullptr);
    CHECK(h.storage.files.count(saved[1]) == 0);
    CHECK(h.engine.index().find(saved[2]) != nullptr);
    CHECK(h.engine.index().find(saved[3]) != nullptr);
    CHECK_EQ(h.storage.clipCount(), 3u);
}

TEST(Engine_IndexPersistsAndRebuildsFromDisk) {
    MemoryStorage storage;
    std::string err;
    {
        Engine e(storage);
        e.init();
        e.setSimRunning(true);
        TraceGen g(false, 3000.0, 150.0);
        g.hold(10.0, false);
        for (const DetectorSample& s : g.samples) e.onFrame(toInput(s), kUtcBase);
        REQUIRE(e.saveLast(kUtcBase, err));
        REQUIRE(e.setKeep(e.index().clips()[0].file, true, err));
    }
    {
        Engine e(storage);
        e.init();
        REQUIRE(e.index().size() == 1);
        CHECK(e.index().clips()[0].keep);
    }
    // Lose the index: the clip is adopted back from its header (the keep flag is lost).
    storage.files.erase("index.json");
    {
        Engine e(storage);
        e.init();
        REQUIRE(e.index().size() == 1);
        CHECK(!e.index().clips()[0].keep);
        CHECK(e.index().clips()[0].event == EventType::Manual);
        CHECK_NEAR(e.index().clips()[0].durationSec, 10.0, 0.1);
    }
    // A clip file that vanished is dropped from the index.
    storage.files.erase("2026-10-17_1432_manual.rpl");
    {
        Engine e(storage);
        e.init();
        CHECK_EQ(e.index().size(), 0u);
    }
}

TEST(Engine_IndexFallbacks) {
    MemoryStorage storage;
    storage.failRename = true;  // rename unsupported: index.json is written directly
    std::string err;
    {
        Engine e(storage);
        e.init();
        e.setSimRunning(true);
        TraceGen g(false, 3000.0, 150.0);
        g.hold(5.0, false);
        for (const DetectorSample& s : g.samples) e.onFrame(toInput(s), kUtcBase);
        REQUIRE(e.saveLast(kUtcBase, err));
    }
    CHECK(storage.files.count("index.json") == 1);

    // Corrupt index.json but a good .tmp (crash between write and rename): .tmp wins.
    storage.files["index.json.tmp"] = storage.files["index.json"];
    storage.files["index.json"] = std::vector<uint8_t>{'{', 'x'};
    storage.failList = true;  // also skip reconciliation, to prove the .tmp was used
    Engine e(storage);
    e.init();
    CHECK_EQ(e.index().size(), 1u);
}

TEST(Engine_KeepAndDeleteValidateNames) {
    Harness h;
    TraceGen g(false, 3000.0, 150.0);
    g.hold(5.0, false);
    h.feedAll(g.samples);
    std::string err;
    REQUIRE(h.engine.saveLast(kUtcBase, err));
    const std::string file = h.engine.index().clips()[0].file;
    CHECK(!h.engine.deleteClip("../index.json", err));
    CHECK(!h.engine.deleteClip("nope.rpl", err));
    CHECK(!h.engine.setKeep("nope.rpl", true, err));
    const int rev = h.engine.clipsRevision();
    REQUIRE(h.engine.deleteClip(file, err));
    CHECK(h.engine.clipsRevision() > rev);
    CHECK_EQ(h.storage.clipCount(), 0u);
    CHECK_EQ(h.engine.index().size(), 0u);
}

TEST(Engine_SuspendReasons) {
    Harness h;
    TraceGen g(false, 3000.0, 150.0);
    g.hold(4.0, false);
    h.feed(g.samples, 0, 60);
    const size_t before = h.engine.bufferedFrames();
    CHECK(before > 50);
    h.engine.setSimRunning(false);
    h.feed(g.samples, 60, 90);
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("sim not running"));
    CHECK_EQ(h.engine.bufferedFrames(), before);
    h.engine.setSimRunning(true);
    h.engine.setAtMainMenu(true);
    h.feed(g.samples, 90, 100);
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("main menu"));
    h.engine.setAtMainMenu(false);
    h.engine.setSimReplayActive(true);
    h.feed(g.samples, 100, 110);
    CHECK_EQ(std::string(h.engine.suspendReason()), std::string("sim replay"));
    h.engine.setSimReplayActive(false);
    h.feed(g.samples, 110, 120);
    CHECK(h.engine.recording());
    CHECK(!h.engine.detector().armed());  // relocation-type suspensions re-arm the detector
}

TEST(Engine_FlightLoadedClearsBuffer) {
    Harness h;
    TraceGen g(false, 3000.0, 150.0);
    g.hold(5.0, false);
    h.feedAll(g.samples);
    CHECK(h.engine.bufferedFrames() > 100);
    h.engine.onFlightLoaded("new flight");
    CHECK_EQ(h.engine.bufferedFrames(), 0u);
}

TEST(Engine_DecimatesToSampleRate) {
    for (double fps : {60.0, 45.0, 30.0, 144.0}) {
        Harness h;
        TraceGen g(false, 3000.0, 150.0);
        g.dt = 1.0 / fps;
        g.hold(10.0, false);
        h.feedAll(g.samples);
        const double rate = (h.engine.bufferedFrames() - 1) / h.engine.bufferedSeconds();
        CHECK_NEAR(rate, std::min(30.0, fps), 1.0);
    }
}

TEST(Engine_BufferHoldsAboutFiveMinutes) {
    Harness h;
    TraceGen g(false, 30000.0, 450.0);
    g.hold(1200.0, false);  // 20 minutes
    h.feedAll(g.samples);
    CHECK(h.engine.bufferedSeconds() >= 300.0);
    CHECK(h.engine.bufferedSeconds() <= 320.0);
}

TEST(Engine_StatusAndClipsJson) {
    Harness h;
    TraceGen g(false, 3000.0, 150.0);
    g.hold(15.0, false);
    h.feedAll(g.samples);
    std::string err;
    REQUIRE(h.engine.saveLast(kUtcBase, err));

    std::string json;
    h.engine.statusJson(json);
    JsonValue v;
    REQUIRE(parseJson(json.data(), json.size(), v, &err));
    CHECK(v.getBool("rec", false));
    CHECK(v.getBool("armed", false));
    CHECK_EQ(v.getString("phase", ""), std::string("airborne"));
    CHECK_NEAR(v.getNumber("bufferSec", 0), 15.0, 0.1);
    CHECK_EQ(v.getString("aircraft", ""), std::string("Test Aircraft"));

    h.engine.clipsJson(0, 20, json);
    REQUIRE(parseJson(json.data(), json.size(), v, &err));
    CHECK_NEAR(v.getNumber("total", 0), 1.0, 0);
    REQUIRE(v.get("clips") && v.get("clips")->array.size() == 1);
    CHECK_EQ(v.get("clips")->array[0].getString("event", ""), std::string("manual"));
    h.engine.clipsJson(1, 20, json);
    REQUIRE(parseJson(json.data(), json.size(), v, &err));
    CHECK(v.get("clips")->array.empty());
}
