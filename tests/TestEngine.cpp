#include <cmath>
#include <cstdio>
#include <cstdlib>
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
    std::map<std::string, std::vector<uint8_t>> package;  // read-only
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
    bool readPackageFile(const std::string& name, std::vector<uint8_t>& out) override {
        const auto it = package.find(name);
        if (it == package.end()) return false;
        out = it->second;
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
    // What the detector reads them from.
    for (int e = 0; e < 2; ++e) in.raw.v[kEngineN1_1 + e] = std::isfinite(s.enginePower) ? s.enginePower : 0.0;
    in.facts.onRunway = s.onRunway;
    in.raw.v[kFlapsHandleIndex] = s.flapsRetracted ? 0.0 : 2.0;
    in.raw.v[kFlapsLeft] = in.raw.v[kFlapsRight] = s.flapsRetracted ? 0.0 : 0.4;
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

// Approach from 1500 ft and land, rolling to a stop; returns the touchdown sample index.
size_t landingTrace(TraceGen& g) {
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(30.0, 0.0, 0.0, true);
    g.hold(10.0, true);
    return td;
}

constexpr const char* kPmdg800 = "simobjects/airplanes/pmdg 737-800/presets/pmdg/pmdg 737-800 bw tc/config";
constexpr const char* kPmdg700 = "simobjects/airplanes/pmdg 737-700/presets/pmdg/pmdg 737-700 bw/config";

}  // namespace

TEST(Engine_FullFlightSavesTakeoffAndLanding) {
    TraceGen g(true, 0.0, 0.0);
    g.dt = 1.0 / 60.0;  // the sim runs faster than the recording rate
    g.power = 20.0;
    g.hold(30.0, true);
    const double spool = g.now();
    g.spool(4.0, 90.0, true);
    g.segment(30.0, 70.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(40.0, 90.0, 1200.0, false);
    g.segment(60.0, 140.0, 3000.0, false);
    g.hold(60.0, false);
    g.segment(120.0, 120.0, 1500.0, false);
    const size_t td = landingTrace(g);

    Harness h;
    h.engine.setAircraftKey(kPmdg800);
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
    CHECK_NEAR(landings[0]->durationSec, 180.0 + 29.75, 0.1);  // 115 -> 0 kt is below 1 kt after 29.75 s

    // Set BR_SAMPLE_OUT=<path> to keep this clip on disk (handy for tools/rpl-dump.mjs).
    if (const char* out = std::getenv("BR_SAMPLE_OUT")) {
        const std::vector<uint8_t>& bytes = h.storage.files[landings[0]->file];
        if (FILE* f = std::fopen(out, "wb")) {
            std::fwrite(bytes.data(), 1, bytes.size(), f);
            std::fclose(f);
        }
    }

    Clip clip;
    REQUIRE(h.load(landings[0]->file, clip));
    CHECK(clip.meta.event == EventType::Landing);
    CHECK_NEAR(clip.meta.eventOffsetSec, 180.0, 0.1);
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
    CHECK_NEAR(static_cast<double>(clip.meta.utcStartMs), static_cast<double>(tdUtc - 180000), 100.0);

    CHECK_EQ(clip.meta.aircraftPath, std::string(kPmdg800));

    // The takeoff: from 5 s before the engines spooled up (N1) to 2 minutes after liftoff.
    Clip to;
    REQUIRE(h.load(takeoffs[0]->file, to));
    CHECK(to.meta.event == EventType::Takeoff);
    CHECK(std::isnan(to.meta.touchdownVsFpm));
    CHECK(to.meta.eventOffsetSec >= liftoff - (spool - 5.0) - 0.3);  // seen a moment into the spool-up
    CHECK(to.meta.eventOffsetSec <= liftoff - (spool - 5.0) + 0.05);
    CHECK_NEAR(takeoffs[0]->durationSec, to.meta.eventOffsetSec + 120.0, 0.1);
}

TEST(Engine_TakeoffsAndLandingsRecordTheirAirport) {
    // Lifts off near YSSY and lands 30 km further south.
    TraceGen g(true, 0.0, 0.0);
    g.power = 20.0;
    g.hold(30.0, true);
    g.spool(4.0, 90.0, true);
    g.segment(30.0, 150.0, 0.0, true);
    const size_t liftoff = g.samples.size();
    g.segment(100.0, 220.0, 3000.0, false);
    g.segment(120.0, 120.0, 1500.0, false);
    const size_t td = landingTrace(g);

    Harness h;
    const DetectorSample& up = g.samples[liftoff];
    const DetectorSample& down = g.samples[td];
    h.engine.addAirport("YSSY", "YM", up.latDeg + 0.01, up.lonDeg);         // 1.1 km
    h.engine.addAirport("34NS", "YM", up.latDeg, up.lonDeg + 0.005);        // a heliport 460 m away
    h.engine.addAirport("YSBK", "YM", up.latDeg + 0.03, up.lonDeg - 0.15);  // Bankstown, 14 km
    h.engine.addAirport("YWOL", "YM", down.latDeg - 0.02, down.lonDeg);     // 2.2 km
    h.engine.addAirport("YWOL", "YM", down.latDeg - 0.02, down.lonDeg);     // reported again
    CHECK_EQ(h.engine.airports().size(), 4u);
    h.feedAll(g.samples);
    const auto takeoffs = h.clipsOf(EventType::Takeoff);
    const auto landings = h.clipsOf(EventType::Landing);
    REQUIRE(takeoffs.size() == 1);
    REQUIRE(landings.size() == 1);
    CHECK_EQ(takeoffs[0]->airport, std::string("YSSY"));
    CHECK_EQ(landings[0]->airport, std::string("YWOL"));
    Clip clip;
    REQUIRE(h.load(landings[0]->file, clip));
    CHECK_EQ(clip.meta.airport, std::string("YWOL"));

    // Manual clips have none; a new flight forgets the airports.
    std::string err;
    REQUIRE(h.engine.saveLast(kUtcBase + 900000, err));
    const auto manual = h.clipsOf(EventType::Manual);
    REQUIRE(manual.size() == 1);
    CHECK(manual[0]->airport.empty());
    h.engine.onFlightLoaded("next flight");
    CHECK_EQ(h.engine.airports().size(), 0u);
}

TEST(Airports_NearestWithinReachPreferringIcaoIdents) {
    AirportList a;
    CHECK(a.at(-37.67, 144.84).empty());
    a.add("YMML", "YM", -37.6733, 144.8433);
    a.add("YMEN", "YM", -37.7281, 144.9019);  // Essendon, 8 km from YMML
    a.add("0S9", "K1", -37.6800, 144.8433);   // a strip 750 m from YMML
    CHECK_EQ(a.at(-37.6733, 144.8433), std::string("YMML"));
    CHECK_EQ(a.at(-37.6900, 144.8433), std::string("YMML"));  // 0S9 is closer, by less than its penalty
    CHECK_EQ(a.at(-37.7250, 144.8950), std::string("YMEN"));
    CHECK(a.at(-37.9, 144.8433).empty());  // 25 km from YMML
    a.add("", "YM", -37.0, 144.0);
    a.add("YNAN", "YM", NAN, 144.0);
    CHECK_EQ(a.size(), 3u);
    // Across the antimeridian.
    a.add("NFFN", "NF", -17.7554, 177.4434);
    a.add("XDAT", "NF", -16.0, 179.99);
    CHECK_EQ(a.at(-16.0, -179.99), std::string("XDAT"));
    a.clear();
    CHECK_EQ(a.size(), 0u);
}

TEST(Engine_LandingEndsTenSecondsOffTheRunway) {
    TraceGen g(false, 1500.0, 120.0);
    g.flapsUp = false;
    g.hold(120.0, false);  // the buffer holds the whole pre-roll
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(25.0, 15.0, 0.0, true);
    g.onRunway = false;
    const double exit = g.now() + g.dt;
    g.hold(20.0, true);

    Harness h;
    h.feedAll(g.samples);
    const auto landings = h.clipsOf(EventType::Landing);
    REQUIRE(landings.size() == 1);
    CHECK_NEAR(landings[0]->durationSec, 180.0 + (exit + 10.0 - g.samples[td].t), 0.1);
}

TEST(Engine_TakeoffFromTheRunwayEntryUntilClean) {
    // ON ANY RUNWAY and the flap readings, as the module passes them.
    TraceGen g(true, 0.0, 0.0);
    g.onRunway = false;
    g.flapsUp = false;
    g.power = 20.0;
    g.hold(40.0, true);
    g.segment(5.0, 10.0, 0.0, true);
    const double entry = g.now() + g.dt;
    g.onRunway = true;
    g.segment(5.0, 0.0, 0.0, true);
    g.spool(4.0, 95.0, true);
    g.segment(30.0, 150.0, 0.0, true);
    const double liftoff = g.now() + g.dt;
    g.segment(100.0, 220.0, 3000.0, false);
    g.flapsUp = true;
    const double clean = g.now() + g.dt;
    g.segment(30.0, 250.0, 4000.0, false);

    Harness h;
    h.feedAll(g.samples);
    const auto takeoffs = h.clipsOf(EventType::Takeoff);
    REQUIRE(takeoffs.size() == 1);
    Clip to;
    REQUIRE(h.load(takeoffs[0]->file, to));
    CHECK_NEAR(to.meta.eventOffsetSec, liftoff - (entry - 30.0), 0.1);
    CHECK_NEAR(takeoffs[0]->durationSec, clean + 10.0 - (entry - 30.0), 0.1);
}

TEST(Engine_LandingInProgressIsSavedWhenTheFlightEnds) {
    TraceGen g(false, 1500.0, 120.0);
    g.hold(120.0, false);  // the buffer holds the whole pre-roll
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(15.0, 50.0, 0.0, true);  // still rolling
    Harness h;
    h.feedAll(g.samples);
    CHECK(h.clipsOf(EventType::Landing).empty());
    h.engine.setAtMainMenu(true);
    const auto landings = h.clipsOf(EventType::Landing);
    REQUIRE(landings.size() == 1);
    CHECK_NEAR(landings[0]->durationSec, 180.0 + (g.now() - g.samples[td].t), 0.1);
    h.engine.onFlightLoaded("next flight");  // nothing left to save
    CHECK_EQ(h.clipsOf(EventType::Landing).size(), 1u);
}

TEST(Engine_VariantsShareOneControlProfile) {
    // aircraft.json from before: kept under the 737-800 preset's own path.
    MemoryStorage storage;
    {
        ProfileStore old;
        AircraftProfile& p = old.upsert(kPmdg800);
        p.title = "737-800 PAX BW TC";
        p[Control::Flaps].state = ControlState::Ok;
        p[Control::Flaps].method.kind = MethodKind::KeyEvent;
        p[Control::Flaps].method.var = kKeyFlapsNotches;
        std::string json;
        old.toJson(json);
        storage.files["aircraft.json"].assign(json.begin(), json.end());
    }
    Engine e(storage);
    e.setLogger(&testLog, nullptr);
    e.init();
    e.setAircraftKey(kPmdg700);
    CHECK_EQ(e.aircraftKey(), std::string("simobjects/airplanes/pmdg 737"));
    const AircraftProfile* p = e.currentProfile();
    REQUIRE(p != nullptr);
    CHECK((*p)[Control::Flaps].method.var == std::string(kKeyFlapsNotches));
    // Saved back under the family.
    ProfileStore saved;
    const std::vector<uint8_t>& bytes = storage.files["aircraft.json"];
    std::string err;
    REQUIRE(saved.fromJson(reinterpret_cast<const char*>(bytes.data()), bytes.size(), &err));
    REQUIRE(saved.size() == 1);
    CHECK_EQ(saved.profiles()[0].key, std::string("simobjects/airplanes/pmdg 737"));
}

// A profile with the flaps on key events `var`, and two ways to put the gear down.
static void writeProfiles(std::vector<uint8_t>& file, const char* key, const char* flapsVar) {
    ProfileStore store;
    AircraftProfile& p = store.upsert(key);
    p.title = "737";
    p[Control::Flaps].state = ControlState::Ok;
    p[Control::Flaps].method.kind = MethodKind::KeyEvent;
    p[Control::Flaps].method.var = flapsVar;
    ControlProfile& gear = p[Control::Gear];
    gear.state = ControlState::Ok;
    gear.method.kind = MethodKind::KeyEvent;
    gear.method.var = "#69927";
    gear.ways[static_cast<int>(Direction::Extend)] = {gear.method, ControlMethod{MethodKind::KeyEvent, "#69928", 1.0}};
    gear.ways[static_cast<int>(Direction::Retract)] = {gear.method};
    std::string json;
    store.toJson(json);
    file.assign(json.begin(), json.end());
}

static bool statusIncluded(const Engine& e) {
    std::string json;
    e.statusJson(json);
    CHECK(json.find("\"included\":") != std::string::npos);
    return json.find("\"included\":true") != std::string::npos;
}

TEST(Engine_IncludedProfileWhereTheAircraftHasNone) {
    MemoryStorage storage;
    writeProfiles(storage.package["profiles/aircraft.json"], "simobjects/airplanes/pmdg 737", kKeyFlapsAxis);
    Engine e(storage);
    e.setLogger(&testLog, nullptr);
    e.init();
    CHECK_EQ(e.includedProfiles().size(), static_cast<size_t>(1));
    e.setAircraftKey(kPmdg800);
    const AircraftProfile* p = e.currentProfile();
    REQUIRE(p != nullptr);
    CHECK(e.profileIncluded());
    CHECK(statusIncluded(e));
    CHECK((*p)[Control::Flaps].method.var == std::string(kKeyFlapsAxis));
    CHECK(storage.files.count("aircraft.json") == 0);  // nothing of the pilot's to keep

    // A way that worked in a replay goes first for this session, without making it the pilot's.
    const ControlMethod second{MethodKind::KeyEvent, "#69928", 1.0};
    CHECK(e.promoteWay(Control::Gear, Direction::Extend, second));
    CHECK(e.currentProfile()->controls[static_cast<int>(Control::Gear)].waysTo(Direction::Extend)[0] == second);
    CHECK(e.profileIncluded());
    CHECK(storage.files.count("aircraft.json") == 0);

    // So does a surface its fly-by-wire keeps, steered by the stick.
    CHECK(e.learnStickSurface(kStickElevator));
    CHECK_EQ(e.currentProfile()->stickSurfaces, static_cast<unsigned>(kStickElevator));
    CHECK(storage.files.count("aircraft.json") == 0);

    // Another aircraft has none, so there's nowhere to keep one.
    e.setAircraftKey("simobjects/airplanes/a350/presets/airbus/config");
    CHECK(e.currentProfile() == nullptr);
    CHECK(!e.profileIncluded());
    CHECK(!statusIncluded(e));
    CHECK(!e.learnStickSurface(kStickElevator));
    CHECK(storage.files.count("aircraft.json") == 0);
}

TEST(Engine_StickSurfacesAreKeptInTheAircraftsProfile) {
    MemoryStorage storage;
    writeProfiles(storage.files["aircraft.json"], "simobjects/airplanes/pmdg 737", kKeyFlapsAxis);
    Engine e(storage);
    e.setLogger(&testLog, nullptr);
    e.init();
    e.setAircraftKey(kPmdg800);
    REQUIRE(e.currentProfile() != nullptr);
    CHECK(!e.profileIncluded());
    CHECK_EQ(e.currentProfile()->stickSurfaces, 0u);
    CHECK(e.learnStickSurface(kStickRudder));
    CHECK(e.learnStickSurface(kStickElevator));
    CHECK(!e.learnStickSurface(kStickElevator));  // kept already
    CHECK_EQ(e.currentProfile()->stickSurfaces, static_cast<unsigned>(kStickElevator | kStickRudder));

    Engine again(storage);
    again.init();
    again.setAircraftKey(kPmdg800);
    REQUIRE(again.currentProfile() != nullptr);
    CHECK_EQ(again.currentProfile()->stickSurfaces, static_cast<unsigned>(kStickElevator | kStickRudder));
}

TEST(Engine_OwnProfileComesBeforeTheIncludedOne) {
    MemoryStorage storage;
    writeProfiles(storage.files["aircraft.json"], kPmdg800, kKeyFlapsNotches);  // (moved to the family)
    writeProfiles(storage.package["profiles/aircraft.json"], "simobjects/airplanes/pmdg 737", kKeyFlapsAxis);
    Engine e(storage);
    e.setLogger(&testLog, nullptr);
    e.init();
    e.setAircraftKey(kPmdg700);
    REQUIRE(e.currentProfile() != nullptr);
    CHECK(!e.profileIncluded());
    CHECK(!statusIncluded(e));
    CHECK((*e.currentProfile())[Control::Flaps].method.var == std::string(kKeyFlapsNotches));

    // Reset: the included one again.
    std::string err;
    REQUIRE(e.setupReset(err));
    REQUIRE(e.currentProfile() != nullptr);
    CHECK(e.profileIncluded());
    CHECK((*e.currentProfile())[Control::Flaps].method.var == std::string(kKeyFlapsAxis));
}

TEST(Engine_NeverOnAnIncludedProfileKeepsItsControls) {
    MemoryStorage storage;
    writeProfiles(storage.package["profiles/aircraft.json"], "simobjects/airplanes/pmdg 737", kKeyFlapsAxis);
    Engine e(storage);
    e.setLogger(&testLog, nullptr);
    e.init();
    e.setAircraftKey(kPmdg800);
    std::string err;
    REQUIRE(e.setupNever(err));
    CHECK(!e.profileIncluded());
    REQUIRE(e.currentProfile() != nullptr);
    CHECK(e.currentProfile()->declined);
    CHECK((*e.currentProfile())[Control::Flaps].method.var == std::string(kKeyFlapsAxis));
    // Kept in aircraft.json.
    ProfileStore saved;
    const std::vector<uint8_t>& bytes = storage.files["aircraft.json"];
    REQUIRE(saved.fromJson(reinterpret_cast<const char*>(bytes.data()), bytes.size(), &err));
    REQUIRE(saved.size() == 1);
    CHECK(saved.profiles()[0].declined);
    CHECK(saved.profiles()[0][Control::Flaps].method.var == std::string(kKeyFlapsAxis));
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

TEST(Engine_SlewOnTheRolloutSavesTheLandingUpToIt) {
    TraceGen g(false, 1500.0, 120.0);
    g.hold(120.0, false);  // the buffer holds the whole pre-roll
    const size_t td = approachAndTouchdown(g, 115.0);
    g.segment(10.0, 60.0, 0.0, true);
    const size_t slewFrom = g.index();
    g.hold(5.0, true);
    Harness h;
    h.feed(g.samples, 0, slewFrom);
    for (size_t i = slewFrom; i < g.samples.size(); ++i) {
        FrameInput in = toInput(g.samples[i]);
        in.slewActive = true;
        h.engine.onFrame(in, kUtcBase);
    }
    CHECK(h.clipsOf(EventType::Landing).empty());
    h.feed(g.samples, g.samples.size() - 1, g.samples.size());  // out of slew
    const auto landings = h.clipsOf(EventType::Landing);
    REQUIRE(landings.size() == 1);
    CHECK_NEAR(landings[0]->durationSec, 180.0 + (g.samples[slewFrom - 1].t - g.samples[td].t), 0.1);
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

TEST(Engine_RetentionKeepsNewestAutoClipsOfEachAircraft) {
    EngineConfig cfg;
    cfg.maxAutoClipsPerAircraft = 2;
    Harness h(cfg);
    std::vector<std::string> saved;
    for (int flight = 0; flight < 5; ++flight) {
        h.utcOffsetMs = flight * 3600000LL;
        if (flight == 4) h.engine.setAircraftTitle("Other Aircraft");
        h.engine.onFlightLoaded("test flight");
        TraceGen g(false, 1500.0, 120.0);
        landingTrace(g);
        // Each new flight restarts sim time from 0.
        h.feedAll(g.samples);
        saved.push_back(h.engine.index().clips()[0].file);
    }
    // Only the first aircraft's newest two survive, in the index and on disk; the other
    // aircraft's clip doesn't push them out.
    CHECK_EQ(h.engine.index().size(), 3u);
    CHECK(h.engine.index().find(saved[0]) == nullptr);
    CHECK(h.engine.index().find(saved[1]) == nullptr);
    CHECK(h.storage.files.count(saved[0]) == 0);
    CHECK(h.storage.files.count(saved[1]) == 0);
    CHECK(h.engine.index().find(saved[2]) != nullptr);
    CHECK(h.engine.index().find(saved[3]) != nullptr);
    CHECK(h.engine.index().find(saved[4]) != nullptr);
    CHECK_EQ(h.storage.clipCount(), 3u);

    // A lower limit applies to the existing library at startup.
    EngineConfig lower;
    lower.maxAutoClipsPerAircraft = 1;
    Engine restarted(h.storage, lower);
    restarted.init();
    CHECK_EQ(restarted.index().size(), 2u);
    CHECK(restarted.index().find(saved[3]) != nullptr);
    CHECK(restarted.index().find(saved[4]) != nullptr);
    CHECK_EQ(h.storage.clipCount(), 2u);
}

TEST(Engine_DefaultRetentionIsTenAutoClipsPerAircraft) { CHECK_EQ(EngineConfig().maxAutoClipsPerAircraft, 10u); }

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
    }
    {
        Engine e(storage);
        e.init();
        REQUIRE(e.index().size() == 1);
        CHECK(e.index().clips()[0].event == EventType::Manual);
    }
    // Lose the index: the clip is adopted back from its header.
    storage.files.erase("index.json");
    {
        Engine e(storage);
        e.init();
        REQUIRE(e.index().size() == 1);
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

TEST(Engine_DeleteValidatesNames) {
    Harness h;
    TraceGen g(false, 3000.0, 150.0);
    g.hold(5.0, false);
    h.feedAll(g.samples);
    std::string err;
    REQUIRE(h.engine.saveLast(kUtcBase, err));
    const std::string file = h.engine.index().clips()[0].file;
    CHECK(!h.engine.deleteClip("../index.json", err));
    CHECK(!h.engine.deleteClip("nope.rpl", err));
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

TEST(Engine_BufferHoldsAboutTenMinutes) {
    Harness h;
    TraceGen g(false, 30000.0, 450.0);
    g.hold(1200.0, false);  // 20 minutes
    h.feedAll(g.samples);
    CHECK(h.engine.bufferedSeconds() >= 600.0);
    CHECK(h.engine.bufferedSeconds() <= 640.0);
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
    // The aircraft goes by its title until its path is known, and the clip was recorded in it.
    CHECK(!v.getBool("aircraftKnown", true));
    CHECK(v.get("clips")->array[0].getBool("playable", false));
    CHECK(v.get("clips")->array[0].get("aircraftPath") == nullptr);  // index.json's, not the panel's
    h.engine.clipsJson(1, 20, json);
    REQUIRE(parseJson(json.data(), json.size(), v, &err));
    CHECK(v.get("clips")->array.empty());
}

// How a surface answers the stick, as a replay fitted it: kept in the aircraft's own profile and
// there after a restart. The package's profile stays at hand beside the aircraft's own, for its
// figures where the own one has none.
TEST(Engine_StickPlantsAreKeptInTheAircraftsProfile) {
    MemoryStorage storage;
    writeProfiles(storage.files["aircraft.json"], "simobjects/airplanes/pmdg 737", kKeyFlapsAxis);
    writeProfiles(storage.package["profiles/aircraft.json"], "simobjects/airplanes/pmdg 737", kKeyFlapsNotches);
    Engine e(storage);
    e.setLogger(&testLog, nullptr);
    e.init();
    e.setAircraftKey(kPmdg800);
    REQUIRE(e.currentProfile() != nullptr);
    CHECK(!e.profileIncluded());
    REQUIRE(e.includedProfile() != nullptr);
    CHECK((*e.includedProfile())[Control::Flaps].method.var == std::string(kKeyFlapsNotches));
    StickPlant p;
    p.form = StickPlant::Form::Direct;
    p.gain = 14.0;
    p.lagSec = 0.06;
    p.fit = 0.7;
    p.seconds = 40.0;
    CHECK(!e.learnStickPlant(1, StickPlant()));  // nothing to keep
    CHECK(e.learnStickPlant(1, p));
    CHECK(e.currentProfile()->stickPlants[1].valid());
    CHECK(!e.currentProfile()->stickPlants[0].valid());

    Engine again(storage);
    again.init();
    again.setAircraftKey(kPmdg800);
    REQUIRE(again.currentProfile() != nullptr);
    const StickPlant& back = again.currentProfile()->stickPlants[1];
    CHECK(back.form == StickPlant::Form::Direct);
    CHECK_NEAR(back.gain, 14.0, 1e-9);
    CHECK_NEAR(back.lagSec, 0.06, 1e-9);
}
