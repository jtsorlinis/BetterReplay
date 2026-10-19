#include <string>

#include "ClipIndex.h"
#include "TestHarness.h"

using namespace br;

namespace {

ClipEntry entry(const std::string& file, EventType ev, int64_t utc) {
    ClipEntry e;
    e.file = file;
    e.event = ev;
    e.utcStartMs = utc;
    e.autoClip = ev != EventType::Manual;
    return e;
}

}  // namespace

TEST(ClipIndex_KeepsNewestFirst) {
    ClipIndex idx;
    idx.add(entry("b.rpl", EventType::Landing, 200));
    idx.add(entry("a.rpl", EventType::Takeoff, 100));
    idx.add(entry("c.rpl", EventType::Manual, 300));
    REQUIRE(idx.size() == 3);
    CHECK_EQ(idx.clips()[0].file, std::string("c.rpl"));
    CHECK_EQ(idx.clips()[2].file, std::string("a.rpl"));
    idx.add(entry("a.rpl", EventType::Takeoff, 400));  // re-adding replaces
    CHECK_EQ(idx.size(), 3u);
    CHECK_EQ(idx.clips()[0].file, std::string("a.rpl"));
}

TEST(ClipIndex_OrderIsByWhenClipsEnd) {
    // Save last 5 minutes right after a takeoff clip: the save starts earlier but ends later.
    ClipIndex idx;
    ClipEntry takeoff = entry("takeoff.rpl", EventType::Takeoff, 1000000 + 260000);
    takeoff.durationSec = 39.2;
    ClipEntry manual = entry("manual.rpl", EventType::Manual, 1000000);
    manual.durationSec = 317.3;
    idx.add(takeoff);
    idx.add(manual);
    CHECK_EQ(idx.clips()[0].file, std::string("manual.rpl"));

    // and after a reload
    std::string json;
    idx.toJson(json);
    ClipIndex loaded;
    REQUIRE(loaded.fromJson(json.data(), json.size(), nullptr));
    CHECK_EQ(loaded.clips()[0].file, std::string("manual.rpl"));
}

TEST(ClipIndex_UniqueFileNames) {
    ClipIndex idx;
    const int64_t t = 1792247525000;  // 2026-10-17 14:32:05 UTC
    const std::string first = idx.uniqueFileName(t, EventType::Landing);
    CHECK_EQ(first, std::string("2026-10-17_1432_landing.rpl"));
    idx.add(entry(first, EventType::Landing, t));
    CHECK_EQ(idx.uniqueFileName(t + 20000, EventType::Landing), std::string("2026-10-17_1432_landing_2.rpl"));
    idx.add(entry("2026-10-17_1432_landing_2.rpl", EventType::Landing, t));
    CHECK_EQ(idx.uniqueFileName(t, EventType::Landing), std::string("2026-10-17_1432_landing_3.rpl"));
    CHECK_EQ(idx.uniqueFileName(t, EventType::Takeoff), std::string("2026-10-17_1432_takeoff.rpl"));
}

TEST(ClipIndex_RetentionKeepsNewestAutoAndAllManualClips) {
    ClipIndex idx;
    for (int i = 0; i < 30; ++i) {
        idx.add(entry("auto" + std::to_string(i) + ".rpl", i % 2 ? EventType::Landing : EventType::Takeoff, i));
    }
    idx.add(entry("manual_old.rpl", EventType::Manual, -5));
    const std::vector<std::string> removed = idx.enforceRetention(20);
    // The newest 20 auto clips are auto29..auto10; auto0..auto9 go. Manual clips always stay.
    CHECK_EQ(removed.size(), 10u);
    CHECK(idx.find("manual_old.rpl") != nullptr);
    CHECK(idx.find("auto0.rpl") == nullptr);
    CHECK(idx.find("auto9.rpl") == nullptr);
    CHECK(idx.find("auto10.rpl") != nullptr);
    CHECK(idx.find("auto29.rpl") != nullptr);
    CHECK_EQ(idx.size(), 21u);  // 20 auto + 1 manual
    CHECK(idx.enforceRetention(20).empty());
}

TEST(ClipIndex_OldKeepFlagIsIgnored) {
    // index.json from before Keep was removed: the flag no longer protects a clip.
    const std::string json =
        "{\"clips\":[{\"file\":\"new.rpl\",\"event\":\"landing\",\"utcMs\":2,\"auto\":true},"
        "{\"file\":\"old.rpl\",\"event\":\"takeoff\",\"utcMs\":1,\"keep\":true,\"auto\":true}]}";
    ClipIndex idx;
    REQUIRE(idx.fromJson(json.data(), json.size(), nullptr));
    const std::vector<std::string> removed = idx.enforceRetention(1);
    REQUIRE(removed.size() == 1);
    CHECK_EQ(removed[0], std::string("old.rpl"));
    std::string out;
    idx.toJson(out);
    CHECK(out.find("keep") == std::string::npos);
}

TEST(ClipIndex_OldIndexHasNoAircraftPaths) {
    // Version 1 (or none): the entries' paths are yet to be read from the clips.
    const std::string v1 = "{\"version\":1,\"clips\":[{\"file\":\"a.rpl\",\"event\":\"landing\",\"utcMs\":2}]}";
    ClipIndex idx;
    REQUIRE(idx.fromJson(v1.data(), v1.size(), nullptr));
    CHECK(!idx.pathsKnown());
    CHECK(idx.find("a.rpl")->aircraftPath.empty());
    idx.setPathsKnown();
    std::string out;
    idx.toJson(out);
    CHECK(out.find("\"version\":2") != std::string::npos);
    ClipIndex back;
    REQUIRE(back.fromJson(out.data(), out.size(), nullptr));
    CHECK(back.pathsKnown());

    const std::string unversioned = "{\"clips\":[]}";
    REQUIRE(back.fromJson(unversioned.data(), unversioned.size(), nullptr));
    CHECK(!back.pathsKnown());
    back.clear();  // a new library
    CHECK(back.pathsKnown());
}

TEST(ClipIndex_JsonRoundTrip) {
    ClipIndex idx;
    ClipEntry e = entry("2026-10-17_1432_landing.rpl", EventType::Landing, 1792247525000);
    e.aircraft = "Airbus A320neo \"Asobo\"";
    e.aircraftPath = "simobjects/airplanes/asobo_a320_neo";
    e.airport = "YMML";
    e.durationSec = 154.25;
    e.touchdownVsFpm = -142.5;
    e.peakG = 1.31;
    e.bounces = 1;
    idx.add(e);
    idx.add(entry("2026-10-17_1300_manual.rpl", EventType::Manual, 1792242000000));
    std::string json;
    idx.toJson(json);

    ClipIndex back;
    std::string err;
    REQUIRE(back.fromJson(json.data(), json.size(), &err));
    REQUIRE(back.size() == 2);
    const ClipEntry* l = back.find("2026-10-17_1432_landing.rpl");
    REQUIRE(l != nullptr);
    CHECK(l->event == EventType::Landing);
    CHECK_EQ(l->utcStartMs, static_cast<int64_t>(1792247525000));
    CHECK_EQ(l->aircraft, e.aircraft);
    CHECK_EQ(l->aircraftPath, e.aircraftPath);
    CHECK(back.pathsKnown());
    CHECK_EQ(l->airport, std::string("YMML"));
    CHECK_NEAR(l->durationSec, 154.25, 1e-9);
    CHECK_NEAR(l->touchdownVsFpm, -142.5, 1e-9);
    CHECK_NEAR(l->peakG, 1.31, 1e-9);
    CHECK_EQ(l->bounces, 1);
    CHECK(l->autoClip);
    const ClipEntry* m = back.find("2026-10-17_1300_manual.rpl");
    REQUIRE(m != nullptr);
    CHECK(!m->autoClip);
    CHECK(std::isnan(m->touchdownVsFpm));
    CHECK(json.find("\"utc\":\"2026-10-17T14:32:05Z\"") != std::string::npos);
}

TEST(ClipIndex_FromJsonDropsUnsafeNames) {
    const std::string json =
        "{\"clips\":[{\"file\":\"..\\\\evil.rpl\"},{\"file\":\"a/b.rpl\"},{\"file\":\"ok.rpl\"},{\"file\":\"x.txt\"}]}";
    ClipIndex idx;
    REQUIRE(idx.fromJson(json.data(), json.size(), nullptr));
    CHECK_EQ(idx.size(), 1u);
    CHECK(idx.find("ok.rpl") != nullptr);
}

TEST(ClipIndex_SafeNames) {
    CHECK(isSafeClipFileName("2026-10-17_1432_landing_2.rpl"));
    CHECK(!isSafeClipFileName("index.json"));
    CHECK(!isSafeClipFileName("../x.rpl"));
    CHECK(!isSafeClipFileName("a\\b.rpl"));
    CHECK(!isSafeClipFileName(".rpl"));
}
