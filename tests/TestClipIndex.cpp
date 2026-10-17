#include <string>

#include "ClipIndex.h"
#include "TestHarness.h"

using namespace br;

namespace {

ClipEntry entry(const std::string& file, EventType ev, int64_t utc, bool keep = false) {
    ClipEntry e;
    e.file = file;
    e.event = ev;
    e.utcStartMs = utc;
    e.autoClip = ev != EventType::Manual;
    e.keep = keep;
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

TEST(ClipIndex_RetentionSparesKeptAndManualClips) {
    ClipIndex idx;
    for (int i = 0; i < 60; ++i) {
        const bool keep = (i == 3 || i == 10);  // two old kept clips
        idx.add(entry("auto" + std::to_string(i) + ".rpl", i % 2 ? EventType::Landing : EventType::Takeoff, i, keep));
    }
    idx.add(entry("manual_old.rpl", EventType::Manual, -5));
    const std::vector<std::string> removed = idx.enforceRetention(50);
    // 58 non-kept auto clips; the newest 50 of those are auto59..auto11 and auto9 (auto10 is
    // kept, so it doesn't count). auto0..auto8 go, except the kept auto3.
    CHECK_EQ(removed.size(), 8u);
    CHECK(idx.find("auto3.rpl") != nullptr);
    CHECK(idx.find("auto10.rpl") != nullptr);
    CHECK(idx.find("manual_old.rpl") != nullptr);
    CHECK(idx.find("auto0.rpl") == nullptr);
    CHECK(idx.find("auto8.rpl") == nullptr);
    CHECK(idx.find("auto9.rpl") != nullptr);
    CHECK(idx.find("auto11.rpl") != nullptr);
    CHECK(idx.find("auto59.rpl") != nullptr);
    CHECK_EQ(idx.size(), 53u);  // 60 + 1 manual - 8
    CHECK(idx.enforceRetention(50).empty());
}

TEST(ClipIndex_JsonRoundTrip) {
    ClipIndex idx;
    ClipEntry e = entry("2026-10-17_1432_landing.rpl", EventType::Landing, 1792247525000, true);
    e.aircraft = "Airbus A320neo \"Asobo\"";
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
    CHECK_NEAR(l->durationSec, 154.25, 1e-9);
    CHECK_NEAR(l->touchdownVsFpm, -142.5, 1e-9);
    CHECK_NEAR(l->peakG, 1.31, 1e-9);
    CHECK_EQ(l->bounces, 1);
    CHECK(l->keep);
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
