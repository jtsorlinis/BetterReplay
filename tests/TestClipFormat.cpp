#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "ByteIO.h"
#include "ClipFormat.h"
#include "TestHarness.h"

using namespace br;

namespace {

std::vector<Frame> makeFrames(size_t n, double t0) {
    std::vector<Frame> frames(n);
    for (size_t i = 0; i < n; ++i) {
        Frame& f = frames[i];
        f.t = t0 + i / 30.0;
        for (int c = 0; c < kChannelCount; ++c) f.v[c] = 0.25 * c + 0.001 * static_cast<double>(i);
        f.v[kLat] = -33.946111 + 1e-7 * static_cast<double>(i);  // f64 channels keep full precision
        f.v[kLon] = 151.177222;
        f.v[kOnGround] = i % 2 ? 1.0 : 0.0;
        f.v[kLightNav] = 1.0;
        f.v[kLightTaxi] = NAN;  // u8 channels encode NaN distinctly
    }
    return frames;
}

ClipMeta makeMeta() {
    ClipMeta m;
    m.event = EventType::Landing;
    m.utcStartMs = 1792247525000;
    m.sampleRateHz = 30.0f;
    m.eventOffsetSec = 120.0f;
    m.touchdownVsFpm = -142.5f;
    m.peakG = 1.31f;
    m.bounces = 2;
    m.aircraft = "Cessna 172 Skyhawk \xC3\xA9";
    return m;
}

// Splices an extra chunk into a serialized clip right after the 8-byte file header.
std::vector<uint8_t> withExtraChunk(const std::vector<uint8_t>& file, const char id[4], size_t payload) {
    std::vector<uint8_t> chunk;
    ByteWriter w(chunk);
    const size_t pos = w.beginChunk(id);
    for (size_t i = 0; i < payload; ++i) w.u8(static_cast<uint8_t>(i));
    w.endChunk(pos);
    std::vector<uint8_t> out(file.begin(), file.begin() + 8);
    out.insert(out.end(), chunk.begin(), chunk.end());
    out.insert(out.end(), file.begin() + 8, file.end());
    return out;
}

}  // namespace

TEST(ClipFormat_RoundTripPreservesMetaAndSamples) {
    const std::vector<Frame> frames = makeFrames(300, 1000.0);
    std::vector<uint8_t> bytes;
    writeClip(makeMeta(), VectorFrameSource(frames), bytes);

    Clip clip;
    std::string err;
    REQUIRE(readClip(bytes.data(), bytes.size(), clip, &err));
    CHECK(err.empty());
    CHECK(clip.meta.event == EventType::Landing);
    CHECK_EQ(clip.meta.utcStartMs, static_cast<int64_t>(1792247525000));
    CHECK_NEAR(clip.meta.sampleRateHz, 30.0, 0);
    CHECK_EQ(clip.meta.frames, 300u);
    CHECK_NEAR(clip.meta.eventOffsetSec, 120.0, 0);
    CHECK_NEAR(clip.meta.touchdownVsFpm, -142.5, 1e-4);
    CHECK_NEAR(clip.meta.peakG, 1.31, 1e-6);
    CHECK_EQ(clip.meta.bounces, 2);
    CHECK_EQ(clip.meta.aircraft, makeMeta().aircraft);
    CHECK_EQ(clip.present.count(), static_cast<size_t>(kChannelCount));

    REQUIRE(clip.frames.size() == frames.size());
    for (size_t i = 0; i < frames.size(); i += 37) {
        const Frame& a = frames[i];
        const Frame& b = clip.frames[i];
        CHECK_NEAR(b.t, a.t - 1000.0, 1e-4);  // times become clip-relative
        CHECK_NEAR(b.v[kLat], a.v[kLat], 1e-12);
        CHECK_NEAR(b.v[kLon], a.v[kLon], 1e-12);
        CHECK_NEAR(b.v[kPitch], a.v[kPitch], 1e-5);
        CHECK_NEAR(b.v[kOnGround], a.v[kOnGround], 0);
        CHECK_NEAR(b.v[kLightNav], 1.0, 0);
        CHECK(std::isnan(b.v[kLightTaxi]));
    }
    CHECK_NEAR(clip.duration(), 299.0 / 30.0, 1e-4);
}

TEST(ClipFormat_EmptyClip) {
    std::vector<Frame> none;
    std::vector<uint8_t> bytes;
    writeClip(makeMeta(), VectorFrameSource(none), bytes);
    Clip clip;
    REQUIRE(readClip(bytes.data(), bytes.size(), clip, nullptr));
    CHECK(clip.frames.empty());
    CHECK_NEAR(clip.duration(), 0.0, 0);
}

TEST(ClipFormat_ReadMetaOnly) {
    const std::vector<Frame> frames = makeFrames(10, 0.0);
    std::vector<uint8_t> bytes;
    writeClip(makeMeta(), VectorFrameSource(frames), bytes);
    ClipMeta meta;
    REQUIRE(readClipMeta(bytes.data(), bytes.size(), meta, nullptr));
    CHECK_EQ(meta.frames, 10u);
    CHECK_EQ(meta.aircraft, makeMeta().aircraft);
}

TEST(ClipFormat_SkipsUnknownChunks) {
    const std::vector<Frame> frames = makeFrames(20, 0.0);
    std::vector<uint8_t> bytes;
    writeClip(makeMeta(), VectorFrameSource(frames), bytes);
    const std::vector<uint8_t> patched = withExtraChunk(bytes, "LVAR", 123);
    Clip clip;
    std::string err;
    REQUIRE(readClip(patched.data(), patched.size(), clip, &err));
    CHECK_EQ(clip.frames.size(), 20u);
    CHECK_NEAR(clip.frames[5].v[kLat], frames[5].v[kLat], 1e-12);
}

TEST(ClipFormat_UnknownAndMissingChannelsMapByKey) {
    // Hand-build a file with three channels: a key from the future, "alt" and "gs" (in that order).
    std::vector<uint8_t> bytes;
    ByteWriter w(bytes);
    w.bytes("BRPL", 4);
    w.u16(kClipFormatVersion);
    w.u16(0);
    size_t c = w.beginChunk("META");
    w.u8(1);
    w.u8(0);
    w.u16(0);
    w.i64(5);
    w.f32(30.0f);
    w.u32(2);
    w.f32(0);
    w.f32(NAN);
    w.f32(NAN);
    w.str("Future Jet");
    w.f32(99.0f);  // a field appended by a future version: must be ignored
    w.endChunk(c);
    c = w.beginChunk("CHAN");
    w.u16(3);
    w.u8(static_cast<uint8_t>(Storage::F64));
    w.u8(0);
    w.u8(8);
    w.str("lvar_future_thing");
    w.u8(static_cast<uint8_t>(Storage::F64));
    w.u8(0);
    w.u8(8);
    w.str("alt");
    w.u8(static_cast<uint8_t>(Storage::F32));
    w.u8(0);
    w.u8(4);
    w.str("gs");
    w.endChunk(c);
    c = w.beginChunk("SAMP");
    w.u32(2);
    w.u32(4 + 8 + 8 + 4);
    for (int i = 0; i < 2; ++i) {
        w.f32(static_cast<float>(i));
        w.f64(-1.0);
        w.f64(1000.0 + i);
        w.f32(120.0f + i);
    }
    w.endChunk(c);

    Clip clip;
    std::string err;
    REQUIRE(readClip(bytes.data(), bytes.size(), clip, &err));
    CHECK_EQ(clip.meta.aircraft, std::string("Future Jet"));
    CHECK_EQ(clip.present.count(), 2u);
    CHECK(clip.present.test(kAlt));
    CHECK(clip.present.test(kGroundSpeed));
    CHECK(!clip.present.test(kLat));
    CHECK_NEAR(clip.frames[1].v[kAlt], 1001.0, 0);
    CHECK_NEAR(clip.frames[1].v[kGroundSpeed], 121.0, 0);
    CHECK(std::isnan(clip.frames[1].v[kLat]));
}

TEST(ClipFormat_RejectsCorruptFiles) {
    const std::vector<Frame> frames = makeFrames(50, 0.0);
    std::vector<uint8_t> bytes;
    writeClip(makeMeta(), VectorFrameSource(frames), bytes);
    Clip clip;
    std::string err;

    std::vector<uint8_t> badMagic = bytes;
    badMagic[0] = 'X';
    CHECK(!readClip(badMagic.data(), badMagic.size(), clip, &err));

    std::vector<uint8_t> future = bytes;
    future[4] = 0xFF;  // format version 255
    CHECK(!readClip(future.data(), future.size(), clip, &err));

    for (size_t cut : {size_t(0), size_t(5), size_t(9), bytes.size() / 2, bytes.size() - 1}) {
        err.clear();
        CHECK(!readClip(bytes.data(), cut, clip, &err));
        CHECK(!err.empty());
    }
}

TEST(ClipFormat_FileSizeIsCompact) {
    const std::vector<Frame> frames = makeFrames(9000, 0.0);  // 5 minutes at 30 Hz
    std::vector<uint8_t> bytes;
    writeClip(makeMeta(), VectorFrameSource(frames), bytes);
    std::printf("       5-minute clip: %zu bytes\n", bytes.size());
    // About 270 bytes a frame since the control setup's channels (N1, RPM, spoiler position,
    // L:vars) were added: ~1.3 MB for a typical landing clip.
    CHECK(bytes.size() < 3u * 1024u * 1024u);
}
