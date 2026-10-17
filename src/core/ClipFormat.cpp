#include "ClipFormat.h"

#include <cmath>
#include <cstring>

#include "ByteIO.h"

namespace br {

namespace {

constexpr char kMagic[4] = {'B', 'R', 'P', 'L'};
constexpr uint8_t kU8Missing = 0xFF;  // u8 encoding of NaN (channel had no value)

void writeValue(ByteWriter& w, Storage s, double v) {
    switch (s) {
        case Storage::F64: w.f64(v); break;
        case Storage::F32: w.f32(static_cast<float>(v)); break;
        case Storage::U8: {
            if (std::isnan(v)) {
                w.u8(kU8Missing);
            } else {
                const double r = std::round(v);
                w.u8(static_cast<uint8_t>(r < 0.0 ? 0.0 : (r > 254.0 ? 254.0 : r)));
            }
            break;
        }
    }
}

double readValue(const uint8_t* p, Storage s) {
    ByteReader r(p, 8);
    switch (s) {
        case Storage::F64: return r.f64();
        case Storage::F32: return r.f32();
        case Storage::U8: {
            const uint8_t b = r.u8();
            return b == kU8Missing ? NAN : static_cast<double>(b);
        }
    }
    return NAN;
}

bool setError(std::string* error, const char* msg) {
    if (error) *error = msg;
    return false;
}

bool readHeader(ByteReader& r, std::string* error) {
    char magic[4];
    if (!r.bytes(magic, 4) || std::memcmp(magic, kMagic, 4) != 0) return setError(error, "not a BetterReplay clip");
    const uint16_t version = r.u16();
    r.u16();  // flags
    if (!r.ok()) return setError(error, "truncated header");
    if (version == 0 || version > kClipFormatVersion) return setError(error, "unsupported clip format version");
    return true;
}

void parseMeta(const uint8_t* payload, size_t size, ClipMeta& meta) {
    ByteReader r(payload, size);
    meta.event = static_cast<EventType>(r.u8());
    r.u8();
    meta.bounces = r.u16();
    meta.utcStartMs = r.i64();
    meta.sampleRateHz = r.f32();
    meta.frames = r.u32();
    meta.eventOffsetSec = r.f32();
    meta.touchdownVsFpm = r.f32();
    meta.peakG = r.f32();
    meta.aircraft = r.str();
}

struct ChunkHeader {
    char id[4];
    uint32_t size;
    const uint8_t* payload;
};

bool nextChunk(ByteReader& r, ChunkHeader& c, std::string* error) {
    r.bytes(c.id, 4);
    c.size = r.u32();
    c.payload = r.cursor();
    if (!r.ok() || !r.skip(c.size)) return setError(error, "truncated chunk");
    return true;
}

}  // namespace

const char* eventName(EventType e) {
    switch (e) {
        case EventType::Manual: return "manual";
        case EventType::Takeoff: return "takeoff";
        case EventType::Landing: return "landing";
    }
    return "manual";
}

bool parseEventName(const std::string& name, EventType& out) {
    if (name == "manual") out = EventType::Manual;
    else if (name == "takeoff") out = EventType::Takeoff;
    else if (name == "landing") out = EventType::Landing;
    else return false;
    return true;
}

void writeClip(const ClipMeta& meta, const FrameSource& frames, std::vector<uint8_t>& out) {
    out.clear();
    ByteWriter w(out);
    w.bytes(kMagic, 4);
    w.u16(kClipFormatVersion);
    w.u16(0);

    const size_t n = frames.size();

    size_t chunk = w.beginChunk("META");
    w.u8(static_cast<uint8_t>(meta.event));
    w.u8(0);
    w.u16(meta.bounces);
    w.i64(meta.utcStartMs);
    w.f32(meta.sampleRateHz);
    w.u32(static_cast<uint32_t>(n));
    w.f32(meta.eventOffsetSec);
    w.f32(meta.touchdownVsFpm);
    w.f32(meta.peakG);
    w.str(meta.aircraft);
    w.endChunk(chunk);

    if (!meta.vars.empty()) {
        chunk = w.beginChunk("VARS");
        w.u16(static_cast<uint16_t>(meta.vars.size()));
        for (const auto& kv : meta.vars) {
            w.str(kv.first);
            w.str(kv.second);
        }
        w.endChunk(chunk);
    }

    uint32_t stride = 4;
    chunk = w.beginChunk("CHAN");
    w.u16(static_cast<uint16_t>(kChannelCount));
    for (const ChannelDef& c : kChannels) {
        const size_t bytes = storageBytes(c.storage);
        stride += static_cast<uint32_t>(bytes);
        w.u8(static_cast<uint8_t>(c.storage));
        w.u8(static_cast<uint8_t>(c.interp));
        w.u8(static_cast<uint8_t>(bytes));
        w.str(c.key);
    }
    w.endChunk(chunk);

    out.reserve(out.size() + 16 + n * stride);
    chunk = w.beginChunk("SAMP");
    w.u32(static_cast<uint32_t>(n));
    w.u32(stride);
    const double t0 = n > 0 ? frames.at(0).t : 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Frame& f = frames.at(i);
        w.f32(static_cast<float>(f.t - t0));
        for (int c = 0; c < kChannelCount; ++c) writeValue(w, kChannels[c].storage, f.v[c]);
    }
    w.endChunk(chunk);
}

bool readClipMeta(const uint8_t* data, size_t size, ClipMeta& out, std::string* error) {
    ByteReader r(data, size);
    if (!readHeader(r, error)) return false;
    ChunkHeader c;
    while (r.remaining() > 0 && nextChunk(r, c, error)) {
        if (std::memcmp(c.id, "META", 4) == 0) {
            out = ClipMeta();
            parseMeta(c.payload, c.size, out);
            return true;
        }
    }
    return setError(error, "missing META chunk");
}

bool readClip(const uint8_t* data, size_t size, Clip& out, std::string* error) {
    out = Clip();
    ByteReader r(data, size);
    if (!readHeader(r, error)) return false;

    struct Column {
        int slot;  // -1 = not known to this build, skipped
        Storage storage;
        uint32_t offset;
    };
    std::vector<Column> columns;
    bool haveMeta = false;
    bool haveChan = false;
    bool haveSamp = false;
    uint32_t minStride = 4;

    ChunkHeader c;
    while (r.remaining() > 0) {
        if (!nextChunk(r, c, error)) return false;
        if (std::memcmp(c.id, "META", 4) == 0) {
            parseMeta(c.payload, c.size, out.meta);
            haveMeta = true;
        } else if (std::memcmp(c.id, "VARS", 4) == 0) {
            ByteReader vr(c.payload, c.size);
            const uint16_t count = vr.u16();
            out.meta.vars.clear();
            for (uint16_t i = 0; i < count && vr.ok(); ++i) {
                std::string key = vr.str();
                std::string var = vr.str();
                if (vr.ok()) out.meta.vars.emplace_back(std::move(key), std::move(var));
            }
        } else if (std::memcmp(c.id, "CHAN", 4) == 0) {
            ByteReader cr(c.payload, c.size);
            const uint16_t count = cr.u16();
            uint32_t offset = 4;
            for (uint16_t i = 0; i < count && cr.ok(); ++i) {
                const Storage storage = static_cast<Storage>(cr.u8());
                cr.u8();  // interp (informational; this build's table decides)
                const uint8_t bytes = cr.u8();
                const std::string key = cr.str();
                int slot = findChannel(key.c_str());
                if (slot >= 0 && storageBytes(storage) != bytes) slot = -1;
                columns.push_back({slot, storage, offset});
                offset += bytes;
            }
            if (!cr.ok()) return setError(error, "corrupt CHAN chunk");
            minStride = offset;
            haveChan = true;
        } else if (std::memcmp(c.id, "SAMP", 4) == 0) {
            if (!haveChan) return setError(error, "SAMP before CHAN");
            ByteReader sr(c.payload, c.size);
            const uint32_t frames = sr.u32();
            const uint32_t stride = sr.u32();
            if (!sr.ok() || stride < minStride) return setError(error, "corrupt SAMP chunk");
            if (static_cast<uint64_t>(frames) * stride > sr.remaining()) return setError(error, "truncated samples");
            out.frames.resize(frames);
            const uint8_t* base = sr.cursor();
            for (uint32_t i = 0; i < frames; ++i) {
                const uint8_t* row = base + static_cast<size_t>(i) * stride;
                Frame& f = out.frames[i];
                for (double& v : f.v) v = NAN;
                f.t = readValue(row, Storage::F32);
                for (const Column& col : columns) {
                    if (col.slot >= 0) f.v[col.slot] = readValue(row + col.offset, col.storage);
                }
            }
            haveSamp = true;
        }
        // Unknown chunks are skipped for forward compatibility.
    }

    if (!haveMeta || !haveChan || !haveSamp) return setError(error, "missing required chunk");
    for (const Column& col : columns) {
        if (col.slot >= 0) out.present.set(static_cast<size_t>(col.slot));
    }
    out.meta.frames = static_cast<uint32_t>(out.frames.size());
    return true;
}

}  // namespace br
