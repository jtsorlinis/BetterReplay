// .rpl clip files: little-endian, chunked.
//
//   "BRPL" u16 formatVersion u16 flags
//   chunk*: char[4] id, u32 payloadSize, payload     (unknown chunks are skipped)
//
//   META  u8 event, u8 reserved, u16 bounces, i64 utcStartMs, f32 sampleRateHz, u32 frames,
//         f32 eventOffsetSec, f32 touchdownVsFpm, f32 peakG, str aircraft, str aircraftPath
//         (new fields are appended; readers stop at the payload size)
//   CHAN  u16 count, count x { u8 storage, u8 interp, u8 bytes, str key }
//   SAMP  u32 frames, u32 stride, frames x { f32 t, channel values in CHAN order }
//   VARS  u16 count, count x { str channelKey, str variable }   (optional: which aircraft
//         variable each input event / L:var channel recorded)
//
//   str = u16 byteLength + UTF-8 bytes
//
// Channels are matched by key on load: unknown keys are skipped, and channels missing from
// the file are marked absent (playback leaves them alone).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Clip.h"

namespace br {

constexpr uint16_t kClipFormatVersion = 1;

// Random access to the frames being written, so callers can stream straight out of a ring
// buffer without copying into a temporary vector.
class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual size_t size() const = 0;
    virtual const Frame& at(size_t i) const = 0;
};

class VectorFrameSource : public FrameSource {
public:
    explicit VectorFrameSource(const std::vector<Frame>& frames) : frames_(frames) {}
    size_t size() const override { return frames_.size(); }
    const Frame& at(size_t i) const override { return frames_[i]; }

private:
    const std::vector<Frame>& frames_;
};

// Serializes into `out` (cleared first; capacity is kept). Frame times are stored relative
// to the first frame. meta.frames is ignored and taken from `frames`.
void writeClip(const ClipMeta& meta, const FrameSource& frames, std::vector<uint8_t>& out);

bool readClip(const uint8_t* data, size_t size, Clip& out, std::string* error);

// Parses only the META chunk (used to rebuild the index from files on disk).
bool readClipMeta(const uint8_t* data, size_t size, ClipMeta& out, std::string* error);

}  // namespace br
