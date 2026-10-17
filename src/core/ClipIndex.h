// The clip library: what index.json stores, file naming and retention.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Clip.h"

namespace br {

struct ClipEntry {
    std::string file;
    EventType event = EventType::Manual;
    int64_t utcStartMs = 0;
    std::string aircraft;
    double durationSec = 0.0;
    double touchdownVsFpm = NAN;
    double peakG = NAN;
    int bounces = 0;
    bool keep = false;
    bool autoClip = false;  // takeoff/landing clips are subject to retention; manual ones aren't
};

ClipEntry makeEntry(const std::string& file, const ClipMeta& meta, double durationSec);

class JsonWriter;
void writeClipEntryJson(JsonWriter& w, const ClipEntry& e);  // one entry as a JSON object

class ClipIndex {
public:
    const std::vector<ClipEntry>& clips() const { return clips_; }  // newest first
    size_t size() const { return clips_.size(); }

    void add(const ClipEntry& entry);
    ClipEntry* find(const std::string& file);
    const ClipEntry* find(const std::string& file) const;
    bool remove(const std::string& file);
    void clear() { clips_.clear(); }

    // YYYY-MM-DD_HHMM_<event>.rpl (UTC), with _2, _3... appended on collision.
    std::string uniqueFileName(int64_t utcMs, EventType event) const;

    // Removes the oldest non-kept auto clips beyond `maxAuto` and returns their file names.
    std::vector<std::string> enforceRetention(size_t maxAuto);

    void toJson(std::string& out) const;
    bool fromJson(const char* text, size_t length, std::string* error);

private:
    std::vector<ClipEntry> clips_;
};

// True for bare "*.rpl" names with no path components; guards commands coming from JS.
bool isSafeClipFileName(const std::string& name);

}  // namespace br
