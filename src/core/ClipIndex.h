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
    std::string aircraftPath;  // ClipMeta::aircraftPath: the aircraft it replays on ("" in older clips)
    std::string airport;  // takeoffs and landings ("" if not known)
    double durationSec = 0.0;
    double touchdownVsFpm = NAN;
    double peakG = NAN;
    int bounces = 0;
    bool autoClip = false;  // takeoff/landing clips are subject to retention; manual ones aren't
};

ClipEntry makeEntry(const std::string& file, const ClipMeta& meta, double durationSec);

class JsonWriter;
void writeClipEntryJson(JsonWriter& w, const ClipEntry& e);  // one entry as index.json stores it
void writeClipFields(JsonWriter& w, const ClipEntry& e);     // what the panel shows of it, in an open object

class ClipIndex {
public:
    const std::vector<ClipEntry>& clips() const { return clips_; }  // newest first
    size_t size() const { return clips_.size(); }
    // False after loading an index.json from before entries recorded their aircraft's path
    // (version 1): theirs are empty until read from the clips (Engine::fillAircraftPaths).
    bool pathsKnown() const { return pathsKnown_; }
    void setPathsKnown() { pathsKnown_ = true; }

    void add(const ClipEntry& entry);
    ClipEntry* find(const std::string& file);
    const ClipEntry* find(const std::string& file) const;
    bool remove(const std::string& file);
    void clear() {
        clips_.clear();
        pathsKnown_ = true;
    }

    // YYYY-MM-DD_HHMM_<event>.rpl (UTC), with _2, _3... appended on collision.
    std::string uniqueFileName(int64_t utcMs, EventType event) const;

    // Removes each aircraft's oldest auto clips beyond `maxAutoPerAircraft` and returns their file
    // names. An aircraft is what a clip replays on: its aircraft.cfg folder, or its title without one.
    std::vector<std::string> enforceRetention(size_t maxAutoPerAircraft);

    void toJson(std::string& out) const;
    bool fromJson(const char* text, size_t length, std::string* error);

private:
    std::vector<ClipEntry> clips_;
    bool pathsKnown_ = true;
};

// True for bare "*.rpl" names with no path components; guards commands coming from JS.
bool isSafeClipFileName(const std::string& name);

}  // namespace br
