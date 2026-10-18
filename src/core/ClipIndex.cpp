#include "ClipIndex.h"

#include <algorithm>
#include <cmath>

#include "Json.h"
#include "TimeUtil.h"

namespace br {

namespace {

// Newest first by when a clip ends: a manual save of the last 5 minutes starts long before the
// takeoff clip saved a minute earlier, but it's the newer of the two.
int64_t endMs(const ClipEntry& e) { return e.utcStartMs + static_cast<int64_t>(std::llround(e.durationSec * 1000.0)); }

bool newerFirst(const ClipEntry& a, const ClipEntry& b) {
    const int64_t ea = endMs(a);
    const int64_t eb = endMs(b);
    if (ea != eb) return ea > eb;
    return a.file > b.file;
}

}  // namespace

ClipEntry makeEntry(const std::string& file, const ClipMeta& meta, double durationSec) {
    ClipEntry e;
    e.file = file;
    e.event = meta.event;
    e.utcStartMs = meta.utcStartMs;
    e.aircraft = meta.aircraft;
    e.airport = meta.airport;
    e.durationSec = durationSec;
    e.touchdownVsFpm = meta.touchdownVsFpm;
    e.peakG = meta.peakG;
    e.bounces = meta.bounces;
    e.autoClip = meta.event != EventType::Manual;
    return e;
}

void ClipIndex::add(const ClipEntry& entry) {
    remove(entry.file);
    const auto it = std::lower_bound(clips_.begin(), clips_.end(), entry, newerFirst);
    clips_.insert(it, entry);
}

ClipEntry* ClipIndex::find(const std::string& file) {
    for (ClipEntry& e : clips_) {
        if (e.file == file) return &e;
    }
    return nullptr;
}

const ClipEntry* ClipIndex::find(const std::string& file) const {
    for (const ClipEntry& e : clips_) {
        if (e.file == file) return &e;
    }
    return nullptr;
}

bool ClipIndex::remove(const std::string& file) {
    for (auto it = clips_.begin(); it != clips_.end(); ++it) {
        if (it->file == file) {
            clips_.erase(it);
            return true;
        }
    }
    return false;
}

std::string ClipIndex::uniqueFileName(int64_t utcMs, EventType event) const {
    const std::string stem = formatFileStamp(utcMs) + "_" + eventName(event);
    std::string name = stem + ".rpl";
    for (int n = 2; find(name) != nullptr; ++n) name = stem + "_" + std::to_string(n) + ".rpl";
    return name;
}

std::vector<std::string> ClipIndex::enforceRetention(size_t maxAuto) {
    std::vector<std::string> removed;
    size_t autoCount = 0;
    // clips_ is newest first, so everything past the first maxAuto eligible clips goes.
    for (auto it = clips_.begin(); it != clips_.end();) {
        if (it->autoClip) {
            if (++autoCount > maxAuto) {
                removed.push_back(it->file);
                it = clips_.erase(it);
                continue;
            }
        }
        ++it;
    }
    return removed;
}

void writeClipEntryJson(JsonWriter& w, const ClipEntry& e) {
    w.beginObject();
    w.kv("file", e.file);
    w.kv("event", eventName(e.event));
    w.kv("utc", formatIsoUtc(e.utcStartMs));
    w.kv("utcMs", e.utcStartMs);
    w.kv("aircraft", e.aircraft);
    w.kv("airport", e.airport);
    w.kv("durationSec", e.durationSec);
    w.kv("touchdownVsFpm", e.touchdownVsFpm);
    w.kv("peakG", e.peakG);
    w.kv("bounces", e.bounces);
    w.kv("auto", e.autoClip);
    w.endObject();
}

void ClipIndex::toJson(std::string& out) const {
    JsonWriter w(out);
    w.beginObject();
    w.kv("version", 1);
    w.key("clips").beginArray();
    for (const ClipEntry& e : clips_) writeClipEntryJson(w, e);
    w.endArray();
    w.endObject();
}

bool ClipIndex::fromJson(const char* text, size_t length, std::string* error) {
    JsonValue root;
    if (!parseJson(text, length, root, error)) return false;
    const JsonValue* list = root.get("clips");
    if (!list || !list->isArray()) {
        if (error) *error = "index has no clips array";
        return false;
    }
    std::vector<ClipEntry> loaded;
    for (const JsonValue& v : list->array) {
        if (!v.isObject()) continue;
        ClipEntry e;
        e.file = v.getString("file", "");
        if (!isSafeClipFileName(e.file)) continue;
        if (!parseEventName(v.getString("event", "manual"), e.event)) e.event = EventType::Manual;
        e.utcStartMs = static_cast<int64_t>(v.getNumber("utcMs", 0.0));
        if (e.utcStartMs == 0) parseIsoUtc(v.getString("utc", ""), e.utcStartMs);
        e.aircraft = v.getString("aircraft", "");
        e.airport = v.getString("airport", "");
        e.durationSec = v.getNumber("durationSec", 0.0);
        e.touchdownVsFpm = v.getNumber("touchdownVsFpm", NAN);
        e.peakG = v.getNumber("peakG", NAN);
        e.bounces = static_cast<int>(v.getNumber("bounces", 0.0));  // an old "keep" flag is ignored
        e.autoClip = v.getBool("auto", e.event != EventType::Manual);
        loaded.push_back(e);
    }
    std::sort(loaded.begin(), loaded.end(), newerFirst);
    clips_.swap(loaded);
    return true;
}

bool isSafeClipFileName(const std::string& name) {
    if (name.size() < 5 || name.size() > 128) return false;
    if (name.compare(name.size() - 4, 4, ".rpl") != 0) return false;
    for (const char c : name) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' ||
                        c == '_' || c == '.';
        if (!ok) return false;
    }
    return name.find("..") == std::string::npos;
}

}  // namespace br
