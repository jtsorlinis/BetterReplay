#include "FileStore.h"

#include <dirent.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "Log.h"

namespace br {

bool FileStore::probe() {
    static const char* kCandidates[] = {"\\work\\", "/work/", "work/"};
    for (const char* dir : kCandidates) {
        const std::string probePath = std::string(dir) + ".probe";
        FILE* f = std::fopen(probePath.c_str(), "wb");
        if (!f) {
            log::debug("work folder candidate '%s' not writable (errno %d)", dir, errno);
            continue;
        }
        const bool wrote = std::fwrite("ok", 1, 2, f) == 2;
        std::fclose(f);
        std::remove(probePath.c_str());
        if (wrote) {
            dir_ = dir;
            log::info("work folder: '%s'", dir);
            return true;
        }
    }
    log::error("no writable work folder; clips cannot be saved");
    return false;
}

bool FileStore::writeFile(const std::string& name, const uint8_t* data, size_t size) {
    if (!ready()) return false;
    FILE* f = std::fopen(path(name).c_str(), "wb");
    if (!f) {
        log::error("open %s for writing failed (errno %d)", name.c_str(), errno);
        return false;
    }
    const size_t written = size > 0 ? std::fwrite(data, 1, size, f) : 0;
    const bool closed = std::fclose(f) == 0;
    if (written != size || !closed) {
        log::error("writing %s failed (%zu of %zu bytes)", name.c_str(), written, size);
        return false;
    }
    return true;
}

bool FileStore::readFile(const std::string& name, std::vector<uint8_t>& out) {
    if (!ready()) return false;
    FILE* f = std::fopen(path(name).c_str(), "rb");
    if (!f) return false;
    out.clear();
    bool ok = std::fseek(f, 0, SEEK_END) == 0;
    const long size = ok ? std::ftell(f) : -1;
    ok = ok && size >= 0 && std::fseek(f, 0, SEEK_SET) == 0;
    if (ok) {
        out.resize(static_cast<size_t>(size));
        ok = size == 0 || std::fread(out.data(), 1, out.size(), f) == out.size();
    }
    std::fclose(f);
    if (!ok) log::warn("reading %s failed", name.c_str());
    return ok;
}

bool FileStore::removeFile(const std::string& name) {
    if (!ready()) return false;
    return std::remove(path(name).c_str()) == 0;
}

bool FileStore::replaceFile(const std::string& from, const std::string& to) {
    if (!ready()) return false;
    std::remove(path(to).c_str());  // rename() won't overwrite on every host
    if (std::rename(path(from).c_str(), path(to).c_str()) != 0) {
        log::debug("rename %s -> %s failed (errno %d)", from.c_str(), to.c_str(), errno);
        return false;
    }
    return true;
}

bool FileStore::listFiles(std::vector<std::string>& names) {
    if (!ready()) return false;
    names.clear();
    std::string dir = dir_;
    DIR* d = opendir(dir.c_str());
    if (!d && dir.size() > 1) {
        dir.pop_back();  // try without the trailing separator
        d = opendir(dir.c_str());
    }
    if (!d) {
        log::warn("cannot list work folder (errno %d)", errno);
        return false;
    }
    while (const dirent* e = readdir(d)) {
        if (e->d_name[0] == '.') continue;
        names.emplace_back(e->d_name);
    }
    closedir(d);
    return true;
}

}  // namespace br
