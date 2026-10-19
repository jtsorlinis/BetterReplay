// IStorage on the package's persistent work folder, using plain stdio (WASI), and reads from the
// package folder itself (the included control profiles).
// On disk (Steam): %APPDATA%\Microsoft Flight Simulator 2024\WASM\MSFS2024\<package>\work
#pragma once

#include <string>
#include <vector>

#include "Engine.h"

namespace br {

class FileStore : public IStorage {
public:
    // Finds a work-folder spelling the runtime accepts (the docs say "\work") by writing and
    // removing a probe file. Returns false if none worked; the store then fails every call.
    bool probe();
    bool ready() const { return !dir_.empty(); }
    const std::string& dir() const { return dir_; }

    bool writeFile(const std::string& name, const uint8_t* data, size_t size) override;
    bool readFile(const std::string& name, std::vector<uint8_t>& out) override;
    bool removeFile(const std::string& name) override;
    bool replaceFile(const std::string& from, const std::string& to) override;
    bool listFiles(std::vector<std::string>& names) override;
    // From the package folder (read-only), with whichever spelling of its root opens the file.
    bool readPackageFile(const std::string& name, std::vector<uint8_t>& out) override;

private:
    std::string path(const std::string& name) const { return dir_ + name; }
    std::string dir_;
};

}  // namespace br
