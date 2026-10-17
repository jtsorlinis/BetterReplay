#include "Log.h"

#include <cerrno>
#include <cstdarg>
#include <cstdio>

#include "Platform.h"

namespace br {
namespace log {

#ifdef NDEBUG
bool g_debug = false;  // Release: enable from the panel's CommBus "setDebug" command
#else
bool g_debug = true;  // Debug builds log everything
#endif

namespace {

constexpr long kMaxFileBytes = 8L * 1024 * 1024;

FILE* g_file = nullptr;
long g_fileBytes = 0;
std::string g_path;
std::string g_prevPath;

bool startFile() {
    std::remove(g_prevPath.c_str());
    std::rename(g_path.c_str(), g_prevPath.c_str());  // fails harmlessly if there's no file yet
    g_file = std::fopen(g_path.c_str(), "wb");
    g_fileBytes = 0;
    return g_file != nullptr;
}

void writeFile(const char* level, const char* msg) {
    if (!g_file) return;
    const int64_t ms = utcNowMs();
    const int64_t s = ms / 1000;
    const int n = std::fprintf(g_file, "%02d:%02d:%02d.%03d %s%s\n", static_cast<int>(s / 3600 % 24),
                               static_cast<int>(s / 60 % 60), static_cast<int>(s % 60), static_cast<int>(ms % 1000),
                               level, msg);
    std::fflush(g_file);
    if (n > 0) g_fileBytes += n;
    if (g_fileBytes > kMaxFileBytes) {
        std::fclose(g_file);
        g_file = nullptr;
        if (startFile()) std::fprintf(g_file, "(continued from %s)\n", g_prevPath.c_str());
    }
}

void write(FILE* stream, const char* level, const char* fmt, va_list args) {
    char buf[512];
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    std::fprintf(stream, "[BetterReplay] %s%s\n", level, buf);
    std::fflush(stream);
    writeFile(level, buf);
}

}  // namespace

void openFile(const std::string& path, const std::string& prevPath) {
    closeFile();
    g_path = path;
    g_prevPath = prevPath;
    if (!startFile()) {
        warn("could not open log file %s (errno %d); logging to the console only", path.c_str(), errno);
        return;
    }
    const int64_t s = utcNowMs() / 1000;
    info("log file started (UTC times; Unix time %lld)", static_cast<long long>(s));
}

void closeFile() {
    if (!g_file) return;
    std::fclose(g_file);
    g_file = nullptr;
}

void debug(const char* fmt, ...) {
    if (!g_debug) return;
    va_list args;
    va_start(args, fmt);
    write(stdout, "DEBUG ", fmt, args);
    va_end(args);
}

void info(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    write(stdout, "", fmt, args);
    va_end(args);
}

void warn(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    write(stderr, "WARNING ", fmt, args);
    va_end(args);
}

void error(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    write(stderr, "ERROR ", fmt, args);
    va_end(args);
}

}  // namespace log
}  // namespace br
