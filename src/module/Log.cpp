#include "Log.h"

#include <cstdarg>
#include <cstdio>

namespace br {
namespace log {

bool g_debug = false;

namespace {

void write(FILE* stream, const char* level, const char* fmt, va_list args) {
    char buf[512];
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    std::fprintf(stream, "[BetterReplay] %s%s\n", level, buf);
    std::fflush(stream);
}

}  // namespace

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
