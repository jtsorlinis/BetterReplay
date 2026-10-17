// Console logging. Everything is prefixed with "[BetterReplay]" so it can be filtered in the
// DevMode console; warnings and errors go to stderr.
#pragma once

namespace br {
namespace log {

extern bool g_debug;  // toggled from the panel ("setDebug" command)

void debug(const char* fmt, ...);
void info(const char* fmt, ...);
void warn(const char* fmt, ...);
void error(const char* fmt, ...);

}  // namespace log
}  // namespace br
