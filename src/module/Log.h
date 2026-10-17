// Console logging. Everything is prefixed with "[BetterReplay]" so it can be filtered in the
// DevMode console; warnings and errors go to stderr. Once openFile() has been called every line
// is also written, with a UTC timestamp, to a log file in the work folder.
#pragma once

#include <string>

namespace br {
namespace log {

extern bool g_debug;  // toggled from the panel ("setDebug" command)

// Starts the log file at `path`. The previous session's file is kept as `prevPath`; a file that
// grows past a few MB is rotated the same way.
void openFile(const std::string& path, const std::string& prevPath);
void closeFile();

void debug(const char* fmt, ...);
void info(const char* fmt, ...);
void warn(const char* fmt, ...);
void error(const char* fmt, ...);

}  // namespace log
}  // namespace br
