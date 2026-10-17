// UTC calendar conversion without relying on gmtime (identical results natively and in WASM).
#pragma once

#include <cstdint>
#include <string>

namespace br {

struct UtcTime {
    int year = 1970;
    int month = 1;  // 1..12
    int day = 1;    // 1..31
    int hour = 0;
    int minute = 0;
    int second = 0;
    int millisecond = 0;
};

UtcTime utcFromUnixMs(int64_t unixMs);
int64_t unixMsFromUtc(const UtcTime& t);

std::string formatIsoUtc(int64_t unixMs);    // 2026-10-17T14:32:05Z
std::string formatFileStamp(int64_t unixMs);  // 2026-10-17_1432
bool parseIsoUtc(const std::string& s, int64_t& unixMs);

}  // namespace br
