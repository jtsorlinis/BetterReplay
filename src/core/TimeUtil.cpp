#include "TimeUtil.h"

#include <cstdio>

namespace br {

namespace {

// Howard Hinnant's days-from-civil / civil-from-days algorithms (proleptic Gregorian).
int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned mp = m > 2 ? m - 3 : m + 9;
    const unsigned doy = (153 * mp + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

void civilFromDays(int64_t z, int& year, int& month, int& day) {
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t y = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    const unsigned d = doy - (153 * mp + 2) / 5 + 1;
    const unsigned m = mp < 10 ? mp + 3 : mp - 9;
    year = static_cast<int>(y + (m <= 2));
    month = static_cast<int>(m);
    day = static_cast<int>(d);
}

int64_t floorDiv(int64_t a, int64_t b) {
    int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
    return q;
}

}  // namespace

UtcTime utcFromUnixMs(int64_t unixMs) {
    UtcTime t;
    const int64_t days = floorDiv(unixMs, 86400000);
    int64_t msOfDay = unixMs - days * 86400000;
    civilFromDays(days, t.year, t.month, t.day);
    t.hour = static_cast<int>(msOfDay / 3600000);
    msOfDay %= 3600000;
    t.minute = static_cast<int>(msOfDay / 60000);
    msOfDay %= 60000;
    t.second = static_cast<int>(msOfDay / 1000);
    t.millisecond = static_cast<int>(msOfDay % 1000);
    return t;
}

int64_t unixMsFromUtc(const UtcTime& t) {
    const int64_t days = daysFromCivil(t.year, static_cast<unsigned>(t.month), static_cast<unsigned>(t.day));
    return days * 86400000 + t.hour * 3600000LL + t.minute * 60000LL + t.second * 1000LL + t.millisecond;
}

std::string formatIsoUtc(int64_t unixMs) {
    const UtcTime t = utcFromUnixMs(unixMs);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ", t.year, t.month, t.day, t.hour, t.minute,
                  t.second);
    return buf;
}

std::string formatFileStamp(int64_t unixMs) {
    const UtcTime t = utcFromUnixMs(unixMs);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d_%02d%02d", t.year, t.month, t.day, t.hour, t.minute);
    return buf;
}

bool parseIsoUtc(const std::string& s, int64_t& unixMs) {
    UtcTime t;
    if (std::sscanf(s.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d", &t.year, &t.month, &t.day, &t.hour, &t.minute,
                    &t.second) != 6) {
        return false;
    }
    if (t.month < 1 || t.month > 12 || t.day < 1 || t.day > 31) return false;
    unixMs = unixMsFromUtc(t);
    return true;
}

}  // namespace br
