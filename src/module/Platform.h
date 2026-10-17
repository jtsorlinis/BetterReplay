#pragma once

#include <stdint.h>
#include <time.h>

namespace br {

// Real-world UTC (WASI clock_time_get), in milliseconds since the Unix epoch.
inline int64_t utcNowMs() {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return 0;
    return static_cast<int64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}

}  // namespace br
