// Takeoff / landing detection state machine. Pure logic: it is fed samples on the recording
// clock (which stops while recording is suspended) and reports finished events.
#pragma once

#include <cmath>
#include <cstdint>

#include "Clip.h"
#include "RingBuffer.h"

namespace br {

// Every threshold lives here.
struct DetectorConfig {
    double armDelaySec = 10.0;  // after flight load, teleport, slew, crash

    // Takeoff
    double takeoffMinAirborneSec = 5.0;
    double takeoffMinAglFt = 50.0;
    double takeoffSlowGroundSpeedKt = 10.0;  // clip starts at the last moment below this
    double takeoffEndAglFt = 1000.0;
    double takeoffMaxAfterLiftoffSec = 60.0;
    double takeoffMaxPreRollSec = 180.0;  // never start a takeoff clip earlier than this before liftoff

    // Landing
    double landingMinAirborneSec = 5.0;  // a touchdown after a shorter hop isn't a landing
    double landingMinGroundSec = 3.0;
    double landingPreRollSec = 120.0;
    double landingEndGroundSpeedKt = 30.0;
    double landingMaxAfterTouchdownSec = 170.0;  // keeps pre-roll + rollout inside the 5-minute buffer
    double landingMaxBounceAirborneSec = 60.0;   // longer than this airborne = not a bounce any more
    double touchdownVsReadDelaySec = 0.25;       // when to read PLANE TOUCHDOWN NORMAL VELOCITY
    double peakGWindowBeforeTouchdownSec = 1.0;

    // Pause handling. After a pause, an in-progress event survives unless the aircraft moved
    // more than this while paused (e.g. the user relocated it from the pause menu).
    double continuityMaxDistanceM = 200.0;
    double continuityMaxAltFt = 100.0;
    // true: also hold off *new* events for armDelaySec after every pause (strict reading of the
    // spec). false (default): only re-arm when the position jumped, so a pause on short final
    // doesn't lose the landing.
    bool holdOffNewEventsAfterPause = false;
};

struct DetectorSample {
    double t = 0.0;  // recording clock, seconds
    bool onGround = false;
    double aglFt = 0.0;
    double groundSpeedKt = 0.0;
    double verticalSpeedFpm = 0.0;
    double gForce = 1.0;
    double latDeg = 0.0;
    double lonDeg = 0.0;
    double altFt = 0.0;
    double touchdownNormalVelFps = NAN;  // PLANE TOUCHDOWN NORMAL VELOCITY, NaN if unavailable
};

struct DetectedEvent {
    EventType type = EventType::Manual;
    double startT = 0.0;
    double endT = 0.0;
    double eventT = 0.0;  // liftoff or first touchdown
    float touchdownVsFpm = NAN;
    float peakG = NAN;
    uint16_t bounces = 0;
};

class EventDetector {
public:
    using LogFn = void (*)(void* ctx, const char* message);

    explicit EventDetector(const DetectorConfig& config = DetectorConfig());

    void setLogger(LogFn fn, void* ctx) {
        log_ = fn;
        logCtx_ = ctx;
    }
    const DetectorConfig& config() const { return cfg_; }

    // Flight loaded, teleport, slew, crash, end of our own playback: abort everything in
    // progress and stay disarmed for armDelaySec (on the recording clock, from `t`).
    void disarm(double t, const char* reason);

    // Recording resumed after a pause; the next sample is checked for a position jump.
    void notifyResumedAfterPause();

    // Feeds one sample. Writes up to two finished events to `out` and returns how many.
    int update(const DetectorSample& s, DetectedEvent out[2]);

    bool armed() const { return armed_; }
    double armRemaining(double t) const { return armed_ ? 0.0 : (armAt_ > t ? armAt_ - t : 0.0); }
    const char* phase() const;  // short status for the panel / logs

private:
    struct Takeoff {
        bool active = false;
        bool confirmed = false;
        double liftoffT = 0.0;
        double startT = 0.0;
    };
    struct Landing {
        bool active = false;
        bool confirmed = false;
        bool bouncing = false;  // airborne again after touchdown, not yet back on the ground
        bool vsRead = false;
        double touchdownT = 0.0;
        double bounceLiftoffT = 0.0;
        double touchdownVsFpm = NAN;
        double peakG = NAN;
        uint16_t bounces = 0;
    };
    struct GSample {
        double t;
        double g;
    };

    void onLiftoff(const DetectorSample& s, DetectedEvent* out, int& n);
    void onTouchdown(const DetectorSample& s, DetectedEvent* out, int& n);
    void airborneTick(const DetectorSample& s, DetectedEvent* out, int& n);
    void groundTick(const DetectorSample& s, DetectedEvent* out, int& n);
    void finishTakeoff(double endT, DetectedEvent* out, int& n);
    void finishLanding(double endT, DetectedEvent* out, int& n);
    bool newEventsHeld(double t) const { return t < holdNewUntil_; }
    void logf(const char* fmt, ...);

    DetectorConfig cfg_;
    LogFn log_ = nullptr;
    void* logCtx_ = nullptr;

    bool armed_ = false;
    double armAt_ = 0.0;
    double holdNewUntil_ = -1e300;
    bool pendingContinuityCheck_ = false;

    // Tracked whether or not armed.
    bool haveTrack_ = false;
    bool prevOnGround_ = false;
    double groundRunStart_ = NAN;  // start of the current continuous ground contact
    double airRunStart_ = NAN;     // start of the current (or just ended) airborne run
    double lastSlowGround_ = NAN;  // last time on the ground below takeoffSlowGroundSpeedKt
    DetectorSample last_;
    RingBuffer<GSample> gHistory_{64};

    Takeoff takeoff_;
    Landing landing_;
};

}  // namespace br
