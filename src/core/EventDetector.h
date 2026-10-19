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

    // Takeoff: from takeoffPreRunwaySec before entering the runway until takeoffAfterCleanSec after
    // the flaps are up, or back on the ground if that's sooner.
    double takeoffMinAirborneSec = 5.0;
    double takeoffMinAglFt = 50.0;
    double takeoffPreRunwaySec = 30.0;
    // Lined up and waiting: the clip starts no earlier than this before the engines spool up (so
    // the whole takeoffPreRunwaySec stays for a wait of up to 30 s).
    double takeoffMaxWaitBeforeSpoolSec = 60.0;
    // Entered the runway before the recording (spawned on it), or on a strip the sim doesn't flag
    // as one: from this long before the spool-up.
    double takeoffPreSpoolSec = 5.0;
    double takeoffAfterCleanSec = 5.0;
    // Flaps up at liftoff (or never reported): the clip ends this long after liftoff. A takeoff
    // not confirmed by then is dropped.
    double takeoffAfterLiftoffSec = 120.0;
    double takeoffMaxAfterLiftoffSec = 300.0;  // never clean (circuits, forgotten flaps)
    double takeoffMaxPreRollSec = 175.0;       // never start a takeoff clip earlier than this before liftoff
    // The spool-up: the engines' idle is the lowest power (DetectorSample::enginePower) they held
    // steady for takeoffIdleSteadySec, within takeoffIdleBand of takeoff power (the most before
    // liftoff). They start spooling up the last time before liftoff they were that close to idle.
    double takeoffIdleSteadySec = 5.0;
    double takeoffIdleBand = 0.03;
    // Without a runway entry or engine readings the clip starts at the last moment on the ground
    // below this.
    double takeoffSlowGroundSpeedKt = 10.0;

    // Landing: from landingPreRollSec before the first touchdown until a full stop, off the runway
    // for landingOffRunwaySec, or airborne again for landingGoAroundSec (a shorter time in the air
    // is a bounce).
    double landingMinAirborneSec = 5.0;  // a touchdown after a shorter hop isn't a landing
    double landingMinGroundSec = 3.0;    // then "landing-rollout" (status only)
    double landingPreRollSec = 300.0;
    double landingStopGroundSpeedKt = 1.0;
    double landingOffRunwaySec = 10.0;
    double landingGoAroundSec = 15.0;
    double landingMaxAfterTouchdownSec = 170.0;  // with the pre-roll, a clip of 7:50 at most
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
    // The engines' power, to find the takeoff's spool-up: the highest N1 (%), RPM on pistons.
    // NaN without readings.
    double enginePower = NAN;
    bool onRunway = false;        // ON ANY RUNWAY
    bool flapsRetracted = true;   // lever and surfaces up, or not reported
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

    // The flight is ending or the aircraft is about to be moved (main menu, flight load, slew,
    // teleport): a landing in progress and a confirmed takeoff end at `t`, the last recorded
    // moment, rather than being lost. Writes them to `out` and returns how many. Call before
    // disarm().
    int finishInProgress(double t, DetectedEvent out[2]);

    // Feeds one sample. Writes up to two finished events to `out` and returns how many.
    int update(const DetectorSample& s, DetectedEvent out[2]);

    bool armed() const { return armed_; }
    double armRemaining(double t) const { return armed_ ? 0.0 : (armAt_ > t ? armAt_ - t : 0.0); }
    const char* phase() const;  // short status for the panel / logs

private:
    struct Takeoff {
        bool active = false;
        bool confirmed = false;
        bool flapsOut = false;  // at liftoff: the clip ends once they're up
        double liftoffT = 0.0;
        double startT = 0.0;
        double cleanT = NAN;  // when the flaps were last seen coming up
    };
    struct Landing {
        bool active = false;
        bool confirmed = false;
        bool bouncing = false;  // airborne again after touchdown, not yet back on the ground
        bool vsRead = false;
        bool runwaySeen = false;  // on a runway since touching down (so it can leave it)
        double offRunwayT = NAN;  // when it last left one
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
    struct PowerSample {
        double t;
        double p;
    };

    void onLiftoff(const DetectorSample& s);
    // When the engines started spooling up for a takeoff, from the power history in [from, to];
    // NaN if it can't tell (no readings, or no rise from idle in there).
    double spoolUpStart(double from, double to) const;
    double peakGAround(const DetectorSample& s) const;  // G at a touchdown and the second before
    void finishPending(double t, DetectedEvent* out, int& n);
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
    double runwayEntry_ = NAN;     // last time it rolled onto a runway
    DetectorSample last_;
    RingBuffer<GSample> gHistory_{64};
    RingBuffer<PowerSample> power_;  // enginePower every 0.25 s, back to takeoffMaxPreRollSec

    Takeoff takeoff_;
    Landing landing_;
};

}  // namespace br
