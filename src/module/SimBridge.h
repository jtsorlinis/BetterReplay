// SimConnect adapter: builds data definitions from the channel table, subscribes to system and
// flow events, feeds every sim frame into the Engine, and drives the aircraft during playback.
#pragma once

#include <MSFS/MSFS_WindowsTypes.h>
#include <MSFS/MSFS_Events.h>
#include <MSFS/MSFS_Vars.h>
#include <SimConnect.h>

#include <cmath>
#include <string>
#include <vector>

#include "ControlDriver.h"
#include "Engine.h"

namespace br {

class SimBridge : public ISimOutput, public IControlSim {
public:
    explicit SimBridge(Engine& engine) : engine_(engine) {}

    bool open();
    void close();
    void update(float dTime);  // from a module update callback

    bool connected() const { return h_ != 0; }

    // ISimOutput
    void beginPlayback(const Clip& clip, const Frame& peak) override;
    void writePlaybackFrame(const Frame& frame, bool jumped) override;
    void sendChannelEvent(int channel, double value) override;
    void endPlayback(const Frame& snapshot, bool restore) override;
    bool restoring() const override { return restoreWaitFrames_ > 0; }
    bool prepareJump(const Frame& target) override;
    void applyProfile(const AircraftProfile* profile) override;

    // IControlSim (the control setup)
    void scanBegin(std::vector<ScanVar>& vars) override;
    void scanRead(std::vector<double>& values) override;
    void scanEnd() override;
    bool beginControl(Control c, const ControlMethod& m, std::string& why) override;
    void applyControl(Control c, const Frame& f, bool force) override;
    void endControl(Control c) override;
    double readVar(const ScanVar& v) override;
    void setReverser(bool on) override;

private:
    enum DataDef : DWORD { DEF_RECORD = 1, DEF_TITLE = 2, DEF_PLAYBACK = 3, DEF_RESTORE = 4, DEF_SNAP = 5 };
    enum ClientEvent : DWORD {
        EVT_FREEZE_SET_FIRST = 100,    // + Freeze
        EVT_FREEZE_TOGGLE_FIRST = 110, // + Freeze
        EVT_CHANNEL_FIRST = 200,       // + channel index, for PlayMode::Event channels
        EVT_CHANNEL_ON_FIRST = 300,    // + channel index, ChannelDef::playEventOn
        EVT_CHANNEL_OFF_FIRST = 400,   // + channel index, ChannelDef::playEventOff
        EVT_MASK_FIRST = 500,          // + index into kMaskedInputEvents
        EVT_THROTTLE_AXIS_FIRST = 600, // + engine - 1: THROTTLEn_AXIS_SET_EX1, to drive the levers
    };
    enum Group : DWORD { GROUP_INPUT_MASK = 1 };
    enum Freeze { kFreezeLatLon, kFreezeAlt, kFreezeAtt, kFreezeCount };
    enum Request : DWORD {
        REQ_RECORD = 1,
        REQ_TITLE = 2,
        REQ_INPUT_EVENTS = 3,
        REQ_AIRCRAFT_PATH = 4,  // RequestSystemState("AircraftLoaded"): the aircraft's profile key
        REQ_INPUT_VALUE_FIRST = 100,  // + channel index: GetInputEvent for an input event channel
    };
    enum SysEvent : DWORD {
        EVT_FLIGHT_LOADED = 1,
        EVT_SIM,
        EVT_SIM_START,
        EVT_SIM_STOP,
        EVT_PAUSE_EX1,
        EVT_CRASHED,
        EVT_CRASH_RESET,
        EVT_POSITION_CHANGED,
    };
    // Extra values appended after the channels in DEF_RECORD.
    enum Extra {
        kExSimTime,
        kExSlew,
        kExTouchdownNormalVel,
        kExFreezeLatLon,
        kExFreezeAlt,
        kExFreezeAtt,
        kExNumEngines,
        kExThrottleLowerLimit,
        // for the control setup (SetupFacts)
        kExEngineType,
        kExParkingBrake,
        kExOnRunway,
        kExCombustion1,
        kExCombustion2,
        kExCombustion3,
        kExCombustion4,
        kExGearRetractable,
        kExSpoilerAvailable,
        kExFlapPositions,
        kExtraCount
    };

    static void CALLBACK dispatchThunk(SIMCONNECT_RECV* data, DWORD size, void* ctx);
    static void keyEventThunk(FsEventId id, FsVarParamArray* params, void* ctx);
    // Hardware throttle key events: tracks the pilot's throttle axis for the thr_axis_N channels.
    void onKeyEvent(FsEventId id, const FsVarParamArray* params);
    void dispatch(SIMCONNECT_RECV* data, DWORD size);
    void onSystemEvent(DWORD id, DWORD data);
    void onFlowEvent(DWORD flow, const char* path);
    void onRecordData(const SIMCONNECT_RECV_SIMOBJECT_DATA* data, DWORD size);
    void onTitleData(const SIMCONNECT_RECV_SIMOBJECT_DATA* data, DWORD size);
    void heartbeat();
    void sendFreeze(bool on);
    void verifyFreeze();
    void checkRestoreGear();
    void checkGearResponse();
    // (Re)builds a SetData definition from the channels with play mode `mode`, limited to `present`
    // (if given) and to engines the current aircraft has. Returns the channel count.
    int buildDefinition(DWORD def, bool& defined, PlayMode mode, const ChannelMask* present, int* channels);
    void setInputMasking(bool on);
    void checkOverrides();
    void logOverrides();
    void writeSnap(const Frame& frame);
    void checkSnap();
    // Throttle via axis events, for aircraft that override the lever simvar.
    void switchThrottleToEvents(int overriddenPct);
    void sendThrottleEvents(const Frame& frame, bool force);
    void checkThrottleEvents();
    // Diagnostics: do the aircraft's lights/reversers follow the states playback sends?
    void trackEventChannels(const Frame& frame, bool jumped);
    void checkEventChannels();
    // Input event channels: resolve them against the aircraft's list, replay and check them.
    void resetInputEvents(bool unsubscribe);
    void matchInputChannels();   // input event channels <- the aircraft's list (profile names first)
    void resolveLVarChannels();  // L:var channels <- the profile's L:vars
    void reresolveForProfile();
    void requestAircraftPath();
    void onSystemState(const SIMCONNECT_RECV_SYSTEM_STATE* s);
    void logLVars();       // lists the aircraft's L:vars for gear/throttle/flaps/spoilers (research)
    void tracePlayback();  // debug: once a second, what playback sent vs what the aircraft reports
    void resolveInputEvents();
    void pollInputEvents();
    void sendInputEvents(const Frame& frame, bool force);
    void checkInputEvents();
    // Set channels the aircraft keeps overriding (flaps, spoilers, surfaces) are dropped for the
    // rest of the playback, so the two don't fight (visible flicker).
    void yieldChannel(int channel, int overriddenPct);
    void rebuildPlaybackDefinition();
    void onInputEvents(const SIMCONNECT_RECV_ENUMERATE_INPUT_EVENTS* list, DWORD size);
    void onAircraftChanged();
    double liveGearExtension() const;

    // Remembers what each SimConnect call was so exceptions can be logged meaningfully.
    void track(HRESULT hr, const char* what);
    const char* describe(DWORD sendId) const;

    Engine& engine_;
    HANDLE h_ = 0;

    FrameInput input_;
    uint64_t dispatchCount_ = 0;
    uint64_t frameCount_ = 0;
    float sinceOpen_ = 0.0f;
    float pendingDt_ = 0.0f;
    float sinceHeartbeat_ = 0.0f;
    bool pollDispatch_ = false;
    bool sizeWarned_ = false;
    bool simReplay_ = false;
    bool rtc_ = false;

    // recording: DEF_RECORD slot -> channel (channels with a simvar)
    int recordChannels_[kChannelCount] = {};
    int recordCount_ = 0;

    // Input event channels (PlayMode::Input): hash on the current aircraft (0 = it doesn't have
    // one), latest value from the subscription, and playback state.
    uint64_t ieHash_[kChannelCount] = {};
    char ieName_[kChannelCount][64] = {};
    double ieValue_[kChannelCount] = {};
    // Replay goes through the input event's "<NAME>_SET" preset (fsVarsBVarCall), the same path
    // a cockpit click or hardware binding takes. Writing the value with SimConnect_SetInputEvent
    // doesn't run the aircraft's code, so the A320neo V2 just recomputed it. The preset takes a
    // 0..1 fraction; events reported in percent (e.g. throttle 16.7 = idle) are scaled by 0.01.
    FsBVarId ieSetId_[kChannelCount] = {};
    double ieScale_[kChannelCount] = {};
    int ieSetErrors_ = 0;
    bool ieActive_[kChannelCount] = {};  // replayed in this playback
    double ieSent_[kChannelCount] = {};  // last value sent, in the recorded (native) units
    int ieSinceSend_[kChannelCount] = {};
    int ieSettled_[kChannelCount] = {};  // frames since the sent value last changed
    int ieChecked_[kChannelCount] = {};
    int ieOff_[kChannelCount] = {};
    bool ieNotified_[kChannelCount] = {};  // a subscription message has arrived (logged once)
    int iePollFrames_ = 0;
    int traceFrames_ = 0;
    bool firstInputSet_ = false;
    static constexpr int kInputResendFrames = 15;
    // Every numeric input event the aircraft has (for the setup's scan and profile names).
    std::vector<InputEventInfo> inputEvents_;
    bool inputEventsListed_ = false;

    // The aircraft's control profile (applyProfile) and what follows from it.
    bool haveProfile_ = false;
    AircraftProfile profile_;
    bool profilePending_ = false;  // changed during playback: re-resolve when it ends
    FsLVarId lvId_[kChannelCount] = {};
    FsUnitId numberUnit_ = FS_INVALID_UNIT;
    ControlDriver driver_;
    bool profileDrives_[kControlCount] = {};  // this playback: the profile drives the control
    bool throttleByProfile_ = false;
    bool setupMasking_ = false;  // the setup is driving the throttle simvars: hardware masked
    // the setup's scan: input events (B: vars) then L:vars
    std::vector<FsBVarId> scanB_;
    std::vector<FsLVarId> scanL_;
    int flapPositions_ = 0;
    bool aircraftPathRequested_ = false;

    // playback
    ChannelMask playPresent_;                  // channels the playing clip has
    ChannelMask playSkip_;                     // Set channels not written (covered or yielded)
    int windowFrames_ = 0;                     // rolling override window (yield/throttle)
    int windowOverrides_[kChannelCount] = {};
    int playChannels_[kChannelCount] = {};     // PlayMode::Set channels written every frame
    int playChannelCount_ = 0;
    int restoreChannels_[kChannelCount] = {};  // PlayMode::Set channels (incl. body velocities)
    int restoreChannelCount_ = 0;
    double setBuffer_[kChannelCount] = {};
    double lastSentEvent_[kChannelCount] = {};  // event channel states, to log only real changes
    bool playDefDefined_ = false;
    bool restoreDefDefined_ = false;
    int numEngines_ = 4;
    bool playing_ = false;
    // PlayMode::Snap channels (gear positions) are written for a few frames after play/seek/stop,
    // until the gear command sent alongside has taken over.
    static constexpr int kSnapFrames = 20;  // ~1/3 s at 60 fps
    int snapChannels_[kChannelCount] = {};
    int snapChannelCount_ = 0;
    double snapBuffer_[kChannelCount] = {};
    bool snapDefDefined_ = false;
    int snapFramesLeft_ = 0;
    static constexpr int kSnapCheckFrames = 30;
    int snapCheckFrames_ = 0;  // countdown to checkSnap()
    bool snapRejected_ = false;  // the aircraft moved snapped values straight back; learned per aircraft
    Frame restoreSnap_;  // snapshot to snap back to after a restore
    // Hardware throttle input is masked during playback so the physical lever can't fight it.
    bool masking_ = false;
    int maskedInputs_ = 0;
    // Throttle. By default the lever simvars are written and the hardware throttle events masked.
    // If the aircraft keeps overriding the simvar (its own throttle handling, e.g. 2024 input
    // events), playback switches to sending THROTTLEn_AXIS_SET_EX1, which the aircraft handles
    // like hardware input. Learned per aircraft.
    static constexpr int kOverrideWindowFrames = 30;  // overridden in half of these: switch/yield
    static constexpr int kThrottleResendFrames = 15;  // re-assert an unchanged lever this often
    bool clipHasThrottle_ = false;
    bool throttleViaEvents_ = false;
    bool throttleViaInput_ = false;  // this playback drives the levers through input events
    int yieldedCount_ = 0;
    int32_t throttleSent_[4] = {};
    int throttleSinceSend_[4] = {};
    double throttleTarget_[4] = {};
    int throttleCheckedFrames_ = 0;
    int throttleOffFrames_ = 0;
    bool firstThrottleEvent_ = false;
    double throttleLowerLimit_ = NAN;  // THROTTLE LOWER LIMIT (percent, negative with reversers)
    // The pilot's hardware throttle axis per engine (0..1), from the key events; NaN until it moves
    // (and after a flight load, when the aircraft's levers start from scratch).
    double throttleAxis_[4] = {NAN, NAN, NAN, NAN};
    bool throttleAxisLogged_ = false;
    // Before a jump onto the ground with the gear down, on an aircraft whose gear can't be snapped:
    // lower it where the aircraft is first (see prepareJump).
    static constexpr int64_t kPrepareGearMaxMs = 30000;
    int64_t prepareStartMs_ = 0;
    bool prepareGearSent_ = false;
    bool snapAccepted_ = false;    // a gear snap held on this aircraft: no need to lower it first
    bool restoreDeferred_ = false;  // Stop: lowering the gear before putting the aircraft back
    static double gearExtensionOf(const Frame& f);
    void commandGearDown(const Frame& target);
    void writeRestore(const Frame& snapshot);
    // After a jump, and after Stop puts the aircraft back, a flap lever the profile or an input
    // event drives waits until the aircraft is where the clip has it (on the ground or not, at its
    // speed) for a moment. Aircraft that pick the flap setting from the lever and their speed
    // otherwise see the lever move while still fast: the A320neo V2 then gives CONF 1 (slats
    // only) instead of 1+F on the runway.
    static constexpr int64_t kFlapsSettleMs = 1500;
    static constexpr int64_t kFlapsHoldMaxMs = 5000;
    static constexpr double kFlapsSpeedMatchKt = 20.0;
    bool flapsHold_ = false;            // playback: the flap lever is left where the aircraft has it
    bool restoreFlapsPending_ = false;  // Stop: the profile puts the flaps back once settled
    int64_t flapsHoldStartMs_ = 0;
    int64_t flapsMatchSinceMs_ = 0;
    void holdFlaps();
    bool flapsSettled(const Frame& target);
    void checkRestoreFlaps();
    void cancelRestoreFlaps();
    // Engines (trial): on turbine aircraft playback writes the clip's N1 and N2 every frame, so
    // the engine sound and animation follow the recording instead of what the aircraft's own
    // thrust logic (autothrust holding a detent) makes of the replayed lever. If the aircraft
    // doesn't keep the values, the engines are left to it for the rest of the playback.
    static constexpr double kEngineKeptPct = 2.0;  // read back within this of what was written
    static constexpr int kEngineWindowFrames = 60;
    FsAVarId engineVarId_[2] = {FS_VAR_INVALID_ID, FS_VAR_INVALID_ID};  // TURB ENG N1, N2
    FsUnitId percentUnit_ = FS_INVALID_UNIT;
    bool engineWrite_[2] = {};  // this playback writes N1 / N2
    bool engineWritesRejected_ = false;  // this aircraft kept its own N1: don't write it again
    double engineWritten_[2][4] = {};
    int engineWriteErrors_ = 0;
    int engineWrites_ = 0;      // frames written
    int engineChecked_ = 0;     // frames whose N1 read back was compared with what was written
    int engineKept_ = 0;
    int engineWindowChecked_ = 0;
    int engineWindowOff_ = 0;
    int64_t engineCheckFromMs_ = 0;  // a moment after each jump
    int engineType_ = 1;             // ENGINE TYPE: 0 piston, 1 jet, 3 helo turbine, 5 turboprop
    void beginEngineWrites(const ChannelMask& present);
    void writeEngines(const Frame& frame, bool jumped);
    // Diagnostics for the engine sound: does each engine's N1 follow the clip, or hunt?
    int64_t engineStartMs_ = 0;
    int64_t engineLastMs_ = 0;
    double n1DiffSum_[4] = {};
    double n1DiffMax_[4] = {};
    int n1Frames_[4] = {};
    double n1Last_[4] = {};      // live, then the clip: direction changes of each
    double n1ClipLast_[4] = {};
    int n1Dir_[4] = {};
    int n1ClipDir_[4] = {};
    int n1Reversals_[4] = {};
    int n1ClipReversals_[4] = {};
    void trackEngines(const Frame& frame, bool jumped);
    void logEngines();
    // Event channel diagnostics (gear has its own check): the state each should be in, checked
    // after a grace period following every change, since reversers take a moment to move.
    static constexpr int kEventGraceFrames = 180;  // ~3 s at 60 fps
    double eventWant_[kChannelCount] = {};
    int eventGrace_[kChannelCount] = {};
    int eventChecked_[kChannelCount] = {};
    int eventOff_[kChannelCount] = {};
    char lastTitle_[64] = {};
    // Diagnostics: how often the aircraft overrode what playback wrote, per channel.
    double written_[kChannelCount] = {};
    bool haveWritten_ = false;
    int overrideFrames_[kChannelCount] = {};
    int checkedFrames_ = 0;
    bool firstPlaybackWrite_ = false;
    // Restoring onto the ground: stay frozen until the gear is back where the snapshot had it.
    static constexpr int kRestoreGearMaxFrames = 1200;  // ~20 s at 60 fps
    int restoreWaitFrames_ = 0;
    double restoreGear_[3] = {};
    // After a gear command during playback: did the aircraft's gear actually start moving?
    static constexpr int kGearResponseFrames = 180;  // ~3 s at 60 fps
    int gearCheckFrames_ = 0;
    double gearCheckFrom_ = NAN;
    double gearCheckTarget_ = NAN;
    // Aircraft input events (B: vars) are listed once per flight, to find gear and throttle
    // controls that ignore the legacy key events.
    uint64_t enumerateAtFrame_ = 120;
    int inputEventsLogged_ = 0;
    bool wantFreeze_ = false;
    bool freezeState_[kFreezeCount] = {};
    bool freezeToggled_[kFreezeCount] = {};
    int freezeCheckFrames_ = 0;

    struct Packet {
        DWORD id;
        const char* what;
    };
    static constexpr size_t kPacketHistory = 128;
    Packet packets_[kPacketHistory] = {};
    size_t packetCount_ = 0;
};

}  // namespace br
