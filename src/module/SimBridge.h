// SimConnect adapter: builds data definitions from the channel table, subscribes to system and
// flow events, feeds every sim frame into the Engine, and drives the aircraft during playback.
#pragma once

#include <MSFS/MSFS_WindowsTypes.h>
#include <SimConnect.h>

#include <cmath>

#include "Engine.h"

namespace br {

class SimBridge : public ISimOutput {
public:
    explicit SimBridge(Engine& engine) : engine_(engine) {}

    bool open();
    void close();
    void update(float dTime);  // from a module update callback

    bool connected() const { return h_ != 0; }

    // ISimOutput
    void beginPlayback(const ChannelMask& present) override;
    void writePlaybackFrame(const Frame& frame, bool jumped) override;
    void sendChannelEvent(int channel, double value) override;
    void endPlayback(const Frame& snapshot, bool restore) override;
    bool restoring() const override { return restoreWaitFrames_ > 0; }

private:
    enum DataDef : DWORD { DEF_RECORD = 1, DEF_TITLE = 2, DEF_PLAYBACK = 3, DEF_RESTORE = 4, DEF_SNAP = 5 };
    enum ClientEvent : DWORD {
        EVT_FREEZE_SET_FIRST = 100,    // + Freeze
        EVT_FREEZE_TOGGLE_FIRST = 110, // + Freeze
        EVT_CHANNEL_FIRST = 200,       // + channel index, for PlayMode::Event channels
        EVT_CHANNEL_ON_FIRST = 300,    // + channel index, ChannelDef::playEventOn
        EVT_CHANNEL_OFF_FIRST = 400,   // + channel index, ChannelDef::playEventOff
        EVT_MASK_FIRST = 500,          // + index into kMaskedInputEvents; the first four
                                       // (THROTTLE1..4_AXIS_SET_EX1) also drive the levers
    };
    enum Group : DWORD { GROUP_INPUT_MASK = 1 };
    enum Freeze { kFreezeLatLon, kFreezeAlt, kFreezeAtt, kFreezeCount };
    enum Request : DWORD { REQ_RECORD = 1, REQ_TITLE = 2, REQ_INPUT_EVENTS = 3 };
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
        kExtraCount
    };

    static void CALLBACK dispatchThunk(SIMCONNECT_RECV* data, DWORD size, void* ctx);
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

    // playback
    ChannelMask playPresent_;                  // channels the playing clip has
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
    bool snapCheckPending_ = false;
    Frame restoreSnap_;  // snapshot to snap back to after a restore
    // Hardware throttle input is masked during playback so the physical lever can't fight it.
    bool masking_ = false;
    int maskedInputs_ = 0;
    // Throttle. By default the lever simvars are written and the hardware throttle events masked.
    // If the aircraft keeps overriding the simvar (its own throttle handling, e.g. 2024 input
    // events), playback switches to sending THROTTLEn_AXIS_SET_EX1, which the aircraft handles
    // like hardware input. Learned per aircraft.
    static constexpr int kThrottleWindowFrames = 30;  // overridden in half of these: switch
    static constexpr int kThrottleResendFrames = 15;  // re-assert an unchanged lever this often
    bool clipHasThrottle_ = false;
    bool throttleViaEvents_ = false;
    int throttleWindowFrames_ = 0;
    int throttleWindowOverrides_ = 0;
    int32_t throttleSent_[4] = {};
    int throttleSinceSend_[4] = {};
    double throttleTarget_[4] = {};
    int throttleCheckedFrames_ = 0;
    int throttleOffFrames_ = 0;
    bool firstThrottleEvent_ = false;
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
