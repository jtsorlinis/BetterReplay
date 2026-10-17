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
    void writePlaybackFrame(const Frame& frame) override;
    void sendChannelEvent(int channel, double value) override;
    void endPlayback(const Frame& snapshot, bool restore) override;
    bool restoring() const override { return restoreWaitFrames_ > 0; }

private:
    enum DataDef : DWORD { DEF_RECORD = 1, DEF_TITLE = 2, DEF_PLAYBACK = 3, DEF_RESTORE = 4 };
    enum ClientEvent : DWORD {
        EVT_FREEZE_SET_FIRST = 100,    // + Freeze
        EVT_FREEZE_TOGGLE_FIRST = 110, // + Freeze
        EVT_CHANNEL_FIRST = 200,       // + channel index, for PlayMode::Event channels
        EVT_CHANNEL_ON_FIRST = 300,    // + channel index, ChannelDef::playEventOn
        EVT_CHANNEL_OFF_FIRST = 400,   // + channel index, ChannelDef::playEventOff
    };
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
    enum Extra { kExSimTime, kExSlew, kExTouchdownNormalVel, kExFreezeLatLon, kExFreezeAlt, kExFreezeAtt, kExtraCount };

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
    void onInputEvents(const SIMCONNECT_RECV_ENUMERATE_INPUT_EVENTS* list, DWORD size);
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
    int playChannels_[kChannelCount] = {};     // PlayMode::Set channels present in the clip
    int playChannelCount_ = 0;
    int restoreChannels_[kChannelCount] = {};  // PlayMode::Set channels + body velocities
    int restoreChannelCount_ = 0;
    double setBuffer_[kChannelCount] = {};
    bool playDefDefined_ = false;
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
    // Aircraft input events (B: vars) are listed once per flight, to find gear controls that
    // ignore the legacy key events.
    uint64_t enumerateAtFrame_ = 120;
    int gearInputEventsLogged_ = 0;
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
