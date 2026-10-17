// SimConnect adapter: builds data definitions from the channel table, subscribes to system and
// flow events, and feeds every sim frame into the Engine.
#pragma once

#include <MSFS/MSFS_WindowsTypes.h>
#include <SimConnect.h>

#include "Engine.h"

namespace br {

class SimBridge {
public:
    explicit SimBridge(Engine& engine) : engine_(engine) {}

    bool open();
    void close();
    void update(float dTime);  // from a module update callback

    bool connected() const { return h_ != 0; }

private:
    enum DataDef : DWORD { DEF_RECORD = 1, DEF_TITLE = 2 };
    enum Request : DWORD { REQ_RECORD = 1, REQ_TITLE = 2 };
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

    struct Packet {
        DWORD id;
        const char* what;
    };
    static constexpr size_t kPacketHistory = 128;
    Packet packets_[kPacketHistory] = {};
    size_t packetCount_ = 0;
};

}  // namespace br
