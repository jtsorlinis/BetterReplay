#include "SimBridge.h"

#include <cstdint>
#include <cstring>

#include "Log.h"
#include "Platform.h"

namespace br {

namespace {

const char* exceptionName(DWORD e) {
    static const char* kNames[] = {
        "NONE", "ERROR", "SIZE_MISMATCH", "UNRECOGNIZED_ID", "UNOPENED", "VERSION_MISMATCH", "TOO_MANY_GROUPS",
        "NAME_UNRECOGNIZED", "TOO_MANY_EVENT_NAMES", "EVENT_ID_DUPLICATE", "TOO_MANY_MAPS", "TOO_MANY_OBJECTS",
        "TOO_MANY_REQUESTS", "WEATHER_INVALID_PORT", "WEATHER_INVALID_METAR", "WEATHER_UNABLE_TO_GET_OBSERVATION",
        "WEATHER_UNABLE_TO_CREATE_STATION", "WEATHER_UNABLE_TO_REMOVE_STATION", "INVALID_DATA_TYPE",
        "INVALID_DATA_SIZE", "DATA_ERROR", "INVALID_ARRAY", "CREATE_OBJECT_FAILED", "LOAD_FLIGHTPLAN_FAILED",
        "OPERATION_INVALID_FOR_OBJECT_TYPE", "ILLEGAL_OPERATION", "ALREADY_SUBSCRIBED", "INVALID_ENUM",
        "DEFINITION_ERROR", "DUPLICATE_ID", "DATUM_ID", "OUT_OF_BOUNDS", "ALREADY_CREATED",
        "OBJECT_OUTSIDE_REALITY_BUBBLE", "OBJECT_CONTAINER", "OBJECT_AI", "OBJECT_ATC", "OBJECT_SCHEDULE",
        "JETWAY_DATA", "ACTION_NOT_FOUND", "NOT_AN_ACTION", "INCORRECT_ACTION_PARAMS", "GET_INPUT_EVENT_FAILED",
        "SET_INPUT_EVENT_FAILED", "EVENT_NAME_RESERVED", "INTERNAL", "CAMERA_API",
    };
    return e < sizeof(kNames) / sizeof(kNames[0]) ? kNames[e] : "?";
}

const char* flowName(DWORD f) {
    static const char* kNames[] = {
        "NONE", "FLT_LOAD", "FLT_LOADED", "TELEPORT_START", "TELEPORT_DONE", "BACK_ON_TRACK_START",
        "BACK_ON_TRACK_DONE", "SKIP_START", "SKIP_DONE", "BACK_TO_MAIN_MENU", "RTC_START", "RTC_END",
        "REPLAY_START", "REPLAY_END", "FLIGHT_START", "FLIGHT_END", "PLANE_CRASH",
    };
    return f < sizeof(kNames) / sizeof(kNames[0]) ? kNames[f] : "?";
}

// Byte offset of the variable-length payload inside SIMCONNECT_RECV_SIMOBJECT_DATA.
size_t payloadOffset(const SIMCONNECT_RECV_SIMOBJECT_DATA* d) {
    return static_cast<size_t>(reinterpret_cast<const uint8_t*>(&d->dwData) - reinterpret_cast<const uint8_t*>(d));
}

}  // namespace

void SimBridge::track(HRESULT hr, const char* what) {
    if (FAILED(hr)) {
        log::error("SimConnect call failed (0x%08lx): %s", static_cast<unsigned long>(hr), what);
        return;
    }
    DWORD id = 0;
    if (SUCCEEDED(SimConnect_GetLastSentPacketID(h_, &id))) {
        packets_[packetCount_ % kPacketHistory] = {id, what};
        ++packetCount_;
    }
}

const char* SimBridge::describe(DWORD sendId) const {
    for (size_t i = 0; i < kPacketHistory && i < packetCount_; ++i) {
        if (packets_[i].id == sendId && packets_[i].what) return packets_[i].what;
    }
    return "unknown call";
}

bool SimBridge::open() {
    HRESULT hr = SimConnect_Open(&h_, "BetterReplay", nullptr, 0, 0, 0);
    if (FAILED(hr)) {
        log::error("SimConnect_Open failed (0x%08lx)", static_cast<unsigned long>(hr));
        h_ = 0;
        return false;
    }

    // Recording definition: every channel, then the extras, all as FLOAT64.
    for (const ChannelDef& c : kChannels) {
        track(SimConnect_AddToDataDefinition(h_, DEF_RECORD, c.simvar, c.units, SIMCONNECT_DATATYPE_FLOAT64), c.simvar);
    }
    static const struct {
        const char* simvar;
        const char* units;
    } kExtras[kExtraCount] = {
        {"SIMULATION TIME", "seconds"},
        {"IS SLEW ACTIVE", "bool"},
        {"PLANE TOUCHDOWN NORMAL VELOCITY", "feet per second"},
        {"IS LATITUDE LONGITUDE FREEZE ON", "bool"},
        {"IS ALTITUDE FREEZE ON", "bool"},
        {"IS ATTITUDE FREEZE ON", "bool"},
    };
    for (const auto& e : kExtras) {
        track(SimConnect_AddToDataDefinition(h_, DEF_RECORD, e.simvar, e.units, SIMCONNECT_DATATYPE_FLOAT64), e.simvar);
    }
    track(SimConnect_AddToDataDefinition(h_, DEF_TITLE, "TITLE", nullptr, SIMCONNECT_DATATYPE_STRING256), "TITLE");

    track(SimConnect_RequestDataOnSimObject(h_, REQ_RECORD, DEF_RECORD, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT,
                                            SIMCONNECT_PERIOD_SIM_FRAME),
          "request recording data (every sim frame)");
    track(SimConnect_RequestDataOnSimObject(h_, REQ_TITLE, DEF_TITLE, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT,
                                            SIMCONNECT_PERIOD_SECOND, SIMCONNECT_DATA_REQUEST_FLAG_CHANGED),
          "request TITLE");

    static const struct {
        DWORD id;
        const char* name;
    } kSystemEvents[] = {
        {EVT_FLIGHT_LOADED, "FlightLoaded"}, {EVT_SIM, "Sim"},         {EVT_SIM_START, "SimStart"},
        {EVT_SIM_STOP, "SimStop"},           {EVT_PAUSE_EX1, "Pause_EX1"}, {EVT_CRASHED, "Crashed"},
        {EVT_CRASH_RESET, "CrashReset"},     {EVT_POSITION_CHANGED, "PositionChanged"},
    };
    for (const auto& e : kSystemEvents) track(SimConnect_SubscribeToSystemEvent(h_, e.id, e.name), e.name);
    track(SimConnect_SubscribeToFlowEvent(h_), "SubscribeToFlowEvent");

    // In a WASM module this registers the dispatch procedure; if no messages show up, update()
    // falls back to polling it every frame.
    hr = SimConnect_CallDispatch(h_, &SimBridge::dispatchThunk, this);
    if (FAILED(hr)) log::error("SimConnect_CallDispatch failed (0x%08lx)", static_cast<unsigned long>(hr));

    log::info("SimConnect ready: %d channels + %d extras per frame", static_cast<int>(kChannelCount),
              static_cast<int>(kExtraCount));
    return true;
}

void SimBridge::close() {
    if (h_) {
        SimConnect_Close(h_);
        h_ = 0;
    }
}

void SimBridge::update(float dTime) {
    if (!h_) return;
    pendingDt_ += dTime;
    sinceOpen_ += dTime;
    sinceHeartbeat_ += dTime;
    if (sinceHeartbeat_ >= 60.0f) {
        sinceHeartbeat_ = 0.0f;
        heartbeat();
    }
    if (pollDispatch_) {
        SimConnect_CallDispatch(h_, &SimBridge::dispatchThunk, this);
    } else if (dispatchCount_ == 0 && sinceOpen_ > 2.0f) {
        pollDispatch_ = true;
        log::warn("no SimConnect messages 2 s after init; polling SimConnect_CallDispatch every frame");
    }
}

void SimBridge::heartbeat() {
    log::info("status: %s%s%s, phase %s, buffer %.0f s, %llu frames received, %zu clip(s)",
              engine_.recording() ? "recording" : "suspended", engine_.recording() ? "" : " - ",
              engine_.recording() ? "" : engine_.suspendReason(), engine_.detector().phase(), engine_.bufferedSeconds(),
              static_cast<unsigned long long>(frameCount_), engine_.index().size());
}

void CALLBACK SimBridge::dispatchThunk(SIMCONNECT_RECV* data, DWORD size, void* ctx) {
    static_cast<SimBridge*>(ctx)->dispatch(data, size);
}

void SimBridge::dispatch(SIMCONNECT_RECV* p, DWORD size) {
    ++dispatchCount_;
    switch (p->dwID) {
        case SIMCONNECT_RECV_ID_OPEN: {
            const auto* o = reinterpret_cast<const SIMCONNECT_RECV_OPEN*>(p);
            log::info("connected to %s %lu.%lu (SimConnect %lu.%lu)", o->szApplicationName,
                      static_cast<unsigned long>(o->dwApplicationVersionMajor),
                      static_cast<unsigned long>(o->dwApplicationVersionMinor),
                      static_cast<unsigned long>(o->dwSimConnectVersionMajor),
                      static_cast<unsigned long>(o->dwSimConnectVersionMinor));
            break;
        }
        case SIMCONNECT_RECV_ID_EXCEPTION: {
            const auto* e = reinterpret_cast<const SIMCONNECT_RECV_EXCEPTION*>(p);
            log::warn("SimConnect exception %s (%lu) from \"%s\", parameter %lu", exceptionName(e->dwException),
                      static_cast<unsigned long>(e->dwException), describe(e->dwSendID),
                      static_cast<unsigned long>(e->dwIndex));
            break;
        }
        case SIMCONNECT_RECV_ID_EVENT: {
            const auto* e = reinterpret_cast<const SIMCONNECT_RECV_EVENT*>(p);
            onSystemEvent(e->uEventID, e->dwData);
            break;
        }
        case SIMCONNECT_RECV_ID_EVENT_FILENAME: {
            const auto* e = reinterpret_cast<const SIMCONNECT_RECV_EVENT_FILENAME*>(p);
            if (e->uEventID == EVT_FLIGHT_LOADED) {
                log::info("FlightLoaded: %s", e->szFileName);
                engine_.onFlightLoaded("FlightLoaded");
            }
            break;
        }
        case SIMCONNECT_RECV_ID_FLOW_EVENT: {
            const auto* f = reinterpret_cast<const SIMCONNECT_RECV_FLOW_EVENT*>(p);
            onFlowEvent(static_cast<DWORD>(f->FlowEvent), f->FltPath);
            break;
        }
        case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
            const auto* d = reinterpret_cast<const SIMCONNECT_RECV_SIMOBJECT_DATA*>(p);
            if (d->dwRequestID == REQ_RECORD) {
                onRecordData(d, size);
            } else if (d->dwRequestID == REQ_TITLE) {
                onTitleData(d, size);
            }
            break;
        }
        case SIMCONNECT_RECV_ID_QUIT:
            log::info("SimConnect quit");
            break;
        default:
            break;
    }
}

void SimBridge::onSystemEvent(DWORD id, DWORD data) {
    switch (id) {
        case EVT_SIM:
            engine_.setSimRunning(data != 0);
            break;
        case EVT_SIM_START:
            engine_.setSimRunning(true);
            break;
        case EVT_SIM_STOP:
            engine_.setSimRunning(false);
            break;
        case EVT_PAUSE_EX1:
            engine_.setPauseFlags(static_cast<uint32_t>(data));
            break;
        case EVT_CRASHED:
            engine_.onRelocated("crashed");
            break;
        case EVT_CRASH_RESET:
            engine_.onRelocated("crash reset");
            break;
        case EVT_POSITION_CHANGED:
            engine_.onRelocated("position changed");
            break;
        default:
            break;
    }
}

void SimBridge::onFlowEvent(DWORD flow, const char* path) {
    log::info("flow event %s%s%s", flowName(flow), path && path[0] ? ": " : "", path ? path : "");
    switch (flow) {
        case SIMCONNECT_FLOW_EVENT_FLT_LOADED:
            engine_.onFlightLoaded("flight loaded");
            engine_.setAtMainMenu(false);
            break;
        case SIMCONNECT_FLOW_EVENT_FLIGHT_START:
            engine_.setAtMainMenu(false);
            break;
        case SIMCONNECT_FLOW_EVENT_BACK_TO_MAIN_MENU:
        case SIMCONNECT_FLOW_EVENT_FLIGHT_END:
            engine_.setAtMainMenu(true);
            break;
        case SIMCONNECT_FLOW_EVENT_TELEPORT_START:
        case SIMCONNECT_FLOW_EVENT_BACK_ON_TRACK_START:
        case SIMCONNECT_FLOW_EVENT_SKIP_START:
            engine_.setRelocating(true);
            break;
        case SIMCONNECT_FLOW_EVENT_TELEPORT_DONE:
        case SIMCONNECT_FLOW_EVENT_BACK_ON_TRACK_DONE:
        case SIMCONNECT_FLOW_EVENT_SKIP_DONE:
            engine_.setRelocating(false);
            engine_.onRelocated(flowName(flow));
            break;
        case SIMCONNECT_FLOW_EVENT_REPLAY_START:
        case SIMCONNECT_FLOW_EVENT_REPLAY_END:
            simReplay_ = flow == SIMCONNECT_FLOW_EVENT_REPLAY_START;
            engine_.setSimReplayActive(simReplay_ || rtc_);
            break;
        case SIMCONNECT_FLOW_EVENT_RTC_START:
        case SIMCONNECT_FLOW_EVENT_RTC_END:
            rtc_ = flow == SIMCONNECT_FLOW_EVENT_RTC_START;
            engine_.setSimReplayActive(simReplay_ || rtc_);
            break;
        case SIMCONNECT_FLOW_EVENT_PLANE_CRASH:
            engine_.onRelocated("plane crash");
            break;
        default:
            break;
    }
}

void SimBridge::onRecordData(const SIMCONNECT_RECV_SIMOBJECT_DATA* d, DWORD size) {
    constexpr size_t kValues = kChannelCount + kExtraCount;
    const size_t offset = payloadOffset(d);
    if (size < offset + kValues * sizeof(double)) {
        if (!sizeWarned_) {
            sizeWarned_ = true;
            log::error("recording data too small: %lu bytes, expected %zu", static_cast<unsigned long>(size),
                       offset + kValues * sizeof(double));
        }
        return;
    }
    double v[kValues];
    std::memcpy(v, reinterpret_cast<const uint8_t*>(d) + offset, sizeof(v));
    for (int c = 0; c < kChannelCount; ++c) input_.raw.v[c] = v[c];
    input_.simTimeSec = v[kChannelCount + kExSimTime];
    input_.slewActive = v[kChannelCount + kExSlew] >= 0.5;
    input_.touchdownNormalVelFps = v[kChannelCount + kExTouchdownNormalVel];
    input_.fallbackDtSec = pendingDt_;
    pendingDt_ = 0.0f;

    if (frameCount_++ == 0) {
        log::info("first data frame: lat %.5f lon %.5f alt %.0f ft, AGL %.0f ft, on ground %d, sim time %.1f",
                  v[kLat], v[kLon], v[kAlt], v[kAgl], v[kOnGround] >= 0.5 ? 1 : 0, input_.simTimeSec);
    }
    engine_.onFrame(input_, utcNowMs());
}

void SimBridge::onTitleData(const SIMCONNECT_RECV_SIMOBJECT_DATA* d, DWORD size) {
    const size_t offset = payloadOffset(d);
    if (size <= offset) return;
    const char* s = reinterpret_cast<const char*>(d) + offset;
    const size_t maxLen = size - offset < 256 ? size - offset : 256;
    size_t len = 0;
    while (len < maxLen && s[len]) ++len;
    engine_.setAircraftTitle(std::string(s, len));
}

}  // namespace br
