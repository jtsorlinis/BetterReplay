#include "SimBridge.h"

#include <cmath>
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

// Throttle events a hardware lever or keyboard sends (2024 Key Events docs, Throttle section).
// The first four, in engine order, are also mapped under EVT_THROTTLE_AXIS_FIRST to drive the
// levers on aircraft that override the throttle simvar (SimBridge::sendThrottleEvents).
const char* const kMaskedInputEvents[] = {
    "THROTTLE1_AXIS_SET_EX1", "THROTTLE2_AXIS_SET_EX1", "THROTTLE3_AXIS_SET_EX1", "THROTTLE4_AXIS_SET_EX1",
    "THROTTLE_AXIS_SET_EX1",  "THROTTLE_SET",           "THROTTLE1_SET",          "THROTTLE2_SET",
    "THROTTLE3_SET",          "THROTTLE4_SET",          "AXIS_THROTTLE_SET",      "AXIS_THROTTLE1_SET",
    "AXIS_THROTTLE2_SET",     "AXIS_THROTTLE3_SET",     "AXIS_THROTTLE4_SET",     "AXIS_THROTTLE_PLUS",
    "AXIS_THROTTLE_MINUS",
};
constexpr size_t kMaskedInputEventCount = sizeof(kMaskedInputEvents) / sizeof(kMaskedInputEvents[0]);

bool isThrottle(int channel) { return channel >= kThrottle1 && channel <= kThrottle4; }

// Set channels playback stops writing when the aircraft keeps overriding them: flaps, spoilers
// and control surfaces. Position, attitude and velocities are always written (they're frozen or
// drive the instruments); the throttle has its own fallback.
bool canYield(int channel) { return channel >= kFlapsLeft && channel <= kRudder; }

double onOff(double v) { return std::isnan(v) ? NAN : (v >= 0.5 ? 1.0 : 0.0); }

// Counts a change of direction in a series once it has moved 0.05 (% N1) the other way.
void countReversal(double v, double& last, int& dir, int& reversals) {
    if (!std::isfinite(last)) {
        last = v;
        return;
    }
    const double d = v - last;
    if (std::fabs(d) < 0.05) return;
    const int now = d > 0.0 ? 1 : -1;
    if (dir != 0 && now != dir) ++reversals;
    dir = now;
    last = v;
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

    // Recording definition: every simvar channel, then the extras, all as FLOAT64. Input event
    // channels are filled from their subscriptions instead.
    recordCount_ = 0;
    numberUnit_ = fsVarsGetUnitId("number");
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        ieValue_[c] = NAN;
        ieSetId_[c] = FS_VAR_INVALID_ID;
        lvId_[c] = FS_VAR_INVALID_ID;
        if (!ch.simvar) continue;
        recordChannels_[recordCount_++] = c;
        track(SimConnect_AddToDataDefinition(h_, DEF_RECORD, ch.simvar, ch.units, SIMCONNECT_DATATYPE_FLOAT64),
              ch.simvar);
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
        {"NUMBER OF ENGINES", "number"},
        {"THROTTLE LOWER LIMIT", "percent"},
        {"ENGINE TYPE", "enum"},
        {"BRAKE PARKING POSITION", "bool"},
        {"ON ANY RUNWAY", "bool"},
        {"GENERAL ENG COMBUSTION:1", "bool"},
        {"GENERAL ENG COMBUSTION:2", "bool"},
        {"GENERAL ENG COMBUSTION:3", "bool"},
        {"GENERAL ENG COMBUSTION:4", "bool"},
        {"IS GEAR RETRACTABLE", "bool"},
        {"SPOILER AVAILABLE", "bool"},
        {"FLAPS NUM HANDLE POSITIONS", "number"},
    };
    for (const auto& e : kExtras) {
        track(SimConnect_AddToDataDefinition(h_, DEF_RECORD, e.simvar, e.units, SIMCONNECT_DATATYPE_FLOAT64), e.simvar);
    }
    track(SimConnect_AddToDataDefinition(h_, DEF_TITLE, "TITLE", nullptr, SIMCONNECT_DATATYPE_STRING256), "TITLE");
    snapChannelCount_ = buildDefinition(DEF_SNAP, snapDefDefined_, PlayMode::Snap, nullptr, snapChannels_);

    // Hardware throttle input events, masked while a clip plays (see setInputMasking).
    for (size_t i = 0; i < kMaskedInputEventCount; ++i) {
        track(SimConnect_MapClientEventToSimEvent(h_, EVT_MASK_FIRST + static_cast<DWORD>(i), kMaskedInputEvents[i]),
              kMaskedInputEvents[i]);
    }
    // The same axis events under their own IDs for driving the levers: transmitting the IDs
    // that had been in the mask group failed with SimConnect ERROR in the sim.
    for (DWORD e = 0; e < 4; ++e) {
        track(SimConnect_MapClientEventToSimEvent(h_, EVT_THROTTLE_AXIS_FIRST + e, kMaskedInputEvents[e]),
              kMaskedInputEvents[e]);
    }

    // Key events used by playback (names verified against the 2024 Key Events docs).
    static const char* kFreezeSet[kFreezeCount] = {"FREEZE_LATITUDE_LONGITUDE_SET", "FREEZE_ALTITUDE_SET",
                                                   "FREEZE_ATTITUDE_SET"};
    static const char* kFreezeToggle[kFreezeCount] = {"FREEZE_LATITUDE_LONGITUDE_TOGGLE", "FREEZE_ALTITUDE_TOGGLE",
                                                      "FREEZE_ATTITUDE_TOGGLE"};
    for (int i = 0; i < kFreezeCount; ++i) {
        track(SimConnect_MapClientEventToSimEvent(h_, EVT_FREEZE_SET_FIRST + i, kFreezeSet[i]), kFreezeSet[i]);
        track(SimConnect_MapClientEventToSimEvent(h_, EVT_FREEZE_TOGGLE_FIRST + i, kFreezeToggle[i]), kFreezeToggle[i]);
    }
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        if (ch.play != PlayMode::Event) continue;
        if (ch.playEvent) {
            track(SimConnect_MapClientEventToSimEvent(h_, EVT_CHANNEL_FIRST + c, ch.playEvent), ch.playEvent);
        }
        if (ch.playEventOn) {
            track(SimConnect_MapClientEventToSimEvent(h_, EVT_CHANNEL_ON_FIRST + c, ch.playEventOn), ch.playEventOn);
        }
        if (ch.playEventOff) {
            track(SimConnect_MapClientEventToSimEvent(h_, EVT_CHANNEL_OFF_FIRST + c, ch.playEventOff), ch.playEventOff);
        }
    }

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

    ControlDriver::Context ctx;
    ctx.h = h_;
    ctx.live = &input_.raw;
    ctx.engines = &numEngines_;
    ctx.flapPositions = &flapPositions_;
    ctx.throttleLowerLimit = &throttleLowerLimit_;
    ctx.inputEvents = &inputEvents_;
    ctx.clipChannels = &playPresent_;
    driver_.setContext(ctx);
    fsEventsRegisterKeyEventHandler(&SimBridge::keyEventThunk, this);
    requestAircraftPath();

    log::info("SimConnect ready: %d simvar channels + %d extras per frame, %d input event / L:var channels",
              recordCount_, static_cast<int>(kExtraCount), static_cast<int>(kChannelCount) - recordCount_);
    return true;
}

// The aircraft.cfg path identifies the aircraft for its control profile. Asked for after every
// flight load and aircraft change.
void SimBridge::requestAircraftPath() {
    aircraftPathRequested_ = true;
    track(SimConnect_RequestSystemState(h_, REQ_AIRCRAFT_PATH, "AircraftLoaded"), "RequestSystemState AircraftLoaded");
}

void SimBridge::onSystemState(const SIMCONNECT_RECV_SYSTEM_STATE* s) {
    if (s->dwRequestID != REQ_AIRCRAFT_PATH) return;
    aircraftPathRequested_ = false;
    std::string key;
    for (const char* p = s->szString; *p && key.size() < 255; ++p) {
        const char ch = *p == '\\' ? '/' : *p;
        key.push_back(static_cast<char>(ch >= 'A' && ch <= 'Z' ? ch + 32 : ch));
    }
    const std::string cfg = "/aircraft.cfg";
    if (key.size() > cfg.size() && key.compare(key.size() - cfg.size(), cfg.size(), cfg) == 0) key.resize(key.size() - cfg.size());
    if (key.empty()) {
        if (!lastTitle_[0]) return;  // try again once the title is known
        key = std::string("title:") + lastTitle_;
    }
    log::info("aircraft path: %s", s->szString[0] ? s->szString : "(none; using the title)");
    engine_.setAircraftKey(key);
}

void SimBridge::close() {
    if (h_) {
        fsEventsUnregisterKeyEventHandler(&SimBridge::keyEventThunk, this);
        SimConnect_Close(h_);
        h_ = 0;
    }
}

void SimBridge::keyEventThunk(FsEventId id, FsVarParamArray* params, void* ctx) {
    static_cast<SimBridge*>(ctx)->onKeyEvent(id, params);
}

// The hardware throttle arrives as key events. Their values are the pilot's actual lever, which
// the lever simvar isn't on every aircraft (on the A320neo V2 under autothrust it's the thrust
// demand), so they're recorded as the thr_axis_N channels for the key-event throttle method.
void SimBridge::onKeyEvent(FsEventId id, const FsVarParamArray* params) {
    // While playback or the control setup drives the throttle, these are our own events.
    if (playing_ || driver_.active(Control::Throttle)) return;
    struct Mapping {
        int id;
        int engine;  // 1-4, 0 = all engines
        int kind;    // 0: axis -16384..16384, 1: 0..16383, 2: full, 3: cut, 4: unknown (increment)
    };
    static const Mapping kMappings[] = {
        {KEY_THROTTLE_AXIS_SET_EX1, 0, 0},  {KEY_THROTTLE1_AXIS_SET_EX1, 1, 0}, {KEY_THROTTLE2_AXIS_SET_EX1, 2, 0},
        {KEY_THROTTLE3_AXIS_SET_EX1, 3, 0}, {KEY_THROTTLE4_AXIS_SET_EX1, 4, 0}, {KEY_AXIS_THROTTLE_SET, 0, 0},
        {KEY_AXIS_THROTTLE1_SET, 1, 0},     {KEY_AXIS_THROTTLE2_SET, 2, 0},     {KEY_AXIS_THROTTLE3_SET, 3, 0},
        {KEY_AXIS_THROTTLE4_SET, 4, 0},     {KEY_THROTTLE_SET, 0, 1},           {KEY_THROTTLE1_SET, 1, 1},
        {KEY_THROTTLE2_SET, 2, 1},          {KEY_THROTTLE3_SET, 3, 1},          {KEY_THROTTLE4_SET, 4, 1},
        {KEY_THROTTLE_FULL, 0, 2},          {KEY_THROTTLE_FULL_EX1, 0, 2},      {KEY_THROTTLE_CUT, 0, 3},
        {KEY_THROTTLE_CUT_EX1, 0, 3},       {KEY_THROTTLE_INCR, 0, 4},          {KEY_THROTTLE_DECR, 0, 4},
        {KEY_THROTTLE_INCR_SMALL, 0, 4},    {KEY_THROTTLE_DECR_SMALL, 0, 4},    {KEY_THROTTLE_INCREASE_EX1, 0, 4},
        {KEY_THROTTLE_DECREASE_EX1, 0, 4},
    };
    const Mapping* m = nullptr;
    for (const Mapping& k : kMappings) {
        if (k.id == id) m = &k;
    }
    if (!m) return;
    const int32_t raw = params && params->size > 0 && params->array ? static_cast<int32_t>(params->array[0].intValue) : 0;
    double v = NAN;  // increments: the lever's position is no longer known
    switch (m->kind) {
        case 0: v = (raw + 16384.0) / 32768.0; break;
        case 1: v = raw / 16383.0; break;
        case 2: v = 1.0; break;
        case 3: v = 0.0; break;
        default: break;
    }
    if (std::isfinite(v)) v = v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
    for (int e = 0; e < 4; ++e) {
        if (m->engine == 0 || m->engine == e + 1) throttleAxis_[e] = v;
    }
    if (!throttleAxisLogged_) {
        throttleAxisLogged_ = true;
        log::info("throttle hardware: key event %d, value %ld; recording the throttle axis from now on", id,
                  static_cast<long>(raw));
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
                engine_.onFlightLoaded("FlightLoaded event");
                onAircraftChanged();
                resetInputEvents(false);  // the old aircraft's input events are gone; don't poll them
                for (double& a : throttleAxis_) a = NAN;  // the new aircraft's levers start from scratch
                throttleAxisLogged_ = false;
                inputEvents_.clear();
                inputEventsListed_ = false;
                enumerateAtFrame_ = frameCount_ + 120;  // list the new aircraft's input events once it's settled
                requestAircraftPath();
            }
            break;
        }
        case SIMCONNECT_RECV_ID_SYSTEM_STATE:
            onSystemState(reinterpret_cast<const SIMCONNECT_RECV_SYSTEM_STATE*>(p));
            break;
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
        case SIMCONNECT_RECV_ID_ENUMERATE_INPUT_EVENTS:
            onInputEvents(reinterpret_cast<const SIMCONNECT_RECV_ENUMERATE_INPUT_EVENTS*>(p), size);
            break;
        case SIMCONNECT_RECV_ID_GET_INPUT_EVENT: {
            const auto* g = reinterpret_cast<const SIMCONNECT_RECV_GET_INPUT_EVENT*>(p);
            const DWORD c = g->dwRequestID - REQ_INPUT_VALUE_FIRST;
            const size_t offset = reinterpret_cast<const uint8_t*>(&g->Value) - reinterpret_cast<const uint8_t*>(g);
            if (c < static_cast<DWORD>(kChannelCount) && g->eType == SIMCONNECT_INPUT_EVENT_TYPE_DOUBLE &&
                size >= offset + sizeof(double)) {
                std::memcpy(&ieValue_[c], reinterpret_cast<const uint8_t*>(g) + offset, sizeof(double));
            }
            break;
        }
        case SIMCONNECT_RECV_ID_SUBSCRIBE_INPUT_EVENT: {
            const auto* s = reinterpret_cast<const SIMCONNECT_RECV_SUBSCRIBE_INPUT_EVENT*>(p);
            const size_t offset = reinterpret_cast<const uint8_t*>(&s->Value) - reinterpret_cast<const uint8_t*>(s);
            if (s->eType != SIMCONNECT_INPUT_EVENT_TYPE_DOUBLE || size < offset + sizeof(double)) break;
            for (int c = 0; c < kChannelCount; ++c) {
                if (ieHash_[c] != 0 && ieHash_[c] == s->Hash) {
                    std::memcpy(&ieValue_[c], reinterpret_cast<const uint8_t*>(s) + offset, sizeof(double));
                    if (!ieNotified_[c]) {
                        ieNotified_[c] = true;
                        log::debug("input event %s changed to %.4f (subscription works)", ieName_[c], ieValue_[c]);
                    }
                }
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
            if (id >= EVT_MASK_FIRST && id < EVT_MASK_FIRST + kMaskedInputEventCount) ++maskedInputs_;
            break;
    }
}

void SimBridge::onFlowEvent(DWORD flow, const char* path) {
    log::info("flow event %s%s%s", flowName(flow), path && path[0] ? ": " : "", path ? path : "");
    switch (flow) {
        case SIMCONNECT_FLOW_EVENT_FLT_LOADED:
            engine_.onFlightLoaded("FLT_LOADED flow event");
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
    const size_t values = static_cast<size_t>(recordCount_) + kExtraCount;
    const size_t offset = payloadOffset(d);
    if (size < offset + values * sizeof(double)) {
        if (!sizeWarned_) {
            sizeWarned_ = true;
            log::error("recording data too small: %lu bytes, expected %zu", static_cast<unsigned long>(size),
                       offset + values * sizeof(double));
        }
        return;
    }
    double v[kChannelCount + kExtraCount];
    std::memcpy(v, reinterpret_cast<const uint8_t*>(d) + offset, values * sizeof(double));
    for (int i = 0; i < recordCount_; ++i) input_.raw.v[recordChannels_[i]] = v[i];
    for (int c = 0; c < kChannelCount; ++c) {
        if (kChannels[c].play == PlayMode::Input) {
            input_.raw.v[c] = ieValue_[c];
        } else if (kChannels[c].play == PlayMode::LVar) {
            double lv = NAN;
            if (lvId_[c] != FS_VAR_INVALID_ID && fsVarsLVarGet(lvId_[c], numberUnit_, &lv) != FS_VAR_ERROR_NONE) lv = NAN;
            input_.raw.v[c] = lv;
        }
    }
    for (int e = 0; e < 4; ++e) input_.raw.v[kThrottleAxis1 + e] = e < numEngines_ ? throttleAxis_[e] : NAN;
    const double* ex = v + recordCount_;
    input_.simTimeSec = ex[kExSimTime];
    input_.slewActive = ex[kExSlew] >= 0.5;
    input_.touchdownNormalVelFps = ex[kExTouchdownNormalVel];
    input_.fallbackDtSec = pendingDt_;
    pendingDt_ = 0.0f;
    freezeState_[kFreezeLatLon] = ex[kExFreezeLatLon] >= 0.5;
    freezeState_[kFreezeAlt] = ex[kExFreezeAlt] >= 0.5;
    freezeState_[kFreezeAtt] = ex[kExFreezeAtt] >= 0.5;
    const double engines = ex[kExNumEngines];
    if (std::isfinite(engines) && engines >= 0.0 && engines <= 16.0) numEngines_ = static_cast<int>(engines);
    throttleLowerLimit_ = ex[kExThrottleLowerLimit];
    const double flaps = ex[kExFlapPositions];
    flapPositions_ = std::isfinite(flaps) && flaps >= 0.0 && flaps < 64.0 ? static_cast<int>(flaps) : 0;
    SetupFacts& facts = input_.facts;
    facts.engines = numEngines_;
    facts.engineType = std::isfinite(ex[kExEngineType]) ? static_cast<int>(ex[kExEngineType]) : 1;
    facts.parkingBrake = ex[kExParkingBrake] >= 0.5;
    facts.onRunway = ex[kExOnRunway] >= 0.5;
    facts.engineRunning = false;
    for (int e = 0; e < 4 && e < numEngines_; ++e) facts.engineRunning |= ex[kExCombustion1 + e] >= 0.5;
    facts.gearRetractable = ex[kExGearRetractable] >= 0.5;
    facts.hasSpoilers = ex[kExSpoilerAvailable] >= 0.5;
    facts.flapPositions = flapPositions_;
    facts.hasReverse = std::isfinite(throttleLowerLimit_) && throttleLowerLimit_ < -1.0;
    if (freezeCheckFrames_ > 0 && --freezeCheckFrames_ == 0) verifyFreeze();
    if (playChannelCount_ > 0 && haveWritten_) checkOverrides();
    if (playing_ && throttleViaEvents_ && clipHasThrottle_) checkThrottleEvents();
    if (playing_) checkEventChannels();
    if (playing_) checkInputEvents();
    if (playing_ && log::g_debug && ++traceFrames_ >= 60) {
        traceFrames_ = 0;
        tracePlayback();
    }
    pollInputEvents();
    if (!playing_ && profilePending_) reresolveForProfile();  // the profile changed during a replay
    if (snapCheckFrames_ > 0 && --snapCheckFrames_ == 0) checkSnap();
    if (!playing_ && snapFramesLeft_ > 0) writeSnap(restoreSnap_);
    if (restoreWaitFrames_ > 0) checkRestoreGear();
    if (restoreFlapsPending_ && !restoreDeferred_) checkRestoreFlaps();
    if (gearCheckFrames_ > 0 && --gearCheckFrames_ == 0) checkGearResponse();
    if (frameCount_ == enumerateAtFrame_) {
        inputEventsLogged_ = 0;
        resetInputEvents(true);
        track(SimConnect_EnumerateInputEvents(h_, REQ_INPUT_EVENTS), "EnumerateInputEvents");
    }

    if (frameCount_++ == 0) {
        const double* r = input_.raw.v;
        log::info("first data frame: lat %.5f lon %.5f alt %.0f ft, AGL %.0f ft, on ground %d, sim time %.1f",
                  r[kLat], r[kLon], r[kAlt], r[kAgl], r[kOnGround] >= 0.5 ? 1 : 0, input_.simTimeSec);
    }
    engine_.onFrame(input_, utcNowMs());
}

// ---------------------------------------------------------------------------- playback

void SimBridge::sendFreeze(bool on) {
    static const char* kWhat[kFreezeCount] = {"FREEZE_LATITUDE_LONGITUDE_SET", "FREEZE_ALTITUDE_SET",
                                              "FREEZE_ATTITUDE_SET"};
    for (int i = 0; i < kFreezeCount; ++i) {
        freezeToggled_[i] = false;
        track(SimConnect_TransmitClientEvent_EX1(h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, EVT_FREEZE_SET_FIRST + i,
                                                 SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                 SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, on ? 1 : 0),
              kWhat[i]);
    }
    wantFreeze_ = on;
    freezeCheckFrames_ = 20;  // read back IS ... FREEZE ON a few frames later
}

// The *_SET events' parameter isn't documented, so confirm the result and fall back to the
// toggle events if a SET didn't take.
void SimBridge::verifyFreeze() {
    static const char* kNames[kFreezeCount] = {"lat/lon", "altitude", "attitude"};
    bool retry = false;
    for (int i = 0; i < kFreezeCount; ++i) {
        if (freezeState_[i] == wantFreeze_) {
            log::debug("%s freeze %s confirmed", kNames[i], wantFreeze_ ? "on" : "off");
            continue;
        }
        if (!freezeToggled_[i]) {
            log::warn("%s freeze still %s after SET; sending TOGGLE", kNames[i], freezeState_[i] ? "on" : "off");
            track(SimConnect_TransmitClientEvent_EX1(h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, EVT_FREEZE_TOGGLE_FIRST + i,
                                                     SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                     SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, 0),
                  "FREEZE_*_TOGGLE");
            freezeToggled_[i] = true;
            retry = true;
        } else {
            log::error("could not turn %s freeze %s", kNames[i], wantFreeze_ ? "on" : "off");
        }
    }
    if (retry) freezeCheckFrames_ = 20;
}

int SimBridge::buildDefinition(DWORD def, bool& defined, PlayMode mode, const ChannelMask* present, int* channels) {
    // Clearing a definition that was never built raises UNRECOGNIZED_ID, so only clear after the first.
    if (defined) track(SimConnect_ClearDataDefinition(h_, def), "ClearDataDefinition");
    defined = true;
    int count = 0;
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        if (ch.play != mode) continue;
        if (present && !present->test(static_cast<size_t>(c))) continue;
        if (ch.engine > numEngines_) continue;  // e.g. throttle 3/4 on a twin
        channels[count++] = c;
        track(SimConnect_AddToDataDefinition(h_, def, ch.simvar, ch.units, SIMCONNECT_DATATYPE_FLOAT64), ch.simvar);
    }
    return count;
}

void SimBridge::setInputMasking(bool on) {
    if (on == masking_) return;
    if (on) {
        // Events in a maskable group at the highest maskable priority are consumed by this client
        // and never reach the aircraft.
        for (size_t i = 0; i < kMaskedInputEventCount; ++i) {
            track(SimConnect_AddClientEventToNotificationGroup(h_, GROUP_INPUT_MASK,
                                                               EVT_MASK_FIRST + static_cast<DWORD>(i), TRUE),
                  "AddClientEventToNotificationGroup (throttle mask)");
        }
        track(SimConnect_SetNotificationGroupPriority(h_, GROUP_INPUT_MASK, SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE),
              "SetNotificationGroupPriority (throttle mask)");
        maskedInputs_ = 0;
    } else {
        track(SimConnect_ClearNotificationGroup(h_, GROUP_INPUT_MASK), "ClearNotificationGroup (throttle mask)");
        log::info("playback: hardware throttle input released (%d input event(s) were blocked)", maskedInputs_);
    }
    masking_ = on;
}

// Compares what the sim reports this frame with what playback wrote last frame. A channel the
// aircraft keeps overriding is being driven by its own systems (or by hardware input).
void SimBridge::checkOverrides() {
    ++checkedFrames_;
    bool throttleOverridden = false;
    for (int i = 0; i < playChannelCount_; ++i) {
        const int c = playChannels_[i];
        const double w = written_[c];
        const double live = input_.raw.v[c];
        if (!std::isfinite(w) || !std::isfinite(live)) continue;
        const Interp kind = kChannels[c].interp;
        double diff = live - w;
        if (kind == Interp::Angle180 || kind == Interp::Angle360) {
            diff = std::fmod(diff + 540.0, 360.0) - 180.0;
        }
        const double tolerance = 0.02 * (std::fabs(w) > 1.0 ? std::fabs(w) : 1.0);
        if (std::fabs(diff) > tolerance) {
            ++overrideFrames_[c];
            if (isThrottle(c)) {
                throttleOverridden = true;
            } else {
                ++windowOverrides_[c];
            }
        }
    }
    if (throttleOverridden) ++windowOverrides_[kThrottle1];  // any lever counts once per frame

    // Checked in rolling half-second windows rather than once, because an override only shows
    // once the clip's value differs from the aircraft's.
    if (++windowFrames_ < kOverrideWindowFrames) return;
    const int frames = windowFrames_;
    windowFrames_ = 0;
    int throttlePct = 0;
    for (int c = 0; c < kChannelCount; ++c) {
        const int pct = windowOverrides_[c] * 100 / frames;
        windowOverrides_[c] = 0;
        if (c == kThrottle1) throttlePct = pct;
        // Flaps, spoilers and surfaces the aircraft keeps moving itself: stop fighting it.
        if (canYield(c) && pct >= 50 && !playSkip_.test(static_cast<size_t>(c))) yieldChannel(c, pct);
    }
    // If the aircraft keeps overriding the throttle simvar, it handles the levers itself, so
    // drive them the way the hardware does.
    if (!throttleViaEvents_ && clipHasThrottle_ && !throttleViaInput_ && throttlePct >= 50) {
        switchThrottleToEvents(throttlePct);
    }
}

void SimBridge::yieldChannel(int channel, int overriddenPct) {
    log::warn("playback: the aircraft overrode %s (%s) in %d%% of the last %d frames; leaving it to the aircraft "
              "for the rest of this playback",
              kChannels[channel].key, kChannels[channel].simvar, overriddenPct, kOverrideWindowFrames);
    playSkip_.set(static_cast<size_t>(channel));
    ++yieldedCount_;
    rebuildPlaybackDefinition();
}

void SimBridge::rebuildPlaybackDefinition() {
    const ChannelMask set = playPresent_ & ~playSkip_;
    playChannelCount_ = buildDefinition(DEF_PLAYBACK, playDefDefined_, PlayMode::Set, &set, playChannels_);
    firstPlaybackWrite_ = true;
    haveWritten_ = false;  // the next comparison needs a write with the new layout
}

void SimBridge::logOverrides() {
    if (checkedFrames_ >= 30) {
        int reported = 0;
        for (int i = 0; i < playChannelCount_; ++i) {
            const int c = playChannels_[i];
            const int pct = overrideFrames_[c] * 100 / checkedFrames_;
            if (pct < 20) continue;
            ++reported;
            log::warn("playback: %s (%s) was overridden by the aircraft in %d%% of frames", kChannels[c].key,
                      kChannels[c].simvar, pct);
        }
        if (reported == 0) log::info("playback: all %d written channels held their values", playChannelCount_);
    }
    int eventsChecked = 0;
    int eventsOff = 0;
    for (int c = 0; c < kChannelCount; ++c) {
        if (eventChecked_[c] < 30) continue;
        ++eventsChecked;
        const int pct = eventOff_[c] * 100 / eventChecked_[c];
        if (pct < 20) continue;
        ++eventsOff;
        const ChannelDef& ch = kChannels[c];
        log::warn("playback: %s (%s) didn't follow the clip in %d%% of frames; the aircraft seems to ignore %s",
                  ch.key, ch.simvar, pct, ch.playEvent ? ch.playEvent : (ch.playEventOn ? ch.playEventOn : "its events"));
    }
    if (eventsChecked > 0 && eventsOff == 0) {
        log::info("playback: all %d checked light/reverser channels followed the clip", eventsChecked);
    }
    char followed[256] = {};
    for (int c = 0; c < kChannelCount; ++c) {
        if (!ieActive_[c] || ieChecked_[c] < 30) continue;
        const int pct = ieOff_[c] * 100 / ieChecked_[c];
        if (pct >= 20) {
            log::warn("playback: %s (input event %s) didn't hold the clip's value in %d%% of frames (last sent %.4f, "
                      "aircraft reports %.4f)",
                      kChannels[c].key, ieName_[c], pct, ieSent_[c], ieValue_[c]);
        } else {
            const size_t used = std::strlen(followed);
            std::snprintf(followed + used, sizeof(followed) - used, "%s%s", used ? ", " : "", ieName_[c]);
        }
    }
    if (followed[0]) log::info("playback: input events followed the clip: %s", followed);
    if (yieldedCount_ > 0) log::info("playback: %d channel(s) were left to the aircraft (see above)", yieldedCount_);
    if (throttleViaEvents_ && clipHasThrottle_ && throttleCheckedFrames_ >= 30) {
        const int pct = throttleOffFrames_ * 100 / throttleCheckedFrames_;
        if (pct >= 20) {
            log::warn("playback: with axis events the throttle levers were still more than 3%% away from the clip in "
                      "%d%% of frames; this aircraft needs its own throttle variables (future LVar support)",
                      pct);
        } else {
            log::info("playback: throttle levers followed the clip via axis events (%d%% of frames off)", pct);
        }
    }
}

void SimBridge::switchThrottleToEvents(int overriddenPct) {
    log::warn("playback: the aircraft overrode the throttle lever simvar in %d%% of the last 30 frames; driving the levers with "
              "THROTTLEn_AXIS_SET_EX1 events for this aircraft from now on (the hardware lever can't be blocked "
              "that way, so leave it alone while a clip plays)",
              overriddenPct);
    throttleViaEvents_ = true;
    setInputMasking(false);  // the mask would swallow our own throttle events too
    for (int c = kThrottle1; c <= kThrottle4; ++c) playSkip_.set(static_cast<size_t>(c));
    rebuildPlaybackDefinition();
    firstThrottleEvent_ = true;
    throttleCheckedFrames_ = 0;
    throttleOffFrames_ = 0;
    for (int e = 0; e < 4; ++e) throttleSinceSend_[e] = kThrottleResendFrames;  // send on the next frame
}

void SimBridge::sendThrottleEvents(const Frame& frame, bool force) {
    for (int e = 0; e < 4 && e < numEngines_; ++e) {
        const int c = kThrottle1 + e;
        if (!playPresent_.test(static_cast<size_t>(c))) continue;
        const double pct = frame.v[c];
        if (!std::isfinite(pct)) continue;
        // The axis runs -16384..16384. Out of reverse that's idle..full. With the reverser engaged
        // the event sets the reverse amount instead: idle..full reverse (the throttle lower
        // limit). The reverser itself is switched by the rev_N on/off events, so go by the
        // aircraft's live reverser state and hold idle until it matches the clip.
        const double rev = frame.v[kReverser1 + e];
        const bool clipReverse = std::isfinite(rev) ? rev >= 0.5 : pct < 0.0;
        const bool liveReverse = input_.raw.v[kReverser1 + e] >= 0.5;
        const double lower = std::isfinite(throttleLowerLimit_) && throttleLowerLimit_ < -1.0 ? throttleLowerLimit_ : 0.0;
        double fraction = 0.0;
        double target = 0.0;
        if (clipReverse && liveReverse && lower < 0.0) {
            fraction = pct / lower;
            target = pct < lower ? lower : (pct > 0.0 ? 0.0 : pct);
        } else if (!clipReverse && !liveReverse) {
            fraction = pct / 100.0;
            target = pct < 0.0 ? 0.0 : (pct > 100.0 ? 100.0 : pct);
        }
        fraction = fraction < 0.0 ? 0.0 : (fraction > 1.0 ? 1.0 : fraction);
        const int32_t axis = static_cast<int32_t>(std::lround(fraction * 32768.0 - 16384.0));
        throttleTarget_[e] = target;
        if (!force && axis == throttleSent_[e] && ++throttleSinceSend_[e] < kThrottleResendFrames) continue;
        throttleSent_[e] = axis;
        throttleSinceSend_[e] = 0;
        const HRESULT hr = SimConnect_TransmitClientEvent_EX1(
            h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, EVT_THROTTLE_AXIS_FIRST + static_cast<DWORD>(e),
            SIMCONNECT_GROUP_PRIORITY_HIGHEST, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, static_cast<DWORD>(axis));
        if (firstThrottleEvent_) {
            firstThrottleEvent_ = false;
            track(hr, kMaskedInputEvents[e]);
        }
    }
}

// Diagnostics for axis-event mode: does the lever simvar now follow the clip?
void SimBridge::checkThrottleEvents() {
    bool any = false;
    bool off = false;
    for (int e = 0; e < 4 && e < numEngines_; ++e) {
        const double target = throttleTarget_[e];
        const double live = input_.raw.v[kThrottle1 + e];
        if (!std::isfinite(target) || !std::isfinite(live)) continue;
        any = true;
        if (std::fabs(live - target) > 3.0) off = true;
    }
    if (!any) return;
    ++throttleCheckedFrames_;
    if (off) ++throttleOffFrames_;
}

void SimBridge::trackEventChannels(const Frame& frame, bool jumped) {
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        if (ch.play != PlayMode::Event || c == kGearHandle) continue;
        if (!playPresent_.test(static_cast<size_t>(c)) || ch.engine > numEngines_) continue;
        const double want = onOff(frame.v[c]);
        if (jumped || !(want == eventWant_[c])) eventGrace_[c] = kEventGraceFrames;
        eventWant_[c] = want;
    }
}

void SimBridge::checkEventChannels() {
    for (int c = 0; c < kChannelCount; ++c) {
        if (!std::isfinite(eventWant_[c])) continue;
        if (eventGrace_[c] > 0) {
            --eventGrace_[c];
            continue;
        }
        const double live = onOff(input_.raw.v[c]);
        if (std::isnan(live)) continue;
        ++eventChecked_[c];
        if (live != eventWant_[c]) ++eventOff_[c];
    }
}

void SimBridge::beginPlayback(const Clip& clip, const Frame& peak) {
    if (!h_) return;
    const ChannelMask& present = clip.present;
    restoreWaitFrames_ = 0;  // a new clip started while still settling a restore: playback wins
    restoreDeferred_ = false;
    prepareGearSent_ = false;
    snapFramesLeft_ = 0;
    snapCheckFrames_ = 0;
    flapsHold_ = false;
    cancelRestoreFlaps();
    engineStartMs_ = 0;
    engineLastMs_ = 0;
    for (int e = 0; e < 4; ++e) {
        n1DiffSum_[e] = 0.0;
        n1DiffMax_[e] = 0.0;
        n1Frames_[e] = 0;
        n1Last_[e] = NAN;
        n1ClipLast_[e] = NAN;
        n1Dir_[e] = 0;
        n1ClipDir_[e] = 0;
        n1Reversals_[e] = 0;
        n1ClipReversals_[e] = 0;
    }
    if (playing_) driver_.endAll();  // switching clips
    playing_ = true;
    playPresent_ = present;
    playSkip_.reset();
    yieldedCount_ = 0;

    // Controls the aircraft's profile drives: its method, through the driver, instead of the
    // defaults below (or nothing, if the setup found no way). Their channels are left alone here.
    ChannelMask owned;
    throttleByProfile_ = false;
    for (int i = 0; i < kControlCount; ++i) {
        const Control c = controlAt(i);
        profileDrives_[i] = false;
        if (!haveProfile_ || !profile_.drives(c)) continue;
        const ControlProfile& cp = profile_[c];
        std::string why;
        if (cp.state == ControlState::Ok && !clipSupports(c, cp.method, clip, numEngines_, &why)) {
            log::info("playback: the %s's profile method (%s) doesn't fit this clip (%s); using the defaults",
                      controlSpec(c).label, describeMethod(c, cp.method).c_str(), why.c_str());
            continue;
        }
        if (cp.state == ControlState::Ok && !driver_.begin(c, cp.method, why)) {
            log::warn("playback: can't drive the %s with its profile method (%s): %s; using the defaults", controlSpec(c).label,
                      describeMethod(c, cp.method).c_str(), why.c_str());
            continue;
        }
        profileDrives_[i] = true;
        controlChannels(c, owned);
        if (c == Control::Throttle) throttleByProfile_ = true;
        log::info("playback: %s %s (control profile)", controlSpec(c).label,
                  cp.state == ControlState::Ok ? ("via " + describeMethod(c, cp.method)).c_str()
                                               : "left to the aircraft: the setup found no way to replay it");
        if (c == Control::Throttle && cp.state == ControlState::Ok && cp.lever.valid()) {
            // The lever simvar isn't the lever on this aircraft: replay the lever's own variable.
            if (clipHasLever(cp.lever, clip, numEngines_, &why)) {
                driver_.setLever(cp.lever);
            } else {
                log::info("playback: the throttle levers follow %s: %s (recorded before the lever setup?)",
                          kChannels[kThrottle1].simvar, why.c_str());
            }
        }
        // Reverse thrust from the clip's reversers, with or without a lever calibration.
        if (c == Control::Throttle && cp.state == ControlState::Ok) driver_.setReverseBy(cp.reverseBy);
    }
    for (int c = 0; c < kChannelCount; ++c) {
        if (owned.test(static_cast<size_t>(c)) && kChannels[c].play == PlayMode::Set) playSkip_.set(static_cast<size_t>(c));
    }

    // Input events this aircraft has and the clip recorded: replay them, and don't write the
    // simvars they drive.
    throttleViaInput_ = false;
    int inputs = 0;
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        ieActive_[c] = ch.play == PlayMode::Input && present.test(static_cast<size_t>(c)) && ieHash_[c] != 0 &&
                       ch.engine <= numEngines_ && !owned.test(static_cast<size_t>(c));
        ieSinceSend_[c] = kInputResendFrames;
        ieSent_[c] = NAN;
        ieSettled_[c] = 0;
        ieChecked_[c] = 0;
        ieOff_[c] = 0;
        // Presets that take a fraction get values recorded in percent (e.g. throttle: idle 16.7,
        // TOGA 86) scaled by 0.01; the others get the value as recorded.
        ieScale_[c] = ch.inputSetFraction && (peak.v[c] > 1.0001 || std::fabs(ieValue_[c]) > 1.0001) ? 0.01 : 1.0;
        if (!ieActive_[c]) continue;
        ++inputs;
        for (int k = 0; k < ch.coversCount; ++k) playSkip_.set(static_cast<size_t>(ch.coversFirst + k));
        if (ch.coversFirst == kThrottle1 + ch.engine - 1 && ch.engine > 0) throttleViaInput_ = true;
    }
    firstInputSet_ = true;

    // The legacy throttle paths (simvars with masking, axis events) all depend on this.
    clipHasThrottle_ = false;
    for (int e = 0; e < 4 && e < numEngines_ && !throttleByProfile_; ++e) {
        clipHasThrottle_ |= present.test(static_cast<size_t>(kThrottle1 + e));
    }
    if (throttleViaEvents_ || throttleViaInput_) {
        for (int c = kThrottle1; c <= kThrottle4; ++c) playSkip_.set(static_cast<size_t>(c));
    }
    rebuildPlaybackDefinition();
    checkedFrames_ = 0;
    for (int& n : overrideFrames_) n = 0;
    windowFrames_ = 0;
    for (int& n : windowOverrides_) n = 0;
    throttleCheckedFrames_ = 0;
    throttleOffFrames_ = 0;
    firstThrottleEvent_ = true;
    for (int e = 0; e < 4; ++e) {
        throttleTarget_[e] = NAN;
        throttleSinceSend_[e] = kThrottleResendFrames;
    }
    for (int c = 0; c < kChannelCount; ++c) {
        lastSentEvent_[c] = onOff(input_.raw.v[c]);
        eventWant_[c] = NAN;
        eventGrace_[c] = 0;
        eventChecked_[c] = 0;
        eventOff_[c] = 0;
    }
    sendFreeze(true);

    const char* throttle;
    if (throttleByProfile_) {
        setInputMasking(driver_.wantsThrottleMask());
        throttle = driver_.wantsThrottleMask() ? "throttle via the control profile, hardware throttle input blocked"
                                               : "throttle via the control profile";
    } else if (throttleViaInput_) {
        setInputMasking(false);
        throttle = "throttle via the aircraft's input events";
    } else if (!clipHasThrottle_) {
        setInputMasking(false);
        throttle = "no throttle data in this clip (recorded before throttle support), so the levers stay with your "
                   "hardware";
    } else if (throttleViaEvents_) {
        setInputMasking(false);
        throttle = "throttle via axis events (this aircraft overrides the lever simvar)";
    } else {
        setInputMasking(true);
        throttle = "throttle via simvars, hardware throttle input blocked";
    }
    log::info("playback: writing %d channels per frame and %d input event(s) (%d engine(s)); freezing "
              "position/altitude/attitude; %s",
              playChannelCount_, inputs, numEngines_, throttle);
    for (int c = 0; c < kChannelCount; ++c) {
        if (!ieActive_[c]) continue;
        if (ieSetId_[c] != FS_VAR_INVALID_ID) {
            log::debug("playback: replaying %s via %s_SET (x%g)", kChannels[c].key, ieName_[c], ieScale_[c]);
        } else {
            log::debug("playback: replaying %s via SetInputEvent %s", kChannels[c].key, ieName_[c]);
        }
    }
    ieSetErrors_ = 0;
}

void SimBridge::writeSnap(const Frame& frame) {
    if (snapChannelCount_ == 0) {
        snapFramesLeft_ = 0;
        return;
    }
    for (int i = 0; i < snapChannelCount_; ++i) {
        const int c = snapChannels_[i];
        const double v = frame.v[c];
        snapBuffer_[i] = std::isfinite(v) ? v : input_.raw.v[c];
    }
    const HRESULT hr = SimConnect_SetDataOnSimObject(h_, DEF_SNAP, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, 0, 0,
                                                     static_cast<DWORD>(snapChannelCount_ * sizeof(double)), snapBuffer_);
    if (snapFramesLeft_ == kSnapFrames) {
        track(hr, "SetDataOnSimObject (snap gear)");
        log::debug("playback: setting %s %.2f, %s %.2f, %s %.2f", kChannels[snapChannels_[0]].key, snapBuffer_[0],
                   snapChannelCount_ > 1 ? kChannels[snapChannels_[1]].key : "-", snapChannelCount_ > 1 ? snapBuffer_[1] : 0.0,
                   snapChannelCount_ > 2 ? kChannels[snapChannels_[2]].key : "-", snapChannelCount_ > 2 ? snapBuffer_[2] : 0.0);
    }
    if (--snapFramesLeft_ == 0) snapCheckFrames_ = kSnapCheckFrames;
}

// Half a second after the last snap write: did the values stick, or did the aircraft move them
// back? The A320neo V2 does (its gear has its own state), and the snap then only shows as a
// flicker, so stop snapping on this aircraft.
void SimBridge::checkSnap() {
    for (int i = 0; i < snapChannelCount_; ++i) {
        const int c = snapChannels_[i];
        const double live = input_.raw.v[c];
        if (std::isfinite(live) && std::fabs(live - snapBuffer_[i]) > 0.25) {
            snapRejected_ = true;
            log::info("playback: %s went back to %.2f after being set to %.2f; the aircraft keeps its own gear state, "
                      "so the gear won't be snapped on this aircraft again",
                      kChannels[c].key, live, snapBuffer_[i]);
            return;
        }
    }
    snapAccepted_ = true;
    log::debug("playback: snapped values held");
}

double SimBridge::gearExtensionOf(const Frame& f) {
    double sum = 0.0;
    int n = 0;
    for (int c : {static_cast<int>(kGearCenter), static_cast<int>(kGearLeft), static_cast<int>(kGearRight)}) {
        if (std::isfinite(f.v[c])) {
            sum += f.v[c];
            ++n;
        }
    }
    return n > 0 ? sum / n : NAN;
}

void SimBridge::commandGearDown(const Frame& target) {
    if (profileDrives_[static_cast<int>(Control::Gear)]) {
        driver_.apply(Control::Gear, target, true);
    } else {
        sendChannelEvent(kGearHandle, 1.0);
    }
}

// A jump onto the ground with the gear down while the aircraft's gear is up: unless gear
// positions are known to snap on this aircraft, lower the gear first, here, frozen where the
// aircraft is. Lowering it after the jump puts the extending gear into the runway, and on the
// A320neo V2 the gear then stops half way.
bool SimBridge::prepareJump(const Frame& target) {
    if (!h_ || !playing_) return true;
    const double live = liveGearExtension();
    const bool needed = !snapAccepted_ && target.v[kOnGround] >= 0.5 && gearExtensionOf(target) >= 0.95 &&
                        std::isfinite(live) && live < 0.95;
    const int64_t now = utcNowMs();
    if (!needed) {
        if (prepareGearSent_) log::info("playback: gear down after %.1f s; going to the clip", (now - prepareStartMs_) / 1000.0);
        prepareGearSent_ = false;
        return true;
    }
    if (!prepareGearSent_) {
        prepareGearSent_ = true;
        prepareStartMs_ = now;
        log::info("playback: lowering the gear before going to the runway (gear at %.2f)", live);
        commandGearDown(target);
    }
    if (now - prepareStartMs_ >= kPrepareGearMaxMs) {
        log::warn("playback: the gear didn't come down within %d s; going to the clip anyway",
                  static_cast<int>(kPrepareGearMaxMs / 1000));
        prepareGearSent_ = false;
        return true;
    }
    return false;
}

void SimBridge::holdFlaps() {
    const bool driven = profileDrives_[static_cast<int>(Control::Flaps)] || ieActive_[kInputFlaps];
    if (!driven) return;
    flapsHold_ = true;
    flapsHoldStartMs_ = utcNowMs();
    flapsMatchSinceMs_ = 0;
}

// Whether the aircraft has been where `target` is (on the ground or not, at its ground speed)
// long enough for its own systems to see it, or the hold has gone on for too long.
bool SimBridge::flapsSettled(const Frame& target) {
    const int64_t now = utcNowMs();
    const double* live = input_.raw.v;
    const double gs = target.v[kGroundSpeed];
    const bool there = (live[kOnGround] >= 0.5) == (target.v[kOnGround] >= 0.5) &&
                       (!std::isfinite(gs) || !std::isfinite(live[kGroundSpeed]) ||
                        std::fabs(live[kGroundSpeed] - gs) < kFlapsSpeedMatchKt);
    if (!there) {
        flapsMatchSinceMs_ = 0;
    } else if (flapsMatchSinceMs_ == 0) {
        flapsMatchSinceMs_ = now;
    }
    if (flapsMatchSinceMs_ != 0 && now - flapsMatchSinceMs_ >= kFlapsSettleMs) {
        log::debug("flaps: the aircraft settled %.1f s after the jump; moving the flap lever",
                   (now - flapsHoldStartMs_) / 1000.0);
        return true;
    }
    if (now - flapsHoldStartMs_ >= kFlapsHoldMaxMs) {
        log::info("flaps: the aircraft isn't where the clip is after %d s (on ground %d/%d, %.0f/%.0f kt); moving "
                  "the flap lever anyway",
                  static_cast<int>(kFlapsHoldMaxMs / 1000), live[kOnGround] >= 0.5 ? 1 : 0,
                  target.v[kOnGround] >= 0.5 ? 1 : 0, live[kGroundSpeed], gs);
        return true;
    }
    return false;
}

void SimBridge::checkRestoreFlaps() {
    if (!flapsSettled(restoreSnap_)) return;
    restoreFlapsPending_ = false;
    driver_.apply(Control::Flaps, restoreSnap_, true);
    driver_.end(Control::Flaps);
}

void SimBridge::cancelRestoreFlaps() {
    if (!restoreFlapsPending_) return;
    restoreFlapsPending_ = false;
    driver_.end(Control::Flaps);
}

void SimBridge::trackEngines(const Frame& frame, bool jumped) {
    const int64_t now = utcNowMs();
    if (engineStartMs_ == 0) engineStartMs_ = now;
    engineLastMs_ = now;
    for (int e = 0; e < numEngines_ && e < 4; ++e) {
        const double live = input_.raw.v[kEngineN1_1 + e];
        const double clip = frame.v[kEngineN1_1 + e];
        if (!std::isfinite(live) || !std::isfinite(clip)) continue;
        if (jumped) {  // a jump isn't hunting
            n1Last_[e] = NAN;
            n1ClipLast_[e] = NAN;
            n1Dir_[e] = 0;
            n1ClipDir_[e] = 0;
        }
        const double diff = std::fabs(live - clip);
        n1DiffSum_[e] += diff;
        if (diff > n1DiffMax_[e]) n1DiffMax_[e] = diff;
        ++n1Frames_[e];
        countReversal(live, n1Last_[e], n1Dir_[e], n1Reversals_[e]);
        countReversal(clip, n1ClipLast_[e], n1ClipDir_[e], n1ClipReversals_[e]);
    }
}

void SimBridge::logEngines() {
    const double secs = (engineLastMs_ - engineStartMs_) / 1000.0;
    if (secs < 1.0) return;
    for (int e = 0; e < numEngines_ && e < 4; ++e) {
        if (n1Frames_[e] == 0) continue;
        log::info("playback: engine %d N1 was %.1f%% off the clip on average (at most %.1f%%) and changed direction "
                  "%.1f times a second (the clip: %.1f)",
                  e + 1, n1DiffSum_[e] / n1Frames_[e], n1DiffMax_[e], n1Reversals_[e] / secs, n1ClipReversals_[e] / secs);
    }
}

void SimBridge::writePlaybackFrame(const Frame& frame, bool jumped) {
    if (!h_ || !playing_) return;
    if (jumped && !snapRejected_) snapFramesLeft_ = kSnapFrames;
    if (snapFramesLeft_ > 0) writeSnap(frame);
    if (jumped) holdFlaps();
    bool releaseFlaps = false;
    if (flapsHold_ && flapsSettled(frame)) {
        flapsHold_ = false;
        releaseFlaps = true;
        ieSent_[kInputFlaps] = NAN;  // sent on this frame
    }
    sendInputEvents(frame, jumped);
    for (int i = 0; i < kControlCount; ++i) {
        const Control c = controlAt(i);
        if (!profileDrives_[i] || (c == Control::Flaps && flapsHold_)) continue;
        driver_.apply(c, frame, jumped || (c == Control::Flaps && releaseFlaps));
    }
    if (throttleViaEvents_ && clipHasThrottle_) sendThrottleEvents(frame, jumped);
    trackEventChannels(frame, jumped);
    trackEngines(frame, jumped);
    if (playChannelCount_ == 0) return;
    for (int i = 0; i < playChannelCount_; ++i) {
        const int c = playChannels_[i];
        const double v = frame.v[c];
        setBuffer_[i] = std::isnan(v) ? input_.raw.v[c] : v;  // gap in the clip: hold the live value
        written_[c] = setBuffer_[i];
    }
    haveWritten_ = true;
    const HRESULT hr = SimConnect_SetDataOnSimObject(h_, DEF_PLAYBACK, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, 0, 0,
                                                     static_cast<DWORD>(playChannelCount_ * sizeof(double)), setBuffer_);
    if (firstPlaybackWrite_) {
        firstPlaybackWrite_ = false;
        track(hr, "SetDataOnSimObject (playback frame)");
    }
}

void SimBridge::sendChannelEvent(int channel, double value) {
    if (!h_ || channel < 0 || channel >= kChannelCount) return;
    const ChannelDef& ch = kChannels[channel];
    if (ch.engine > numEngines_) return;  // e.g. reverser 3/4 on a twin
    if (channel == kGearHandle && profileDrives_[static_cast<int>(Control::Gear)]) return;  // the profile's method
    const DWORD state = value >= 0.5 ? 1 : 0;
    // Lights: parameter 0 on/off, parameter 1 circuit index (0 = all circuits).
    // GEAR_SET: parameter 0 is 1 = down, 0 = up; parameter 1 is ignored.
    if (ch.playEvent) {
        track(SimConnect_TransmitClientEvent_EX1(h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, EVT_CHANNEL_FIRST + channel,
                                                 SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                 SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, state, 0),
              ch.playEvent);
    }
    const char* extra = state ? ch.playEventOn : ch.playEventOff;
    if (extra) {
        track(SimConnect_TransmitClientEvent_EX1(h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT,
                                                 (state ? EVT_CHANNEL_ON_FIRST : EVT_CHANNEL_OFF_FIRST) + channel,
                                                 SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                 SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, 0),
              extra);
    }
    // Play/seek/stop resend every event channel; only log the ones that actually change.
    const bool changed = lastSentEvent_[channel] != static_cast<double>(state);
    lastSentEvent_[channel] = state;
    const char* name = ch.playEvent ? ch.playEvent : ch.key;
    if (changed) {
        log::info("playback: %s %lu%s%s", name, static_cast<unsigned long>(state), extra ? " + " : "", extra ? extra : "");
    } else {
        log::debug("playback: %s %lu (resync)", name, static_cast<unsigned long>(state));
    }

    if (channel == kGearHandle && changed && playing_) {
        // Watch whether the aircraft's gear follows; some aircraft ignore the legacy gear events.
        gearCheckFrames_ = kGearResponseFrames;
        gearCheckFrom_ = liveGearExtension();
        gearCheckTarget_ = state ? 1.0 : 0.0;
    }
}

double SimBridge::liveGearExtension() const {
    double sum = 0.0;
    int n = 0;
    for (int c : {static_cast<int>(kGearCenter), static_cast<int>(kGearLeft), static_cast<int>(kGearRight)}) {
        if (std::isfinite(input_.raw.v[c])) {
            sum += input_.raw.v[c];
            ++n;
        }
    }
    return n > 0 ? sum / n : NAN;
}

void SimBridge::checkGearResponse() {
    const double now = liveGearExtension();
    if (!std::isfinite(now) || !std::isfinite(gearCheckFrom_)) return;
    const bool alreadyThere = std::fabs(gearCheckFrom_ - gearCheckTarget_) < 0.05;
    const bool moved = std::fabs(now - gearCheckFrom_) > 0.05;
    if (alreadyThere || moved) {
        log::debug("playback: gear responded (%.2f -> %.2f)", gearCheckFrom_, now);
        return;
    }
    log::warn("playback: the aircraft's gear did not move 3 s after GEAR_SET/GEAR_%s (still %.2f). It probably "
              "ignores these events; see the 'aircraft input event' lines logged after the flight loaded.",
              gearCheckTarget_ > 0.5 ? "DOWN" : "UP", now);
}

void SimBridge::onInputEvents(const SIMCONNECT_RECV_ENUMERATE_INPUT_EVENTS* list, DWORD size) {
    const size_t header = sizeof(SIMCONNECT_RECV_ENUMERATE_INPUT_EVENTS) - sizeof(SIMCONNECT_INPUT_EVENT_DESCRIPTOR);
    const size_t fits = size > header ? (size - header) / sizeof(SIMCONNECT_INPUT_EVENT_DESCRIPTOR) : 0;
    const size_t count = list->dwArraySize < fits ? list->dwArraySize : fits;
    static const char* const kInteresting[] = {"GEAR", "THROTTLE", "FLAP", "SPOILER", "SPEED_BRAKE", "REVERS"};
    for (size_t i = 0; i < count; ++i) {
        const SIMCONNECT_INPUT_EVENT_DESCRIPTOR& d = list->rgData[i];
        char name[sizeof(d.Name) + 1] = {};
        std::memcpy(name, d.Name, sizeof(d.Name));
        char upper[sizeof(name)] = {};
        for (size_t k = 0; name[k]; ++k) upper[k] = static_cast<char>(name[k] >= 'a' && name[k] <= 'z' ? name[k] - 32 : name[k]);
        const bool number = d.eType == SIMCONNECT_INPUT_EVENT_TYPE_DOUBLE;
        bool interesting = false;
        for (const char* word : kInteresting) interesting |= std::strstr(upper, word) != nullptr;
        if (interesting) {
            ++inputEventsLogged_;
            log::info("aircraft input event: %s (%s)", name, number ? "number" : "string");
        } else {
            log::debug("aircraft input event: %s (%s)", name, number ? "number" : "string");
        }
        if (number) inputEvents_.push_back({name, upper, d.Hash});
    }
    if (list->dwEntryNumber + 1 >= list->dwOutOf) {
        if (inputEventsLogged_ == 0) log::info("aircraft input events: none for gear, throttle, flaps or spoilers");
        log::info("aircraft input events: %zu numeric in total", inputEvents_.size());
        inputEventsListed_ = true;
        matchInputChannels();
        resolveInputEvents();
        resolveLVarChannels();
        logLVars();
    }
}

// Each input event channel records the input event its control's profile names (if it chose
// one), else the first of the channel's default names the aircraft has.
void SimBridge::matchInputChannels() {
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        if (ch.play != PlayMode::Input) continue;
        ieHash_[c] = 0;
        ieName_[c][0] = '\0';
        std::string wanted;
        for (int i = 0; haveProfile_ && i < kControlCount; ++i) {
            const Control control = controlAt(i);
            const ControlSpec& spec = controlSpec(control);
            const ControlProfile& cp = profile_[control];
            const int e = c - spec.inputChannel;
            if (cp.state != ControlState::Ok || e < 0 || e >= (spec.perEngine ? 4 : 1)) continue;
            if (cp.method.kind != MethodKind::InputSet && cp.method.kind != MethodKind::InputValue) continue;
            if (e > 0 && cp.method.var.find('#') == std::string::npos) continue;  // one variable for all engines
            wanted = engineVar(cp.method.var, e + 1);
            for (char& x : wanted) x = static_cast<char>(x >= 'a' && x <= 'z' ? x - 32 : x);
        }
        for (const InputEventInfo& ie : inputEvents_) {
            bool match = false;
            if (!wanted.empty()) {
                match = ie.upper == wanted;
            } else if (ch.inputEvent) {
                const char* alt = ch.inputEvent;
                while (*alt && !match) {
                    const char* end = std::strchr(alt, '|');
                    const size_t len = end ? static_cast<size_t>(end - alt) : std::strlen(alt);
                    match = ie.upper.size() == len && ie.upper.compare(0, len, alt, len) == 0;
                    alt += len + (end ? 1 : 0);
                }
            }
            if (!match) continue;
            ieHash_[c] = ie.hash;
            std::snprintf(ieName_[c], sizeof(ieName_[c]), "%s", ie.name.c_str());
            break;
        }
        engine_.setChannelVar(c, ieName_[c]);
    }
}

// L:var channels record the L:vars the profile chose, if any: the L:var method's, or the
// throttle lever's own variable when the key events drive a lever the simvar isn't.
void SimBridge::resolveLVarChannels() {
    for (int c = 0; c < kChannelCount; ++c) {
        if (kChannels[c].play != PlayMode::LVar) continue;
        lvId_[c] = FS_VAR_INVALID_ID;
        std::string name;
        for (int i = 0; haveProfile_ && i < kControlCount; ++i) {
            const Control control = controlAt(i);
            const ControlSpec& spec = controlSpec(control);
            const ControlProfile& cp = profile_[control];
            const int e = c - spec.lvarChannel;
            if (cp.state != ControlState::Ok) continue;
            const bool lever = control == Control::Throttle && cp.method.kind == MethodKind::KeyEvent && cp.lever.valid();
            if (cp.method.kind != MethodKind::LVar && !lever) continue;
            const std::string& var = lever ? cp.lever.var : cp.method.var;
            if (e < 0 || e >= (spec.perEngine ? 4 : 1)) continue;
            if (e > 0 && var.find('#') == std::string::npos) continue;
            if (e >= numEngines_ && e > 0) continue;
            name = engineVar(var, e + 1);
        }
        if (!name.empty()) {
            lvId_[c] = fsVarsGetLVarId(name.c_str());
            if (lvId_[c] == FS_VAR_INVALID_ID) {
                log::warn("L:var %s (control profile) not found on this aircraft yet", name.c_str());
            } else {
                log::info("recording %s from L:%s (control profile)", kChannels[c].key, name.c_str());
            }
        }
        engine_.setChannelVar(c, lvId_[c] != FS_VAR_INVALID_ID ? name : std::string());
    }
}

void SimBridge::applyProfile(const AircraftProfile* profile) {
    haveProfile_ = profile != nullptr;
    profile_ = profile ? *profile : AircraftProfile();
    if (playing_) {
        profilePending_ = true;  // the playing clip keeps what it started with
        return;
    }
    reresolveForProfile();
}

void SimBridge::reresolveForProfile() {
    profilePending_ = false;
    resolveLVarChannels();
    if (!inputEventsListed_) return;  // done once the aircraft's input events are listed
    resetInputEvents(true);
    matchInputChannels();
    resolveInputEvents();
}

void SimBridge::resetInputEvents(bool unsubscribe) {
    for (int c = 0; c < kChannelCount; ++c) {
        if (unsubscribe && ieHash_[c] != 0) SimConnect_UnsubscribeInputEvent(h_, ieHash_[c]);
        ieHash_[c] = 0;
        ieSetId_[c] = FS_VAR_INVALID_ID;
        ieName_[c][0] = '\0';
        ieValue_[c] = NAN;
        ieNotified_[c] = false;
    }
}

// Subscriptions report changes; this also reads the values twice a second, in case a change
// doesn't get reported (e.g. one the aircraft makes itself).
void SimBridge::pollInputEvents() {
    if (++iePollFrames_ < 30) return;
    iePollFrames_ = 0;
    for (int c = 0; c < kChannelCount; ++c) {
        if (ieHash_[c] == 0) continue;
        SimConnect_GetInputEvent(h_, REQ_INPUT_VALUE_FIRST + static_cast<DWORD>(c), ieHash_[c]);
    }
}

// After the aircraft's input events are listed: subscribe to the ones we record, and read their
// current values (the subscription only reports changes).
void SimBridge::resolveInputEvents() {
    char found[512] = {};
    char missing[512] = {};
    for (int c = 0; c < kChannelCount; ++c) {
        const ChannelDef& ch = kChannels[c];
        if (ch.play != PlayMode::Input || ch.engine > numEngines_) continue;
        char* list = ieHash_[c] ? found : missing;
        const size_t used = std::strlen(list);
        std::snprintf(list + used, sizeof(found) - used, "%s%s%s%s", used ? ", " : "", ch.key, ieHash_[c] ? "=" : "",
                      ieHash_[c] ? ieName_[c] : "");
        if (!ieHash_[c]) continue;
        track(SimConnect_SubscribeInputEvent(h_, ieHash_[c]), ieName_[c]);
        track(SimConnect_GetInputEvent(h_, REQ_INPUT_VALUE_FIRST + static_cast<DWORD>(c), ieHash_[c]), ieName_[c]);
        char setName[80];
        std::snprintf(setName, sizeof(setName), "%s_SET", ieName_[c]);
        ieSetId_[c] = fsVarsGetBVarId(setName, true);
        if (ieSetId_[c] == FS_VAR_INVALID_ID) {
            log::warn("input event %s has no callable %s preset; replaying it with SimConnect_SetInputEvent instead",
                      ieName_[c], setName);
        } else {
            log::debug("input event %s: replayed through %s", ieName_[c], setName);
        }
    }
    log::info("input event channels: recording %s%s%s", found[0] ? found : "none", missing[0] ? "; not on this aircraft: " : "",
              missing);
}

// Research aid for aircraft that keep their controls in their own L:vars (the A320neo V2): list
// the ones whose names look like gear, throttle, flap or spoiler controls, with their values.
void SimBridge::logLVars() {
    static const char* const kWords[] = {"THROTTLE", "TLA", "THRUST", "LEVER", "FLAP", "SLAT", "GEAR",
                                         "SPOILER", "SPEEDBRAKE", "SPEED_BRAKE", "SPD_BRK", "REVERS", "DETENT"};
    const FsUnitId number = fsVarsGetUnitId("number");
    int total = 0;
    int listed = 0;
    char name[128];
    for (FsLVarId id = 0; id < 50000; ++id) {
        if (fsVarsGetLVarName(id, name, sizeof(name)) != FS_VAR_ERROR_NONE) break;
        ++total;
        char upper[sizeof(name)] = {};
        for (size_t k = 0; name[k] && k < sizeof(name) - 1; ++k) {
            upper[k] = static_cast<char>(name[k] >= 'a' && name[k] <= 'z' ? name[k] - 32 : name[k]);
        }
        bool match = false;
        for (const char* w : kWords) match |= std::strstr(upper, w) != nullptr;
        if (!match) continue;
        double value = NAN;
        fsVarsLVarGet(id, number, &value);
        if (listed++ < 300) log::info("aircraft L:var %s = %.4f", name, value);
    }
    log::info("aircraft L:vars: %d in total, %d look like gear/throttle/flap/spoiler controls%s", total, listed,
              listed > 300 ? " (first 300 listed)" : "");
}

void SimBridge::tracePlayback() {
    char line[512];
    int n = std::snprintf(line, sizeof(line), "trace t=%.1f:", engine_.playback().time());
    for (int c = 0; c < kChannelCount && n < static_cast<int>(sizeof(line)) - 64; ++c) {
        if (!ieActive_[c]) continue;
        n += std::snprintf(line + n, sizeof(line) - n, " %s clip %.2f arg %.4f live %.2f;", kChannels[c].key, ieSent_[c],
                           ieSent_[c] * ieScale_[c], ieValue_[c]);
    }
    const double* r = input_.raw.v;
    std::snprintf(line + n, sizeof(line) - n, " thr_1 %.1f flap_idx %.0f gear_c %.2f spoil_h %.2f rnoz_1 %.0f", r[kThrottle1],
                  r[kFlapsHandleIndex], r[kGearCenter], r[kSpoilersHandle], r[kReverserNozzle1]);
    log::debug("%s", line);
}

void SimBridge::sendInputEvents(const Frame& frame, bool force) {
    for (int c = 0; c < kChannelCount; ++c) {
        if (!ieActive_[c] || (flapsHold_ && c == kInputFlaps)) continue;
        double v = frame.v[c];
        if (!std::isfinite(v)) continue;
        const bool changed = !(std::fabs(v - ieSent_[c]) <= 1e-6 * (std::fabs(v) > 1.0 ? std::fabs(v) : 1.0));
        if (changed) ieSettled_[c] = 0;
        // Re-assert an unchanged value now and then, in case the hardware moved it.
        if (!force && !changed && ++ieSinceSend_[c] < kInputResendFrames) continue;
        ieSent_[c] = v;
        ieSinceSend_[c] = 0;
        if (ieSetId_[c] != FS_VAR_INVALID_ID) {
            if (kChannels[c].inputSetFraction && ieScale_[c] == 1.0 && std::fabs(v) > 1.0001) {
                ieScale_[c] = 0.01;  // the clip turned out to be in percent after all
                log::debug("playback: %s is in percent; scaling by 0.01 for %s_SET", kChannels[c].key, ieName_[c]);
            }
            FsVarParamVariant param;
            param.type = FsVarParamTypeDouble;
            param.doubleValue = v * ieScale_[c];
            FsVarParamArray params;
            params.size = 1;
            params.array = &param;
            const FsVarError err = fsVarsBVarCall(ieSetId_[c], params);
            if (err != FS_VAR_ERROR_NONE && ieSetErrors_++ < 5) {
                log::warn("playback: %s_SET(%.4f) failed (error 0x%x)", ieName_[c], param.doubleValue,
                          static_cast<unsigned>(err));
            }
        } else {
            const HRESULT hr = SimConnect_SetInputEvent(h_, ieHash_[c], sizeof(double), &v);
            if (firstInputSet_) {
                firstInputSet_ = false;
                track(hr, "SetInputEvent");
            }
        }
    }
}

// Diagnostics: once a sent value has been steady for a few frames, does the aircraft report it?
void SimBridge::checkInputEvents() {
    for (int c = 0; c < kChannelCount; ++c) {
        if (!ieActive_[c] || !std::isfinite(ieSent_[c])) continue;
        if (++ieSettled_[c] < 10) continue;
        const double live = ieValue_[c];
        if (!std::isfinite(live)) continue;
        ++ieChecked_[c];
        const double w = ieSent_[c];
        if (std::fabs(live - w) > 0.02 * (std::fabs(w) > 1.0 ? std::fabs(w) : 1.0)) ++ieOff_[c];
    }
}

void SimBridge::onAircraftChanged() {
    if (throttleViaEvents_) log::info("aircraft changed: trying the throttle simvars first again");
    throttleViaEvents_ = false;
    snapRejected_ = false;
    snapAccepted_ = false;
    cancelRestoreFlaps();
}

void SimBridge::endPlayback(const Frame& snapshot, bool restore) {
    if (!h_) return;
    logOverrides();
    logEngines();
    playing_ = false;
    playChannelCount_ = 0;
    snapFramesLeft_ = 0;
    snapCheckFrames_ = 0;
    flapsHold_ = false;
    setInputMasking(false);
    // Profile-driven controls back to where they were, the same way; the flaps once the aircraft
    // is back where it was (see flapsSettled).
    const bool deferFlaps = restore && profileDrives_[static_cast<int>(Control::Flaps)];
    for (int i = 0; i < kControlCount; ++i) {
        const Control c = controlAt(i);
        if (profileDrives_[i] && restore && !(deferFlaps && c == Control::Flaps)) driver_.apply(c, snapshot, true);
        profileDrives_[i] = false;
        if (!(deferFlaps && c == Control::Flaps)) driver_.end(c);
    }
    if (deferFlaps) {
        restoreFlapsPending_ = true;
        flapsHoldStartMs_ = utcNowMs();
        flapsMatchSinceMs_ = 0;
    }
    throttleByProfile_ = false;
    if (!restore) {
        for (bool& a : ieActive_) a = false;
        sendFreeze(false);
        return;
    }
    // Input events back to the aircraft's own values from before playback.
    sendInputEvents(snapshot, true);
    for (bool& a : ieActive_) a = false;
    if (throttleViaEvents_ && clipHasThrottle_) sendThrottleEvents(snapshot, true);
    playPresent_.reset();

    // The gear handle was just put back (GEAR_SET). Snap the gear positions back too, for a few
    // frames, so it doesn't animate from wherever the clip left it. If the aircraft won't take
    // that and it's on the ground, stay frozen until the gear is back down, or it would drop
    // onto its belly. In the air it can finish moving while flying.
    restoreSnap_ = snapshot;
    restoreGear_[0] = snapshot.v[kGearCenter];
    restoreGear_[1] = snapshot.v[kGearLeft];
    restoreGear_[2] = snapshot.v[kGearRight];
    const bool ground = snapshot.v[kOnGround] >= 0.5;
    const double live = liveGearExtension();
    if (ground && !snapAccepted_ && gearExtensionOf(snapshot) >= 0.95 && std::isfinite(live) && live < 0.95) {
        // Going back onto the ground with the gear up: lower it here first (still frozen where the
        // clip left the aircraft), then put the aircraft back; see prepareJump.
        restoreDeferred_ = true;
        restoreWaitFrames_ = kRestoreGearMaxFrames;
        log::info("playback: lowering the gear before putting the aircraft back on the ground (gear at %.2f)", live);
        return;
    }
    writeRestore(snapshot);
    if (!snapRejected_) {
        snapFramesLeft_ = kSnapFrames;
        writeSnap(restoreSnap_);
    }
    if (ground) {
        restoreWaitFrames_ = kRestoreGearMaxFrames;
        checkRestoreGear();
    } else {
        sendFreeze(false);
    }
}

void SimBridge::writeRestore(const Frame& snapshot) {
    restoreChannelCount_ =
        buildDefinition(DEF_RESTORE, restoreDefDefined_, PlayMode::Set, nullptr, restoreChannels_);
    for (int i = 0; i < restoreChannelCount_; ++i) {
        const int c = restoreChannels_[i];
        // The profile puts the flaps back once the aircraft has settled (checkRestoreFlaps).
        const bool flaps = c == kFlapsLeft || c == kFlapsRight || c == kFlapsHandleIndex;
        setBuffer_[i] = restoreFlapsPending_ && flaps && std::isfinite(input_.raw.v[c]) ? input_.raw.v[c] : snapshot.v[c];
    }
    if (restoreFlapsPending_) {
        flapsHoldStartMs_ = utcNowMs();  // from when the aircraft is put back
        flapsMatchSinceMs_ = 0;
    }
    track(SimConnect_SetDataOnSimObject(h_, DEF_RESTORE, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, 0, 0,
                                        static_cast<DWORD>(restoreChannelCount_ * sizeof(double)), setBuffer_),
          "SetDataOnSimObject (restore snapshot)");
    log::info("playback: restored aircraft to lat %.5f lon %.5f alt %.0f ft", snapshot.v[kLat], snapshot.v[kLon],
              snapshot.v[kAlt]);
}

void SimBridge::checkRestoreGear() {
    static const int kGear[3] = {kGearCenter, kGearLeft, kGearRight};
    bool settled = true;
    for (int i = 0; i < 3; ++i) {
        const double target = restoreGear_[i];
        const double live = input_.raw.v[kGear[i]];
        if (std::isfinite(target) && std::isfinite(live) && std::fabs(live - target) > 0.02) settled = false;
    }
    const bool timedOut = !settled && --restoreWaitFrames_ <= 0;
    if (restoreDeferred_) {
        // The gear was being lowered before putting the aircraft back on the ground (endPlayback).
        if (!settled && !timedOut) return;
        if (timedOut) log::warn("playback: the gear didn't come down within 20 s; putting the aircraft back anyway");
        restoreDeferred_ = false;
        writeRestore(restoreSnap_);
        restoreWaitFrames_ = 10;  // a few frames for the position to land before unfreezing
        return;
    }
    if (settled) {
        if (restoreWaitFrames_ < kRestoreGearMaxFrames) log::info("playback: gear back in place; unfreezing");
    } else if (!timedOut) {
        if (restoreWaitFrames_ == kRestoreGearMaxFrames - 1) log::info("playback: waiting for the gear before unfreezing");
        return;
    } else {
        log::warn("playback: gear did not return to its pre-playback position within 20 s; unfreezing anyway");
    }
    restoreWaitFrames_ = 0;
    sendFreeze(false);
}

// ---------------------------------------------------------------------------- control setup

void SimBridge::scanBegin(std::vector<ScanVar>& vars) {
    vars.clear();
    scanB_.clear();
    scanL_.clear();
    if (!inputEventsListed_) log::warn("setup scan: the aircraft's input events aren't listed yet");
    int readable = 0;
    for (const InputEventInfo& ie : inputEvents_) {
        vars.push_back({ie.name, false});
        scanB_.push_back(fsVarsGetBVarId(ie.name.c_str(), false));
        double v = NAN;
        if (scanB_.back() != FS_VAR_INVALID_ID && fsVarsBVarGet(scanB_.back(), numberUnit_, &v) == FS_VAR_ERROR_NONE) {
            ++readable;
        }
    }
    char name[128];
    for (FsLVarId id = 0; id < 50000 && scanL_.size() < 8000; ++id) {
        if (fsVarsGetLVarName(id, name, sizeof(name)) != FS_VAR_ERROR_NONE) break;
        vars.push_back({name, true});
        scanL_.push_back(id);
    }
    log::info("setup scan: %zu input event(s), %d of them readable, and %zu L:var(s)", scanB_.size(), readable,
              scanL_.size());
}

void SimBridge::scanRead(std::vector<double>& values) {
    values.resize(scanB_.size() + scanL_.size());
    size_t i = 0;
    for (const FsBVarId id : scanB_) {
        double v = NAN;
        if (id == FS_VAR_INVALID_ID || fsVarsBVarGet(id, numberUnit_, &v) != FS_VAR_ERROR_NONE) v = NAN;
        values[i++] = v;
    }
    for (const FsLVarId id : scanL_) {
        double v = NAN;
        if (fsVarsLVarGet(id, numberUnit_, &v) != FS_VAR_ERROR_NONE) v = NAN;
        values[i++] = v;
    }
}

void SimBridge::scanEnd() {
    std::vector<FsBVarId>().swap(scanB_);
    std::vector<FsLVarId>().swap(scanL_);
}

bool SimBridge::beginControl(Control c, const ControlMethod& m, std::string& why) {
    if (playing_) {
        why = "a replay is playing";
        return false;
    }
    cancelRestoreFlaps();
    if (!driver_.begin(c, m, why)) return false;
    if (c == Control::Throttle && driver_.wantsThrottleMask()) {
        setInputMasking(true);  // the simvars only hold with the hardware throttle blocked
        setupMasking_ = true;
    }
    return true;
}

void SimBridge::applyControl(Control c, const Frame& f, bool force) {
    if (!playing_) driver_.apply(c, f, force);
}

void SimBridge::endControl(Control c) {
    driver_.restoreLVars(c);
    driver_.end(c);
    if (c == Control::Throttle && setupMasking_) {
        setupMasking_ = false;
        setInputMasking(false);
    }
}

double SimBridge::readVar(const ScanVar& v) {
    double out = NAN;
    if (v.lvar) {
        const FsLVarId id = fsVarsGetLVarId(v.name.c_str());
        if (id == FS_VAR_INVALID_ID || fsVarsLVarGet(id, numberUnit_, &out) != FS_VAR_ERROR_NONE) out = NAN;
    } else {
        const FsBVarId id = fsVarsGetBVarId(v.name.c_str(), false);
        if (id == FS_VAR_INVALID_ID || fsVarsBVarGet(id, numberUnit_, &out) != FS_VAR_ERROR_NONE) out = NAN;
    }
    return out;
}

void SimBridge::setReverser(bool on) {
    for (int e = 0; e < 4 && e < numEngines_; ++e) sendChannelEvent(kReverser1 + e, on ? 1.0 : 0.0);
}

void SimBridge::throttleDecrease() {
    if (!playing_) driver_.throttleDecrease();
}

void SimBridge::onTitleData(const SIMCONNECT_RECV_SIMOBJECT_DATA* d, DWORD size) {
    const size_t offset = payloadOffset(d);
    if (size <= offset) return;
    const char* s = reinterpret_cast<const char*>(d) + offset;
    const size_t maxLen = size - offset < 256 ? size - offset : 256;
    size_t len = 0;
    while (len < maxLen && s[len]) ++len;
    const size_t keep = len < sizeof(lastTitle_) - 1 ? len : sizeof(lastTitle_) - 1;
    const bool changed = std::strncmp(lastTitle_, s, keep) != 0 || lastTitle_[keep] != '\0';
    if (lastTitle_[0] && changed) onAircraftChanged();
    std::memcpy(lastTitle_, s, keep);
    lastTitle_[keep] = '\0';
    engine_.setAircraftTitle(std::string(s, len));
    if (changed && !aircraftPathRequested_) requestAircraftPath();  // a new aircraft: its profile key
}

}  // namespace br
