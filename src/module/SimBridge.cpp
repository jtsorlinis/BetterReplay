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

    // Restore definition: everything playback writes, plus body velocities so the aircraft
    // continues where it was when playback started.
    for (int c = 0; c < kChannelCount; ++c) {
        const bool velocity = c >= kVelBodyX && c <= kRotVelBodyZ;
        if (kChannels[c].play != PlayMode::Set && !velocity) continue;
        restoreChannels_[restoreChannelCount_++] = c;
        track(SimConnect_AddToDataDefinition(h_, DEF_RESTORE, kChannels[c].simvar, kChannels[c].units,
                                             SIMCONNECT_DATATYPE_FLOAT64),
              kChannels[c].simvar);
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
        if (ch.play != PlayMode::Event || !ch.playEvent) continue;
        track(SimConnect_MapClientEventToSimEvent(h_, EVT_CHANNEL_FIRST + c, ch.playEvent), ch.playEvent);
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
                engine_.onFlightLoaded("FlightLoaded event");
                enumerateAtFrame_ = frameCount_ + 120;  // list the new aircraft's input events once it's settled
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
        case SIMCONNECT_RECV_ID_ENUMERATE_INPUT_EVENTS:
            onInputEvents(reinterpret_cast<const SIMCONNECT_RECV_ENUMERATE_INPUT_EVENTS*>(p), size);
            break;
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
    freezeState_[kFreezeLatLon] = v[kChannelCount + kExFreezeLatLon] >= 0.5;
    freezeState_[kFreezeAlt] = v[kChannelCount + kExFreezeAlt] >= 0.5;
    freezeState_[kFreezeAtt] = v[kChannelCount + kExFreezeAtt] >= 0.5;
    if (freezeCheckFrames_ > 0 && --freezeCheckFrames_ == 0) verifyFreeze();
    if (restoreWaitFrames_ > 0) checkRestoreGear();
    if (gearCheckFrames_ > 0 && --gearCheckFrames_ == 0) checkGearResponse();
    if (frameCount_ == enumerateAtFrame_) {
        gearInputEventsLogged_ = 0;
        track(SimConnect_EnumerateInputEvents(h_, REQ_INPUT_EVENTS), "EnumerateInputEvents");
    }

    if (frameCount_++ == 0) {
        log::info("first data frame: lat %.5f lon %.5f alt %.0f ft, AGL %.0f ft, on ground %d, sim time %.1f",
                  v[kLat], v[kLon], v[kAlt], v[kAgl], v[kOnGround] >= 0.5 ? 1 : 0, input_.simTimeSec);
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

void SimBridge::beginPlayback(const ChannelMask& present) {
    if (!h_) return;
    restoreWaitFrames_ = 0;  // a new clip started while still settling a restore: playback wins
    // The definition is rebuilt per clip (channels the clip has). Clearing one that was never
    // defined raises UNRECOGNIZED_ID, so only clear after the first build.
    if (playDefDefined_) track(SimConnect_ClearDataDefinition(h_, DEF_PLAYBACK), "ClearDataDefinition (playback)");
    playDefDefined_ = true;
    playChannelCount_ = 0;
    for (int c = 0; c < kChannelCount; ++c) {
        if (kChannels[c].play != PlayMode::Set || !present.test(static_cast<size_t>(c))) continue;
        playChannels_[playChannelCount_++] = c;
        track(SimConnect_AddToDataDefinition(h_, DEF_PLAYBACK, kChannels[c].simvar, kChannels[c].units,
                                             SIMCONNECT_DATATYPE_FLOAT64),
              kChannels[c].simvar);
    }
    firstPlaybackWrite_ = true;
    sendFreeze(true);
    log::info("playback: writing %d channels per frame; freezing position/altitude/attitude", playChannelCount_);
}

void SimBridge::writePlaybackFrame(const Frame& frame) {
    if (!h_ || playChannelCount_ == 0) return;
    for (int i = 0; i < playChannelCount_; ++i) {
        const int c = playChannels_[i];
        const double v = frame.v[c];
        setBuffer_[i] = std::isnan(v) ? input_.raw.v[c] : v;  // gap in the clip: hold the live value
    }
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
    const DWORD state = value >= 0.5 ? 1 : 0;
    // Lights: parameter 0 on/off, parameter 1 circuit index (0 = all circuits).
    // GEAR_SET: parameter 0 is 1 = down, 0 = up; parameter 1 is ignored.
    track(SimConnect_TransmitClientEvent_EX1(h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, EVT_CHANNEL_FIRST + channel,
                                             SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                             SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, state, 0),
          ch.playEvent);
    const char* extra = state ? ch.playEventOn : ch.playEventOff;
    if (extra) {
        track(SimConnect_TransmitClientEvent_EX1(h_, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT,
                                                 (state ? EVT_CHANNEL_ON_FIRST : EVT_CHANNEL_OFF_FIRST) + channel,
                                                 SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                 SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, 0),
              extra);
    }
    log::info("playback: %s %lu%s%s", ch.playEvent, static_cast<unsigned long>(state), extra ? " + " : "",
              extra ? extra : "");

    if (channel == kGearHandle && playChannelCount_ > 0) {
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
    for (size_t i = 0; i < count; ++i) {
        const SIMCONNECT_INPUT_EVENT_DESCRIPTOR& d = list->rgData[i];
        char name[sizeof(d.Name) + 1] = {};
        std::memcpy(name, d.Name, sizeof(d.Name));
        char upper[sizeof(name)] = {};
        for (size_t k = 0; name[k]; ++k) upper[k] = static_cast<char>(name[k] >= 'a' && name[k] <= 'z' ? name[k] - 32 : name[k]);
        if (std::strstr(upper, "GEAR")) {
            ++gearInputEventsLogged_;
            log::info("aircraft input event: %s (%s)", name,
                      d.eType == SIMCONNECT_INPUT_EVENT_TYPE_DOUBLE ? "number" : "string");
        }
    }
    if (list->dwEntryNumber + 1 >= list->dwOutOf && gearInputEventsLogged_ == 0) {
        log::info("aircraft input events: none with GEAR in the name");
    }
}

void SimBridge::endPlayback(const Frame& snapshot, bool restore) {
    if (!h_) return;
    playChannelCount_ = 0;
    if (!restore) {
        sendFreeze(false);
        return;
    }
    for (int i = 0; i < restoreChannelCount_; ++i) setBuffer_[i] = snapshot.v[restoreChannels_[i]];
    track(SimConnect_SetDataOnSimObject(h_, DEF_RESTORE, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, 0, 0,
                                        static_cast<DWORD>(restoreChannelCount_ * sizeof(double)), setBuffer_),
          "SetDataOnSimObject (restore snapshot)");
    log::info("playback: restored aircraft to lat %.5f lon %.5f alt %.0f ft", snapshot.v[kLat], snapshot.v[kLon],
              snapshot.v[kAlt]);

    // The gear handle was just put back (GEAR_SET) and the sim animates the gear from wherever the
    // clip left it. On the ground, stay frozen until it's back down, or the aircraft would drop
    // onto its belly. In the air it can finish moving while flying.
    restoreGear_[0] = snapshot.v[kGearCenter];
    restoreGear_[1] = snapshot.v[kGearLeft];
    restoreGear_[2] = snapshot.v[kGearRight];
    if (snapshot.v[kOnGround] >= 0.5) {
        restoreWaitFrames_ = kRestoreGearMaxFrames;
        checkRestoreGear();
    } else {
        sendFreeze(false);
    }
}

void SimBridge::checkRestoreGear() {
    static const int kGear[3] = {kGearCenter, kGearLeft, kGearRight};
    bool settled = true;
    for (int i = 0; i < 3; ++i) {
        const double target = restoreGear_[i];
        const double live = input_.raw.v[kGear[i]];
        if (std::isfinite(target) && std::isfinite(live) && std::fabs(live - target) > 0.02) settled = false;
    }
    if (settled) {
        if (restoreWaitFrames_ < kRestoreGearMaxFrames) log::info("playback: gear back in place; unfreezing");
    } else if (--restoreWaitFrames_ > 0) {
        if (restoreWaitFrames_ == kRestoreGearMaxFrames - 1) log::info("playback: waiting for the gear before unfreezing");
        return;
    } else {
        log::warn("playback: gear did not return to its pre-playback position within 20 s; unfreezing anyway");
    }
    restoreWaitFrames_ = 0;
    sendFreeze(false);
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
