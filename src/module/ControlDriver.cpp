#include "ControlDriver.h"

#include <MSFS/MSFS_Events.h>

#include <cstdlib>
#include <cstring>

#include "Log.h"
#include "Platform.h"

namespace br {

namespace {

const int kThrottleAxisKeys[4] = {KEY_THROTTLE1_AXIS_SET_EX1, KEY_THROTTLE2_AXIS_SET_EX1, KEY_THROTTLE3_AXIS_SET_EX1,
                                  KEY_THROTTLE4_AXIS_SET_EX1};
const int kReverseOnKeys[4] = {KEY_SET_THROTTLE1_REVERSE_THRUST_ON, KEY_SET_THROTTLE2_REVERSE_THRUST_ON,
                               KEY_SET_THROTTLE3_REVERSE_THRUST_ON, KEY_SET_THROTTLE4_REVERSE_THRUST_ON};
const int kReverseOffKeys[4] = {KEY_SET_THROTTLE1_REVERSE_THRUST_OFF, KEY_SET_THROTTLE2_REVERSE_THRUST_OFF,
                                KEY_SET_THROTTLE3_REVERSE_THRUST_OFF, KEY_SET_THROTTLE4_REVERSE_THRUST_OFF};
constexpr int kEngageRetryFrames = 30;
constexpr int kLeverGapFrames = 30;  // a clip without its lever value this long: from the simvar

double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

std::string upper(const std::string& s) {
    std::string out = s;
    for (char& ch : out) ch = static_cast<char>(ch >= 'a' && ch <= 'z' ? ch - 32 : ch);
    return out;
}

}  // namespace

bool ControlDriver::due(Slot& s, int i, double v, bool force) {
    const double prev = s.sent[i];
    const bool changed = !(std::fabs(v - prev) <= 1e-6 * (std::fabs(v) > 1.0 ? std::fabs(v) : 1.0));
    if (!force && !changed && ++s.since[i] < kResendFrames) return false;
    s.sent[i] = v;
    s.since[i] = 0;
    ++s.sends;
    return true;
}

bool ControlDriver::dueOnChange(Slot& s, int i, double v, bool force, double minStep) {
    if (!force && std::isfinite(s.sent[i]) && std::fabs(v - s.sent[i]) < minStep) return false;
    s.sent[i] = v;
    s.since[i] = 0;
    ++s.sends;
    return true;
}

bool ControlDriver::dueWhenLasting(Slot& s, int i, double v, bool force, double minStep, double nowStep,
                                   int64_t lastingMs, int64_t nowMs) {
    if (force || !std::isfinite(s.sent[i]) || std::fabs(v - s.sent[i]) >= nowStep) {
        s.lastingFromMs[i] = 0;
        return dueOnChange(s, i, v, true, 0.0);
    }
    if (std::fabs(v - s.sent[i]) < minStep) {
        s.lastingFromMs[i] = 0;
        return false;
    }
    if (s.lastingFromMs[i] == 0) s.lastingFromMs[i] = nowMs;
    if (nowMs - s.lastingFromMs[i] < lastingMs) return false;
    s.lastingFromMs[i] = 0;
    return dueOnChange(s, i, v, true, 0.0);
}

bool ControlDriver::begin(Control c, const ControlMethod& m, std::string& why) {
    const int i = static_cast<int>(c);
    extra_[i].clear();
    ControlProfile one;
    one.method = m;
    picker_[i].begin(c, one);
    return beginSlot(c, slots_[i], m, why);
}

bool ControlDriver::beginWays(Control c, const ControlProfile& cp, std::string& why) {
    if (c == Control::Throttle || !cp.perDirection()) return begin(c, cp.method, why);
    const int ci = static_cast<int>(c);
    WayPicker& p = picker_[ci];
    p.begin(c, cp);
    extra_[ci].clear();
    std::vector<Slot> slots;
    for (size_t i = 0; i < p.ways().size();) {
        Slot s;
        std::string whyNot;
        if (beginSlot(c, s, p.ways()[i], whyNot)) {
            slots.push_back(s);
            ++i;
            continue;
        }
        log::warn("controls: can't drive the %s with %s: %s; leaving it out", controlSpec(c).label,
                  describeMethod(c, p.ways()[i]).c_str(), whyNot.c_str());
        if (!p.drop(i)) {
            why = "no way left one way (" + whyNot + ")";
            return false;
        }
    }
    slots_[ci] = slots.front();
    extra_[ci].assign(slots.begin() + 1, slots.end());
    for (int d = 0; d < kDirections; ++d) {
        std::string list;
        for (const int w : p.order(static_cast<Direction>(d))) list += (list.empty() ? "" : ", then ") + describeMethod(c, p.ways()[w]);
        log::info("controls: the %s %s: %s", controlSpec(c).label, directionKey(c, static_cast<Direction>(d)), list.c_str());
    }
    return true;
}

bool ControlDriver::beginSlot(Control c, Slot& s, const ControlMethod& m, std::string& why) {
    if (number_ == FS_INVALID_UNIT) number_ = fsVarsGetUnitId("number");
    s = Slot();
    s.method = m;
    const ControlSpec& spec = controlSpec(c);
    const int engines = ctx_.engines ? (*ctx_.engines < 1 ? 1 : (*ctx_.engines > 4 ? 4 : *ctx_.engines)) : 1;
    const bool pattern = m.var.find('#') != std::string::npos;
    s.count = spec.perEngine && (pattern || m.kind == MethodKind::SimVar || m.kind == MethodKind::KeyEvent) ? engines : 1;

    switch (m.kind) {
        case MethodKind::None:
            why = "no method";
            return false;
        case MethodKind::InputSet:
            for (int e = 0; e < s.count; ++e) {
                const std::string name = engineVar(m.var, e + 1) + "_SET";
                s.setId[e] = fsVarsGetBVarId(name.c_str(), true);
                if (s.setId[e] == FS_VAR_INVALID_ID) {
                    why = "the aircraft has no " + name + " preset";
                    return false;
                }
            }
            break;
        case MethodKind::InputValue:
            for (int e = 0; e < s.count; ++e) {
                const std::string name = upper(engineVar(m.var, e + 1));
                bool found = false;
                for (size_t i = 0; ctx_.inputEvents && i < ctx_.inputEvents->size() && !found; ++i) {
                    const InputEventInfo& ie = (*ctx_.inputEvents)[i];
                    if (ie.upper == name) {
                        s.hash[e] = ie.hash;
                        found = true;
                    }
                }
                if (!found) {
                    why = "the aircraft has no input event " + name;
                    return false;
                }
            }
            break;
        case MethodKind::LVar:
            for (int e = 0; e < s.count; ++e) {
                const std::string name = engineVar(m.var, e + 1);
                s.lvar[e] = fsVarsGetLVarId(name.c_str());
                if (s.lvar[e] == FS_VAR_INVALID_ID) {
                    why = "the aircraft has no L:var " + name;
                    return false;
                }
                if (fsVarsLVarGet(s.lvar[e], number_, &s.original[e]) != FS_VAR_ERROR_NONE) s.original[e] = NAN;
            }
            break;
        case MethodKind::SimVar: {
            if (spec.simvarFirst < 0) {
                why = "no simvars to write";
                return false;
            }
            const int n = spec.perEngine ? s.count : spec.simvarCount;
            for (int i = 0; i < n && i < 4; ++i) {
                const int ch = spec.simvarFirst + i;
                const ChannelDef& def = kChannels[ch];
                std::string name = def.simvar;
                unsigned index = 0;
                const size_t colon = name.find(':');
                if (colon != std::string::npos) {
                    index = static_cast<unsigned>(std::atoi(name.c_str() + colon + 1));
                    name.resize(colon);
                }
                SimVarTarget& t = s.simvars[s.simvarCount++];
                t.channel = ch;
                t.id = fsVarsGetAVarId(name.c_str());
                t.unit = fsVarsGetUnitId(def.units);
                t.index = index;
                if (t.id == FS_VAR_INVALID_ID || t.id == FS_VAR_AVAR_NONE || t.unit == FS_INVALID_UNIT) {
                    why = std::string("can't write ") + def.simvar;
                    return false;
                }
            }
            break;
        }
        case MethodKind::KeyEvent:
            if (isOwnEvent(m)) {
                if (c == Control::Throttle) {
                    why = "an event by its number doesn't drive the throttle";
                    return false;
                }
                if (!parseEventMethod(m, s.event)) {
                    why = "can't read the event " + m.var;
                    return false;
                }
                // The aircraft's own go through SimConnect, mapped by number; sim key events are
                // triggered directly.
                const uint32_t ids[2] = {s.event.id, s.event.mode == EventMethod::Mode::Steps ? s.event.lowerId : 0u};
                for (int i = 0; i < 2; ++i) {
                    if (ids[i] < kThirdPartyEventFirst) continue;
                    s.eventClient[i] = ownEventId("#" + std::to_string(ids[i]));
                    if (s.eventClient[i] == 0) {
                        why = "can't map the event #" + std::to_string(ids[i]);
                        return false;
                    }
                }
                break;
            }
            if (c == Control::Flaps && m.var == kKeyFlapsNotches) notches_.begin(utcNowMs());
            if (c == Control::Flaps && m.var != kKeyFlapsNotches && (!ctx_.flapPositions || *ctx_.flapPositions < 2)) {
                why = "FLAPS_SET needs the number of flap positions";
                return false;
            }
            break;
    }
    s.active = true;
    for (int i = 0; i < 4; ++i) s.since[i] = kResendFrames;
    log::info("controls: driving the %s with %s", spec.label, describeMethod(c, m).c_str());
    return true;
}

void ControlDriver::setLever(const LeverCalibration& cal) {
    Slot& s = slots_[static_cast<int>(Control::Throttle)];
    if (!s.active || s.method.kind != MethodKind::KeyEvent || !cal.valid()) return;
    s.lever = cal;
    s.useLever = true;
    log::info("controls: throttle levers from L:var %s (%zu calibration point(s)%s)", cal.var.c_str(), cal.forward.size(),
              cal.reverse.empty() ? "" : ", reverse too");
}

void ControlDriver::findAutothrust(const std::vector<Frame>& clipFrames) {
    Slot& s = slots_[static_cast<int>(Control::Throttle)];
    clearAutothrust();
    if (!s.active || !s.useLever) return;
    size_t stretches = 0;
    double seconds = 0.0;
    for (int e = 0; e < s.count; ++e) {
        s.autothrust[e] = autothrustStretches(s.lever, clipFrames, e);
        if (e > 0) continue;
        for (const AutothrustStretch& a : s.autothrust[e]) {
            ++stretches;
            seconds += a.endT - a.startT;
        }
    }
    if (stretches > 0) {
        log::info("controls: autothrust in %zu stretch(es) of the clip (%.0f s): the throttle follows its thrust demand "
                  "there, the levers no further forward than the clip's",
                  stretches, seconds);
    }
}

void ControlDriver::clearAutothrust() {
    Slot& s = slots_[static_cast<int>(Control::Throttle)];
    for (int e = 0; e < 4; ++e) {
        s.autothrust[e].clear();
        s.follower[e].reset();
    }
    s.followMs = 0;
}

bool ControlDriver::inAutothrust(const Slot& s, int e, double t) {
    for (const AutothrustStretch& a : s.autothrust[e]) {
        if (t >= a.startT && t <= a.endT) return true;
    }
    return false;
}

void ControlDriver::setReverseBy(ReverseBy by) {
    Slot& s = slots_[static_cast<int>(Control::Throttle)];
    if (!s.active || s.method.kind != MethodKind::KeyEvent) return;
    s.reverseBy = by;
    if (by == ReverseBy::Hold) log::info("controls: reverse thrust by holding THROTTLE_DECR while the clip's reversers are out");
    if (by == ReverseBy::Events) log::info("controls: reverse thrust with the reverse thrust events");
    if (by == ReverseBy::Toggle) log::info("controls: reverse thrust with toggle reverse while the clip's reversers are out");
}

void ControlDriver::throttleDecrease() { keyEvent(KEY_THROTTLE_DECR, 0); }

void ControlDriver::toggleReverse() { keyEvent(KEY_THROTTLE_REVERSE_THRUST_TOGGLE, 0); }

bool ControlDriver::reverseToggled() const {
    const Slot& s = slots_[static_cast<int>(Control::Throttle)];
    return s.active && s.reverseBy == ReverseBy::Toggle && s.toggle.busy();
}

double ControlDriver::throttleOff(const Frame& clip) const {
    const Slot& s = slots_[static_cast<int>(Control::Throttle)];
    if (!s.active || s.method.kind != MethodKind::KeyEvent || isOwnEvent(s.method) || !ctx_.live) return NAN;
    if (s.holding || s.toggle.busy()) return NAN;
    if (!s.useLever && s.reverseBy == ReverseBy::Axis) return NAN;
    const Frame& live = *ctx_.live;
    double off = NAN;
    for (int e = 0; e < s.count; ++e) {
        // Reverse, by what the reversers did, or the flags (not on a "hold" aircraft, where the
        // clip's flag isn't its reverse).
        if (s.engaged[e] || reversing(clip, e) || reversing(live, e)) return NAN;
        if (s.reverseBy != ReverseBy::Hold && (clip.v[kReverser1 + e] >= 0.5 || live.v[kReverser1 + e] >= 0.5)) return NAN;
        if (!std::isfinite(s.sent[e])) continue;
        double at = NAN;
        if (s.useLever) {
            const double lv = live.v[kLVarThrottle1 + (s.lever.var.find('#') != std::string::npos ? e : 0)];
            if (!leverAxis(s.lever, lv, 0.0, false, at)) continue;
        } else {
            const double pct = live.v[kThrottle1 + e];
            if (!std::isfinite(pct)) continue;
            at = pct / 100.0;
        }
        const double gap = std::fabs(at - (s.sent[e] + 16384.0) / 32768.0);
        if (!(gap <= off)) off = gap;
    }
    return off;
}

void ControlDriver::forgetReverse() {
    slots_[static_cast<int>(Control::Throttle)].toggle.forget();
    notches_.forget();
}

bool ControlDriver::flapsSentTo(const Frame& f) const {
    const int ci = static_cast<int>(Control::Flaps);
    const auto notched = [](const Slot& s) {
        return s.method.kind == MethodKind::KeyEvent && s.method.var == kKeyFlapsNotches;
    };
    bool notches = slots_[ci].active && notched(slots_[ci]);
    for (const Slot& s : extra_[ci]) notches |= notched(s);
    return !notches || notches_.at(std::round(f.v[kFlapsHandleIndex]));
}

namespace {

// "the gear went down", "the flaps retracted" (and "didn't go down", "didn't retract").
std::string movedText(Control c, Direction d, bool did) {
    const std::string label = controlSpec(c).label;
    const bool out = d == Direction::Extend;
    if (c == Control::Gear) return "the gear " + std::string(did ? "went " : "didn't go ") + (out ? "down" : "up");
    return "the " + label + (did ? (out ? " extended" : " retracted") : (out ? " didn't extend" : " didn't retract"));
}

}  // namespace

void ControlDriver::apply(Control c, const Frame& f, bool force) {
    const int ci = static_cast<int>(c);
    Slot& first = slots_[ci];
    if (!first.active) return;
    WayPicker& p = picker_[ci];
    if (!p.multi() || !ctx_.live) {
        applySlot(c, first, f, force);
        return;
    }
    // Each direction's ways: the picker's, which goes on to the next one that way if the control
    // hasn't moved after a while. The one that moved it is logged, and where it wasn't first, the
    // profile gets it first from now on.
    const WayPicker::Pick pick = p.step(eventTarget(c, f), *ctx_.live);
    auto way = [&](int w) { return describeMethod(c, p.ways()[static_cast<size_t>(w)]); };
    if (pick.gaveUp >= 0) {
        log::info("controls: %s with %s within %d frames; now %s", movedText(c, pick.dir, false).c_str(), way(pick.gaveUp).c_str(),
                  EventStepper::waitFrames(c), way(pick.way).c_str());
    }
    if (pick.worked >= 0 && p.order(pick.dir).size() > 1) {
        log::info("controls: %s with %s%s", movedText(c, pick.dir, true).c_str(), way(pick.worked).c_str(),
                  pick.promoted ? " (not the first way: first from now on)" : "");
    }
    if (pick.promoted && ctx_.wayWorked) ctx_.wayWorked(ctx_.wayCtx, c, pick.dir, p.ways()[static_cast<size_t>(pick.worked)]);
    Slot& s = pick.way <= 0 ? first : extra_[ci][static_cast<size_t>(pick.way - 1)];
    applySlot(c, s, f, force || pick.fresh);
}

void ControlDriver::applySlot(Control c, Slot& s, const Frame& f, bool force) {
    const ControlMethod& m = s.method;
    switch (m.kind) {
        case MethodKind::InputSet:
            for (int e = 0; e < s.count; ++e) {
                const double v = f.v[methodSourceChannel(c, m.kind, e)];
                if (!std::isfinite(v) || !due(s, e, v, force)) continue;
                FsVarParamVariant param;
                param.type = FsVarParamTypeDouble;
                param.doubleValue = v * m.scale;
                FsVarParamArray params;
                params.size = 1;
                params.array = &param;
                const FsVarError err = fsVarsBVarCall(s.setId[e], params);
                if (err != FS_VAR_ERROR_NONE && s.errors++ < 5) {
                    log::warn("controls: %s_SET(%.4f) failed (error 0x%x)", engineVar(m.var, e + 1).c_str(),
                              param.doubleValue, static_cast<unsigned>(err));
                }
            }
            break;
        case MethodKind::InputValue:
            for (int e = 0; e < s.count; ++e) {
                double v = f.v[methodSourceChannel(c, m.kind, e)];
                if (!std::isfinite(v) || !due(s, e, v, force)) continue;
                const HRESULT hr = SimConnect_SetInputEvent(ctx_.h, s.hash[e], sizeof(double), &v);
                if (FAILED(hr) && s.errors++ < 5) log::warn("controls: SetInputEvent %s failed", engineVar(m.var, e + 1).c_str());
            }
            break;
        case MethodKind::LVar:
            for (int e = 0; e < s.count; ++e) {
                const double v = f.v[methodSourceChannel(c, m.kind, e)];
                if (!std::isfinite(v) || !due(s, e, v, force)) continue;
                const FsVarError err = fsVarsLVarSet(s.lvar[e], number_, v);
                if (err != FS_VAR_ERROR_NONE && s.errors++ < 5) {
                    log::warn("controls: setting L:%s failed (error 0x%x)", engineVar(m.var, e + 1).c_str(),
                              static_cast<unsigned>(err));
                }
            }
            break;
        case MethodKind::SimVar:
            // Written every frame, like playback's own simvar channels.
            ++s.sends;
            for (int i = 0; i < s.simvarCount; ++i) {
                const SimVarTarget& t = s.simvars[i];
                const double v = f.v[t.channel];
                if (!std::isfinite(v)) continue;
                FsVarParamVariant param;
                param.type = FsVarParamTypeInteger;
                param.intValue = t.index;
                FsVarParamArray params;
                params.size = t.index > 0 ? 1 : 0;
                params.array = t.index > 0 ? &param : nullptr;
                const FsVarError err = fsVarsAVarSet(t.id, t.unit, params, v);
                if (err != FS_VAR_ERROR_NONE && s.errors++ < 5) {
                    log::warn("controls: writing %s failed (error 0x%x)", kChannels[t.channel].simvar, static_cast<unsigned>(err));
                }
            }
            break;
        case MethodKind::KeyEvent: applyKeyEvent(c, s, f, force); break;
        case MethodKind::None: break;
    }
}

bool ControlDriver::sending_ = false;

void ControlDriver::keyEvent(int id, int32_t value) {
    FsVarParamVariant param;
    param.type = FsVarParamTypeInteger;
    param.intValue = static_cast<unsigned>(value);
    FsVarParamArray params;
    params.size = 1;
    params.array = &param;
    sending_ = true;
    fsEventsTriggerKeyEvent(id, params);
    sending_ = false;
}

void ControlDriver::sendEvent(Slot& s, int which, int32_t param) {
    const uint32_t id = which == 0 ? s.event.id : s.event.lowerId;
    const DWORD client = s.eventClient[which];
    if (client == 0) {
        keyEvent(static_cast<int>(id), param);
        return;
    }
    sending_ = true;
    const HRESULT hr = SimConnect_TransmitClientEvent_EX1(ctx_.h, SIMCONNECT_OBJECT_ID_USER_AIRCRAFT, client,
                                                          SIMCONNECT_GROUP_PRIORITY_HIGHEST,
                                                          SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY, static_cast<DWORD>(param), 0);
    sending_ = false;
    if (FAILED(hr) && s.errors++ < 5) log::warn("controls: sending event #%lu failed", static_cast<unsigned long>(id));
}

void ControlDriver::applyEvent(Control c, Slot& s, const Frame& f, bool force) {
    const EventMethod& ev = s.event;
    // Where the aircraft's control is: the gear's command from its legs (its handle simvar may
    // lag them, or not be the lever at all), every frame.
    double now = NAN;
    if (ctx_.live) {
        const Frame& live = *ctx_.live;
        if (c == Control::Gear) {
            double sum = 0.0;
            int n = 0;
            for (const int ch : {kGearCenter, kGearLeft, kGearRight}) {
                if (!std::isfinite(live.v[ch])) continue;
                sum += live.v[ch];
                ++n;
            }
            now = s.gearLive.update(n > 0 ? sum / n : NAN);
        } else {
            now = live.v[c == Control::Flaps ? kFlapsHandleIndex : kSpoilersHandle];
        }
    }
    const double target = eventTarget(c, f);
    if (!std::isfinite(target)) return;
    const char* label = controlSpec(c).label;
    if (ev.mode == EventMethod::Mode::Position) {
        const int32_t param = eventParam(ev, target);
        const double minStep = 0.003 * std::fabs(ev.scale);  // an axis: as AXIS_SPOILER_SET's 48
        if (!dueOnChange(s, 0, param, force, minStep > 1.0 ? minStep : 1.0)) return;
        sendEvent(s, 0, param);
        if (c == Control::Gear) {
            log::info("controls: gear %s (event %s, parameter %ld)", target > 0.5 ? "down" : "up", s.method.var.c_str(),
                      static_cast<long>(param));
        }
        return;
    }
    const int dir = s.stepper.step(target, now, ev.scale, EventStepper::waitFrames(c), force);
    if (dir == 0) return;
    const bool back = dir < 0 && ev.mode == EventMethod::Mode::Steps;
    sendEvent(s, back ? 1 : 0, back ? ev.lowerParam : ev.param);
    if (back ? ev.lowerClick : ev.click) sendEvent(s, back ? 1 : 0, back ? ev.lowerRelease : ev.release);  // a click's release
    ++s.sends;
    log::info("controls: %s %s (event %s; at %.3g, the clip %.3g)", label,
              ev.mode == EventMethod::Mode::Toggle ? "toggled" : (dir > 0 ? "a step up" : "a step down"),
              s.method.var.c_str(), now, target);
}

DWORD ControlDriver::ownEventId(const std::string& name) {
    for (size_t i = 0; i < ownEvents_.size(); ++i) {
        if (ownEvents_[i] == name) return kOwnEventFirst + static_cast<DWORD>(i);
    }
    const DWORD id = kOwnEventFirst + static_cast<DWORD>(ownEvents_.size());
    if (FAILED(SimConnect_MapClientEventToSimEvent(ctx_.h, id, name.c_str()))) return 0;
    ownEvents_.push_back(name);
    return id;
}

void ControlDriver::applyKeyEvent(Control c, Slot& s, const Frame& f, bool force) {
    if (isOwnEvent(s.method)) {
        applyEvent(c, s, f, force);
        return;
    }
    switch (c) {
        case Control::Throttle: {
            // Reverse thrust as the setup found this aircraft takes it. "Hold": THROTTLE_DECR every
            // frame while the clip's reversers are out, as a held "decrease throttle" key does; the
            // levers stay where the clip has them (idle), and let go the aircraft brings them back.
            const ReverseBy by = s.reverseBy;
            if (by == ReverseBy::Hold) {
                bool out = false;
                for (int e = 0; e < s.count; ++e) out |= reversing(f, e);
                if (out) {
                    keyEvent(KEY_THROTTLE_DECR, 0);
                    ++s.holdFrames;
                    if (!s.holding) log::info("controls: holding THROTTLE_DECR (the clip's reversers are out)");
                } else if (s.holding) {
                    force = true;  // let go: and the levers again, in case the aircraft stayed in reverse
                    log::info("controls: let go of THROTTLE_DECR");
                }
                s.holding = out;
            }
            // "Toggle": toggle reverse with the levers at idle when the clip's reversers come out,
            // its reverse amount once the aircraft is in reverse, and back out at idle when they go
            // in. Neither the clip nor the aircraft may have the reverse flag (the A350 doesn't set
            // it), so both go by what the reversers did; ReverseToggle keeps track of the rest.
            ReverseToggle::Step toggled;
            if (by == ReverseBy::Toggle) {
                bool clipOut = false, liveOut = false;
                double liveThrottle = NAN;
                for (int e = 0; e < s.count; ++e) {
                    clipOut |= reversing(f, e);
                    if (!ctx_.live) continue;
                    liveOut |= reversing(*ctx_.live, e);
                    const double t = ctx_.live->v[kThrottle1 + e];
                    if (std::isfinite(t) && !(t <= liveThrottle)) liveThrottle = t;
                }
                toggled = s.toggle.step(clipOut, liveOut, liveThrottle);
                if (toggled.corrected) {
                    log::info("controls: the aircraft %s in reverse after all", s.toggle.on() ? "is" : "isn't");
                }
            }
            const double limit = ctx_.throttleLowerLimit ? *ctx_.throttleLowerLimit : NAN;
            const double lower = std::isfinite(limit) && limit < -1.0 ? limit : 0.0;
            const int64_t nowMs = utcNowMs();
            const double followDt = s.followMs > 0 ? clamp((nowMs - s.followMs) / 1000.0, 0.0, 0.1) : 0.0;
            s.followMs = nowMs;
            for (int e = 0; e < s.count; ++e) {
                double fraction = NAN;
                bool lever = true;  // the source is the lever itself
                bool following = false;  // the clip's thrust demand (autothrust)
                const double pct = f.v[kThrottle1 + e];
                const double flag = f.v[kReverser1 + e];
                if (by == ReverseBy::Toggle && toggled.drive != ReverseToggle::Drive::Forward) {
                    // Around a toggle the levers at idle; in reverse the clip's amount from the
                    // throttle simvar (the lever variable doesn't show it).
                    fraction = 0.0;
                    if (toggled.drive == ReverseToggle::Drive::Reverse) {
                        if (!std::isfinite(pct)) continue;
                        if (lower < 0.0 && pct < 0.0) fraction = std::fmin(pct / lower, toggled.cap);
                        ++s.toggleFrames;
                    }
                    const double axis = static_cast<double>(std::lround(clamp(fraction, 0.0, 1.0) * 32768.0 - 16384.0));
                    if (dueOnChange(s, e, axis, force, 48.0)) keyEvent(kThrottleAxisKeys[e], static_cast<int32_t>(axis));
                    continue;
                }
                bool liveReverse = ctx_.live && ctx_.live->v[kReverser1 + e] >= 0.5;
                // Whether the clip is in reverse here. Its reverse flag, except: on a "hold"
                // aircraft the flag isn't its reverse (the hold above is); on an "events" one the
                // reversers may be out without it ("decrease throttle" held), and then we put the
                // reverser in ourselves, retrying until the aircraft has it, and take it out after.
                double clipFlag = flag;
                bool clipReverse = std::isfinite(flag) ? flag >= 0.5 : pct < 0.0;
                if (by == ReverseBy::Hold) {
                    clipFlag = NAN;
                    clipReverse = false;
                } else if (by == ReverseBy::Events) {
                    const bool flagOn = std::isfinite(flag) && flag >= 0.5;
                    const bool out = reversing(f, e);
                    if (out && !flagOn) {
                        if (!liveReverse && --s.engageWait[e] <= 0) {
                            keyEvent(kReverseOnKeys[e], 0);
                            s.engageWait[e] = kEngageRetryFrames;
                            if (!s.engaged[e]) log::info("controls: engine %d reversing without the reverse flag: reverser in", e + 1);
                        }
                        s.engaged[e] = true;
                    } else if (s.engaged[e] && !out) {
                        keyEvent(kReverseOffKeys[e], 0);
                        s.engaged[e] = false;
                        s.engageWait[e] = 0;
                    }
                    clipReverse = flagOn || out;
                    clipFlag = clipReverse ? 1.0 : 0.0;
                } else if (by == ReverseBy::Toggle) {
                    // Out of reverse (above): forward, whatever the flags say.
                    clipFlag = 0.0;
                    clipReverse = liveReverse = false;
                }
                const double hw = f.v[kThrottleAxis1 + e];
                if (by == ReverseBy::Toggle && std::isfinite(pct) && pct < 0.0) {
                    // Below idle without reversing yet (going in or out of reverse): idle, not the
                    // pilot's axis, which set the reverse amount there.
                    fraction = 0.0;
                } else if (std::isfinite(hw)) {
                    // The pilot's own hardware axis: puts the lever exactly where it was, detents
                    // and any reverse zone on the axis included.
                    fraction = hw;
                } else if (ctx_.clipChannels && ctx_.clipChannels->test(static_cast<size_t>(kThrottleAxis1 + e))) {
                    continue;  // the clip has the axis, but not yet here (before the hardware moved): hold
                } else if (s.useLever) {
                    // The lever's own variable, through the setup's calibration: the axis value
                    // that put the lever there, the middle of a detent when it was in one. The
                    // calibration can't place reverse without a reverse table; then the simvar.
                    const int ch = kLVarThrottle1 + (s.lever.var.find('#') != std::string::npos ? e : 0);
                    const double lv = f.v[ch];
                    if (!std::isfinite(lv)) {
                        // A gap in the clip: hold for a moment. A longer stretch without it (recorded
                        // before the setup found the variable, or before the aircraft had it): from
                        // the simvar below meanwhile.
                        if (++s.leverGap[e] <= kLeverGapFrames) continue;
                        ++s.gapFrames;
                    } else {
                        s.leverGap[e] = 0;
                        if (leverAxis(s.lever, lv, clipFlag, liveReverse, fraction)) {
                            ++s.leverFrames;
                        } else {
                            fraction = NAN;
                        }
                    }
                }
                if (std::isfinite(fraction) && !clipReverse && !liveReverse && inAutothrust(s, e, f.t)) {
                    // Autothrust in the clip: its thrust demand, the levers no further forward than
                    // the clip's (as the autothrust can't go beyond them either).
                    const double demand = ctx_.live ? ctx_.live->v[kThrottle1 + e] : NAN;
                    if (std::isfinite(pct) && std::isfinite(demand)) {
                        fraction = s.follower[e].step(pct, demand, s.lever.forward.front().axis, fraction, followDt);
                        following = true;
                        ++s.followFrames;
                    }
                }
                if (!std::isfinite(fraction)) {
                    // Older clips (and reverse the calibration can't place): from the lever
                    // simvar. Idle..full out of reverse; with the
                    // reverser engaged (switched by the clip's reverser events), idle..full
                    // reverse. Hold idle until the aircraft's reverser matches the clip.
                    if (!std::isfinite(pct)) continue;
                    lever = false;
                    fraction = 0.0;
                    if (clipReverse && liveReverse && lower < 0.0) {
                        fraction = pct / lower;
                    } else if (!clipReverse && !liveReverse) {
                        fraction = pct / 100.0;
                    }
                }
                if (by == ReverseBy::Hold && s.holding) {
                    // Held in reverse: the forward levers at idle, whatever the lever variable reads
                    // (the A350's reads part throttle in reverse).
                    fraction = 0.0;
                    lever = true;
                }
                if (by == ReverseBy::Events) {
                    if (clipReverse != liveReverse) {
                        fraction = 0.0;  // idle in whichever mode the aircraft is in, until it matches
                    } else if (clipReverse && lower < 0.0 && std::isfinite(pct) && pct < 0.0) {
                        // The reverse amount: the lever variable may not show it (with "decrease
                        // throttle" held it stays at idle), the simvar does.
                        const double amount = pct / lower;
                        fraction = std::isfinite(fraction) && fraction > amount ? fraction : amount;
                    }
                }
                const double axis = static_cast<double>(std::lround(clamp(fraction, 0.0, 1.0) * 32768.0 - 16384.0));
                // The simvar isn't always the lever: on the A320neo V2 under autothrust it's the
                // thrust demand, which creeps with speed. Every event makes the aircraft re-evaluate
                // its lever (and the engine sound follows), so from the simvar a change of 2% goes
                // at once and a smaller one only once it has lasted. The thrust demand follower
                // corrects a little every frame: from it 3% at once, 0.5% once it has lasted (an
                // event almost every frame, about 20 a second, likely distorted the A350's sound).
                bool send = false;
                if (following) {
                    send = dueWhenLasting(s, e, axis, force, 164.0, 983.0, kLastingMs, nowMs);
                } else {
                    send = lever ? dueOnChange(s, e, axis, force, 48.0)
                                 : dueWhenLasting(s, e, axis, force, 48.0, 655.0, kLastingMs, nowMs);
                }
                if (send) keyEvent(kThrottleAxisKeys[e], static_cast<int32_t>(axis));
            }
            if (toggled.toggle) {
                // After the levers' idle (sent a frame or more before).
                keyEvent(KEY_THROTTLE_REVERSE_THRUST_TOGGLE, 0);
                log::info("controls: toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE) %s",
                          s.toggle.on() ? "in: the clip's reversers are out" : "back out: the clip's reversers are in");
            }
            break;
        }
        case Control::Flaps: {
            const double idx = f.v[kFlapsHandleIndex];
            if (!std::isfinite(idx)) break;
            if (s.method.var == kKeyFlapsNotches) {
                stepFlaps(s, std::round(idx), force);
                break;
            }
            const int positions = ctx_.flapPositions ? *ctx_.flapPositions : 0;
            if (positions < 2) break;
            const double fraction = clamp(std::round(idx), 0.0, positions - 1.0) / (positions - 1.0);
            if (s.method.var == kKeyFlapsAxis) {
                const double value = std::round(fraction * 32766.0 - 16383.0);
                if (dueOnChange(s, 0, value, force, 1.0)) keyEvent(KEY_AXIS_FLAPS_SET, static_cast<int32_t>(value));
                break;
            }
            const double value = std::round(fraction * 16383.0);
            if (dueOnChange(s, 0, value, force, 1.0)) keyEvent(KEY_FLAPS_SET, static_cast<int32_t>(value));
            break;
        }
        case Control::Spoilers: {
            const double h = f.v[kSpoilersHandle];
            if (!std::isfinite(h)) break;
            if (s.method.var == kKeySpoilersAxis) {
                const double value = std::round(clamp(h, 0.0, 1.0) * 32766.0 - 16383.0);
                if (dueOnChange(s, 0, value, force, 48.0)) keyEvent(KEY_AXIS_SPOILER_SET, static_cast<int32_t>(value));
                break;
            }
            const double value = std::round(clamp(h, 0.0, 1.0) * 16383.0);
            if (dueOnChange(s, 0, value, force, 24.0)) keyEvent(KEY_SPOILERS_SET, static_cast<int32_t>(value));
            break;
        }
        case Control::Gear: {
            const double g = f.v[kGearHandle];
            if (!std::isfinite(g)) break;
            const double state = g >= 0.5 ? 1.0 : 0.0;
            if (!force && state == s.sent[0]) break;  // a command, not a position: only on change
            s.sent[0] = state;
            ++s.sends;
            keyEvent(KEY_GEAR_SET, state > 0.5 ? 1 : 0);
            keyEvent(state > 0.5 ? KEY_GEAR_DOWN : KEY_GEAR_UP, 0);
            log::info("controls: gear %s (key events)", state > 0.5 ? "down" : "up");
            break;
        }
    }
}

// Like a hardware flap switch: a notch at a time toward the clip's handle index (FlapNotches).
void ControlDriver::stepFlaps(Slot& s, double target, bool force) {
    const double live = ctx_.live ? ctx_.live->v[kFlapsHandleIndex] : NAN;
    const int dir = notches_.step(target, live, utcNowMs(), force);
    if (dir == 0) return;
    const bool extend = dir > 0;
    keyEvent(extend ? KEY_FLAPS_INCR : KEY_FLAPS_DECR, 0);
    ++s.sends;
    log::info("controls: flaps a notch %s (FLAPS_%s; the handle at %.0f, the lever to %.0f, the clip %.0f)",
              extend ? "down" : "up", extend ? "INCR" : "DECR", live, notches_.lever(), target);
}

void ControlDriver::restoreLVars(Control c) {
    Slot& s = slots_[static_cast<int>(c)];
    if (!s.active || s.method.kind != MethodKind::LVar) return;
    for (int e = 0; e < s.count; ++e) {
        if (s.lvar[e] != FS_VAR_INVALID_ID && std::isfinite(s.original[e])) fsVarsLVarSet(s.lvar[e], number_, s.original[e]);
    }
}

void ControlDriver::end(Control c) {
    const int ci = static_cast<int>(c);
    Slot& s = slots_[ci];
    if (!s.active) return;
    s.active = false;
    for (const Slot& w : extra_[ci]) {
        log::info("controls: %d value(s) sent to the %s with %s", w.sends, controlSpec(c).label, describeMethod(c, w.method).c_str());
        if (w.errors > 0) log::warn("controls: %d error(s) driving the %s with %s", w.errors, controlSpec(c).label,
                                    describeMethod(c, w.method).c_str());
    }
    extra_[ci].clear();
    log::info(s.method.kind == MethodKind::SimVar ? "controls: %d frame(s) written to the %s with %s"
                                                  : "controls: %d value(s) sent to the %s with %s",
              s.sends, controlSpec(c).label, describeMethod(c, s.method).c_str());
    if (s.useLever) log::info("controls: %d engine frame(s) placed from L:var %s", s.leverFrames, s.lever.var.c_str());
    if (s.followFrames > 0) {
        log::info("controls: %d engine frame(s) followed the clip's thrust demand (autothrust)", s.followFrames);
    }
    if (s.gapFrames > 0) {
        log::info("controls: %d engine frame(s) from the throttle simvar, where the clip has no %s value", s.gapFrames,
                  s.lever.var.c_str());
    }
    if (s.holdFrames > 0) log::info("controls: THROTTLE_DECR held for reverse in %d frame(s)", s.holdFrames);
    for (int e = 0; e < 4; ++e) {
        if (s.engaged[e]) keyEvent(kReverseOffKeys[e], 0);  // the reverser we put in
    }
    if (s.reverseBy == ReverseBy::Toggle) {
        if (s.toggle.on()) {
            // Still toggled into reverse: back out, the levers at idle first. (At the end of a
            // replay SimBridge waits for that before ending, so this is the last resort.)
            for (int e = 0; e < s.count; ++e) keyEvent(kThrottleAxisKeys[e], -16384);
            keyEvent(KEY_THROTTLE_REVERSE_THRUST_TOGGLE, 0);
            log::info("controls: toggle reverse back out at the end");
        }
        if (s.toggle.toggles() > 0 || s.toggle.corrections() > 0) {
            log::info("controls: toggle reverse sent %d time(s) (%d correction(s)); a reverse amount in %d engine frame(s)",
                      s.toggle.toggles(), s.toggle.corrections(), s.toggleFrames);
        }
    }
    if (s.errors > 0) log::warn("controls: %d error(s) driving the %s", s.errors, controlSpec(c).label);
}

void ControlDriver::endAll() {
    for (int i = 0; i < kControlCount; ++i) end(controlAt(i));
}

bool ControlDriver::anyActive() const {
    for (const Slot& s : slots_) {
        if (s.active) return true;
    }
    return false;
}

bool ControlDriver::wantsThrottleMask() const {
    const Slot& s = slots_[static_cast<int>(Control::Throttle)];
    return s.active && s.method.kind == MethodKind::SimVar;
}

}  // namespace br
