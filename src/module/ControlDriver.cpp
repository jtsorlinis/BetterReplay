#include "ControlDriver.h"

#include <MSFS/MSFS_Events.h>

#include <cstdlib>
#include <cstring>

#include "Log.h"

namespace br {

namespace {

const int kThrottleAxisKeys[4] = {KEY_THROTTLE1_AXIS_SET_EX1, KEY_THROTTLE2_AXIS_SET_EX1, KEY_THROTTLE3_AXIS_SET_EX1,
                                  KEY_THROTTLE4_AXIS_SET_EX1};

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

bool ControlDriver::dueWhenLasting(Slot& s, int i, double v, bool force, double minStep, double nowStep, int frames) {
    if (force || !std::isfinite(s.sent[i]) || std::fabs(v - s.sent[i]) >= nowStep) return dueOnChange(s, i, v, true, 0.0);
    if (std::fabs(v - s.sent[i]) < minStep) {
        s.since[i] = 0;
        return false;
    }
    if (++s.since[i] < frames) return false;
    return dueOnChange(s, i, v, true, 0.0);
}

bool ControlDriver::begin(Control c, const ControlMethod& m, std::string& why) {
    if (number_ == FS_INVALID_UNIT) number_ = fsVarsGetUnitId("number");
    Slot& s = slots_[static_cast<int>(c)];
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
            if (c == Control::Flaps && (!ctx_.flapPositions || *ctx_.flapPositions < 2)) {
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

void ControlDriver::apply(Control c, const Frame& f, bool force) {
    Slot& s = slots_[static_cast<int>(c)];
    if (!s.active) return;
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

void ControlDriver::keyEvent(int id, int32_t value) {
    FsVarParamVariant param;
    param.type = FsVarParamTypeInteger;
    param.intValue = static_cast<unsigned>(value);
    FsVarParamArray params;
    params.size = 1;
    params.array = &param;
    fsEventsTriggerKeyEvent(id, params);
}

void ControlDriver::applyKeyEvent(Control c, Slot& s, const Frame& f, bool force) {
    switch (c) {
        case Control::Throttle:
            for (int e = 0; e < s.count; ++e) {
                double fraction = NAN;
                bool lever = true;  // the source is the lever itself
                const double hw = f.v[kThrottleAxis1 + e];
                if (std::isfinite(hw)) {
                    // The pilot's own hardware axis: puts the lever exactly where it was, detents
                    // and any reverse zone on the axis included.
                    fraction = hw;
                } else if (ctx_.clipChannels && ctx_.clipChannels->test(static_cast<size_t>(kThrottleAxis1 + e))) {
                    continue;  // the clip has the axis, but not yet here (before the hardware moved): hold
                } else {
                    // Older clips: from the lever simvar. Idle..full out of reverse; with the
                    // reverser engaged (switched by the clip's reverser events), idle..full
                    // reverse. Hold idle until the aircraft's reverser matches the clip.
                    const double pct = f.v[kThrottle1 + e];
                    if (!std::isfinite(pct)) continue;
                    lever = false;
                    const double rev = f.v[kReverser1 + e];
                    const bool clipReverse = std::isfinite(rev) ? rev >= 0.5 : pct < 0.0;
                    const bool liveReverse = ctx_.live && ctx_.live->v[kReverser1 + e] >= 0.5;
                    const double limit = ctx_.throttleLowerLimit ? *ctx_.throttleLowerLimit : NAN;
                    const double lower = std::isfinite(limit) && limit < -1.0 ? limit : 0.0;
                    fraction = 0.0;
                    if (clipReverse && liveReverse && lower < 0.0) {
                        fraction = pct / lower;
                    } else if (!clipReverse && !liveReverse) {
                        fraction = pct / 100.0;
                    }
                }
                const double axis = static_cast<double>(std::lround(clamp(fraction, 0.0, 1.0) * 32768.0 - 16384.0));
                // The simvar isn't always the lever: on the A320neo V2 under autothrust it's the
                // thrust demand, which creeps with speed. Every event makes the aircraft re-evaluate
                // its lever (and the engine sound follows), so from the simvar a change of 2% goes
                // at once and a smaller one only once it has lasted.
                const bool send = lever ? dueOnChange(s, e, axis, force, 48.0)
                                        : dueWhenLasting(s, e, axis, force, 48.0, 655.0, kLastingFrames);
                if (send) keyEvent(kThrottleAxisKeys[e], static_cast<int32_t>(axis));
            }
            break;
        case Control::Flaps: {
            const double idx = f.v[kFlapsHandleIndex];
            const int positions = ctx_.flapPositions ? *ctx_.flapPositions : 0;
            if (!std::isfinite(idx) || positions < 2) break;
            const double value = std::round(clamp(std::round(idx), 0.0, positions - 1.0) / (positions - 1.0) * 16383.0);
            if (dueOnChange(s, 0, value, force, 1.0)) keyEvent(KEY_FLAPS_SET, static_cast<int32_t>(value));
            break;
        }
        case Control::Spoilers: {
            const double h = f.v[kSpoilersHandle];
            if (!std::isfinite(h)) break;
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

void ControlDriver::end(Control c) {
    Slot& s = slots_[static_cast<int>(c)];
    if (!s.active) return;
    s.active = false;
    log::info(s.method.kind == MethodKind::SimVar ? "controls: %d frame(s) written to the %s with %s"
                                                  : "controls: %d value(s) sent to the %s with %s",
              s.sends, controlSpec(c).label, describeMethod(c, s.method).c_str());
    if (s.errors > 0) log::warn("controls: %d error(s) driving the %s", s.errors, controlSpec(c).label);
}

void ControlDriver::endAll() {
    for (int i = 0; i < kControlCount; ++i) end(controlAt(i));
}

bool ControlDriver::wantsThrottleMask() const {
    const Slot& s = slots_[static_cast<int>(Control::Throttle)];
    return s.active && s.method.kind == MethodKind::SimVar;
}

}  // namespace br
