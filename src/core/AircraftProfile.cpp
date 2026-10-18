#include "AircraftProfile.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "Json.h"

namespace br {

namespace {

// Lever readings this close together (as a fraction of the table's range) are the same position.
constexpr double kLeverFlatFraction = 0.002;

double leverRange(const std::vector<LeverPoint>& t, double& lo, double& hi) {
    lo = INFINITY;
    hi = -INFINITY;
    for (const LeverPoint& p : t) {
        lo = p.lever < lo ? p.lever : lo;
        hi = p.lever > hi ? p.lever : hi;
    }
    return std::max(kLeverFlatFraction * (hi - lo), 1e-9);  // the tolerance for "the same"
}

void writeLeverTable(JsonWriter& w, const char* key, const std::vector<LeverPoint>& t) {
    w.key(key).beginArray();
    for (const LeverPoint& p : t) w.beginArray().value(p.axis).value(p.lever).endArray();
    w.endArray();
}

std::vector<LeverPoint> readLeverTable(const JsonValue* v) {
    std::vector<LeverPoint> t;
    if (!v || !v->isArray()) return t;
    for (const JsonValue& p : v->array) {
        if (!p.isArray() || p.array.size() < 2) continue;
        t.push_back({p.array[0].number, p.array[1].number});
    }
    return leverTable(std::move(t));
}

const ControlSpec kSpecs[kControlCount] = {
    {"spoilers", "spoilers", kInputSpeedBrakes, kLVarSpeedBrakes, kSpoilersHandle, kSpoilersHandle, 1, false},
    {"flaps", "flaps", kInputFlaps, kLVarFlaps, kFlapsHandleIndex, kFlapsLeft, 3, false},
    {"throttle", "throttle", kInputThrottle1, kLVarThrottle1, kThrottle1, kThrottle1, 4, true},
    {"gear", "gear", kInputGear, kLVarGear, kGearHandle, -1, 0, false},
};

const char* const kMethodKeys[] = {"none", "input_set", "input_value", "lvar", "simvar", "key_event"};
const char* const kStateKeys[] = {"pending", "ok", "failed", "skipped", "absent"};

template <typename E, size_t N>
bool parseKey(const char* const (&keys)[N], const std::string& key, E& out) {
    for (size_t i = 0; i < N; ++i) {
        if (key == keys[i]) {
            out = static_cast<E>(i);
            return true;
        }
    }
    return false;
}

}  // namespace

bool AircraftProfile::anyPending() const {
    for (const ControlProfile& c : controls) {
        if (c.state == ControlState::Pending) return true;
    }
    return false;
}

bool AircraftProfile::drives(Control c) const {
    const ControlState s = (*this)[c].state;
    return s == ControlState::Ok || s == ControlState::Failed;
}

const ControlSpec& controlSpec(Control c) { return kSpecs[static_cast<int>(c)]; }

Control controlAt(int i) { return static_cast<Control>(i); }

bool parseControlKey(const std::string& key, Control& out) {
    for (int i = 0; i < kControlCount; ++i) {
        if (key == kSpecs[i].key) {
            out = controlAt(i);
            return true;
        }
    }
    return false;
}

const char* methodKey(MethodKind k) {
    const size_t i = static_cast<size_t>(k);
    return i < sizeof(kMethodKeys) / sizeof(kMethodKeys[0]) ? kMethodKeys[i] : "none";
}

bool parseMethodKey(const std::string& key, MethodKind& out) { return parseKey(kMethodKeys, key, out); }

const char* stateKey(ControlState s) {
    const size_t i = static_cast<size_t>(s);
    return i < sizeof(kStateKeys) / sizeof(kStateKeys[0]) ? kStateKeys[i] : "pending";
}

bool parseStateKey(const std::string& key, ControlState& out) { return parseKey(kStateKeys, key, out); }

std::string engineVar(const std::string& pattern, int engine) {
    const size_t at = pattern.find('#');
    if (at == std::string::npos) return pattern;
    std::string out = pattern;
    out.replace(at, 1, std::to_string(engine));
    return out;
}

std::string describeMethod(Control c, const ControlMethod& m) {
    switch (m.kind) {
        case MethodKind::None: return "nothing";
        case MethodKind::InputSet:
            return "input event " + m.var + " (" + m.var + "_SET" + (m.scale != 1.0 ? ", as a fraction)" : ")");
        case MethodKind::InputValue: return "input event " + m.var + " (value written directly)";
        case MethodKind::LVar: return "L:var " + m.var;
        case MethodKind::SimVar: return "simvars";
        case MethodKind::KeyEvent:
            if (m.var == kKeyFlapsNotches) return "key events FLAPS_INCR/FLAPS_DECR";
            if (!m.var.empty()) return "key event " + m.var;
            switch (c) {
                case Control::Spoilers: return "key event SPOILERS_SET";
                case Control::Flaps: return "key event FLAPS_SET";
                case Control::Throttle: return "key events THROTTLEn_AXIS_SET_EX1";
                case Control::Gear: return "key events GEAR_SET/GEAR_UP/GEAR_DOWN";
            }
            break;
    }
    return "nothing";
}

std::string describeControl(Control c, const ControlProfile& cp) {
    std::string s = describeMethod(c, cp.method);
    if (c == Control::Throttle && cp.lever.valid()) {
        s += ", lever from L:var " + cp.lever.var;
        switch (cp.lever.reverseBy) {
            case ReverseBy::Events: s += ", reverse with SET_THROTTLEn_REVERSE_THRUST_ON"; break;
            case ReverseBy::Hold: s += ", reverse by holding THROTTLE_DECR"; break;
            case ReverseBy::Axis: s += ", reverse on the axis"; break;
            case ReverseBy::None: s += ", no reverse"; break;
            case ReverseBy::Unknown: break;
        }
    }
    return s;
}

// ---------------------------------------------------------------------------- lever

std::vector<LeverPoint> leverTable(std::vector<LeverPoint> samples) {
    samples.erase(std::remove_if(samples.begin(), samples.end(),
                                 [](const LeverPoint& p) { return !std::isfinite(p.axis) || !std::isfinite(p.lever); }),
                  samples.end());
    std::stable_sort(samples.begin(), samples.end(), [](const LeverPoint& a, const LeverPoint& b) { return a.axis < b.axis; });
    return samples;
}

double invertLever(const std::vector<LeverPoint>& t, double x) {
    if (t.empty()) return 0.0;
    double lo = 0.0, hi = 0.0;
    const double tol = leverRange(t, lo, hi);
    x = x < lo ? lo : (x > hi ? hi : x);
    // The widest run of readings at this value: a detent holds the lever over a stretch of the
    // axis, and its middle is the safest place to put it.
    size_t from = 0, to = 0;
    double widest = -1.0;
    for (size_t i = 0; i < t.size();) {
        if (std::fabs(t[i].lever - x) > tol) {
            ++i;
            continue;
        }
        size_t j = i;
        while (j + 1 < t.size() && std::fabs(t[j + 1].lever - x) <= tol) ++j;
        if (t[j].axis - t[i].axis > widest) {
            widest = t[j].axis - t[i].axis;
            from = i;
            to = j;
        }
        i = j + 1;
    }
    if (widest >= 0.0) return 0.5 * (t[from].axis + t[to].axis);
    for (size_t i = 0; i + 1 < t.size(); ++i) {
        const double a = t[i].lever, b = t[i + 1].lever;
        if ((x - a) * (x - b) > 0.0) continue;
        const double f = b != a ? (x - a) / (b - a) : 0.5;
        return t[i].axis + f * (t[i + 1].axis - t[i].axis);
    }
    return t.back().axis;
}

bool leverAxis(const LeverCalibration& cal, double lever, double clipReverser, bool liveReverse, double& axis) {
    if (!std::isfinite(lever) || cal.forward.size() < 2) return false;
    double flo = 0.0, fhi = 0.0;
    const double tol = leverRange(cal.forward, flo, fhi);
    const double idle = cal.forward.front().lever;
    const double toFull = cal.forward.back().lever >= idle ? 1.0 : -1.0;
    const bool beyondIdle = (lever - idle) * toFull < -tol;  // the other way from full
    const bool separate = cal.reverse.size() >= 2;
    const bool clipReverse = std::isfinite(clipReverser) ? clipReverser >= 0.5 : separate && beyondIdle;
    if (separate) {
        if (clipReverse != liveReverse) {
            // The clip's reverser events switch the aircraft's over; until then, idle in whichever
            // mode it's in (the reverse table's end nearest the forward idle).
            if (!liveReverse) {
                axis = invertLever(cal.forward, idle);
                return true;
            }
            double rlo = 0.0, rhi = 0.0;
            leverRange(cal.reverse, rlo, rhi);
            axis = invertLever(cal.reverse, std::fabs(rlo - idle) < std::fabs(rhi - idle) ? rlo : rhi);
            return true;
        }
        axis = invertLever(clipReverse ? cal.reverse : cal.forward, lever);
        return true;
    }
    // Without a separate table only an axis with its own reverse zone reaches beyond idle.
    if (clipReverse && beyondIdle) return false;
    axis = invertLever(cal.forward, lever);
    return true;
}

bool reversing(const Frame& f, int engine) {
    if (engine < 0 || engine > 3) return false;
    const double nozzle = f.v[kReverserNozzle1 + engine];
    const double lever = f.v[kThrottle1 + engine];
    return (std::isfinite(nozzle) && nozzle >= 50.0) || (std::isfinite(lever) && lever < -1.0);
}

const char* reverseByKey(ReverseBy r) {
    switch (r) {
        case ReverseBy::None: return "none";
        case ReverseBy::Axis: return "axis";
        case ReverseBy::Events: return "events";
        case ReverseBy::Hold: return "hold";
        case ReverseBy::Unknown: break;
    }
    return "";
}

bool parseReverseByKey(const std::string& key, ReverseBy& out) {
    for (const ReverseBy r : {ReverseBy::None, ReverseBy::Axis, ReverseBy::Events, ReverseBy::Hold}) {
        if (key == reverseByKey(r)) {
            out = r;
            return true;
        }
    }
    return false;
}

int methodSourceChannel(Control c, MethodKind k, int engine) {
    const ControlSpec& s = controlSpec(c);
    const int e = s.perEngine ? engine : 0;
    switch (k) {
        case MethodKind::InputSet:
        case MethodKind::InputValue: return s.inputChannel + e;
        case MethodKind::LVar: return s.lvarChannel + e;
        case MethodKind::KeyEvent: return s.keyChannel + e;
        case MethodKind::SimVar: return s.simvarFirst < 0 ? -1 : s.simvarFirst + e;
        case MethodKind::None: break;
    }
    return -1;
}

void controlChannels(Control c, ChannelMask& out) {
    const ControlSpec& s = controlSpec(c);
    const int n = s.perEngine ? 4 : 1;
    for (int e = 0; e < n; ++e) {
        out.set(static_cast<size_t>(s.inputChannel + e));
        out.set(static_cast<size_t>(s.lvarChannel + e));
        out.set(static_cast<size_t>(s.keyChannel + e));
    }
    for (int i = 0; s.simvarFirst >= 0 && i < s.simvarCount; ++i) out.set(static_cast<size_t>(s.simvarFirst + i));
}

// ---------------------------------------------------------------------------- store

const AircraftProfile* ProfileStore::find(const std::string& key) const {
    for (const AircraftProfile& p : profiles_) {
        if (p.key == key) return &p;
    }
    return nullptr;
}

AircraftProfile& ProfileStore::upsert(const std::string& key) {
    for (AircraftProfile& p : profiles_) {
        if (p.key == key) return p;
    }
    profiles_.emplace_back();
    profiles_.back().key = key;
    return profiles_.back();
}

bool ProfileStore::remove(const std::string& key) {
    for (auto it = profiles_.begin(); it != profiles_.end(); ++it) {
        if (it->key == key) {
            profiles_.erase(it);
            return true;
        }
    }
    return false;
}

void ProfileStore::toJson(std::string& out) const {
    out.clear();
    JsonWriter w(out);
    w.beginObject();
    w.kv("version", 1);
    w.key("aircraft").beginArray();
    for (const AircraftProfile& p : profiles_) {
        w.beginObject();
        w.kv("key", p.key);
        w.kv("title", p.title);
        w.kv("updatedUtcMs", p.updatedUtcMs);
        w.kv("declined", p.declined);
        w.key("controls").beginObject();
        for (int i = 0; i < kControlCount; ++i) {
            const ControlProfile& c = p.controls[i];
            w.key(kSpecs[i].key).beginObject();
            w.kv("state", stateKey(c.state));
            w.kv("method", methodKey(c.method.kind));
            w.kv("var", c.method.var);
            w.kv("scale", c.method.scale);
            w.kv("confirmed", c.confirmed);
            if (!c.lever.var.empty()) {
                // [axis, lever] pairs
                w.key("lever").beginObject();
                w.kv("var", c.lever.var);
                writeLeverTable(w, "forward", c.lever.forward);
                writeLeverTable(w, "reverse", c.lever.reverse);
                if (c.lever.reverseBy != ReverseBy::Unknown) w.kv("reverseBy", reverseByKey(c.lever.reverseBy));
                w.endObject();
            }
            w.endObject();
        }
        w.endObject();
        w.endObject();
    }
    w.endArray();
    w.endObject();
}

bool ProfileStore::fromJson(const char* text, size_t length, std::string* error) {
    JsonValue root;
    if (!parseJson(text, length, root, error)) return false;
    const JsonValue* list = root.get("aircraft");
    if (!list || !list->isArray()) {
        if (error) *error = "no aircraft array";
        return false;
    }
    std::vector<AircraftProfile> loaded;
    for (const JsonValue& v : list->array) {
        if (!v.isObject()) continue;
        AircraftProfile p;
        p.key = v.getString("key", "");
        if (p.key.empty()) continue;
        p.title = v.getString("title", "");
        p.updatedUtcMs = static_cast<int64_t>(v.getNumber("updatedUtcMs", 0.0));
        p.declined = v.getBool("declined", false);
        const JsonValue* controls = v.get("controls");
        for (int i = 0; controls && i < kControlCount; ++i) {
            const JsonValue* c = controls->get(kSpecs[i].key);
            if (!c || !c->isObject()) continue;
            ControlProfile& cp = p.controls[i];
            if (!parseStateKey(c->getString("state", "pending"), cp.state)) cp.state = ControlState::Pending;
            if (!parseMethodKey(c->getString("method", "none"), cp.method.kind)) cp.method.kind = MethodKind::None;
            cp.method.var = c->getString("var", "");
            cp.method.scale = c->getNumber("scale", 1.0);
            cp.confirmed = c->getBool("confirmed", false);
            if (const JsonValue* lv = c->get("lever")) {
                cp.lever.var = lv->getString("var", "");
                cp.lever.forward = readLeverTable(lv->get("forward"));
                cp.lever.reverse = readLeverTable(lv->get("reverse"));
                parseReverseByKey(lv->getString("reverseBy", ""), cp.lever.reverseBy);
                if (!cp.lever.valid()) cp.lever = LeverCalibration();
            }
            // A profile that says "use this method" without one can't be followed.
            if (cp.state == ControlState::Ok && cp.method.kind == MethodKind::None) cp.state = ControlState::Pending;
        }
        loaded.push_back(p);
    }
    profiles_.swap(loaded);
    return true;
}

}  // namespace br
