#include "AircraftProfile.h"

#include <cstring>

#include "Json.h"

namespace br {

namespace {

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
            // A profile that says "use this method" without one can't be followed.
            if (cp.state == ControlState::Ok && cp.method.kind == MethodKind::None) cp.state = ControlState::Pending;
        }
        loaded.push_back(p);
    }
    profiles_.swap(loaded);
    return true;
}

}  // namespace br
