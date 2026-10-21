#include "AircraftProfile.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

const char* directionKey(Control c, Direction d) {
    if (c == Control::Gear) return d == Direction::Extend ? "down" : "up";
    return d == Direction::Extend ? "extend" : "retract";
}

std::vector<ControlMethod> ControlProfile::waysTo(Direction d) const {
    const std::vector<ControlMethod>& w = ways[static_cast<int>(d)];
    return w.empty() ? std::vector<ControlMethod>{method} : w;
}

bool promoteWay(ControlProfile& cp, Direction d, const ControlMethod& m) {
    std::vector<ControlMethod>& w = cp.ways[static_cast<int>(d)];
    const auto it = std::find(w.begin(), w.end(), m);
    if (it == w.end() || it == w.begin()) return false;
    std::rotate(w.begin(), it, it + 1);
    return true;
}

std::string engineVar(const std::string& pattern, int engine) {
    const size_t at = pattern.find('#');
    if (at == std::string::npos) return pattern;
    std::string out = pattern;
    out.replace(at, 1, std::to_string(engine));
    return out;
}

bool isOwnEvent(const ControlMethod& m) { return m.kind == MethodKind::KeyEvent && !m.var.empty() && m.var[0] == '#'; }

namespace {

// Gear lever events from aircraft SDKs, by the aircraft's profile key (any variant).
struct OwnGearLever {
    const char* key;
    uint32_t event;        // THIRD_PARTY_EVENT_ID_MIN (69632) + the SDK's offset
    double down;           // the lever's DOWN position (UP is 0)
    const int32_t* click;  // a left click in the SDK's mouse flags: the button, then its release
};
// PMDG's MOUSE_FLAG_LEFTSINGLE and MOUSE_FLAG_LEFTRELEASE: its SDKs' events take them as a click.
const int32_t kPmdgLeftClick[2] = {0x20000000, 0x00020000};
const OwnGearLever kOwnGearLevers[] = {
    // PMDG_777X_SDK.h: EVT_GEAR_LEVER, + 295. In the sim its position 0 raised the gear but 1 didn't
    // lower it; GEAR_SET/GEAR_DOWN lowered it after the event had raised it, and after the pilot
    // had, only a left click did.
    {"simobjects/airplanes/pmdg 777", 69927, 1.0, kPmdgLeftClick},
};

}  // namespace

std::vector<ControlMethod> ownGearWays(const std::string& aircraftKey) {
    std::vector<ControlMethod> out;
    for (const OwnGearLever& g : kOwnGearLevers) {
        if (aircraftKey.compare(0, std::strlen(g.key), g.key) != 0) continue;
        EventMethod e;
        e.id = g.event;
        e.scale = g.down;
        out.push_back(eventMethod(e));
        if (!g.click) continue;
        e.mode = EventMethod::Mode::Toggle;
        e.param = g.click[0];
        e.click = true;
        e.release = g.click[1];
        out.push_back(eventMethod(e));
    }
    return out;
}

namespace {

// "#<id>" at s[i], then "(<param>)" or "(<param>,<release>)" when `withParam`; advances i.
bool readEvent(const std::string& s, size_t& i, bool withParam, uint32_t& id, int32_t& param, bool& click,
               int32_t& release) {
    if (i >= s.size() || s[i] != '#') return false;
    const char* begin = s.c_str() + i + 1;
    char* end = nullptr;
    const unsigned long long n = std::strtoull(begin, &end, 10);
    if (end == begin || n == 0 || n > 0xFFFFFFFFull) return false;
    id = static_cast<uint32_t>(n);
    i = static_cast<size_t>(end - s.c_str());
    if (!withParam) return true;
    if (i >= s.size() || s[i] != '(') return false;
    auto number = [&](int32_t& out) {
        const char* from = s.c_str() + i + 1;
        const long long p = std::strtoll(from, &end, 10);
        if (end == from || p < INT32_MIN || p > INT32_MAX) return false;
        out = static_cast<int32_t>(p);
        i = static_cast<size_t>(end - s.c_str());
        return true;
    };
    if (!number(param)) return false;
    click = s[i] == ',';
    if (click && !number(release)) return false;
    if (s[i] != ')') return false;
    ++i;
    return true;
}

}  // namespace

bool parseEventMethod(const ControlMethod& m, EventMethod& out) {
    if (!isOwnEvent(m)) return false;
    EventMethod e;
    e.scale = m.scale;
    const std::string& s = m.var;
    size_t i = 0;
    const bool withParam = s.find('(') != std::string::npos;
    if (!readEvent(s, i, withParam, e.id, e.param, e.click, e.release)) return false;
    if (!withParam) {
        e.mode = EventMethod::Mode::Position;
        if (i < s.size() && s[i] == '@') {
            const char* begin = s.c_str() + i + 1;
            char* end = nullptr;
            e.offset = std::strtod(begin, &end);
            if (end == begin || !std::isfinite(e.offset)) return false;
            i = static_cast<size_t>(end - s.c_str());
        }
    } else if (i == s.size()) {
        e.mode = EventMethod::Mode::Toggle;
    } else {
        e.mode = EventMethod::Mode::Steps;
        if (s[i++] != '/' || !readEvent(s, i, true, e.lowerId, e.lowerParam, e.lowerClick, e.lowerRelease)) return false;
    }
    if (i != s.size() || !std::isfinite(e.scale)) return false;
    out = e;
    return true;
}

ControlMethod eventMethod(const EventMethod& e) {
    auto event = [](uint32_t id, bool withParam, int32_t param, bool click, int32_t release) {
        std::string s = "#" + std::to_string(id);
        if (withParam) s += "(" + std::to_string(param) + (click ? "," + std::to_string(release) : "") + ")";
        return s;
    };
    ControlMethod m{MethodKind::KeyEvent, std::string(), e.scale};
    switch (e.mode) {
        case EventMethod::Mode::Position: {
            m.var = event(e.id, false, 0, false, 0);
            if (e.offset != 0.0) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "@%.10g", e.offset);
                m.var += buf;
            }
            break;
        }
        case EventMethod::Mode::Toggle: m.var = event(e.id, true, e.param, e.click, e.release); break;
        case EventMethod::Mode::Steps:
            m.var = event(e.id, true, e.param, e.click, e.release) + "/" +
                    event(e.lowerId, true, e.lowerParam, e.lowerClick, e.lowerRelease);
            break;
    }
    return m;
}

int32_t eventParam(const EventMethod& e, double value) {
    const double p = std::round(e.offset + value * e.scale);
    if (!(p > static_cast<double>(INT32_MIN))) return INT32_MIN;
    if (!(p < static_cast<double>(INT32_MAX))) return INT32_MAX;
    return static_cast<int32_t>(p);
}

double eventTarget(Control c, const Frame& f) {
    switch (c) {
        case Control::Spoilers: return f.v[kSpoilersHandle];
        case Control::Flaps: return std::isfinite(f.v[kFlapsHandleIndex]) ? std::round(f.v[kFlapsHandleIndex]) : NAN;
        case Control::Gear: return std::isfinite(f.v[kGearHandle]) ? (f.v[kGearHandle] >= 0.5 ? 1.0 : 0.0) : NAN;
        case Control::Throttle: break;
    }
    return NAN;
}

std::string keyEventName(uint32_t id) {
    if (id >= kThirdPartyEventFirst) {
        return "#" + std::to_string(id) + " (third-party +" + std::to_string(id - kThirdPartyEventFirst) + ")";
    }
    // KEY_ID_MIN (0x10000) + n, from MSFS_EventsEnum.h: what the controls' own bindings send, and
    // what's often seen alongside.
    static const struct {
        uint32_t n;
        const char* name;
    } kNames[] = {
        {34, "GEAR_TOGGLE"},         {52, "BRAKES"},          {53, "SPOILERS_TOGGLE"},  {59, "FLAPS_UP"},
        {67, "FLAPS_DOWN"},          {162, "FLAPS_SET"},      {164, "GEAR_SET"},        {216, "PARKING_BRAKES"},
        {222, "FLAPS_INCR"},         {223, "FLAPS_DECR"},     {226, "AXIS_ELEVATOR_SET"}, {227, "AXIS_AILERONS_SET"},
        {228, "AXIS_RUDDER_SET"},    {229, "AXIS_THROTTLE_SET"}, {230, "AXIS_ELEV_TRIM_SET"}, {250, "SPOILERS_SET"},
        {316, "GEAR_PUMP"},          {317, "SPOILERS_ARM_TOGGLE"}, {528, "SPOILERS_ON"},  {529, "SPOILERS_OFF"},
        {530, "SPOILERS_ARM_ON"},    {531, "SPOILERS_ARM_OFF"}, {532, "SPOILERS_ARM_SET"}, {543, "GEAR_UP"},
        {544, "GEAR_DOWN"},          {846, "AXIS_SPOILER_SET"}, {851, "AXIS_LEFT_BRAKE_SET"}, {852, "AXIS_RIGHT_BRAKE_SET"},
        {998, "AXIS_FLAPS_SET"},     {1051, "ROTOR_BRAKE"},   {1560, "THROTTLE_AXIS_SET_EX1"},
    };
    for (const auto& k : kNames) {
        if (id == 0x10000 + k.n) return std::string(k.name) + " (" + std::to_string(id) + ")";
    }
    return "key event " + std::to_string(id);
}

int EventStepper::waitFrames(Control c) { return c == Control::Gear ? kGearWaitFrames : kWaitFrames; }

void FlapNotches::begin(int64_t nowMs) {
    if (!(nowMs - notchMs_ < kSettleMs)) lever_ = NAN;
    since_ = 1 << 20;
}

int FlapNotches::step(double target, double handle, int64_t nowMs, bool force) {
    if (!std::isfinite(handle)) return 0;
    if (since_ < (1 << 20)) ++since_;
    const double h = std::round(handle);
    if (!(h == handle_)) {
        handle_ = h;
        movedMs_ = nowMs;
    }
    if (!std::isfinite(lever_) || nowMs - movedMs_ >= kSettleMs) lever_ = h;
    if (!std::isfinite(target) || std::fabs(lever_ - target) < 0.5) return 0;
    if (!force && since_ < kFrames) return 0;
    const int dir = target > lever_ ? 1 : -1;
    lever_ += dir;
    movedMs_ = notchMs_ = nowMs;
    since_ = 0;
    return dir;
}

bool FlapNotches::at(double target) const {
    return !std::isfinite(lever_) || !std::isfinite(target) || std::fabs(lever_ - target) < 0.5;
}

int EventStepper::step(double target, double live, double size, int waitFrames, bool force) {
    if (since_ < (1 << 20)) ++since_;
    if (!std::isfinite(target) || !std::isfinite(live) || !(size > 0.0)) return 0;
    const double half = 0.5 * size;
    if (std::fabs(live - target) < half) return 0;
    const bool arrived = std::isnan(expect_) || std::fabs(live - expect_) < half;
    if ((!force && since_ < kMinFrames) || (!arrived && since_ < waitFrames)) return 0;
    const int dir = target > live ? 1 : -1;
    expect_ = live + dir * size;
    since_ = 0;
    return dir;
}

void WayPicker::begin(Control c, const ControlProfile& cp) {
    *this = WayPicker();
    c_ = c;
    for (int d = 0; d < kDirections; ++d) {
        for (const ControlMethod& m : cp.waysTo(static_cast<Direction>(d))) {
            const auto it = std::find(ways_.begin(), ways_.end(), m);
            const int i = static_cast<int>(it - ways_.begin());
            if (it == ways_.end()) ways_.push_back(m);
            if (std::find(order_[d].begin(), order_[d].end(), i) == order_[d].end()) order_[d].push_back(i);
        }
    }
    current_ = multi() ? -1 : 0;
}

bool WayPicker::drop(size_t i) {
    if (i >= ways_.size()) return false;
    ways_.erase(ways_.begin() + static_cast<std::ptrdiff_t>(i));
    bool left = true;
    for (std::vector<int>& o : order_) {
        o.erase(std::remove(o.begin(), o.end(), static_cast<int>(i)), o.end());
        for (int& k : o) k -= k > static_cast<int>(i) ? 1 : 0;
        left = left && !o.empty();
    }
    current_ = multi() ? -1 : 0;
    return left;
}

WayPicker::Pick WayPicker::step(double target, const Frame& f) {
    Pick out;
    // What says the control has moved: its value, by half a notch (the gear: its command), or what
    // it moves, a little.
    double outcome = NAN, tol = 0.5, outcomeTol = 0.05;
    switch (c_) {
        case Control::Gear: {
            double sum = 0.0;
            int n = 0;
            for (const int ch : {kGearCenter, kGearLeft, kGearRight}) {
                if (!std::isfinite(f.v[ch])) continue;
                sum += f.v[ch];
                ++n;
            }
            outcome = n > 0 ? sum / n : NAN;
            live_ = gear_.update(outcome);
            break;
        }
        case Control::Flaps:
            live_ = f.v[kFlapsHandleIndex];
            outcome = f.v[kFlapsLeft];
            outcomeTol = 0.02;
            break;
        case Control::Spoilers:
            live_ = f.v[kSpoilersHandle];
            outcome = f.v[kSpoilersLeft];
            tol = 0.05;
            outcomeTol = 0.03;
            break;
        case Control::Throttle: live_ = NAN; break;
    }
    if (!multi()) return out;
    const int before = current_;
    if (current_ < 0) {
        // Nothing sent yet: the way toward where the clip has it, out or in.
        dir_ = std::isfinite(target) && target >= tol ? Direction::Extend : Direction::Retract;
        current_ = order_[static_cast<int>(dir_)].front();
    }
    std::vector<int>& order = order_[static_cast<int>(dir_)];
    const bool off = std::isfinite(target) && std::isfinite(live_) && std::fabs(target - live_) >= tol;
    if (pending_) {
        ++since_;
        const double sign = dir_ == Direction::Extend ? 1.0 : -1.0;
        const bool moved = (std::isfinite(live_) && std::isfinite(from_) && sign * (live_ - from_) >= tol) ||
                           (std::isfinite(outcome) && std::isfinite(fromOutcome_) && sign * (outcome - fromOutcome_) >= outcomeTol);
        if (moved) {
            if (!worked_) {
                worked_ = true;
                out.worked = current_;
                if (at_ > 0) {
                    std::rotate(order.begin(), order.begin() + static_cast<std::ptrdiff_t>(at_),
                                order.begin() + static_cast<std::ptrdiff_t>(at_) + 1);
                    at_ = 0;
                    out.promoted = true;
                }
            }
            from_ = live_;
            fromOutcome_ = outcome;
            since_ = 0;
        } else if (!worked_ && order.size() > 1 && since_ >= EventStepper::waitFrames(c_)) {
            out.gaveUp = current_;
            at_ = (at_ + 1) % order.size();
            current_ = order[at_];
            since_ = 0;
            from_ = live_;
            fromOutcome_ = outcome;
        }
        if (!off) pending_ = false;  // there (or the clip came back to where the aircraft is)
    }
    if (off) {
        const Direction want = target > live_ ? Direction::Extend : Direction::Retract;
        if (!pending_ || want != dir_) {
            // A new command: the first way that way.
            pending_ = true;
            dir_ = want;
            at_ = 0;
            since_ = 0;
            worked_ = false;
            from_ = live_;
            fromOutcome_ = outcome;
            current_ = order_[static_cast<int>(dir_)].front();
        }
    }
    out.way = current_;
    out.fresh = current_ != before;
    out.dir = dir_;
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
            if (isOwnEvent(m)) {
                EventMethod e;
                if (!parseEventMethod(m, e)) return "event " + m.var;
                auto name = [](uint32_t id) {
                    return id >= kThirdPartyEventFirst ? "the aircraft's own event #" + std::to_string(id)
                                                       : "key event #" + std::to_string(id);
                };
                auto params = [](int32_t param, bool click, int32_t release) {
                    return std::to_string(param) + (click ? ", then " + std::to_string(release) : "");
                };
                switch (e.mode) {
                    case EventMethod::Mode::Position:
                        return name(e.id) + (e.offset != 0.0 ? " (from " + m.var + ")" : "");
                    case EventMethod::Mode::Toggle:
                        return name(e.id) + " (" + params(e.param, e.click, e.release) + ") as a toggle";
                    case EventMethod::Mode::Steps:
                        return name(e.id) + " (" + params(e.param, e.click, e.release) + ") and " +
                               (e.lowerId == e.id ? "(" : name(e.lowerId) + " (") +
                               params(e.lowerParam, e.lowerClick, e.lowerRelease) + "), a step at a time";
                }
            }
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
    std::string s;
    if (cp.perDirection()) {
        for (int d = 0; d < kDirections; ++d) {
            s += std::string(d ? "; " : "") + directionKey(c, static_cast<Direction>(d)) + ": ";
            const std::vector<ControlMethod> ways = cp.waysTo(static_cast<Direction>(d));
            for (size_t i = 0; i < ways.size(); ++i) s += (i ? ", then " : "") + describeMethod(c, ways[i]);
        }
    } else {
        s = describeMethod(c, cp.method);
    }
    if (c == Control::Spoilers && cp.lever.valid()) {
        s += ", through " + cp.lever.var + "'s calibration (" + std::to_string(cp.lever.forward.size()) + " points)";
    }
    if (c != Control::Throttle) return s;
    if (cp.lever.valid()) s += ", lever from L:var " + cp.lever.var;
    switch (cp.reverseBy) {
        case ReverseBy::Events: s += ", reverse with SET_THROTTLEn_REVERSE_THRUST_ON"; break;
        case ReverseBy::Hold: s += ", reverse by holding THROTTLE_DECR"; break;
        case ReverseBy::Toggle: s += ", reverse with toggle reverse (THROTTLE_REVERSE_THRUST_TOGGLE)"; break;
        case ReverseBy::Axis: s += ", reverse on the axis"; break;
        case ReverseBy::None: s += ", no reverse"; break;
        case ReverseBy::Unknown: break;
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

namespace {

// Whether `lever` is beyond the forward table's idle, the other way from full.
bool beyondIdle(const LeverCalibration& cal, double lever) {
    double lo = 0.0, hi = 0.0;
    const double tol = leverRange(cal.forward, lo, hi);
    const double idle = cal.forward.front().lever;
    const double toFull = cal.forward.back().lever >= idle ? 1.0 : -1.0;
    return (lever - idle) * toFull < -tol;
}

}  // namespace

bool leverAxis(const LeverCalibration& cal, double lever, double clipReverser, bool liveReverse, double& axis) {
    if (!std::isfinite(lever) || cal.forward.size() < 2) return false;
    const double idle = cal.forward.front().lever;
    const bool beyond = beyondIdle(cal, lever);  // the other way from full
    const bool separate = cal.reverse.size() >= 2;
    const bool clipReverse = std::isfinite(clipReverser) ? clipReverser >= 0.5 : separate && beyond;
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
    if (clipReverse && beyond) return false;
    axis = invertLever(cal.forward, lever);
    return true;
}

bool leverInReverse(const LeverCalibration& cal, double lever) {
    return std::isfinite(lever) && cal.forward.size() >= 2 && cal.reverse.size() >= 2 && beyondIdle(cal, lever);
}

double eventsReverseAmount(double fromLever, bool leverShown, double pct, double lower) {
    if (leverShown && std::isfinite(fromLever)) return fromLever;
    if (!(lower < 0.0) || !std::isfinite(pct) || !(pct < 0.0)) return fromLever;
    const double amount = pct / lower;
    return std::isfinite(fromLever) && fromLever > amount ? fromLever : amount;
}

const char* spoilerPositionEvent(const ControlMethod& m) {
    if (m.kind != MethodKind::KeyEvent) return nullptr;
    if (m.var.empty()) return "SPOILERS_SET";
    return m.var == kKeySpoilersAxis ? kKeySpoilersAxis : nullptr;
}

double spoilerCommand(const LeverCalibration& cal, const ControlMethod& m, double handle) {
    const char* event = spoilerPositionEvent(m);
    if (!std::isfinite(handle) || !event || !cal.valid() || cal.var != event) return handle;
    return invertLever(cal.forward, handle);
}

double spoilerReach(const LeverCalibration& cal) {
    if (!cal.valid()) return NAN;
    double top = -INFINITY;
    for (const LeverPoint& p : cal.forward) top = std::isfinite(p.lever) && p.lever > top ? p.lever : top;
    return std::isfinite(top) ? top : NAN;
}

std::vector<GroundSpoilerStretch> groundSpoilerStretches(const LeverCalibration& cal, const std::vector<Frame>& frames) {
    std::vector<GroundSpoilerStretch> out;
    const double reach = spoilerReach(cal);
    if (!std::isfinite(reach)) return out;
    const double beyond = reach + kGroundSpoilerMargin;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (!(frames[i].v[kSpoilersHandle] > beyond) || !(frames[i].v[kOnGround] >= 0.5)) continue;
        // Until the handle is back within the lever's reach (through gaps in the clip).
        size_t j = i;
        while (j + 1 < frames.size() && !(frames[j + 1].v[kSpoilersHandle] <= beyond)) ++j;
        out.push_back({frames[i].t, frames[j].t});
        i = j;
    }
    return out;
}

void SpoilerArming::begin(std::vector<GroundSpoilerStretch> stretches, double reach) {
    clear();
    stretches_ = std::move(stretches);
    reach_ = reach;
}

void SpoilerArming::clear() {
    stretches_.clear();
    armed_ = deployed_ = gaveUp_ = armChecked_ = false;
    stretch_ = -1;
    armedT_ = waitFrom_ = NAN;
}

SpoilerArming::Step SpoilerArming::step(double t, double clipHandle, double liveHandle, double liveArmed, bool jumped) {
    Step out;
    // The stretch t is in, else the one it's armed ahead of (the clip's lever retracted there).
    int in = -1;
    bool inside = false;
    for (size_t i = 0; i < stretches_.size() && in < 0; ++i) {
        if (t >= stretches_[i].startT && t <= stretches_[i].endT) {
            in = static_cast<int>(i);
            inside = true;
        }
    }
    for (size_t i = 0; i < stretches_.size() && in < 0; ++i) {
        // Armed with the lever retracted; then up to the stretch, as the clip's ground spoilers
        // start coming out (the A380's handle 0.38 to 0.57 before it's beyond the lever's reach).
        const double start = stretches_[i].startT;
        const bool armedFor = stretch_ == static_cast<int>(i) && !jumped;
        if (t >= start - kLeadSec && t < start && (armedFor || !(clipHandle > kRetractedMax))) in = static_cast<int>(i);
    }
    if (in < 0) {
        out.disarm = armed_;
        armed_ = deployed_ = gaveUp_ = false;
        stretch_ = -1;
        return out;
    }
    if (in != stretch_ || jumped) {
        // Another stretch, or a jump: armed (again: the lever may have been moved since), and
        // the ground spoilers waited for afresh.
        stretch_ = in;
        deployed_ = gaveUp_ = armChecked_ = false;
        waitFrom_ = NAN;
        armedT_ = t;
        armed_ = out.arm = true;
    }
    if (gaveUp_) return out;
    out.mode = Mode::Armed;
    if (!armChecked_ && t - armedT_ >= kArmCheckSec) {
        armChecked_ = true;
        out.armAgain = std::isfinite(liveArmed) && liveArmed < 0.5;
    }
    if (!inside) return out;
    if (!std::isfinite(waitFrom_)) waitFrom_ = t;
    if (!deployed_ && liveHandle > reach_ + kGroundSpoilerMargin) deployed_ = out.deployed = true;
    if (!deployed_ && t - waitFrom_ >= kWaitSec) {
        gaveUp_ = out.gaveUp = true;
        out.mode = Mode::Lever;
    }
    return out;
}

bool reversing(const Frame& f, int engine) {
    if (engine < 0 || engine > 3) return false;
    const double nozzle = f.v[kReverserNozzle1 + engine];
    const double lever = f.v[kThrottle1 + engine];
    return (std::isfinite(nozzle) && nozzle >= 50.0) || (std::isfinite(lever) && lever < -1.0);
}

ReverseToggle::Step ReverseToggle::step(bool clipReverse, bool liveReverse, double liveThrottle) {
    Step out;
    const bool liveForward = liveThrottle > kForwardPct;
    const bool liveIdle = !liveReverse && !liveForward;
    const double moved = std::isfinite(liveThrottle) && std::isfinite(lastThrottle_) ? liveThrottle - lastThrottle_ : 0.0;
    lastThrottle_ = liveThrottle;
    if (clipReverse != want_) {
        want_ = clipReverse;
        tries_ = 0;
    }
    if (wait_ > 0) {
        // A toggle settling: the levers stay at idle, unless the aircraft already shows reverse.
        --wait_;
        if (!(on_ && liveReverse)) {
            out.drive = last_ = Drive::Idle;
            return out;
        }
        wait_ = 0;
    }
    // In reverse a reverse amount can't make forward thrust; out of it nothing reverses. Unless
    // that's the throttle simvar still heading back to idle from before the toggle.
    const bool otherwise = on_ ? liveForward && last_ == Drive::Reverse : liveReverse;
    const bool heading = on_ ? moved < -kHeadingPct : moved > kHeadingPct;
    if (!otherwise) evidence_ = 0;
    else if (!heading) ++evidence_;
    if (evidence_ >= kEvidenceFrames) {
        on_ = !on_;
        evidence_ = probe_ = 0;
        ++corrections_;
        out.corrected = true;
    }
    const bool was = confirmed_;
    confirmed_ = on_ && (confirmed_ || liveReverse);
    if (clipReverse == on_) {
        idle_ = 0;
        if (on_ && !confirmed_ && last_ == Drive::Reverse && !liveForward && !(moved > kHeadingPct)) {
            // The first amount sent, and no forward thrust from it (a toggle not taken makes some).
            confirmed_ = ++probe_ >= kProbeFrames;
        }
        if (confirmed_ && !was) out.confirmed = liveReverse ? Confirmed::Shown : Confirmed::NoThrust;
        out.drive = last_ = on_ ? Drive::Reverse : Drive::Forward;
        out.cap = confirmed_ ? 1.0 : kProbeAmount;
        return out;
    }
    // Into or out of reverse: the levers to idle, then the toggle. After kTries it gives up until
    // the clip changes, idle meanwhile in whichever mode the aircraft is in.
    out.drive = last_ = Drive::Idle;
    ++idle_;
    if (tries_ >= kTries || (!(liveIdle && idle_ > 1) && idle_ < kIdleFrames)) return out;
    idle_ = 0;
    on_ = !on_;
    confirmed_ = false;
    evidence_ = probe_ = 0;
    wait_ = kSettleFrames;
    ++tries_;
    ++toggles_;
    out.toggle = true;
    return out;
}

const char* reverseByKey(ReverseBy r) {
    switch (r) {
        case ReverseBy::None: return "none";
        case ReverseBy::Axis: return "axis";
        case ReverseBy::Events: return "events";
        case ReverseBy::Hold: return "hold";
        case ReverseBy::Toggle: return "toggle";
        case ReverseBy::Unknown: break;
    }
    return "";
}

bool parseReverseByKey(const std::string& key, ReverseBy& out) {
    for (const ReverseBy r : {ReverseBy::None, ReverseBy::Axis, ReverseBy::Events, ReverseBy::Hold, ReverseBy::Toggle}) {
        if (key == reverseByKey(r)) {
            out = r;
            return true;
        }
    }
    return false;
}

std::vector<AutothrustStretch> autothrustStretches(const LeverCalibration& cal, const std::vector<Frame>& frames,
                                                   int engine) {
    std::vector<AutothrustStretch> out;
    if (!cal.valid() || engine < 0 || engine > 3) return out;
    const int leverCh = kLVarThrottle1 + (cal.var.find('#') != std::string::npos ? engine : 0);
    const int demandCh = kThrottle1 + engine;
    const double idle = cal.forward.front().axis;
    const size_t n = frames.size();
    auto axisAt = [&](size_t i) {
        const double lv = frames[i].v[leverCh];
        return std::isfinite(lv) ? invertLever(cal.forward, lv) : NAN;
    };
    size_t i = 0;
    while (i < n) {
        const double held = axisAt(i);
        if (!std::isfinite(held)) {
            ++i;
            continue;
        }
        // The hold: as long as the lever stays there, through short gaps in the clip.
        size_t last = i;
        size_t j = i + 1;
        for (; j < n; ++j) {
            const double a = axisAt(j);
            if (!std::isfinite(a)) {
                if (frames[j].t - frames[last].t > kAutothrustGapSec) break;
                continue;
            }
            if (std::fabs(a - held) > kAutothrustHoldAxis) break;
            last = j;
        }
        const double startT = frames[i].t;
        const double holdEnd = frames[last].t;
        if (held > idle + 2.0 * kAutothrustHoldAxis && holdEnd - startT > kAutothrustSettleSec) {
            double lo = INFINITY, hi = -INFINITY;
            for (size_t k = i; k <= last; ++k) {
                const double d = frames[k].v[demandCh];
                if (frames[k].t < startT + kAutothrustSettleSec || !std::isfinite(d)) continue;
                lo = d < lo ? d : lo;
                hi = d > hi ? d : hi;
            }
            if (hi - lo > kAutothrustRangePct) {
                // And the lever coming back from the hold towards idle, if it does next.
                double endT = holdEnd;
                for (size_t k = last + 1; k < n && frames[k].t - holdEnd <= kAutothrustRetardSec; ++k) {
                    const double a = axisAt(k);
                    if (!std::isfinite(a) || a > held || a <= idle + kAutothrustHoldAxis) break;
                    endT = frames[k].t;
                }
                out.push_back({startT, endT});
            }
        }
        i = last + 1;
    }
    return out;
}

double ThrustFollower::step(double clip, double live, double lo, double hi, double dt) {
    if (hi < lo) hi = lo;
    const double target = clip < 0.0 ? 0.0 : (clip > 100.0 ? 100.0 : clip);
    const double guess = lo + target / 100.0 * (1.0 - lo);
    const double off = target - live;
    const double want = guess + kGain * off + integral_;
    const bool atLimit = (want >= hi && off > 0.0) || (want <= lo && off < 0.0);
    if (!atLimit && dt > 0.0) {
        integral_ += kIntegral * off * dt;
        integral_ = integral_ < -1.0 ? -1.0 : (integral_ > 1.0 ? 1.0 : integral_);
    }
    const double axis = guess + kGain * off + integral_;
    return axis < lo ? lo : (axis > hi ? hi : axis);
}

LeverWatch::Result LeverWatch::step(double off, int64_t nowMs) {
    if (!std::isfinite(off)) {  // nothing to tell: start over
        offSinceMs_ = backSinceMs_ = -1;
        return Result::Quiet;
    }
    if (off > kOffAxis) {
        backSinceMs_ = -1;
        if (offSinceMs_ < 0) {
            offSinceMs_ = nowMs;
            worst_ = 0.0;
        }
        worst_ = std::fmax(worst_, off);
        if (nowMs - offSinceMs_ < kOffMs || tries_ >= kTries) return Result::Quiet;
        if (firedMs_ >= 0 && nowMs - firedMs_ < kRetryMs) return Result::Quiet;
        ++tries_;
        firedMs_ = nowMs;
        waitingBack_ = true;
        return Result::Disconnect;
    }
    if (backSinceMs_ < 0) backSinceMs_ = nowMs;
    if (nowMs - backSinceMs_ < kBackMs) return Result::Quiet;
    offSinceMs_ = -1;
    if (!waitingBack_) return Result::Quiet;
    waitingBack_ = false;
    return Result::Back;
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

namespace {

bool isDigit(char c) { return c >= '0' && c <= '9'; }

// "pmdg 737-800" -> "pmdg 737", "a350-1000" -> "a350", "pmdg 737-900er" -> "pmdg 737". Not
// "pc-12" or "an-225": the dash has to follow the model's digits.
std::string withoutVariant(const std::string& folder) {
    for (size_t i = 1; i < folder.size(); ++i) {
        if (folder[i] != '-' || !isDigit(folder[i - 1])) continue;
        size_t j = i + 1;
        while (j < folder.size() && isDigit(folder[j])) ++j;
        const size_t digits = j - (i + 1);
        if (digits == 3 || digits == 4) return folder.substr(0, i);
    }
    return folder;
}

}  // namespace

std::string aircraftFamily(const std::string& path) {
    const std::string root = "simobjects/";
    size_t at = path.compare(0, root.size(), root) == 0 ? 0 : path.find("/" + root);
    if (at == std::string::npos) return path;
    if (path[at] == '/') ++at;
    const size_t category = path.find('/', at + root.size());  // airplanes/, rotorcraft/...
    if (category == std::string::npos) return path;
    const size_t folderEnd = path.find('/', category + 1);
    const std::string folder = path.substr(category + 1, folderEnd == std::string::npos ? std::string::npos
                                                                                         : folderEnd - category - 1);
    if (folder.empty()) return path;
    return path.substr(0, category + 1) + withoutVariant(folder);
}

namespace {

// The gear's "#69927+keys" from the builds before per-direction ways (the PMDG 777): up by that
// event's position, down by the gear key events, then the aircraft's other own ways (ownGearWays).
void readWithKeys(const std::string& key, ControlProfile& cp) {
    const std::string suffix = "+keys";
    std::string& var = cp.method.var;
    if (cp.method.kind != MethodKind::KeyEvent || cp.perDirection() || var.size() <= suffix.size() ||
        var.compare(var.size() - suffix.size(), std::string::npos, suffix) != 0) {
        return;
    }
    var.resize(var.size() - suffix.size());
    std::vector<ControlMethod>& down = cp.ways[static_cast<int>(Direction::Extend)];
    down.push_back({MethodKind::KeyEvent, std::string(), 1.0});
    for (const ControlMethod& m : ownGearWays(key)) {
        if (!(m == cp.method) && std::find(down.begin(), down.end(), m) == down.end()) down.push_back(m);
    }
    cp.ways[static_cast<int>(Direction::Retract)].push_back(cp.method);
}

}  // namespace

bool isTitleKey(const std::string& key) { return key.compare(0, 6, "title:") == 0; }

std::string aircraftFolder(const std::string& raw) {
    std::string path;
    for (const char ch : raw) {
        const char c = ch == '\\' ? '/' : ch;
        path.push_back(c >= 'A' && c <= 'Z' ? static_cast<char>(c + 32) : c);
    }
    const std::string root = "simobjects/";
    const size_t at = path.compare(0, root.size(), root) == 0 ? 0 : path.find("/" + root);
    if (at != std::string::npos && at > 0) path.erase(0, at + 1);
    const size_t slash = path.rfind('/');
    const std::string file = slash == std::string::npos ? std::string() : path.substr(slash + 1);
    const size_t dot = file.rfind('.');
    if (dot != std::string::npos && (file.compare(dot, 4, ".cfg") == 0 || file.compare(dot, 4, ".air") == 0) &&
        dot + 4 == file.size()) {
        path.resize(slash);
    }
    return path;
}

bool AircraftIdentity::onTitle(const std::string& title) {
    if (title.empty() || title == title_) return false;
    title_ = title;
    if (!path_.empty() && pathTitle_.empty()) {
        pathTitle_ = title;  // the path came first: it's this aircraft's
        return false;
    }
    source_ = Source::Title;
    const std::string key = "title:" + title;
    if (key == key_) return false;
    key_ = key;
    return true;
}

bool AircraftIdentity::onEvent(const std::string& path) {
    if (path.empty()) return false;
    // The event can come before the new title: then the title it goes with is the next one.
    if (source_ == Source::Title) pathTitle_ = title_;
    else if (path != path_) pathTitle_.clear();
    return trust(path, Source::Event);
}

bool AircraftIdentity::onState(const std::string& path) {
    stale_ = false;
    if (path.empty() || source_ == Source::Event) return false;  // the event is newer than the state
    if (path == path_ && !pathTitle_.empty() && pathTitle_ != title_) {
        stale_ = true;
        return false;
    }
    pathTitle_ = title_;  // empty before the first title: then it's the next one's
    return trust(path, Source::State);
}

bool AircraftIdentity::trust(const std::string& path, Source source) {
    path_ = path;
    source_ = source;
    if (path == key_) return false;
    key_ = path;
    return true;
}

size_t ProfileStore::groupVariants() {
    size_t changed = 0;
    std::vector<AircraftProfile> grouped;
    for (AircraftProfile& p : profiles_) {
        const std::string family = aircraftFamily(p.key);
        if (family != p.key) {
            p.key = family;
            ++changed;
        }
        auto same = std::find_if(grouped.begin(), grouped.end(),
                                 [&](const AircraftProfile& g) { return g.key == family; });
        if (same == grouped.end()) {
            grouped.push_back(std::move(p));
        } else {
            ++changed;
            if (p.updatedUtcMs > same->updatedUtcMs) *same = std::move(p);
        }
    }
    profiles_.swap(grouped);
    return changed;
}

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
            // Each way's list, in the order playback tries them ("down": [...], "up": [...]).
            for (int d = 0; d < kDirections; ++d) {
                if (c.ways[d].empty()) continue;
                w.key(directionKey(controlAt(i), static_cast<Direction>(d))).beginArray();
                for (const ControlMethod& m : c.ways[d]) {
                    w.beginObject().kv("method", methodKey(m.kind)).kv("var", m.var).kv("scale", m.scale).endObject();
                }
                w.endArray();
            }
            if (c.reverseBy != ReverseBy::Unknown) w.kv("reverseBy", reverseByKey(c.reverseBy));
            if (c.leadSec > 0.0) w.kv("leadSec", c.leadSec);
            if (!c.lever.var.empty()) {
                // [axis, lever] pairs
                w.key("lever").beginObject();
                w.kv("var", c.lever.var);
                writeLeverTable(w, "forward", c.lever.forward);
                writeLeverTable(w, "reverse", c.lever.reverse);
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
            for (int d = 0; d < kDirections; ++d) {
                const JsonValue* ways = c->get(directionKey(controlAt(i), static_cast<Direction>(d)));
                for (size_t k = 0; ways && ways->isArray() && k < ways->array.size(); ++k) {
                    const JsonValue& way = ways->array[k];
                    ControlMethod m;
                    if (!way.isObject() || !parseMethodKey(way.getString("method", "none"), m.kind)) continue;
                    if (m.kind == MethodKind::None) continue;
                    m.var = way.getString("var", "");
                    m.scale = way.getNumber("scale", 1.0);
                    cp.ways[d].push_back(m);
                }
            }
            if (controlAt(i) == Control::Gear) readWithKeys(p.key, cp);
            parseReverseByKey(c->getString("reverseBy", ""), cp.reverseBy);
            const double lead = c->getNumber("leadSec", 0.0);
            cp.leadSec = controlAt(i) != Control::Throttle && lead >= kMinLeadSec ? std::min(lead, kMaxLeadSec) : 0.0;
            if (const JsonValue* lv = c->get("lever")) {
                cp.lever.var = lv->getString("var", "");
                cp.lever.forward = readLeverTable(lv->get("forward"));
                cp.lever.reverse = readLeverTable(lv->get("reverse"));
                parseReverseByKey(lv->getString("reverseBy", ""), cp.reverseBy);  // where it was first kept
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
