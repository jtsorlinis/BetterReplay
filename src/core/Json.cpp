#include "Json.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace br {

// ---------------------------------------------------------------------------------- writer

void JsonWriter::separator() {
    if (afterKey_) {
        afterKey_ = false;
        return;
    }
    if (depth_ > 0) {
        if (!first_[depth_ - 1]) out_ += ',';
        first_[depth_ - 1] = false;
    }
}

void JsonWriter::escaped(const char* s) {
    out_ += '"';
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(s); *p; ++p) {
        const unsigned char c = *p;
        switch (c) {
            case '"': out_ += "\\\""; break;
            case '\\': out_ += "\\\\"; break;
            case '\n': out_ += "\\n"; break;
            case '\r': out_ += "\\r"; break;
            case '\t': out_ += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out_ += buf;
                } else {
                    out_ += static_cast<char>(c);
                }
        }
    }
    out_ += '"';
}

JsonWriter& JsonWriter::beginObject() {
    separator();
    out_ += '{';
    if (depth_ < kMaxDepth) first_[depth_] = true;
    ++depth_;
    return *this;
}

JsonWriter& JsonWriter::endObject() {
    out_ += '}';
    if (depth_ > 0) --depth_;
    return *this;
}

JsonWriter& JsonWriter::beginArray() {
    separator();
    out_ += '[';
    if (depth_ < kMaxDepth) first_[depth_] = true;
    ++depth_;
    return *this;
}

JsonWriter& JsonWriter::endArray() {
    out_ += ']';
    if (depth_ > 0) --depth_;
    return *this;
}

JsonWriter& JsonWriter::key(const char* k) {
    separator();
    escaped(k);
    out_ += ':';
    afterKey_ = true;
    return *this;
}

JsonWriter& JsonWriter::value(const char* s) {
    separator();
    escaped(s ? s : "");
    return *this;
}

JsonWriter& JsonWriter::value(double d) {
    separator();
    if (!std::isfinite(d)) {
        out_ += "null";
        return *this;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.10g", d);
    out_ += buf;
    return *this;
}

JsonWriter& JsonWriter::value(int64_t i) {
    separator();
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(i));
    out_ += buf;
    return *this;
}

JsonWriter& JsonWriter::value(bool b) {
    separator();
    out_ += b ? "true" : "false";
    return *this;
}

JsonWriter& JsonWriter::null() {
    separator();
    out_ += "null";
    return *this;
}

// ---------------------------------------------------------------------------------- parser

namespace {

class Parser {
public:
    Parser(const char* text, size_t length) : p_(text), end_(text + length) {}

    bool parseDocument(JsonValue& out) {
        skipWs();
        if (!parseValue(out, 0)) return false;
        skipWs();
        if (p_ != end_) return fail("trailing characters");
        return true;
    }

    std::string error;

private:
    static constexpr int kMaxDepth = 64;

    bool fail(const char* msg) {
        if (error.empty()) error = msg;
        return false;
    }

    void skipWs() {
        while (p_ < end_ && (*p_ == ' ' || *p_ == '\t' || *p_ == '\n' || *p_ == '\r')) ++p_;
    }

    bool literal(const char* word) {
        const size_t n = std::strlen(word);
        if (static_cast<size_t>(end_ - p_) < n || std::strncmp(p_, word, n) != 0) return fail("bad literal");
        p_ += n;
        return true;
    }

    bool parseValue(JsonValue& out, int depth) {
        if (depth > kMaxDepth) return fail("nesting too deep");
        if (p_ >= end_) return fail("unexpected end");
        switch (*p_) {
            case '{': return parseObject(out, depth);
            case '[': return parseArray(out, depth);
            case '"':
                out.type = JsonValue::Type::String;
                return parseString(out.string);
            case 't':
                out.type = JsonValue::Type::Bool;
                out.boolean = true;
                return literal("true");
            case 'f':
                out.type = JsonValue::Type::Bool;
                out.boolean = false;
                return literal("false");
            case 'n':
                out.type = JsonValue::Type::Null;
                return literal("null");
            default: return parseNumber(out);
        }
    }

    bool parseNumber(JsonValue& out) {
        const char* start = p_;
        if (p_ < end_ && (*p_ == '-' || *p_ == '+')) ++p_;
        while (p_ < end_ && ((*p_ >= '0' && *p_ <= '9') || *p_ == '.' || *p_ == 'e' || *p_ == 'E' ||
                             *p_ == '-' || *p_ == '+')) {
            ++p_;
        }
        if (p_ == start) return fail("unexpected character");
        char buf[64];
        const size_t n = static_cast<size_t>(p_ - start);
        if (n >= sizeof(buf)) return fail("number too long");
        std::memcpy(buf, start, n);
        buf[n] = '\0';
        char* parsedEnd = nullptr;
        out.number = std::strtod(buf, &parsedEnd);
        if (parsedEnd != buf + n) return fail("bad number");
        out.type = JsonValue::Type::Number;
        return true;
    }

    static void appendUtf8(std::string& s, unsigned cp) {
        if (cp < 0x80) {
            s += static_cast<char>(cp);
        } else if (cp < 0x800) {
            s += static_cast<char>(0xC0 | (cp >> 6));
            s += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            s += static_cast<char>(0xE0 | (cp >> 12));
            s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            s += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            s += static_cast<char>(0xF0 | (cp >> 18));
            s += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            s += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            s += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    bool hex4(unsigned& out) {
        if (end_ - p_ < 4) return fail("short \\u escape");
        out = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = *p_++;
            out <<= 4;
            if (c >= '0' && c <= '9') out |= static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') out |= static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') out |= static_cast<unsigned>(c - 'A' + 10);
            else return fail("bad \\u escape");
        }
        return true;
    }

    bool parseString(std::string& out) {
        ++p_;  // opening quote
        out.clear();
        while (p_ < end_) {
            const char c = *p_++;
            if (c == '"') return true;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (p_ >= end_) break;
            const char e = *p_++;
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp = 0;
                    if (!hex4(cp)) return false;
                    if (cp >= 0xD800 && cp <= 0xDBFF && end_ - p_ >= 6 && p_[0] == '\\' && p_[1] == 'u') {
                        p_ += 2;
                        unsigned lo = 0;
                        if (!hex4(lo)) return false;
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    }
                    appendUtf8(out, cp);
                    break;
                }
                default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }

    bool parseArray(JsonValue& out, int depth) {
        ++p_;
        out.type = JsonValue::Type::Array;
        skipWs();
        if (p_ < end_ && *p_ == ']') {
            ++p_;
            return true;
        }
        while (true) {
            out.array.emplace_back();
            skipWs();
            if (!parseValue(out.array.back(), depth + 1)) return false;
            skipWs();
            if (p_ >= end_) return fail("unterminated array");
            if (*p_ == ',') {
                ++p_;
                continue;
            }
            if (*p_ == ']') {
                ++p_;
                return true;
            }
            return fail("expected , or ]");
        }
    }

    bool parseObject(JsonValue& out, int depth) {
        ++p_;
        out.type = JsonValue::Type::Object;
        skipWs();
        if (p_ < end_ && *p_ == '}') {
            ++p_;
            return true;
        }
        while (true) {
            skipWs();
            if (p_ >= end_ || *p_ != '"') return fail("expected key");
            out.object.emplace_back();
            if (!parseString(out.object.back().first)) return false;
            skipWs();
            if (p_ >= end_ || *p_ != ':') return fail("expected :");
            ++p_;
            skipWs();
            if (!parseValue(out.object.back().second, depth + 1)) return false;
            skipWs();
            if (p_ >= end_) return fail("unterminated object");
            if (*p_ == ',') {
                ++p_;
                continue;
            }
            if (*p_ == '}') {
                ++p_;
                return true;
            }
            return fail("expected , or }");
        }
    }

    const char* p_;
    const char* end_;
};

}  // namespace

bool parseJson(const char* text, size_t length, JsonValue& out, std::string* error) {
    out = JsonValue();
    Parser parser(text, length);
    const bool ok = parser.parseDocument(out);
    if (!ok && error) *error = parser.error;
    return ok;
}

const JsonValue* JsonValue::get(const char* key) const {
    if (type != Type::Object) return nullptr;
    for (const auto& kv : object) {
        if (kv.first == key) return &kv.second;
    }
    return nullptr;
}

double JsonValue::getNumber(const char* key, double fallback) const {
    const JsonValue* v = get(key);
    return v && v->type == Type::Number ? v->number : fallback;
}

std::string JsonValue::getString(const char* key, const char* fallback) const {
    const JsonValue* v = get(key);
    return v && v->type == Type::String ? v->string : std::string(fallback);
}

bool JsonValue::getBool(const char* key, bool fallback) const {
    const JsonValue* v = get(key);
    return v && v->type == Type::Bool ? v->boolean : fallback;
}

}  // namespace br
