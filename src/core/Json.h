// Small JSON writer and DOM parser: enough for CommBus messages and index.json.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace br {

// Appends compact JSON to a caller-owned string. Reusing the same string keeps its capacity,
// so steady-state status messages don't allocate.
class JsonWriter {
public:
    explicit JsonWriter(std::string& out) : out_(out) {}

    JsonWriter& beginObject();
    JsonWriter& endObject();
    JsonWriter& beginArray();
    JsonWriter& endArray();
    JsonWriter& key(const char* k);

    JsonWriter& value(const char* s);
    JsonWriter& value(const std::string& s) { return value(s.c_str()); }
    JsonWriter& value(double d);  // NaN/inf are written as null
    JsonWriter& value(int64_t i);
    JsonWriter& value(int i) { return value(static_cast<int64_t>(i)); }
    JsonWriter& value(bool b);
    JsonWriter& null();

    template <typename T>
    JsonWriter& kv(const char* k, const T& v) {
        key(k);
        return value(v);
    }

private:
    void separator();
    void escaped(const char* s);

    static constexpr int kMaxDepth = 32;
    std::string& out_;
    bool first_[kMaxDepth] = {};
    int depth_ = 0;
    bool afterKey_ = false;
};

struct JsonValue {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string string;
    std::vector<JsonValue> array;
    std::vector<std::pair<std::string, JsonValue>> object;

    bool isObject() const { return type == Type::Object; }
    bool isArray() const { return type == Type::Array; }
    const JsonValue* get(const char* key) const;  // null if absent or not an object

    double getNumber(const char* key, double fallback) const;
    std::string getString(const char* key, const char* fallback) const;
    bool getBool(const char* key, bool fallback) const;
};

bool parseJson(const char* text, size_t length, JsonValue& out, std::string* error);

}  // namespace br
