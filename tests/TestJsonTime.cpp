#include <cmath>
#include <cstring>
#include <string>

#include "Json.h"
#include "TestHarness.h"
#include "TimeUtil.h"

using namespace br;

TEST(Json_WriterProducesCompactEscapedOutput) {
    std::string s;
    JsonWriter w(s);
    w.beginObject();
    w.kv("a", 1);
    w.kv("b", "x\"y\\z\n");
    w.kv("c", true);
    w.kv("d", NAN);
    w.key("e").beginArray().value(1.5).value("q").null().endArray();
    w.key("f").beginObject().endObject();
    w.endObject();
    CHECK_EQ(s, std::string("{\"a\":1,\"b\":\"x\\\"y\\\\z\\n\",\"c\":true,\"d\":null,\"e\":[1.5,\"q\",null],\"f\":{}}"));
}

TEST(Json_ParsesNestedDocument) {
    const std::string text = " {\"op\":\"seek\",\"t\":12.5,\"flags\":[true,false,null],\"o\":{\"k\":-3e2}} ";
    JsonValue v;
    std::string err;
    REQUIRE(parseJson(text.data(), text.size(), v, &err));
    CHECK(v.isObject());
    CHECK_EQ(v.getString("op", ""), std::string("seek"));
    CHECK_NEAR(v.getNumber("t", 0), 12.5, 0);
    const JsonValue* flags = v.get("flags");
    REQUIRE(flags && flags->isArray());
    CHECK_EQ(flags->array.size(), 3u);
    CHECK(flags->array[0].boolean);
    CHECK(flags->array[2].type == JsonValue::Type::Null);
    CHECK_NEAR(v.get("o")->getNumber("k", 0), -300.0, 0);
    CHECK_NEAR(v.getNumber("missing", 7.0), 7.0, 0);
}

TEST(Json_ParsesUnicodeEscapes) {
    const std::string text = "{\"s\":\"caf\\u00e9 \\ud83d\\ude00\"}";
    JsonValue v;
    REQUIRE(parseJson(text.data(), text.size(), v, nullptr));
    CHECK_EQ(v.getString("s", ""), std::string("caf\xC3\xA9 \xF0\x9F\x98\x80"));
}

TEST(Json_RejectsMalformedInput) {
    const char* bad[] = {"", "{", "{\"a\":}", "[1,2", "{\"a\":1}x", "tru", "\"unterminated", "{a:1}"};
    for (const char* b : bad) {
        JsonValue v;
        std::string err;
        CHECK(!parseJson(b, std::strlen(b), v, &err));
        CHECK(!err.empty());
    }
}

TEST(Json_RoundTripThroughWriter) {
    std::string s;
    JsonWriter w(s);
    w.beginObject().kv("name", "Cessna 172 \"Skyhawk\"").kv("n", static_cast<int64_t>(1792247525000)).endObject();
    JsonValue v;
    REQUIRE(parseJson(s.data(), s.size(), v, nullptr));
    CHECK_EQ(v.getString("name", ""), std::string("Cessna 172 \"Skyhawk\""));
    CHECK_NEAR(v.getNumber("n", 0), 1792247525000.0, 0);
}

TEST(Time_FormatsKnownInstants) {
    CHECK_EQ(formatIsoUtc(0), std::string("1970-01-01T00:00:00Z"));
    CHECK_EQ(formatIsoUtc(1792247525000), std::string("2026-10-17T14:32:05Z"));
    CHECK_EQ(formatFileStamp(1792247525000), std::string("2026-10-17_1432"));
    CHECK_EQ(formatIsoUtc(951868799000), std::string("2000-02-29T23:59:59Z"));
    CHECK_EQ(formatIsoUtc(-1000), std::string("1969-12-31T23:59:59Z"));
}

TEST(Time_ParseRoundTrip) {
    int64_t ms = 0;
    REQUIRE(parseIsoUtc("2026-10-17T14:32:05Z", ms));
    CHECK_EQ(ms, static_cast<int64_t>(1792247525000));
    CHECK(!parseIsoUtc("not a date", ms));
    for (int64_t t = -86400000LL * 400; t < 86400000LL * 365 * 80; t += 86400000LL * 37 + 12345) {
        int64_t back = 0;
        REQUIRE(parseIsoUtc(formatIsoUtc(t), back));
        CHECK_EQ(back, t - (((t % 1000) + 1000) % 1000));
    }
}
