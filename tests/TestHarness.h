// Minimal dependency-free test harness for the platform-independent core.
#pragma once

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace th {

struct TestCase {
    const char* name;
    void (*fn)();
};

std::vector<TestCase>& registry();
void fail(const char* file, int line, const std::string& msg);

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

template <typename A, typename B>
std::string describe(const char* exprA, const char* exprB, const A& a, const B& b) {
    std::ostringstream os;
    os << exprA << " == " << exprB << "  (" << a << " vs " << b << ")";
    return os.str();
}

}  // namespace th

#define TEST(name)                                         \
    static void name();                                    \
    static th::Registrar name##_registrar(#name, &name);   \
    static void name()

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) th::fail(__FILE__, __LINE__, "CHECK(" #cond ")"); \
    } while (0)

#define REQUIRE(cond)                                                    \
    do {                                                                 \
        if (!(cond)) {                                                   \
            th::fail(__FILE__, __LINE__, "REQUIRE(" #cond ")");          \
            return;                                                      \
        }                                                                \
    } while (0)

#define CHECK_EQ(a, b)                                                          \
    do {                                                                        \
        const auto& th_a = (a);                                                 \
        const auto& th_b = (b);                                                 \
        if (!(th_a == th_b)) th::fail(__FILE__, __LINE__, th::describe(#a, #b, th_a, th_b)); \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                   \
    do {                                                                        \
        const double th_a = static_cast<double>(a);                             \
        const double th_b = static_cast<double>(b);                             \
        if (!(std::fabs(th_a - th_b) <= (eps)))                                 \
            th::fail(__FILE__, __LINE__, th::describe(#a, #b, th_a, th_b) + " eps " #eps); \
    } while (0)
