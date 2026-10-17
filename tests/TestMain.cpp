#include "TestHarness.h"

#include <cstring>

namespace th {

static int g_failures = 0;
static const char* g_current = "";

std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

void fail(const char* file, int line, const std::string& msg) {
    ++g_failures;
    std::printf("  FAIL %s\n       %s:%d: %s\n", g_current, file, line, msg.c_str());
}

}  // namespace th

// Usage: CoreTests.exe [substring-filter]
int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    int failedTests = 0;
    for (const th::TestCase& t : th::registry()) {
        if (filter && !std::strstr(t.name, filter)) continue;
        th::g_current = t.name;
        const int before = th::g_failures;
        t.fn();
        ++run;
        if (th::g_failures != before) {
            ++failedTests;
        } else {
            std::printf("  ok   %s\n", t.name);
        }
    }
    std::printf("\n%d test(s), %d failed, %d failed check(s)\n", run, failedTests, th::g_failures);
    return failedTests == 0 && run > 0 ? 0 : 1;
}
