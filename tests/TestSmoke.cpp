#include "TestHarness.h"

TEST(Harness_Smoke) {
    CHECK(1 + 1 == 2);
    CHECK_EQ(2 * 3, 6);
    CHECK_NEAR(0.1 + 0.2, 0.3, 1e-12);
}
