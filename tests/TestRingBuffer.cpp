#include "RingBuffer.h"
#include "TestHarness.h"

using br::RingBuffer;

TEST(RingBuffer_FillsBeforeWrapping) {
    RingBuffer<int> rb(4);
    CHECK(rb.empty());
    rb.push(1);
    rb.push(2);
    rb.push(3);
    CHECK_EQ(rb.size(), 3u);
    CHECK(!rb.full());
    CHECK_EQ(rb.front(), 1);
    CHECK_EQ(rb.back(), 3);
    CHECK_EQ(rb.at(1), 2);
}

TEST(RingBuffer_WrapsAndOverwritesOldest) {
    RingBuffer<int> rb(4);
    for (int i = 1; i <= 10; ++i) rb.push(i);
    CHECK(rb.full());
    CHECK_EQ(rb.size(), 4u);
    CHECK_EQ(rb.capacity(), 4u);
    // Oldest-to-newest must be 7, 8, 9, 10 regardless of where the head physically is.
    for (size_t i = 0; i < 4; ++i) CHECK_EQ(rb.at(i), static_cast<int>(7 + i));
    CHECK_EQ(rb.front(), 7);
    CHECK_EQ(rb.back(), 10);
}

TEST(RingBuffer_PushSlotDoesNotReallocate) {
    RingBuffer<int> rb(3);
    const int* first = &rb.push();
    for (int i = 0; i < 100; ++i) rb.push() = i;
    // All slots live in the same storage: the address of slot 0 is reused cyclically.
    bool sawFirst = false;
    for (size_t i = 0; i < rb.size(); ++i) sawFirst |= (&rb.at(i) == first);
    CHECK(sawFirst);
    CHECK_EQ(rb.back(), 99);
}

TEST(RingBuffer_ClearKeepsCapacity) {
    RingBuffer<int> rb(5);
    for (int i = 0; i < 7; ++i) rb.push(i);
    rb.clear();
    CHECK(rb.empty());
    CHECK_EQ(rb.capacity(), 5u);
    rb.push(42);
    CHECK_EQ(rb.front(), 42);
    CHECK_EQ(rb.back(), 42);
}

TEST(RingBuffer_LowerBoundAcrossWrap) {
    RingBuffer<double> rb(5);
    for (int i = 0; i < 8; ++i) rb.push(i * 10.0);  // holds 30, 40, 50, 60, 70
    auto key = [](double v) { return v; };
    CHECK_EQ(br::lowerBoundBy(rb, 0.0, key), 0u);
    CHECK_EQ(br::lowerBoundBy(rb, 30.0, key), 0u);
    CHECK_EQ(br::lowerBoundBy(rb, 31.0, key), 1u);
    CHECK_EQ(br::lowerBoundBy(rb, 60.0, key), 3u);
    CHECK_EQ(br::lowerBoundBy(rb, 70.0, key), 4u);
    CHECK_EQ(br::lowerBoundBy(rb, 71.0, key), 5u);
}
