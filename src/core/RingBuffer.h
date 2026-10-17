// Fixed-capacity ring buffer. Storage is allocated once in reset(); push() never allocates
// and overwrites the oldest element when full.
#pragma once

#include <cstddef>
#include <vector>

namespace br {

template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity = 1) { reset(capacity); }

    void reset(size_t capacity) {
        data_.assign(capacity > 0 ? capacity : 1, T{});
        head_ = 0;
        size_ = 0;
    }

    void clear() {
        head_ = 0;
        size_ = 0;
    }

    size_t capacity() const { return data_.size(); }
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == data_.size(); }

    // Returns the slot for a new element (the newest), evicting the oldest when full.
    T& push() {
        const size_t cap = data_.size();
        const size_t slot = (head_ + size_) % cap;
        if (size_ < cap) {
            ++size_;
        } else {
            head_ = (head_ + 1) % cap;
        }
        return data_[slot];
    }

    void push(const T& value) { push() = value; }

    // 0 is the oldest element, size() - 1 the newest.
    const T& at(size_t i) const { return data_[(head_ + i) % data_.size()]; }
    T& at(size_t i) { return data_[(head_ + i) % data_.size()]; }

    const T& front() const { return at(0); }
    const T& back() const { return at(size_ - 1); }

private:
    std::vector<T> data_;
    size_t head_ = 0;
    size_t size_ = 0;
};

// First index whose key is >= value, or size() if none. Keys must be non-decreasing.
template <typename Container, typename KeyFn>
size_t lowerBoundBy(const Container& c, double value, KeyFn key) {
    size_t lo = 0;
    size_t hi = c.size();
    while (lo < hi) {
        const size_t mid = lo + (hi - lo) / 2;
        if (key(c.at(mid)) < value) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

}  // namespace br
