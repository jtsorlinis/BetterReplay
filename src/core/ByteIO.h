// Little-endian byte writer/reader used by the clip format.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace br {

class ByteWriter {
public:
    explicit ByteWriter(std::vector<uint8_t>& out) : out_(out) {}

    size_t pos() const { return out_.size(); }

    void bytes(const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        out_.insert(out_.end(), b, b + n);
    }
    void u8(uint8_t v) { out_.push_back(v); }
    void u16(uint16_t v) {
        u8(static_cast<uint8_t>(v));
        u8(static_cast<uint8_t>(v >> 8));
    }
    void u32(uint32_t v) {
        for (int i = 0; i < 4; ++i) u8(static_cast<uint8_t>(v >> (8 * i)));
    }
    void u64(uint64_t v) {
        for (int i = 0; i < 8; ++i) u8(static_cast<uint8_t>(v >> (8 * i)));
    }
    void i64(int64_t v) { u64(static_cast<uint64_t>(v)); }
    void f32(float v) {
        uint32_t bits;
        std::memcpy(&bits, &v, 4);
        u32(bits);
    }
    void f64(double v) {
        uint64_t bits;
        std::memcpy(&bits, &v, 8);
        u64(bits);
    }
    void str(const std::string& s) {
        const size_t n = s.size() > 0xFFFF ? 0xFFFF : s.size();
        u16(static_cast<uint16_t>(n));
        bytes(s.data(), n);
    }

    // Writes a chunk header with a placeholder size; endChunk() patches it.
    size_t beginChunk(const char id[4]) {
        bytes(id, 4);
        const size_t sizePos = pos();
        u32(0);
        return sizePos;
    }
    void endChunk(size_t sizePos) {
        const uint32_t size = static_cast<uint32_t>(pos() - sizePos - 4);
        for (int i = 0; i < 4; ++i) out_[sizePos + i] = static_cast<uint8_t>(size >> (8 * i));
    }

private:
    std::vector<uint8_t>& out_;
};

class ByteReader {
public:
    ByteReader(const uint8_t* data, size_t size) : p_(data), size_(size) {}

    bool ok() const { return ok_; }
    size_t pos() const { return pos_; }
    size_t remaining() const { return ok_ ? size_ - pos_ : 0; }
    const uint8_t* cursor() const { return p_ + pos_; }

    bool skip(size_t n) {
        if (!need(n)) return false;
        pos_ += n;
        return true;
    }
    bool bytes(void* dst, size_t n) {
        if (!need(n)) return false;
        std::memcpy(dst, p_ + pos_, n);
        pos_ += n;
        return true;
    }
    uint8_t u8() {
        if (!need(1)) return 0;
        return p_[pos_++];
    }
    uint16_t u16() {
        if (!need(2)) return 0;
        const uint16_t v = static_cast<uint16_t>(p_[pos_] | (p_[pos_ + 1] << 8));
        pos_ += 2;
        return v;
    }
    uint32_t u32() {
        if (!need(4)) return 0;
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(p_[pos_ + i]) << (8 * i);
        pos_ += 4;
        return v;
    }
    uint64_t u64() {
        if (!need(8)) return 0;
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(p_[pos_ + i]) << (8 * i);
        pos_ += 8;
        return v;
    }
    int64_t i64() { return static_cast<int64_t>(u64()); }
    float f32() {
        const uint32_t bits = u32();
        float v;
        std::memcpy(&v, &bits, 4);
        return v;
    }
    double f64() {
        const uint64_t bits = u64();
        double v;
        std::memcpy(&v, &bits, 8);
        return v;
    }
    std::string str() {
        const uint16_t n = u16();
        if (!need(n)) return std::string();
        std::string s(reinterpret_cast<const char*>(p_ + pos_), n);
        pos_ += n;
        return s;
    }

private:
    bool need(size_t n) {
        if (!ok_ || size_ - pos_ < n) {
            ok_ = false;
            return false;
        }
        return true;
    }

    const uint8_t* p_;
    size_t size_;
    size_t pos_ = 0;
    bool ok_ = true;
};

}  // namespace br
