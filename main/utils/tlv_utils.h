#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <algorithm>

class TLVWriter {
public:
    TLVWriter(uint8_t* buffer) : out(buffer), i(0) {}

    void begin(uint8_t msgType, uint8_t dataType) {
        out[i++] = msgType;
        out[i++] = dataType;
    }

    void add(uint8_t id, const void* data, uint8_t size) {
        out[i++] = id;
        out[i++] = size;
        memcpy(&out[i], data, size);
        i += size;
    }

    void addUint8(uint8_t id, uint8_t value) {
        add(id, &value, 1);
    }

    void addInt8(uint8_t id, int8_t value) {
        add(id, &value, 1);
    }

    void addBool(uint8_t id, bool value) {
        uint8_t b = value ? 1 : 0;
        addUint8(id, b);
    }

    void addString(uint8_t id, const std::string& str) {
        uint8_t size = std::min<uint8_t>(str.size(), 255);
        out[i++] = id;
        out[i++] = size;
        memcpy(&out[i], str.c_str(), size);
        i += size;
    }

    void addUint16(uint8_t id, uint16_t value) {
        add(id, &value, 2);
    }

    void addUInt32(uint8_t id, uint32_t value) {
        add(id, &value, 4);
    }

    void addInt32(uint8_t id, int32_t value) {
        add(id, &value, 4);
    }

    void addInt64(uint8_t id, int64_t value) {
        add(id, &value, 8);
    }

    void addDouble(uint8_t id, double value) {
        add(id, &value, 8);
    }

    void addFloat(uint8_t id, float value) {
        add(id, &value, sizeof(float));
    }    

    size_t length() const { return i; }

private:
    uint8_t* out;
    size_t i;
};

class TLVReader {
public:
    TLVReader(const uint8_t* buffer, size_t length) : in(buffer), len(length), i(2) {}

    bool checkHeader(uint8_t expectedMsgType, uint8_t expectedDataType) {
        return len >= 2 && in[0] == expectedMsgType && in[1] == expectedDataType;
    }

    bool next(uint8_t& id, uint8_t& size, const uint8_t*& data) {
        if (i + 2 > len) return false;
        id = in[i++];
        size = in[i++];
        if (i + size > len) return false;
        data = &in[i];
        i += size;
        return true;
    }

private:
    const uint8_t* in;
    size_t len;
    size_t i;
};
