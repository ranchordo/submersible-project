#pragma once
#include "baseobject.h"

class PInteger: public BaseObject {
public:
    PInteger(SerializationResult data);
    SerializationResult serialize() override {
        uint32_t bytes = *(reinterpret_cast<uint32_t*>(&this->value));
        uint8_t* buffer = (uint8_t*)malloc(4);
        buffer[0] = (bytes & 0x000000ff);
        buffer[1] = (bytes & 0x0000ff00);
        buffer[2] = (bytes & 0x00ff0000);
        buffer[3] = (bytes & 0xff000000);
        return { buffer, 4 };
    }
    uint16_t getTypeId() override { return 65535; }
    static uint16_t _static_typeId() { return 65535; }
    const char* getTypeString() override { return "Integer"; }

    int value = 0;
};

class PDouble: public BaseObject {
public:
    PDouble(SerializationResult data);
    SerializationResult serialize() override {
        uint64_t bytes = *(reinterpret_cast<uint64_t*>(&this->value));
        uint8_t* buffer = (uint8_t*)malloc(8);
        buffer[0] = (bytes & 0x00000000000000ff);
        buffer[1] = (bytes & 0x000000000000ff00);
        buffer[2] = (bytes & 0x0000000000ff0000);
        buffer[3] = (bytes & 0x00000000ff000000);
        buffer[4] = (bytes & 0x000000ff00000000);
        buffer[5] = (bytes & 0x0000ff0000000000);
        buffer[6] = (bytes & 0x00ff000000000000);
        buffer[7] = (bytes & 0xff00000000000000);
        return { buffer, 8 };
    }
    uint16_t getTypeId() override { return 65534; }
    static uint16_t _static_typeId() { return 65534; }
    const char* getTypeString() override { return "Double"; }

    double value = 0.0;
};

class PString: public BaseObject {
public:
    PString(SerializationResult data);
    PString(const char* value, bool is_exception);
    ~PString() {
        free(value);
    }
    SerializationResult serialize() override {
        uint8_t* buffer = reinterpret_cast<uint8_t*>(this->value);
        return { buffer, len };
    }
    uint16_t getTypeId() override { return 65533; }
    static uint16_t _static_typeId() { return 65533; }
    const char* getTypeString() override { return "String"; }
    bool isException() override { return is_exception; }

    char* value = NULL;
    size_t len = 0;
    bool is_exception = false;
};

extern BaseObject* constructPrimitive(uint16_t type_id, SerializationResult data);