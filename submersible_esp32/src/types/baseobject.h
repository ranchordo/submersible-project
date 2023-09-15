#pragma once

#include <stdint.h>
#include <Arduino.h>

#define CREATE_EXC_SETUP(name) \
bool jmp_init_##name##_ = false; \
jmp_buf jmp_data_##name##_; \
MethodExceptionData exc_data_##name##_ = { NULL, NULL }; \
bool throw_obj_exception_##name##_(BaseObject* obj, const char* message) { \
    if (!jmp_init_##name##_) { return false; } \
    exc_data_##name##_ = { obj, message }; \
    longjmp(jmp_data_##name##_, 1); \
}

#define TRY_CONSTRUCT_PRIMITIVE(primitive, type_id, data) \
if (primitive::_static_typeId() == type_id) { \
    return new primitive (data); \
}

#define SETUP_EXC(name, on_exception) \
if (setjmp(jmp_data_##name##_) != 0) { \
    BaseObject* exc = new PString(exc_data_##name##_.message, true); \
    on_exception(exc_data_##name##_); \
    exc_data_##name##_ = { NULL, NULL }; \
    return exc; \
} \
jmp_init_##name##_ = true

#define throw_obj_exception(name, obj, message) throw_obj_exception_##name##_(obj, message)

struct SerializationResult {
    uint8_t* buffer;
    size_t datalen;
};

class BaseObject {
public:
    // Allocates memory for serialization. Free buffer when done transmitting.
    virtual SerializationResult serialize() = 0;
    virtual uint16_t getTypeId() = 0;
    virtual const char* getTypeString() = 0;
    virtual bool isException() { return false; }
    virtual BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) { return NULL; }
    virtual void periodic() {}
    bool enable_periodic = false; // Optimization
};

struct MethodExceptionData {
    BaseObject* object;
    const char* message;
};