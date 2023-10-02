#include "light_subsystem.h"

#include "../resource_allocator.h"
#include "../types/primitives.h"

LightingSubsystem::LightingSubsystem() {
    bool success = false;
    this->main_ledc_channel = ResourceAllocatorPresets::ledc_channels.allocate(&success);
    if (!success) {
        this->main_ledc_channel = UINT16_MAX;
    }
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    pinMode(LED_BUILTIN, OUTPUT);
    ledcSetup(main_ledc_channel, 20000, 8);
    ledcAttachPin(4, main_ledc_channel);
    ledcWrite(main_ledc_channel, 0);
    this->tenure();
}

LightingSubsystem::~LightingSubsystem() {
    if (this->main_ledc_channel != UINT16_MAX) {
        ResourceAllocatorPresets::ledc_channels.freeAllocation(this->main_ledc_channel);
    }
    ledcDetachPin(4);
    digitalWrite(4, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
}

void LightingSubsystem::setBuiltinLight(bool value) {
    digitalWrite(LED_BUILTIN, value);
}

void LightingSubsystem::setMainLight(uint8_t value) {
    ledcWrite(this->main_ledc_channel, value);
}

BaseObject* LightingSubsystem::callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) {
    switch (slot) {
    case _setMainLight_:
        CHECK_METHOD_TYPE_SIGNATURE(params, num_params, 1, INT_TYPEID);
        setMainLight(((PInteger*)params[0])->value);
        return NULL;
    case _setBuiltinLight_:
        CHECK_METHOD_TYPE_SIGNATURE(params, num_params, 1, INT_TYPEID);
        bool state = (((PInteger*)params[0])->value > 0) ? 1 : 0;
        setBuiltinLight(state);
        return NULL;
    }
    return NULL;
}