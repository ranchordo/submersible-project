#pragma once
#include "../types/objects.h"

class LightingSubsystem: public BaseObject {
public:
    LightingSubsystem();
    LightingSubsystem(BaseObject** params, uint8_t num_params);
    ~LightingSubsystem();
    SerializationResult serialize() override {
        return { NULL, 0 };
    }
    uint16_t getTypeId() override { return TYPEID_LIGHT_SUBSYS; }
    static uint16_t _static_typeId() { return TYPEID_LIGHT_SUBSYS; }
    const char* getTypeString() override { return "Light_Subsys"; }
    BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) override;

    const static uint8_t _setMainLight_ = 0;
    const static uint8_t _setBuiltinLight_ = 1;

    void setMainLight(uint8_t value);
    void setBuiltinLight(bool value);
private:
    uint16_t main_ledc_channel;
};