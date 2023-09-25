#pragma once

#include <Arduino.h>
#include "../types/baseobject.h"
#include "../types/objects.h"

class PropulsionSubsystem: public BaseObject {
public:
    PropulsionSubsystem();
    PropulsionSubsystem(BaseObject** params, uint8_t num_params);
    ~PropulsionSubsystem();
    SerializationResult serialize() override {
        return { NULL, 0 };
    }
    uint16_t getTypeId() override { return TYPEID_COMMS_SUBSYS; }
    static uint16_t _static_typeId() { return TYPEID_COMMS_SUBSYS; }
    const char* getTypeString() override { return "Comms_Subsys"; }
    BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) override;
    void periodic() override;

    // private:
};