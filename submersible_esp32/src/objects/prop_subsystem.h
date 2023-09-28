#pragma once

#include <Arduino.h>
#include "../types/baseobject.h"
#include "../types/objects.h"

extern const uint8_t motor_gpios[4];

struct motor_state;

struct motion_state {
    float linear_x, linear_y, linear_z;
    float torque_x, torque_y, torque_z;
};

class PropulsionSubsystem: public BaseObject {
public:
    PropulsionSubsystem();
    PropulsionSubsystem(BaseObject** params, uint8_t num_params);
    ~PropulsionSubsystem();
    SerializationResult serialize() override {
        return { NULL, 0 };
    }
    uint16_t getTypeId() override { return TYPEID_PROP_SUBSYS; }
    static uint16_t _static_typeId() { return TYPEID_PROP_SUBSYS; }
    const char* getTypeString() override { return "Prop_Subsys"; }
    BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) override;
    void periodic() override;
    const static uint8_t _runStartupProcedure_ = 0;
    const static uint8_t _isStartupComplete_ = 1;
    const static uint8_t _acceptMotionState_ = 2;

    void acceptMotionState(motion_state state, float torque_weight=0.5);

private:
    void writeMotorValue(float value, uint8_t motor);
    void acceptMotorState(motor_state* state);
    uint16_t ledc_motor_channels[4];
    unsigned long startup_time;
    unsigned long last_accept_time;
};