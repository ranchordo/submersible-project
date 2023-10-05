#include "prop_subsystem.h"
#include "../resource_allocator.h"
#include "../types/primitives.h"

#define SAFE_ACCEPT_TIMEOUT 5000

constexpr float r_force_matrix[3][4] = {
    {1, 0, 0, 1},
    {0, 1, 0, 1},
    {0, 0, 1, 0},
};

constexpr float r_torque_matrix[3][4] = {
    {0, 0, 1, 1},
    {1, 0, 0, 0},
    {0, 1, 0, 1},
};

constexpr float compute_matr_denom(const float r_matrix[3][4]) {
#define M(A, B) r_matrix[A - 1][B - 1]
#define QI1 (M(1, 1) * M(2, 2) * M(3, 3))
#define QI2 (M(1, 1) * M(2, 3) * M(3, 2))
#define QI3 (M(1, 2) * M(2, 1) * M(3, 3))
#define QI4 (M(1, 2) * M(2, 3) * M(3, 1))
#define QI5 (M(1, 3) * M(2, 1) * M(3, 2))
#define QI6 (M(1, 3) * M(2, 2) * M(3, 1))
    return (QI1 - QI2 - QI3 + QI4 + QI5 - QI6);
#undef QI6
#undef QI5
#undef QI4
#undef QI3
#undef QI2
#undef QI1
#undef M
}

constexpr float compute_a1(const float r_matrix[3][4]) {
#define M(A, B) r_matrix[A - 1][B - 1]
#define PI1 (M(1, 2) * M(2, 3) * M(3, 4))
#define PI2 (M(1, 2) * M(2, 4) * M(3, 3))
#define PI3 (M(1, 3) * M(2, 2) * M(3, 4))
#define PI4 (M(1, 3) * M(2, 4) * M(3, 2))
#define PI5 (M(1, 4) * M(2, 2) * M(3, 3))
#define PI6 (M(1, 4) * M(2, 3) * M(3, 2))
    return (PI1 - PI2 - PI3 + PI4 + PI5 - PI6);
#undef PI6
#undef PI5
#undef PI4
#undef PI3
#undef PI2
#undef PI1
#undef M
}

constexpr float compute_a2(const float r_matrix[3][4]) {
#define M(A, B) r_matrix[A - 1][B - 1]
#define PI1 (M(1, 1) * M(2, 3) * M(3, 4))
#define PI2 (M(1, 1) * M(2, 4) * M(3, 3))
#define PI3 (M(1, 3) * M(2, 1) * M(3, 4))
#define PI4 (M(1, 3) * M(2, 4) * M(3, 1))
#define PI5 (M(1, 4) * M(2, 1) * M(3, 3))
#define PI6 (M(1, 4) * M(2, 3) * M(3, 1))
    return (-PI1 + PI2 + PI3 - PI4 - PI5 + PI6);
#undef PI6
#undef PI5
#undef PI4
#undef PI3
#undef PI2
#undef PI1
#undef M
}

constexpr float compute_a3(const float r_matrix[3][4]) {
#define M(A, B) r_matrix[A - 1][B - 1]
#define PI1 (M(1, 1) * M(2, 2) * M(3, 4))
#define PI2 (M(1, 1) * M(2, 4) * M(3, 2))
#define PI3 (M(1, 2) * M(2, 1) * M(3, 4))
#define PI4 (M(1, 2) * M(2, 4) * M(3, 1))
#define PI5 (M(1, 4) * M(2, 1) * M(3, 2))
#define PI6 (M(1, 4) * M(2, 2) * M(3, 1))
    return (PI1 - PI2 - PI3 + PI4 + PI5 - PI6);
#undef PI6
#undef PI5
#undef PI4
#undef PI3
#undef PI2
#undef PI1
#undef M
}

#define M(A, B) r_matrix[A - 1][B - 1]
constexpr float compute_b11(const float r_matrix[3][4]) { return (M(2, 2) * M(3, 3)) - (M(2, 3) * M(3, 2)); }
constexpr float compute_b12(const float r_matrix[3][4]) { return (M(1, 3) * M(3, 2)) - (M(1, 2) * M(3, 3)); }
constexpr float compute_b13(const float r_matrix[3][4]) { return (M(1, 2) * M(2, 3)) - (M(1, 3) * M(2, 2)); }

constexpr float compute_b21(const float r_matrix[3][4]) { return (M(2, 3) * M(3, 1)) - (M(2, 1) * M(3, 3)); }
constexpr float compute_b22(const float r_matrix[3][4]) { return (M(1, 1) * M(3, 3)) - (M(1, 3) * M(3, 1)); }
constexpr float compute_b23(const float r_matrix[3][4]) { return (M(1, 3) * M(2, 1)) - (M(1, 1) * M(2, 3)); }

constexpr float compute_b31(const float r_matrix[3][4]) { return (M(2, 1) * M(3, 2)) - (M(2, 2) * M(3, 1)); }
constexpr float compute_b32(const float r_matrix[3][4]) { return (M(1, 2) * M(3, 1)) - (M(1, 1) * M(3, 2)); }
constexpr float compute_b33(const float r_matrix[3][4]) { return (M(1, 1) * M(2, 2)) - (M(1, 2) * M(2, 1)); }
#undef M

struct linalg_precalc {
    float comm_denom;
    float a1, a2, a3;
    float b11, b12, b13;
    float b21, b22, b23;
    float b31, b32, b33;
};

struct motor_state {
    float m1, m2, m3, m4;
    float dotprod(motor_state v) {
        return (v.m1 * m1 + v.m2 * m2 + v.m3 * m3 + v.m4 * m4);
    }
    motor_state subtract(motor_state v) {
        return { m1 - v.m1, m2 - v.m2, m3 - v.m3, m4 - v.m4 };
    }
    motor_state midpoint(motor_state v, float weight) {
        float iw = 1 - weight;
        return { m1 * iw + v.m1 * weight,
                 m2 * iw + v.m2 * weight,
                 m3 * iw + v.m3 * weight,
                 m4 * iw + v.m4 * weight };
    }
};

struct mspace_line {
    float a1, a2, a3, a4;
    float b1, b2, b3, b4;
    motor_state get_pos(float t) {
        return { a1 * t + b1, a2 * t + b2, a3 * t + b3, a4 * t + b4 };
    }
    motor_state get_as() {
        return { a1, a2, a3, a4 };
    }
};

constexpr linalg_precalc do_linalg_precalc(const float rmatrix[3][4]) {
    return {
        compute_matr_denom(rmatrix),
        compute_a1(rmatrix), compute_a2(rmatrix), compute_a3(rmatrix),
        compute_b11(rmatrix), compute_b12(rmatrix), compute_b13(rmatrix),
        compute_b21(rmatrix), compute_b22(rmatrix), compute_b23(rmatrix),
        compute_b31(rmatrix), compute_b32(rmatrix), compute_b33(rmatrix),
    };
}

constexpr linalg_precalc r_force_precalc = do_linalg_precalc(r_force_matrix);
constexpr linalg_precalc r_torque_precalc = do_linalg_precalc(r_torque_matrix);

mspace_line find_motor_space(linalg_precalc lapc, float t1, float t2, float t3) {
    float b1 = lapc.b11 * t1 + lapc.b12 * t2 + lapc.b13 * t3;
    float b2 = lapc.b21 * t1 + lapc.b22 * t2 + lapc.b23 * t3;
    float b3 = lapc.b31 * t1 + lapc.b32 * t2 + lapc.b33 * t3;
    mspace_line line;
    line.a1 = -lapc.a1 / lapc.comm_denom;
    line.a2 = -lapc.a2 / lapc.comm_denom;
    line.a3 = -lapc.a3 / lapc.comm_denom;
    line.a4 = 1.0;
    line.b1 = b1 / lapc.comm_denom;
    line.b2 = b2 / lapc.comm_denom;
    line.b3 = b3 / lapc.comm_denom;
    line.b4 = 0.0;
    return line;
}

#define __abs(f) (((f) < 0) ? -(f) : (f))

// Line2 weight of 1 means we are fully on line2, 0 means we're on line1.
motor_state find_best_state(mspace_line line1, mspace_line line2, float line2_weight) {
    float g1 = 0;
    float g2 = 0;
    for (;;) {
        motor_state state1 = line1.get_pos(g1);
        motor_state state2 = line2.get_pos(g2);
        float dg1 = state2.subtract(state1).dotprod(line1.get_as());
        float dg2 = state1.subtract(state2).dotprod(line2.get_as());
        g1 += dg1 * 0.2;
        g2 += dg2 * 0.2;
        if ((__abs(dg1) + __abs(dg2)) * 0.5 <= 0.005) {
            break;
        }
    }
    return line1.get_pos(g1).midpoint(line2.get_pos(g2), line2_weight);
}

motor_state get_motor_state(motion_state mstate, float torque_weight) {
    float lf1 = mstate.linear_x;
    float lf2 = mstate.linear_y;
    float lf3 = mstate.linear_z;
    float t1 = mstate.torque_x;
    float t2 = mstate.torque_y;
    float t3 = mstate.torque_z;
    mspace_line l_force_line = find_motor_space(r_force_precalc, lf1, lf2, lf3);
    mspace_line torque_line = find_motor_space(r_torque_precalc, t1, t2, t3);
    return find_best_state(l_force_line, torque_line, torque_weight);
}

#define ESC_500US 103
#define ESC_BOTTOM 237
#define ESC_MIDPOINT 306
#define ESC_TOP 443
#define ESC_2400US 491

const uint8_t motor_gpios[4] = { 32, 33, 27, 13 };

PropulsionSubsystem::PropulsionSubsystem() {
    for (uint8_t i = 0; i < 4; i++) {
        bool ledc_success = false;
        uint16_t ledc_channel = ResourceAllocatorPresets::ledc_channels.allocate(&ledc_success);
        ledcSetup(ledc_channel, 50, 12);
        ledcAttachPin(motor_gpios[i], ledc_channel);
        ledcWrite(ledc_channel, 0);
        if (ledc_success) {
            this->ledc_motor_channels[i] = ledc_channel;
        } else {
            this->ledc_motor_channels[i] = 0;
        }
    }
    this->enable_periodic = true;
    this->tenure();
}

PropulsionSubsystem::~PropulsionSubsystem() {
    for (uint8_t i = 0; i < 4; i++) {
        ledcDetachPin(motor_gpios[i]);
        ResourceAllocatorPresets::ledc_channels.freeAllocation(ledc_motor_channels[i]);
    }
}

void PropulsionSubsystem::periodic() {
    if (millis() - this->last_accept_time > SAFE_ACCEPT_TIMEOUT) {
        motor_state state = { 0, 0, 0, 0 };
        this->acceptMotorState(&state);
        this->last_accept_time = millis();
    }
}

BaseObject* PropulsionSubsystem::callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) {
    switch (slot) {
    case _runStartupProcedure_:
        CHECK_METHOD_TYPE_SIGNATURE(params, num_params, 0, );
        for (uint8_t i = 0; i < 4; i++) {
            ledcWrite(ledc_motor_channels[i], ESC_500US);
            this->startup_time = millis();
        }
        return NULL;
    case _isStartupComplete_:
        CHECK_METHOD_TYPE_SIGNATURE(params, num_params, 0, );
        if (millis() - startup_time >= 5500) {
            return new PInteger(1);
        }
        return new PInteger(0);
    case _acceptMotionState_:
        CHECK_METHOD_TYPE_SIGNATURE(params, num_params, 7,
            DOUBLE_TYPEID, DOUBLE_TYPEID, DOUBLE_TYPEID,
            DOUBLE_TYPEID, DOUBLE_TYPEID, DOUBLE_TYPEID,
            DOUBLE_TYPEID);
        motion_state mstate = {
            (float)((PDouble*)params[0])->value,
            (float)((PDouble*)params[1])->value,
            (float)((PDouble*)params[2])->value,
            (float)((PDouble*)params[3])->value,
            (float)((PDouble*)params[4])->value,
            (float)((PDouble*)params[5])->value
        };
        this->acceptMotionState(mstate, (float)((PDouble*)params[6])->value);
        return NULL;
    }
}

void PropulsionSubsystem::writeMotorValue(float value, uint8_t motor) {
    uint16_t ledc_channel = ledc_motor_channels[motor];
    if (value < 0) {
        if (value < -1) {
            value = -1;
        }
        ledcWrite(ledc_channel, map(-value * 10000, 0, 10000, ESC_MIDPOINT, ESC_BOTTOM));
    } else {
        if (value > 1) {
            value = 1;
        }
        ledcWrite(ledc_channel, map(value * 10000, 0, 10000, ESC_MIDPOINT, ESC_TOP));
    }
}

void PropulsionSubsystem::acceptMotorState(motor_state* state) {
    this->writeMotorValue(state->m1, 0);
    this->writeMotorValue(state->m2, 1);
    this->writeMotorValue(state->m3, 2);
    this->writeMotorValue(state->m4, 3);
}

void PropulsionSubsystem::acceptMotionState(motion_state state, float torque_weight) {
    motor_state mstate = get_motor_state(state, torque_weight);
    this->acceptMotorState(&mstate);
    this->last_accept_time = millis();
}