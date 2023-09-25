// #include "prop_subsystem.h"
#include <cstdlib>
#include <cstdio>

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
    motor_state midpoint(motor_state v) {
        return { (m1 + v.m1) * 0.5f,
                 (m2 + v.m2) * 0.5f,
                 (m3 + v.m3) * 0.5f,
                 (m4 + v.m4) * 0.5f };
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

motor_state find_best_state(mspace_line line1, mspace_line line2) {
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
    return line1.get_pos(g1).midpoint(line2.get_pos(g2));
}

motor_state get_motor_state(float lf1, float lf2, float lf3, float t1, float t2, float t3) {
    mspace_line l_force_line = find_motor_space(r_force_precalc, lf1, lf2, lf3);
    mspace_line torque_line = find_motor_space(r_torque_precalc, t1, t2, t3);
    return find_best_state(l_force_line, torque_line);
}

int main() {
    motor_state s = get_motor_state(1, 0, 0, 0, 0, 0);
    printf("%f, %f, %f, %f\n", s.m1, s.m2, s.m3, s.m4);
    return 0;
}