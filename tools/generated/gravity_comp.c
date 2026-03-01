/* Auto-generated RNEA gravity compensation from robo_arm6.urdf */
/* DO NOT EDIT -- regenerate with generate_gravity_comp_rnea.py */

#include "gravity_comp.h"
#include <math.h>

void compute_gravity_torques(const float q[6], float tau[6])
{
    const float g = 9.81f;

    /* Joint sin/cos */
    const float s0 = sinf(q[0]);
    const float c0 = cosf(q[0]);
    const float s1 = sinf(q[1]);
    const float c1 = cosf(q[1]);
    const float s2 = sinf(-q[2]);
    const float c2 = cosf(-q[2]);
    const float s3 = sinf(q[3]);
    const float c3 = cosf(q[3]);
    const float s4 = sinf(q[4]);
    const float c4 = cosf(q[4]);
    const float s5 = sinf(q[5]);
    const float c5 = cosf(q[5]);

    /* Forward pass: rotation matrices and positions */
    /* R_0i = rotation from frame i to world (stored row-major) */
    /* p_0i = origin of frame i in world coordinates */

    /* Joint 0 (joint1): rpy=(0.0000, 0.0000, 0.0000), xyz=(0.0, 0.0, 0.0265) */
    const float Rc0_00 = 1.0f*c0;
    const float Rc0_01 = -1.0f*s0;
    const float Rc0_02 = 0.0f;
    const float Rc0_10 = 1.0f*s0;
    const float Rc0_11 = 1.0f*c0;
    const float Rc0_12 = 0.0f;
    const float Rc0_20 = 0.0f;
    const float Rc0_21 = 0.0f;
    const float Rc0_22 = 1.0f;

    const float R0_00 = Rc0_00;
    const float R0_01 = Rc0_01;
    const float R0_02 = Rc0_02;
    const float R0_10 = Rc0_10;
    const float R0_11 = Rc0_11;
    const float R0_12 = Rc0_12;
    const float R0_20 = Rc0_20;
    const float R0_21 = Rc0_21;
    const float R0_22 = Rc0_22;

    const float p0_x = 0.0f;
    const float p0_y = 0.0f;
    const float p0_z = 0.0265f;

    /* Joint 1 (joint2): rpy=(-1.5708, 0.0000, 1.5708), xyz=(-0.0557, 0.0, 0.063) */
    const float Rc1_00 = 0.0f;
    const float Rc1_01 = 0.0f;
    const float Rc1_02 = -1.0f;
    const float Rc1_10 = 1.0f*c1;
    const float Rc1_11 = -1.0f*s1;
    const float Rc1_12 = 0.0f;
    const float Rc1_20 = -1.0f*s1;
    const float Rc1_21 = -1.0f*c1;
    const float Rc1_22 = 0.0f;

    const float R1_00 = R0_00*Rc1_00 + R0_01*Rc1_10 + R0_02*Rc1_20;
    const float R1_01 = R0_00*Rc1_01 + R0_01*Rc1_11 + R0_02*Rc1_21;
    const float R1_02 = R0_00*Rc1_02 + R0_01*Rc1_12 + R0_02*Rc1_22;
    const float R1_10 = R0_10*Rc1_00 + R0_11*Rc1_10 + R0_12*Rc1_20;
    const float R1_11 = R0_10*Rc1_01 + R0_11*Rc1_11 + R0_12*Rc1_21;
    const float R1_12 = R0_10*Rc1_02 + R0_11*Rc1_12 + R0_12*Rc1_22;
    const float R1_20 = R0_20*Rc1_00 + R0_21*Rc1_10 + R0_22*Rc1_20;
    const float R1_21 = R0_20*Rc1_01 + R0_21*Rc1_11 + R0_22*Rc1_21;
    const float R1_22 = R0_20*Rc1_02 + R0_21*Rc1_12 + R0_22*Rc1_22;

    const float p1_x = p0_x + R0_00*-0.0557f + R0_02*0.063f;
    const float p1_y = p0_y + R0_10*-0.0557f + R0_12*0.063f;
    const float p1_z = p0_z + R0_20*-0.0557f + R0_22*0.063f;

    /* Joint 2 (joint3): rpy=(0.0000, 0.0000, 3.1416), xyz=(0.38, 0.0, -0.01983) */
    const float Rc2_00 = -1.0f*c2;
    const float Rc2_01 = 1.0f*s2;
    const float Rc2_02 = 0.0f;
    const float Rc2_10 = -1.0f*s2;
    const float Rc2_11 = -1.0f*c2;
    const float Rc2_12 = 0.0f;
    const float Rc2_20 = 0.0f;
    const float Rc2_21 = 0.0f;
    const float Rc2_22 = 1.0f;

    const float R2_00 = R1_00*Rc2_00 + R1_01*Rc2_10 + R1_02*Rc2_20;
    const float R2_01 = R1_00*Rc2_01 + R1_01*Rc2_11 + R1_02*Rc2_21;
    const float R2_02 = R1_00*Rc2_02 + R1_01*Rc2_12 + R1_02*Rc2_22;
    const float R2_10 = R1_10*Rc2_00 + R1_11*Rc2_10 + R1_12*Rc2_20;
    const float R2_11 = R1_10*Rc2_01 + R1_11*Rc2_11 + R1_12*Rc2_21;
    const float R2_12 = R1_10*Rc2_02 + R1_11*Rc2_12 + R1_12*Rc2_22;
    const float R2_20 = R1_20*Rc2_00 + R1_21*Rc2_10 + R1_22*Rc2_20;
    const float R2_21 = R1_20*Rc2_01 + R1_21*Rc2_11 + R1_22*Rc2_21;
    const float R2_22 = R1_20*Rc2_02 + R1_21*Rc2_12 + R1_22*Rc2_22;

    const float p2_x = p1_x + R1_00*0.38f + R1_02*-0.01983f;
    const float p2_y = p1_y + R1_10*0.38f + R1_12*-0.01983f;
    const float p2_z = p1_z + R1_20*0.38f + R1_22*-0.01983f;

    /* Joint 3 (joint4): rpy=(1.5708, 0.0000, 0.0000), xyz=(0.054993, -0.15447, -0.039) */
    const float Rc3_00 = 1.0f*c3;
    const float Rc3_01 = -1.0f*s3;
    const float Rc3_02 = 0.0f;
    const float Rc3_10 = 0.0f;
    const float Rc3_11 = 0.0f;
    const float Rc3_12 = -1.0f;
    const float Rc3_20 = 1.0f*s3;
    const float Rc3_21 = 1.0f*c3;
    const float Rc3_22 = 0.0f;

    const float R3_00 = R2_00*Rc3_00 + R2_01*Rc3_10 + R2_02*Rc3_20;
    const float R3_01 = R2_00*Rc3_01 + R2_01*Rc3_11 + R2_02*Rc3_21;
    const float R3_02 = R2_00*Rc3_02 + R2_01*Rc3_12 + R2_02*Rc3_22;
    const float R3_10 = R2_10*Rc3_00 + R2_11*Rc3_10 + R2_12*Rc3_20;
    const float R3_11 = R2_10*Rc3_01 + R2_11*Rc3_11 + R2_12*Rc3_21;
    const float R3_12 = R2_10*Rc3_02 + R2_11*Rc3_12 + R2_12*Rc3_22;
    const float R3_20 = R2_20*Rc3_00 + R2_21*Rc3_10 + R2_22*Rc3_20;
    const float R3_21 = R2_20*Rc3_01 + R2_21*Rc3_11 + R2_22*Rc3_21;
    const float R3_22 = R2_20*Rc3_02 + R2_21*Rc3_12 + R2_22*Rc3_22;

    const float p3_x = p2_x + R2_00*0.054993f + R2_01*-0.15447f + R2_02*-0.039f;
    const float p3_y = p2_y + R2_10*0.054993f + R2_11*-0.15447f + R2_12*-0.039f;
    const float p3_z = p2_z + R2_20*0.054993f + R2_21*-0.15447f + R2_22*-0.039f;

    /* Joint 4 (joint5): rpy=(-1.5708, 1.5708, 0.0000), xyz=(0.0, -0.031, 0.1572) */
    const float Rc4_00 = -1.0f*s4;
    const float Rc4_01 = -1.0f*c4;
    const float Rc4_02 = 0.0f;
    const float Rc4_10 = 0.0f;
    const float Rc4_11 = 0.0f;
    const float Rc4_12 = 1.0f;
    const float Rc4_20 = -1.0f*c4;
    const float Rc4_21 = 1.0f*s4;
    const float Rc4_22 = 0.0f;

    const float R4_00 = R3_00*Rc4_00 + R3_01*Rc4_10 + R3_02*Rc4_20;
    const float R4_01 = R3_00*Rc4_01 + R3_01*Rc4_11 + R3_02*Rc4_21;
    const float R4_02 = R3_00*Rc4_02 + R3_01*Rc4_12 + R3_02*Rc4_22;
    const float R4_10 = R3_10*Rc4_00 + R3_11*Rc4_10 + R3_12*Rc4_20;
    const float R4_11 = R3_10*Rc4_01 + R3_11*Rc4_11 + R3_12*Rc4_21;
    const float R4_12 = R3_10*Rc4_02 + R3_11*Rc4_12 + R3_12*Rc4_22;
    const float R4_20 = R3_20*Rc4_00 + R3_21*Rc4_10 + R3_22*Rc4_20;
    const float R4_21 = R3_20*Rc4_01 + R3_21*Rc4_11 + R3_22*Rc4_21;
    const float R4_22 = R3_20*Rc4_02 + R3_21*Rc4_12 + R3_22*Rc4_22;

    const float p4_x = p3_x + R3_01*-0.031f + R3_02*0.1572f;
    const float p4_y = p3_y + R3_11*-0.031f + R3_12*0.1572f;
    const float p4_z = p3_z + R3_21*-0.031f + R3_22*0.1572f;

    /* Joint 5 (joint6): rpy=(0.0000, 1.5708, 0.0000), xyz=(0.096475, 0.0, 0.032) */
    const float Rc5_00 = 0.0f;
    const float Rc5_01 = 0.0f;
    const float Rc5_02 = 1.0f;
    const float Rc5_10 = 1.0f*s5;
    const float Rc5_11 = 1.0f*c5;
    const float Rc5_12 = 0.0f;
    const float Rc5_20 = -1.0f*c5;
    const float Rc5_21 = 1.0f*s5;
    const float Rc5_22 = 0.0f;

    const float R5_00 = R4_00*Rc5_00 + R4_01*Rc5_10 + R4_02*Rc5_20;
    const float R5_01 = R4_00*Rc5_01 + R4_01*Rc5_11 + R4_02*Rc5_21;
    const float R5_02 = R4_00*Rc5_02 + R4_01*Rc5_12 + R4_02*Rc5_22;
    const float R5_10 = R4_10*Rc5_00 + R4_11*Rc5_10 + R4_12*Rc5_20;
    const float R5_11 = R4_10*Rc5_01 + R4_11*Rc5_11 + R4_12*Rc5_21;
    const float R5_12 = R4_10*Rc5_02 + R4_11*Rc5_12 + R4_12*Rc5_22;
    const float R5_20 = R4_20*Rc5_00 + R4_21*Rc5_10 + R4_22*Rc5_20;
    const float R5_21 = R4_20*Rc5_01 + R4_21*Rc5_11 + R4_22*Rc5_21;
    const float R5_22 = R4_20*Rc5_02 + R4_21*Rc5_12 + R4_22*Rc5_22;

    const float p5_x = p4_x + R4_00*0.096475f + R4_02*0.032f;
    const float p5_y = p4_y + R4_10*0.096475f + R4_12*0.032f;
    const float p5_z = p4_z + R4_20*0.096475f + R4_22*0.032f;

    /* COM positions in world frame and gravity forces */
    const float com0_x = p0_x + R0_00*-0.010369f + R0_01*-1.6053e-05f + R0_02*0.05425f;
    const float com0_y = p0_y + R0_10*-0.010369f + R0_11*-1.6053e-05f + R0_12*0.05425f;
    const float com0_z = p0_z + R0_20*-0.010369f + R0_21*-1.6053e-05f + R0_22*0.05425f;

    const float com1_x = p1_x + R1_00*0.2234f + R1_01*0.00074741f + R1_02*0.015853f;
    const float com1_y = p1_y + R1_10*0.2234f + R1_11*0.00074741f + R1_12*0.015853f;
    const float com1_z = p1_z + R1_20*0.2234f + R1_21*0.00074741f + R1_22*0.015853f;

    const float com2_x = p2_x + R2_00*0.03012228719f + R2_01*-0.07080251747f + R2_02*-0.03273493625f;
    const float com2_y = p2_y + R2_10*0.03012228719f + R2_11*-0.07080251747f + R2_12*-0.03273493625f;
    const float com2_z = p2_z + R2_20*0.03012228719f + R2_21*-0.07080251747f + R2_22*-0.03273493625f;

    const float com3_x = p3_x + R3_00*0.00051276f + R3_01*0.002109f + R3_02*0.057091f;
    const float com3_y = p3_y + R3_10*0.00051276f + R3_11*0.002109f + R3_12*0.057091f;
    const float com3_z = p3_z + R3_20*0.00051276f + R3_21*0.002109f + R3_22*0.057091f;

    const float com4_x = p4_x + R4_00*0.03939999718f + R4_01*-0.001700420834f + R4_02*0.03232324786f;
    const float com4_y = p4_y + R4_10*0.03939999718f + R4_11*-0.001700420834f + R4_12*0.03232324786f;
    const float com4_z = p4_z + R4_20*0.03939999718f + R4_21*-0.001700420834f + R4_22*0.03232324786f;

    const float com5_x = p5_x + R5_00*-2.5199e-12f + R5_01*-1.2682e-11f + R5_02*0.020912f;
    const float com5_y = p5_y + R5_10*-2.5199e-12f + R5_11*-1.2682e-11f + R5_12*0.020912f;
    const float com5_z = p5_z + R5_20*-2.5199e-12f + R5_21*-1.2682e-11f + R5_22*0.020912f;

    /* Backward pass: gravity torques (RNEA) */
    /* tau_i = axis_z * g * sum_{j>=i} m_j * ((com_j_y - p_i_y)*z_i_x - (com_j_x - p_i_x)*z_i_y) */

    const float z0_x = R0_02;
    const float z0_y = R0_12;
    const float z1_x = R1_02;
    const float z1_y = R1_12;
    const float z2_x = R2_02;
    const float z2_y = R2_12;
    const float z3_x = R3_02;
    const float z3_y = R3_12;
    const float z4_x = R4_02;
    const float z4_y = R4_12;
    const float z5_x = R5_02;
    const float z5_y = R5_12;

    tau[0] = 1.0f*g*(0.86231f*((com0_y - p0_y)*z0_x - (com0_x - p0_x)*z0_y) + 0.66132f*((com1_y - p0_y)*z0_x - (com1_x - p0_x)*z0_y) + 0.8076414448f*((com2_y - p0_y)*z0_x - (com2_x - p0_x)*z0_y) + 0.22515f*((com3_y - p0_y)*z0_x - (com3_x - p0_x)*z0_y) + 0.2730227491f*((com4_y - p0_y)*z0_x - (com4_x - p0_x)*z0_y) + 0.19559f*((com5_y - p0_y)*z0_x - (com5_x - p0_x)*z0_y));
    tau[1] = 1.0f*g*(0.66132f*((com1_y - p1_y)*z1_x - (com1_x - p1_x)*z1_y) + 0.8076414448f*((com2_y - p1_y)*z1_x - (com2_x - p1_x)*z1_y) + 0.22515f*((com3_y - p1_y)*z1_x - (com3_x - p1_x)*z1_y) + 0.2730227491f*((com4_y - p1_y)*z1_x - (com4_x - p1_x)*z1_y) + 0.19559f*((com5_y - p1_y)*z1_x - (com5_x - p1_x)*z1_y));
    tau[2] = -1.0f*g*(0.8076414448f*((com2_y - p2_y)*z2_x - (com2_x - p2_x)*z2_y) + 0.22515f*((com3_y - p2_y)*z2_x - (com3_x - p2_x)*z2_y) + 0.2730227491f*((com4_y - p2_y)*z2_x - (com4_x - p2_x)*z2_y) + 0.19559f*((com5_y - p2_y)*z2_x - (com5_x - p2_x)*z2_y));
    tau[3] = 1.0f*g*(0.22515f*((com3_y - p3_y)*z3_x - (com3_x - p3_x)*z3_y) + 0.2730227491f*((com4_y - p3_y)*z3_x - (com4_x - p3_x)*z3_y) + 0.19559f*((com5_y - p3_y)*z3_x - (com5_x - p3_x)*z3_y));
    tau[4] = 1.0f*g*(0.2730227491f*((com4_y - p4_y)*z4_x - (com4_x - p4_x)*z4_y) + 0.19559f*((com5_y - p4_y)*z4_x - (com5_x - p4_x)*z4_y));
    tau[5] = 1.0f*g*(0.19559f*((com5_y - p5_y)*z5_x - (com5_x - p5_x)*z5_y));
}
