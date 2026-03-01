#ifndef GRAVITY_COMP_H
#define GRAVITY_COMP_H

/* Auto-generated gravity compensation from robo_arm6.urdf */
/* DO NOT EDIT — regenerate with generate_gravity_comp.py */

/**
 * Compute gravity compensation torques for a 6-DOF robot arm.
 *
 * @param q   Joint angles in radians [6]  (URDF convention)
 * @param tau Output gravity torques in Nm [6]
 */
void compute_gravity_torques(const float q[6], float tau[6]);

#endif /* GRAVITY_COMP_H */
