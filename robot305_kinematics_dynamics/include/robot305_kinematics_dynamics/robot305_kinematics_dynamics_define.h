/* Author: Mahoro */

#ifndef ROBOT305_KINEMATICS_DYNAMICS_DEFINE_H_
#define ROBOT305_KINEMATICS_DYNAMICS_DEFINE_H_

namespace robotis_op
{
#define MAX_JOINT_ID    (21)
#define ALL_JOINT_ID    (21)

#define MAX_ARM_ID      (3)
#define MAX_LEG_ID      (6)
#define MAX_ITER        (5)

#define ID_HEAD_END     (20)
#define ID_WAIST        (21)
#define ID_TORSO        (29)

#define ID_R_ARM_START  (1)
#define ID_L_ARM_START  (2)
#define ID_R_ARM_END    (21)
#define ID_L_ARM_END    (22)

#define ID_R_LEG_START  (7)
#define ID_L_LEG_START  (8)
#define ID_R_LEG_END    (31)
#define ID_L_LEG_END    (30)

#define GRAVITY_ACCELERATION (9.8)
}

#endif /* ROBOT305_KINEMATICS_DYNAMICS_DEFINE_H_ */
