#include "params.h"

/**
 * @file yona_coaxial_heli_params.c
 * Parameters for Yona attitude and position controller.
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Yona Control
 */
PARAM_DEFINE_FLOAT(YONA_ROLL_P, 1.1f);
PARAM_DEFINE_FLOAT(YONA_ROLL_I, 0.045f);
PARAM_DEFINE_FLOAT(YONA_ROLL_D, 0.001f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Yona Control
 */
PARAM_DEFINE_FLOAT(YONA_PITCH_P, 1.1f);
PARAM_DEFINE_FLOAT(YONA_PITCH_I, 0.045f);
PARAM_DEFINE_FLOAT(YONA_PITCH_D, 0.001f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Yona Control
 */
PARAM_DEFINE_FLOAT(YONA_YAW_P, 0.85f);
PARAM_DEFINE_FLOAT(YONA_YAW_I, 0.01f);
PARAM_DEFINE_FLOAT(YONA_YAW_D, 0.001f);

/**
 * Thrust P gain
 *
 * Thrust proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Yona Control
 */
PARAM_DEFINE_FLOAT(YONA_THRUST_P, 0.1f);
PARAM_DEFINE_FLOAT(YONA_THRUST_I, 0.01f);
PARAM_DEFINE_FLOAT(YONA_THRUST_D, 0.001f);

/**
 * Complementary filter Alpha
 *
 * Complementary Filter tuning variable alpha.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Yona Control
 */
PARAM_DEFINE_FLOAT(YONA_ALPHA, 0.02f);
PARAM_DEFINE_FLOAT(YONA_BETA, 0.02f);
PARAM_DEFINE_FLOAT(YONA_TIME_DIFF, 10.0f);

/**
 * Controller Bias gains for RC inputs.
 *
 * Controller Bias gains to include RCcontroller input to PID.
 *
 * @unit 1/s
 * @min 0.0
 * @max 12
 * @decimal 2
 * @increment 0.1
 * @group Yona Control
 */
PARAM_DEFINE_FLOAT(YONA_ROLL_BIAS, 0.3f);
PARAM_DEFINE_FLOAT(YONA_PITCH_BIAS, 0.3f);
PARAM_DEFINE_FLOAT(YONA_YAW_BIAS, 0.3f);
PARAM_DEFINE_FLOAT(YONA_THRUST_BIAS, 0.3f);

PARAM_DEFINE_FLOAT(YONA_YAW_I_MIN, -0.3f);
PARAM_DEFINE_FLOAT(YONA_YAW_I_MAX, 0.3f);
PARAM_DEFINE_FLOAT(YONA_THR_I_MIN, -0.3f);
PARAM_DEFINE_FLOAT(YONA_THR_I_MAX, 0.3f);

PARAM_DEFINE_INT32(YONA_INV_ROLL, 1);
PARAM_DEFINE_INT32(YONA_INV_PITCH, -1);
PARAM_DEFINE_INT32(YONA_INV_YAW, -1);   
