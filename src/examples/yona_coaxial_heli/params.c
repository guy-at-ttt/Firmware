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
PARAM_DEFINE_FLOAT(YONA_ROLL_P, 2.0f);
PARAM_DEFINE_FLOAT(YONA_ROLL_I, 0.01f);
PARAM_DEFINE_FLOAT(YONA_ROLL_D, 0.001f);

PARAM_DEFINE_FLOAT(YONA_PITCH_P, 2.0f);
PARAM_DEFINE_FLOAT(YONA_PITCH_I, 0.01f);
PARAM_DEFINE_FLOAT(YONA_PITCH_D, 0.001f);

PARAM_DEFINE_FLOAT(YONA_YAW_P, 3.0f);
PARAM_DEFINE_FLOAT(YONA_YAW_I, 0.01f);
PARAM_DEFINE_FLOAT(YONA_YAW_D, 0.001f);

PARAM_DEFINE_FLOAT(YONA_THRUST_P, 0.1f);
PARAM_DEFINE_FLOAT(YONA_THRUST_I, 0.01f);
PARAM_DEFINE_FLOAT(YONA_THRUST_D, 0.001f);
