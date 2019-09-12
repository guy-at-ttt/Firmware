#include "yona_utils.hpp"

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_log.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <matrix/math.hpp>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_rc_input.h>

// Publisher and Subscriber includes
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_local_position.h>

#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <math.h>

# define MAX_GYRO_COUNT 3
# define THRUST_MOVING_AVG_SPAN 10

void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[]);
void control_yaw(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_magnetometer_s *mag, struct actuator_controls_s *actuators, float rc_channel_values[]);
void control_thrust(const struct vehicle_air_data_s *air_data, struct actuator_controls_s *actuators, float rc_channel_values[]);
// void control_thrust(const struct vehicle_local_position_s *local_pos, const struct vehicle_air_data_s *air_data, struct actuator_controls_s *actuators, float rc_channel_values[]);
float smoothen_baro(const struct vehicle_air_data_s *air_data);
int yona_coaxial_heli_main_thread(int argc, char *argv[]);

extern "C" __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);
// __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);

bool first_iteration_flag = false;
bool yaw_sp_reset = false;
bool rc_input_val_reset = true;
bool thrust_sp_reset = false;

static int deamon_task;
static bool thread_should_exit = false;
static bool thread_running = false;

int gyro_count = 0;
int gyro_selected = 0;
int gyro_sub[MAX_GYRO_COUNT];

float rc_input_val = 0.0f;
float p_err = 0.0f, d_err = 0.0f, dt = 0.0f;
float roll_err_acc = 0.0f, pitch_err_acc = 0.0f, yaw_err_acc = 0.0f;
float roll_err_gyro = 0.0f, pitch_err_gyro = 0.0f, yaw_err_gyro = 0.0f;
float yaw_err_mag = 0.0f, yaw_euler_sp = 0.0f, mag_sp[3] = {0.0f, 0.0f, 0.0f};
float last_roll_err = 0.0f, last_pitch_err = 0.0f, last_yaw_err = 0.0f, last_thrust_err = 0.0f;
float tmp_i_err = 0.0f, yaw_i_err = 0.0f, thrust_i_err = 0.0f;
float tmp_yaw = 0.0f;

float thrust_sp_baro = 0.0f, thrust_err_baro = 0.0f, baro_smooth_val = 0.0f;
int baro_smooth_idx = 0, tmp_thrust_counter = 0;
float prev_baro[THRUST_MOVING_AVG_SPAN];

float thrust_sp = 0.0f, thrust_err = 0.0f;

hrt_abstime rp_curr_time, rp_prev_time, y_curr_time, y_prev_time, th_curr_time, th_prev_time, st_time;