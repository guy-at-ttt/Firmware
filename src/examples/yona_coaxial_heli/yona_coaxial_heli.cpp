/**
 * @file yona_coaxial_heli.cpp
 * Yona app thrust and yaw
 * 
 * Motor - mRo Connections :
 *      PWM MAIN 1: Swashplate servomotor, PITCH axis
 *      PWM MAIN 2: Swashplate servomotor, ROLL axis
 *      PWN MAIN 3: Upper rotor (CCW)
 *      PWM MAIN 4: Lower rotor (CW)
 * 
 * RC Channel mapping :
 *      CH 0: ROLL
 *      CH 1: PITCH
 *      CH 2: YAW
 *      CH 3: THRUST
 *      CH ?: Mode Switch
 *      CH ?: Return Switch
 * 
 * @author TreetownTech User <tttuser@example.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <poll.h>
#include <time.h>

#include "params.h"

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

#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

# define MAX_GYRO_COUNT 3
# define THRUST_MOVING_AVG_SPAN 10

int init_parameters(struct param_handles *handle);
int update_parameters(const struct param_handles *handle, struct params *parameters);
void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[]);
void control_yaw(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_magnetometer_s *mag, struct actuator_controls_s *actuators, float rc_channel_values[]);
void control_thrust(const struct vehicle_air_data_s *air_data, struct actuator_controls_s *actuators, float rc_channel_values[]);
float smoothen_baro(const struct vehicle_air_data_s *air_data);
static void console_print(const char *reason);
int yona_coaxial_heli_main_thread(int argc, char *argv[]);

extern "C" __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);
// __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);

bool verbose = false;
bool first_iteration_flag = false;
bool yaw_sp_reset = false;
int rp_controller_select = 0, yaw_controller_select = 0;

static int deamon_task;
static bool thread_should_exit = false;
static bool thread_running = false;
static struct params pp;
static struct param_handles ph;

int gyro_count = 0;
int gyro_selected = 0;
int gyro_sub[MAX_GYRO_COUNT];

float p_err = 0.0f, d_err = 0.0f, dt = 0.0f;
float roll_err_acc = 0.0f, pitch_err_acc = 0.0f, yaw_err_acc = 0.0f;
float roll_err_gyro = 0.0f, pitch_err_gyro = 0.0f, yaw_err_gyro = 0.0f;
float yaw_err_mag = 0.0f, yaw_euler_sp = 0.0f, mag_sp[3] = {0.0f, 0.0f, 0.0f};
float last_roll_err = 0.0f, last_pitch_err = 0.0f, last_yaw_err = 0.0f, last_thrust_err = 0.0f;

float thrust_sp_baro = 0.0f, thrust_err_baro = 0.0f, baro_smooth_val = 0.0f;
int baro_smooth_idx = 0, tmp_thrust_counter = 0;
float prev_baro[THRUST_MOVING_AVG_SPAN];

hrt_abstime rp_curr_time, rp_prev_time, y_curr_time, y_prev_time, th_curr_time, th_prev_time, st_time;


int init_parameters(struct param_handles *handle) {
    handle->roll_p = param_find("YONA_ROLL_P");
    handle->roll_i = param_find("YONA_ROLL_I");
    handle->roll_d = param_find("YONA_ROLL_D");

    handle->pitch_p = param_find("YONA_PITCH_P");
    handle->pitch_i = param_find("YONA_PITCH_I");
    handle->pitch_d = param_find("YONA_PITCH_D");

    handle->yaw_p = param_find("YONA_YAW_P");
    handle->yaw_i = param_find("YONA_YAW_I");
    handle->yaw_d = param_find("YONA_YAW_D");

    handle->thrust_p = param_find("YONA_THRUST_P");
    handle->thrust_i = param_find("YONA_THRUST_I");
    handle->thrust_d = param_find("YONA_THRUST_D");

    handle->alpha = param_find("YONA_ALPHA");
    handle->beta = param_find("YONA_BETA");
    handle->time_diff = param_find("YONA_TIME_DIFF");

    handle->roll_bias = param_find("YONA_ROLL_BIAS");
    handle->pitch_bias = param_find("YONA_PITCH_BIAS");
    handle->yaw_bias = param_find("YONA_YAW_BIAS");
    handle->thrust_bias = param_find("YONA_THRUST_BIAS");
    return 0;
}

int update_parameters(const struct param_handles *handle, struct params *parameters) {
    // Copy values from parameters to the handles
    param_get(handle->roll_p, &(parameters->roll_p));
    param_get(handle->roll_i, &(parameters->roll_i));
    param_get(handle->roll_d, &(parameters->roll_d));

    param_get(handle->pitch_p, &(parameters->pitch_p));
    param_get(handle->pitch_i, &(parameters->pitch_i));
    param_get(handle->pitch_d, &(parameters->pitch_d));

    param_get(handle->yaw_p, &(parameters->yaw_p));
    param_get(handle->yaw_i, &(parameters->yaw_i));
    param_get(handle->yaw_d, &(parameters->yaw_d));

    param_get(handle->thrust_p, &(parameters->thrust_p));
    param_get(handle->thrust_i, &(parameters->thrust_i));
    param_get(handle->thrust_d, &(parameters->thrust_d));

    param_get(handle->alpha, &(parameters->alpha));
    param_get(handle->beta, &(parameters->beta));
    param_get(handle->time_diff, &(parameters->time_diff));

    param_get(handle->roll_bias, &(parameters->roll_bias));
    param_get(handle->pitch_bias, &(parameters->pitch_bias));
    param_get(handle->yaw_bias, &(parameters->yaw_bias));
    param_get(handle->thrust_bias, &(parameters->thrust_bias));
    return 0;
}

void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[]) {
    // Control Roll and Pitch
    if (rp_controller_select == 0) {
        // Setting ROLL and PITCH to RC input values
        actuators->control[0] = rc_channel_values[1];    // ROLL
        actuators->control[1] = rc_channel_values[2];    // PITCH
    }
    
    else if (rp_controller_select == 1) {
        // PD - accelerometer as Proportional
        //      gyro as Derivative
        
        // Calculating (euler-quat) error and applying P-Gain - accelerometer
        // Roll, Pitch, Yaw -> phi, theta, psi
        roll_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi() - matrix::Eulerf(matrix::Quatf(att->q)).phi();
        pitch_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta() - matrix::Eulerf(matrix::Quatf(att->q)).theta();

        roll_err_gyro = att_sp->roll_body - att->rollspeed;
        pitch_err_gyro = att_sp->pitch_body - att->pitchspeed;

        rp_curr_time = hrt_absolute_time();
        dt = (rp_curr_time - rp_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;
        
        p_err = roll_err_acc;
        d_err = roll_err_gyro;
        actuators->control[0] = (p_err * pp.roll_p) + (d_err * pp.roll_d) + (pp.roll_bias * rc_channel_values[1]);          // ROLL
        
        p_err = pitch_err_acc;
        d_err = pitch_err_gyro;
        actuators->control[1] = (-1 * p_err * pp.pitch_p) + (-1 * d_err * pp.pitch_d) + (pp.pitch_bias * rc_channel_values[2]);          // PITCH
        last_roll_err = roll_err_acc;
    }
    
    else if (rp_controller_select == 2) {
        // PD - accelerometer and gyro - Complementary Filter
        roll_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi() - matrix::Eulerf(matrix::Quatf(att->q)).phi();
        pitch_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta() - matrix::Eulerf(matrix::Quatf(att->q)).theta();

        roll_err_gyro = att_sp->roll_body - att->rollspeed;
        pitch_err_gyro = att_sp->pitch_body - att->pitchspeed;
        if (verbose)
            printf("Pitch_gyro_err: %3.4f\t\t", (double)pitch_err_gyro);

        rp_curr_time = hrt_absolute_time();
        dt = (rp_curr_time - rp_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;

        p_err = (pp.alpha * roll_err_gyro * dt) + ((1.0f - pp.alpha) * roll_err_acc);
        d_err = (p_err - last_roll_err) / dt;
        actuators->control[0] = (p_err * pp.roll_p) + (d_err * pp.roll_d) + (pp.roll_bias * rc_channel_values[1]);              // ROLL
        // float tmp_rp = p_err * pp.roll_p;
        // float tmp_rd = d_err * pp.roll_d;
        // float tmp_rb = rc_channel_values[1] * pp.roll_bias;
        if (verbose) {
            // printf("Roll P_err: %3.4f,    D_err: %3.4f\t\t", (double)p_err, (double)d_err);
            // printf("Roll P: %3.4f,    D: %3.4f,     Bias: %3.4f\t\t", (double)tmp_rp, (double)tmp_rd, (double)tmp_rb);
        }
        last_roll_err = p_err;
        
        p_err = (pp.alpha * pitch_err_gyro * dt) + ((1.0f - pp.alpha) * pitch_err_acc);
        d_err = (p_err - last_pitch_err) / dt;
        if (verbose) {
            printf("Pitch P_err: %3.4f,    D_err: %3.4f,    dt: %.9f,     last_p_err: %3.4f\t\t", (double)p_err, (double)d_err, (double)dt, (double)last_pitch_err);
            // printf("Roll P: %3.4f,    D: %3.4f,     Bias: %3.4f\t\t", (double)tmp_rp, (double)tmp_rd, (double)tmp_rb);
        }
        last_pitch_err = p_err;
        actuators->control[1] = (-1 * p_err * pp.pitch_p) + (-1 * d_err * pp.pitch_d) + (pp.pitch_bias * rc_channel_values[2]);              // PITCH
        if (verbose)
            printf("actuator: %3.4f\n", (double)actuators->control[1]);

        rp_prev_time = hrt_absolute_time();
    }
    
    else if (rp_controller_select == 3) {
        // PD - accelerometer only
        roll_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi() - matrix::Eulerf(matrix::Quatf(att->q)).phi();
        pitch_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta() - matrix::Eulerf(matrix::Quatf(att->q)).theta();

        rp_curr_time = hrt_absolute_time();
        dt = (rp_curr_time - rp_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;
        
        p_err = roll_err_acc;
        d_err = ((roll_err_acc - last_roll_err) / dt);
        actuators->control[0] = (p_err * pp.roll_p) + (d_err * pp.roll_d) + (pp.roll_bias * rc_channel_values[1]);              // ROLL
        
        p_err = pitch_err_acc;
        d_err = ((pitch_err_acc - last_pitch_err) / dt);
        actuators->control[1] = (-1 * p_err * pp.pitch_p) + (-1 * d_err * pp.pitch_d) + (pp.pitch_bias * rc_channel_values[2]);            // PITCH

        rp_prev_time = hrt_absolute_time();
        last_roll_err = roll_err_acc;
        last_pitch_err = pitch_err_acc;
    }

    else {
        warnx("Invalid Controller");
    }

    // printf("roll err: %5.5f, tmp: %5.5f\n", (double)(yaw_err_acc * pp.yaw_p), (double)(((yaw_err_acc - last_yaw_err) / dt) * pp.yaw_d));//, (double)pitch_err_acc);

    // rp_prev_time = hrt_absolute_time();
    // last_roll_err = roll_err_acc;
    // last_pitch_err = pitch_err_acc;
}

void control_yaw(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_magnetometer_s *mag, struct actuator_controls_s *actuators, float rc_channel_values[]) {
    if (first_iteration_flag) {
        printf("Resetting YAW Setpoint\n");
        yaw_euler_sp = matrix::Eulerf(matrix::Quatf(att->q)).psi();
        mag_sp[0] = mag->magnetometer_ga[0];
        mag_sp[1] = mag->magnetometer_ga[1];
        mag_sp[2] = mag->magnetometer_ga[2];
        first_iteration_flag = false;
    }
    
    // YAW
    if (yaw_controller_select == 0) {
        // Radio input
        actuators->control[2] = rc_channel_values[3];          // YAW
    }

    else if (yaw_controller_select == 1) {
        // PD - accelerometer as Proportional
        //      gyro as Derivative
        yaw_err_acc = yaw_euler_sp - matrix::Eulerf(matrix::Quatf(att->q)).psi();
        yaw_err_gyro = att_sp->yaw_body - att->yawspeed;

        y_curr_time = hrt_absolute_time();
        dt = (y_curr_time - y_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;

        p_err = yaw_err_acc;
        d_err = (yaw_err_acc * pp.time_diff) - yaw_err_gyro;
        // printf("%3.4f\t%3.4f\t%3.4f\t\t", (double)(yaw_err_acc * pp.time_diff), (double)d_err, (double)yaw_err_gyro);
        actuators->control[2] = (p_err * pp.yaw_p) + (d_err * pp.yaw_d) + (pp.yaw_bias * rc_channel_values[3]);            // YAW
        
        y_prev_time = hrt_absolute_time();
        last_yaw_err = yaw_err_acc;
    }

    else if (yaw_controller_select == 2) {
        // PD - accelerometer and gyro - Complementary Filter
        yaw_err_acc = yaw_euler_sp - matrix::Eulerf(matrix::Quatf(att->q)).psi();
        yaw_err_gyro = att_sp->yaw_body - att->yawspeed;
        yaw_err_mag = atan2f(-1 * mag_sp[1], mag_sp[0]) - atan2f(-1 * mag->magnetometer_ga[1], mag->magnetometer_ga[0]);

        y_curr_time = hrt_absolute_time();
        dt = (y_curr_time - y_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;

        p_err = (pp.alpha * yaw_err_gyro * dt) + ((1 - pp.alpha) * (((1 - pp.beta) * yaw_err_acc) + (pp.beta * yaw_err_mag)));
        d_err = (p_err - last_yaw_err) / dt;
        actuators->control[2] = (p_err * pp.yaw_p) + (d_err * pp.yaw_d) + (pp.yaw_bias * rc_channel_values[3]);            // YAW
        
        y_prev_time = hrt_absolute_time();
        last_yaw_err = yaw_err_acc;
    }

    else if (yaw_controller_select == 3) {
        // PD - accelerometer only
        yaw_err_acc = yaw_euler_sp - matrix::Eulerf(matrix::Quatf(att->q)).psi();

        y_curr_time = hrt_absolute_time();
        dt = (y_curr_time - y_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;
        
        p_err = yaw_err_acc;
        d_err = (yaw_err_acc - last_yaw_err) / dt;
        actuators->control[2] = (p_err * pp.yaw_p) + (d_err * pp.yaw_d) + (pp.yaw_bias * rc_channel_values[3]);            // YAW
        
        y_prev_time = hrt_absolute_time();
        last_yaw_err = yaw_err_acc;
    }

    else {
        warnx("Invalid Yaw Controller");
    }
}

float smoothen_baro(const struct vehicle_air_data_s *air_data) {
    float sum = 0.0f, avg = 0.0f;
    if (baro_smooth_idx >= THRUST_MOVING_AVG_SPAN)
        baro_smooth_idx %= THRUST_MOVING_AVG_SPAN;
    prev_baro[baro_smooth_idx] = air_data->baro_alt_meter;

    for (int i=0; i < THRUST_MOVING_AVG_SPAN; i++)
        sum += prev_baro[i];
    avg = (float)(sum / THRUST_MOVING_AVG_SPAN);
    baro_smooth_idx++;
    return avg;
}

void control_thrust(const struct vehicle_air_data_s *air_data, struct actuator_controls_s *actuators, float rc_channel_values[]) {
    // THRUST
    // printf("%d\n", (int)rc_channel_values[7]);
    baro_smooth_val = smoothen_baro(air_data);

    if (tmp_thrust_counter <= THRUST_MOVING_AVG_SPAN) {
        actuators->control[3] = rc_channel_values[0];
        tmp_thrust_counter++;
    }
    else {
        if ((int)rc_channel_values[7] < 0) {
            actuators->control[3] = rc_channel_values[0];       // Manual Control
        }
        else if ((int)rc_channel_values[7] == 0) {
            // Partial control
            thrust_sp_baro = baro_smooth_val;
            actuators->control[3] = rc_channel_values[0];       // Manual Control
        }
        else {
            // Altitude hold
            thrust_err_baro = thrust_sp_baro - baro_smooth_val;

            th_curr_time = hrt_absolute_time();
            dt = (th_curr_time - th_prev_time)/10e6;
            if (dt < 0.002f)
                dt = 0.002f;
            if (dt > 0.02f)
                dt = 0.02f;

            p_err = thrust_err_baro;
            d_err = (thrust_err_baro - last_thrust_err) / dt;
            actuators->control[3] = (p_err * pp.thrust_p) + (d_err * pp.thrust_d) + (pp.thrust_bias * (rc_channel_values[0] - 0.5f);            // THRUST

            th_prev_time = hrt_absolute_time();
            last_thrust_err = thrust_err_baro;
        }
    }
    actuators->timestamp = hrt_absolute_time();
    // printf("%5.4f, %5.4f, %5.4f\t\t%5.4f, %5.4f, %5.4f\n",
    //                     (double)matrix::Eulerf(matrix::Quatf(att->q)).phi(),
    //                     (double)matrix::Eulerf(matrix::Quatf(att->q)).theta(),
    //                     (double)matrix::Eulerf(matrix::Quatf(att->q)).psi(),
    //                     (double)att->rollspeed, (double)att->pitchspeed, (double)att->yawspeed);
}

static void console_print(const char *reason) {
    if (reason)
        fprintf(stderr, "%s\n", reason);
    
    fprintf(stderr, "Usage: yona_coaxial_heli {start|stop|status}\n\n");
}

int yona_coaxial_heli_main_thread(int argc, char *argv[]) {
    // Main thread
    /* ------ Reading arguments ------ */
	verbose = false;
    // bool rp_controller = false;
    // bool y_controller = false;
    float alpha = 0.02f, beta = 0.0f, time_diff = 0.1f;
    bool tune_alpha = false, tune_beta = false;
    bool tune_timediff = false;
    bool tune_params = false;
    // bool tune_flags[6] = {false, false, false, false, false, false};
    bool tune_flags[16] = {};
    for (int i = 0; i < 16; i++)
        tune_flags[i] = false;
    float tmp_roll[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_pitch[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_yaw[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_thrust[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--controller") == 0) {
            // rp_controller = true;
            if ((i + 1) < argc) {
                if (!strcmp(argv[i+1], "manual") || !strcmp(argv[i+1], "0")) {
                    rp_controller_select = 0;
                    yaw_controller_select = 0;
                }

                else if (!strcmp(argv[i+1], "gyrod") || !strcmp(argv[i+1], "1")) {
                    rp_controller_select = 1;
                    yaw_controller_select = 0;
                    
                    if (!strcmp(argv[i+2], "timediff") || !strcmp(argv[i+2], "td")) {
                        tune_timediff = true;
                        time_diff = atof(argv[i+3]);
                        printf("\tChanging inverse of time difference for gryo derivative to: %2.3f\n", (double)time_diff);
                    }
                    else {
                        tune_timediff = true;
                        time_diff = 10.0f;
                        printf("\tdefault time difference for gryo derivative to: %2.3f\n", (double)time_diff);
                    }
                }
                
                else if (!strcmp(argv[i+1], "comp") || !strcmp(argv[i+1], "2")) {
                    rp_controller_select = 2;
                    yaw_controller_select = 0;

                    if (!strcmp(argv[i+2], "alpha") || !strcmp(argv[i+2], "a")) {
                        tune_alpha = true;
                        alpha = atof(argv[i+3]);
                        printf("\tChanging complimentary filter coefficient ALPHA to: %2.3f\n", (double)alpha);
                    }
                    else {
                        tune_alpha = true;
                        alpha = 0.02f;
                        printf("\tdefault ALPHA to: %2.3f\n", (double)alpha);
                    }
                }
                
                else if (!strcmp(argv[i+1], "accpd") || !strcmp(argv[i+1], "3")) {
                    rp_controller_select = 3;
                    yaw_controller_select = 0;
                }
                
                else {
                    fprintf(stderr, "Usage: yona_coaxial_heli start -c <value> \n\tValues:\n\t\t0 | manual\t\t- Radio Control\n\t\t1 | gyrod\t\t- P (accelerometer) and D (gyro)\n\t\t2 | comp\t\t- PD with complementary filter on accelerometer and gyro\n\t\t3 | accpd\t\t- PD with accelerometer only");
                }
            }
            else {
                fprintf(stderr, "Usage: yona_coaxial_heli start -c <value> \n\tValues:\n\t\t0 | manual\t\t- Radio Control\n\t\t1 | gyrod\t\t- P (accelerometer) and D (gyro)\n\t\t2 | comp\t\t- PD with complementary filter on accelerometer and gyro\n\t\t3 | accpd\t\t- PD with accelerometer only");
            }
        }
        
        if (strcmp(argv[i], "-yc") == 0 || strcmp(argv[i], "--yawcontroller") == 0) {
            // rp_controller = true;
            // y_controller = true;
            if ((i + 1) < argc) {
                if (!strcmp(argv[i+1], "manual") || !strcmp(argv[i+1], "0")) {
                    rp_controller_select = 0;
                    yaw_controller_select = 0;
                }
                else if (!strcmp(argv[i+1], "gyrod") || !strcmp(argv[i+1], "1")) {
                    rp_controller_select = 1;
                    yaw_controller_select = 1;
                    
                    if (!strcmp(argv[i+2], "timediff") || !strcmp(argv[i+2], "td")) {
                        tune_timediff = true;
                        time_diff = atof(argv[i+3]);
                        printf("\tChanging inverse of time difference for gryo derivative to: %2.3f\n", (double)time_diff);
                    }
                    else {
                        tune_timediff = true;
                        time_diff = 10.0f;
                        printf("\tdefault time difference for gryo derivative to: %2.3f\n", (double)time_diff);
                    }
                }
                else if (!strcmp(argv[i+1], "comp") || !strcmp(argv[i+1], "2")) {
                    rp_controller_select = 2;
                    yaw_controller_select = 2;

                    if (!strcmp(argv[i+2], "alpha") || !strcmp(argv[i+2], "a")) {
                        tune_alpha = true;
                        alpha = atof(argv[i+3]);
                        printf("\tChanging complimentary filter coefficient ALPHA to: %2.3f\n", (double)alpha);

                        tune_beta = true;
                        beta = 0.02f;
                        printf("\tdefault BETA to: %2.3f\n", (double)beta);
                    }
                    else if (!strcmp(argv[i+2], "beta") || !strcmp(argv[i+2], "b")) {
                        tune_alpha = true;
                        alpha = 0.02f;
                        printf("\tdefault ALPHA to: %2.3f\n", (double)alpha);

                        tune_beta = true;
                        beta = atof(argv[i+3]);
                        printf("\tChanging complimentary filter coefficient BETA to: %2.3f\n", (double)beta);
                    }
                    else {
                        tune_alpha = true;
                        alpha = 0.02f;
                        printf("\tdefault ALPHA to: %2.3f\n", (double)alpha);

                        tune_beta = true;
                        beta = 0.02f;
                        printf("\tdefault BETA to: %2.3f\n", (double)beta);
                    }
                }
                else if (!strcmp(argv[i+1], "accpd") || !strcmp(argv[i+1], "3")) {
                    rp_controller_select = 3;
                    yaw_controller_select = 3;
                }
                else {
                    fprintf(stderr, "Usage: yona_coaxial_heli start -yc <value> \n\tValues:\n\t\t0 | manual\t\t- Radio Control\n\t\t1 | gyrod\t\t- P (accelerometer) and D (gyro)\n\t\t2 | comp\t\t- PD with complementary filter on accelerometer and gyro\n\t\t3 | accpd\t\t- PD with accelerometer only");
                }
            }
            else {
                fprintf(stderr, "Usage: yona_coaxial_heli start -yc <value> \n\tValues:\n\t\t0 | manual\t\t- Radio Control\n\t\t1 | gyrod\t\t- P (accelerometer) and D (gyro)\n\t\t2 | comp\t\t- PD with complementary filter on accelerometer and gyro\n\t\t3 | accpd\t\t- PD with accelerometer only");
            }
        }
        
        if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tune") == 0) {
            if ((i + 2) < argc) {
                tune_params = true;
                if (strcmp(argv[i+1], "rp") == 0) {
                    tune_flags[0] = true;
                    tmp_roll[0] = atof(argv[i+2]);
                    printf("\tChanging Roll Proportional Gain (rp) value to: %2.3f\n", (double)tmp_roll[0]);
                }
                else if (strcmp(argv[i+1], "ri") == 0) {
                    tune_flags[1] = true;
                    tmp_roll[1] = atof(argv[i+2]);
                    printf("\tChanging Roll Integral Gain (ri) value to: %2.3f\n", (double)tmp_roll[1]);
                }
                else if (strcmp(argv[i+1], "rd") == 0) {
                    tune_flags[2] = true;
                    tmp_roll[2] = atof(argv[i+2]);
                    printf("\tChanging Roll Derivative Gain (rd) value to: %2.3f\n", (double)tmp_roll[2]);
                }
                else if (!strcmp(argv[i+1], "rb")) {
                    tune_flags[3] = true;
                    tmp_roll[3] = atof(argv[i+2]);
                    printf("\tChanging Roll RC input bias (rb) value to: %2.3f\n", (double)tmp_roll[3]);
                }
                
                else if (strcmp(argv[i+1], "pp") == 0) {
                    tune_flags[4] = true;
                    tmp_pitch[0] = atof(argv[i+2]);
                    printf("\tChanging Pitch Proportional Gain (pp) value to: %2.3f\n", (double)tmp_pitch[0]);
                }
                else if (strcmp(argv[i+1], "pi") == 0) {
                    tune_flags[5] = true;
                    tmp_pitch[1] = atof(argv[i+2]);
                    printf("\tChanging Pitch Integral Gain (pi) value to: %2.3f\n", (double)tmp_pitch[1]);
                }
                else if (strcmp(argv[i+1], "pd") == 0) {
                    tune_flags[6] = true;
                    tmp_pitch[2] = atof(argv[i+2]);
                    printf("\tChanging Pitch Derivative Gain (pd) value to: %2.3f\n", (double)tmp_pitch[2]);
                }
                else if (strcmp(argv[i+1], "pb") == 0) {
                    tune_flags[7] = true;
                    tmp_pitch[3] = atof(argv[i+2]);
                    printf("\tChanging Pitch RC input Bias (pb) value to: %2.3f\n", (double)tmp_pitch[3]);
                }
                
                else if (strcmp(argv[i+1], "yp") == 0) {
                    tune_flags[8] = true;
                    tmp_yaw[0] = atof(argv[i+2]);
                    printf("\tChanging Yaw Proportional Gain (yp) value to: %2.3f\n", (double)tmp_yaw[0]);
                }
                else if (strcmp(argv[i+1], "yi") == 0) {
                    tune_flags[9] = true;
                    tmp_yaw[1] = atof(argv[i+2]);
                    printf("\tChanging Yaw Integral Gain (yi) value to: %2.3f\n", (double)tmp_yaw[1]);
                }
                else if (strcmp(argv[i+1], "yd") == 0) {
                    tune_flags[10] = true;
                    tmp_yaw[2] = atof(argv[i+2]);
                    printf("\tChanging Yaw Derivative Gain (yd) value to: %2.3f\n", (double)tmp_yaw[2]);
                }
                else if (strcmp(argv[i+1], "yb") == 0) {
                    tune_flags[11] = true;
                    tmp_yaw[3] = atof(argv[i+2]);
                    printf("\tChanging Yaw RC input Bias (yb) value to: %2.3f\n", (double)tmp_yaw[3]);
                }
                
                else if (strcmp(argv[i+1], "tp") == 0) {
                    tune_flags[12] = true;
                    tmp_thrust[0] = atof(argv[i+2]);
                    printf("\tChanging Thrust Proportional Gain (tp) value to: %2.3f\n", (double)tmp_thrust[0]);
                }
                else if (strcmp(argv[i+1], "ti") == 0) {
                    tune_flags[13] = true;
                    tmp_thrust[1] = atof(argv[i+2]);
                    printf("\tChanging Thrust Integral Gain (ti) value to: %2.3f\n", (double)tmp_thrust[1]);
                }
                else if (strcmp(argv[i+1], "td") == 0) {
                    tune_flags[14] = true;
                    tmp_thrust[2] = atof(argv[i+2]);
                    printf("\tChanging Thrust Derivative Gain (td) value to: %2.3f\n", (double)tmp_thrust[2]);
                }
                else if (strcmp(argv[i+1], "tb") == 0) {
                    tune_flags[15] = true;
                    tmp_thrust[3] = atof(argv[i+2]);
                    printf("\tChanging Thrust RC input Bias (tb) value to: %2.3f\n", (double)tmp_thrust[3]);
                }
                
                else
                    fprintf(stderr, "Usage: yona_coaxial_heli start -t <parameter> <value>\n\n\tparameters:\n\t\trp\tRoll Proportional Gain\n\t\tpp\tPitch Proportional Gain\n\n\tvalues:\n\t\trp\tmin:0, max:12.00\n\t\tpp\tmin:0, max:12.00\n\n");
            }
            else {
                // TODO: Edit
                fprintf(stderr, "Usage: yona_coaxial_heli start -t <parameter> <value>\n\n\tparameters:\n\t\trp\tRoll Proportional Gain\n\t\tpp\tPitch Proportional Gain\n\n\tvalues:\n\t\trp\tmin:0, max:12.00\n\t\tpp\tmin:0, max:12.00\n\n");
            }
        }
	}

    /* ------ Initializing the Input parameters ------ */
    
    init_parameters(&ph);
    if (tune_params) {
        if (tune_flags[0])
            param_set(ph.roll_p, (const void*)&tmp_roll[0]);
        if (tune_flags[1])
            param_set(ph.roll_i, (const void*)&tmp_roll[1]);
        if (tune_flags[2])
            param_set(ph.roll_d, (const void*)&tmp_roll[2]);
        if (tune_flags[3]) {
            param_set(ph.roll_bias, (const void*)&tmp_roll[3]);
            // printf("Roll Buas: %2.3f\n", (double)ph.roll_bias);
        }
        
        if (tune_flags[4])
            param_set(ph.pitch_p, (const void*)&tmp_pitch[0]);
        if (tune_flags[5])
            param_set(ph.pitch_i, (const void*)&tmp_pitch[1]);
        if (tune_flags[6])
            param_set(ph.pitch_d, (const void*)&tmp_pitch[2]);
        if (tune_flags[7])
            param_set(ph.pitch_bias, (const void*)&tmp_pitch[3]);
        
        if (tune_flags[8])
            param_set(ph.yaw_p, (const void*)&tmp_yaw[0]);
        if (tune_flags[9])
            param_set(ph.yaw_i, (const void*)&tmp_yaw[1]);
        if (tune_flags[10])
            param_set(ph.yaw_d, (const void*)&tmp_yaw[2]);
        if (tune_flags[11])
            param_set(ph.yaw_bias, (const void*)&tmp_yaw[3]);
        
        if (tune_flags[12])
            param_set(ph.thrust_p, (const void*)&tmp_thrust[0]);
        if (tune_flags[13])
            param_set(ph.thrust_i, (const void*)&tmp_thrust[1]);
        if (tune_flags[14])
            param_set(ph.thrust_d, (const void*)&tmp_thrust[2]);
        if (tune_flags[15])
            param_set(ph.thrust_bias, (const void*)&tmp_thrust[3]);
    }

    if (tune_alpha)
        param_set(ph.alpha, (const void*)&alpha);
    if (tune_beta)
        param_set(ph.beta, (const void*)&beta);
    if (tune_timediff)
        param_set(ph.time_diff, (const void*)&time_diff);
    
    for (int i = 0; i < 16; i++) {
        // printf(tune_flags[i] ? "true\n" : "false\n");
        tune_flags[i] = false;
    }

    update_parameters(&ph, &pp);
    printf("GAINS:\n\troll p: %2.3f\n\troll i: %2.3f\n\troll d: %2.3f\n\troll bias: %2.3f\n\n\tpitch p: %2.3f\n\tpitch i: %2.3f\n\tpitch d: %2.3f\n\tpitch bias: %2.3f\n\n\tyaw p: %2.3f\n\tyaw i: %2.3f\n\tyaw d: %2.3f\n\tyaw bias: %2.3f\n\n\tthrust p: %2.3f\n\tthrust i: %2.3f\n\tthrust d: %2.3f\n\tthrust bias: %2.3f\n\n\talpha (complementary filter GYRO): %2.3f\n\tbeta (complementary filter MAGNETOMETER): %2.3f\n\ttime_diff: %2.3f\n", (double)pp.roll_p, (double)pp.roll_i, (double)pp.roll_d, (double)pp.roll_bias, (double)pp.pitch_p, (double)pp.pitch_i, (double)pp.pitch_d, (double)pp.pitch_bias, (double)pp.yaw_p, (double)pp.yaw_i, (double)pp.yaw_d, (double)pp.yaw_bias, (double)pp.thrust_p, (double)pp.thrust_i, (double)pp.thrust_d, (double)pp.thrust_bias, (double)pp.alpha, (double)pp.beta, (double)pp.time_diff);


    struct vehicle_attitude_s           att;
    struct vehicle_attitude_setpoint_s  att_sp;
    struct manual_control_setpoint_s    manual_sp;
    struct vehicle_status_s             v_status;
    struct vehicle_rates_setpoint_s     rates_sp;
    struct vehicle_global_position_s    global_pos;
	struct position_setpoint_s          global_sp;
    struct rc_channels_s                rc_channels;
	struct sensor_gyro_s			    gyro_sensor;           // gyro data before thermal correctons and ekf bias estimates are applied
	struct sensor_correction_s		    gyro_correction;    // sensor thermal corrections
	struct sensor_bias_s			    gyro_bias;          // sensor in-run bias corrections
    struct vehicle_magnetometer_s       mag;
    struct vehicle_air_data_s           air_data;

    
    memset(&att, 0, sizeof(att));
    memset(&att_sp, 0, sizeof(att_sp));
    memset(&manual_sp, 0, sizeof(manual_sp));
    memset(&v_status, 0, sizeof(v_status));
    memset(&rates_sp, 0, sizeof(rates_sp));
	memset(&global_pos, 0, sizeof(global_pos));
	memset(&global_sp, 0, sizeof(global_sp));
    memset(&rc_channels, 0, sizeof(rc_channels));
    memset(&gyro_sensor, 0, sizeof(gyro_sensor));
    memset(&gyro_correction, 0, sizeof(gyro_correction));
    memset(&mag, 0, sizeof(mag));
    memset(&air_data, 0, sizeof(air_data));

    /* ------ Arming ------ */
    // TODO: Change to a button trigger
	struct actuator_armed_s arm;
	memset(&arm, 0, sizeof(arm));

	// arm.timestamp = hrt_absolute_time();
	// arm.ready_to_arm = true;
	// arm.armed = true;
	// orb_advert_t arm_pub_ptr = orb_advertise(ORB_ID(actuator_armed), &arm);
	// orb_publish(ORB_ID(actuator_armed), arm_pub_ptr, &arm);

	/* read back values to validate */
	// int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	// orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

	// if (arm.ready_to_arm && arm.armed) {
	// 	warnx("Actuators Armed");
	// } else {
	// 	warnx("Actuators Not Armed");
	// }


    /* ------ Initializing the Output parameters ------ */
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));

    // Publish 0 values to actuators
    for (unsigned i=0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
        actuators.control[i] = 0.0f;
    }

    /* ------ Advertise these controllers (actuator_pub and rates_pub) as publishers of
              these topics (actuator_controls and rates_sp)                     ------ */
    // orb_advertise(topic) returns handles for the topics.
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
    orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

    /* ------ Subscribe to rest of the topics ------ */
    // orb_subscribe(topic) returns handles for the topics.
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    int v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    int global_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
    int param_sub = orb_subscribe(ORB_ID(parameter_update));
    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
    int mag_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
    int air_data_sub = orb_subscribe(ORB_ID(vehicle_air_data));

	gyro_count = math::min(orb_group_count(ORB_ID(sensor_gyro)), MAX_GYRO_COUNT);
	if (gyro_count == 0)
		gyro_count = 1;
	for (int i = 0; i < gyro_count; i++)
		gyro_sub[i] = orb_subscribe_multi(ORB_ID(sensor_gyro), i);

	int gyro_correction_sub = orb_subscribe(ORB_ID(sensor_correction));
	int gyro_bias_sub = orb_subscribe(ORB_ID(sensor_bias));

    st_time = hrt_absolute_time();
    rp_prev_time = st_time;
    y_prev_time = st_time;

    
    for (int i=0; i<THRUST_MOVING_AVG_SPAN; i++)
        prev_baro[i] = 0.0f;


    struct pollfd fds[2] = {};
    fds[0].fd = param_sub;          // Descriptor being polled
    fds[0].events = POLLIN;         // Low/Medium priority data may be read without blocking
    fds[1].fd = att_sub;
    fds[1].events = POLLIN;

    while (!thread_should_exit) {
        // Parameters : list of file descriptors (structures);
        //              number of list items (descriptors);
        //              timeout (in ms)
        // Returns : number of structures that have non-zero revents fields.
        //           0; if times out waiting for a "descriptor ready"

        orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);
        // first_iteration_flag = arm.armed ? true : false;
        if (arm.armed && yaw_sp_reset) {
            // warnx("Actuators Armed");
            first_iteration_flag = true;
            yaw_sp_reset = false;
        }
        if (!arm.armed) {
            // warnx("Actuators Not Armed");
            yaw_sp_reset = true;
        }
        
        int poll_ret_val = poll(fds, 2, 500);

        if (poll_ret_val < 0) {
            warnx("Poll Error");
        }
        else if (poll_ret_val == 0) {
            continue;
        }
        else {
            // Main Control Thread Logic
            // Failsafes, Sanity checks, Read RC inputs, Backup Values, Update att and att_sp, Publish actuator values

            // Check and update parameters, only if any changes
            if (fds[0].revents & POLLIN) {
                struct parameter_update_s value_updates;
                orb_copy(ORB_ID(parameter_update), param_sub, &value_updates);
                
                // TODO: Update gains while running 'start -c'.?

                update_parameters(&ph, &pp);
            }

            // Run controller
            if (fds[1].revents & POLLIN) {
                // Checking for new position setpoint
                bool manual_sp_updated;
                orb_check(manual_sp_sub, &manual_sp_updated);
                // if (manual_sp_updated)
                //     orb_copy(ORB_ID(manual_sp), param_sub, &value_updates);
                bool pos_updated;
                orb_check(global_pos_sub, &pos_updated);
                bool global_sp_updated;
                orb_check(global_sp_sub, &global_sp_updated);
                bool att_sp_updated;
                orb_check(att_sp_sub, &att_sp_updated);
                
                // Creating a local copy
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
                orb_copy(ORB_ID(vehicle_magnetometer), mag_sub, &mag);
                orb_copy(ORB_ID(vehicle_air_data), air_data_sub, &air_data);
                if (att_sp_updated)
                    orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);
                
                bool gyro_updated;
                orb_check(gyro_correction_sub, &gyro_updated);
                if (gyro_updated)
                    orb_copy(ORB_ID(sensor_correction), gyro_correction_sub, &gyro_correction);
                // Update the latest gyro selection
                if (gyro_correction.selected_gyro_instance < gyro_count)
                    gyro_selected = gyro_correction.selected_gyro_instance;
                
                bool gyro_bias_updated;
                orb_check(gyro_bias_sub, &gyro_bias_updated);
                if (gyro_bias_updated)
                    orb_copy(ORB_ID(sensor_bias), gyro_bias_sub, &gyro_bias);
                
                float rates[3] = {0.0f, 0.0f, 0.0f};
                if (gyro_selected == 0) {
                    rates[0] = (gyro_sensor.x - gyro_correction.gyro_offset_0[0]) * gyro_correction.gyro_scale_0[0];
                    rates[1] = (gyro_sensor.y - gyro_correction.gyro_offset_0[1]) * gyro_correction.gyro_scale_0[1];
                    rates[2] = (gyro_sensor.z - gyro_correction.gyro_offset_0[2]) * gyro_correction.gyro_scale_0[2];
                }
                else if (gyro_selected == 1) {
                    rates[0] = (gyro_sensor.x - gyro_correction.gyro_offset_1[0]) * gyro_correction.gyro_scale_1[0];
                    rates[1] = (gyro_sensor.y - gyro_correction.gyro_offset_1[1]) * gyro_correction.gyro_scale_1[1];
                    rates[2] = (gyro_sensor.z - gyro_correction.gyro_offset_1[2]) * gyro_correction.gyro_scale_1[2];
                }
                // correct for in-run bias errors
                rates[0] -= gyro_bias.gyro_x_bias;
                rates[1] -= gyro_bias.gyro_y_bias;
                rates[2] -= gyro_bias.gyro_z_bias;
                
                // Checking RC inputs
                orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels);
                // printf("Input RC - %f, %f, %f, %f\t\t", (double)rc_channels.channels[1]*1000, (double)rc_channels.channels[2]*1000, (double)rc_channels.channels[3]*1000, (double)rc_channels.channels[0]*1000);
                
                control_right_stick(&att, &att_sp, &actuators, rc_channels.channels);
                control_yaw(&att, &att_sp, &mag, &actuators, rc_channels.channels);
                control_thrust(&air_data, &actuators, rc_channels.channels);

                // printf("%5.8f, %5.8f, %5.8f\n", (double)rates[0], (double)rates[1], (double)rates[2]);

                // TODO: Throttle limit check.?

                // Reading info on current vehicle status and flight mode
                orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                // Publishing rates
                orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

                // if (verbose)
                //     printf("Actuator outputs - %2.4f\t%2.4f\t%2.4f\t%2.4f\n", (double)actuators.control[0], (double)actuators.control[1], (double)actuators.control[2], (double)actuators.control[3]);

                // Sanity check and publishing actuator outputs
                if (PX4_ISFINITE(actuators.control[0]) && PX4_ISFINITE(actuators.control[1]) && PX4_ISFINITE(actuators.control[2]) && PX4_ISFINITE(actuators.control[3])) {
                    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                    // printf("Actuator outputs - %d, %d, %d, %d\n\n", (int)actuators.control[0]*100, (int)actuators.control[1]*100, (int)actuators.control[2]*100, (int)actuators.control[3]*100);

                    // if (verbose) {
                    //     warnx("Actuator controls - Published");
                    // }
                }
            }
        }
    }

    printf("[yona_coaxial_heli] exiting, stopping all motors.\n");
    thread_running = false;

    for(unsigned i=0; i<actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
        actuators.control[i] = 0.0f;
    }
    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
    
    fflush(stdout);
    return 0;
}

int yona_coaxial_heli_main(int argc, char *argv[]) {
    PX4_INFO("__ YONA __");

    // Checking for arguments in the command {start|status|stop|tune}        // TODO: Keep this.?
    if (argc < 2) {
        console_print("missing command");
        return 1;
    }

    printf("\nMAIN argc: %d\n", argc);
    // Command: yona_coaxial_heli start
    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("running");
            return 0;
        }
        
        thread_should_exit = false;
        // px4_task_spawn_cmd - name, (int) scheduler, (int) priority, (int) stack_size, (px4_main_t) entry, argv[]
        deamon_task = px4_task_spawn_cmd("yona_coaxial_heli",
                                        SCHED_DEFAULT,
                                        SCHED_PRIORITY_MAX - 20,
                                        2048,
                                        yona_coaxial_heli_main_thread,
                                        (argv) ? (char *const *)argv : (char *const *)nullptr);
        thread_running = true;
        return 0;
    }
    
    // Command: yona_coaxial_heli stop
    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        return 0;
    }

    // Command: yona_coaxial_heli status
    if (!strcmp(argv[1], "status")) {
        if (thread_running)
            warnx("running");
        else
            warnx("not running");
        return 0;
    }

    console_print("unrecognized command");
    return 1;
}


// Check disarm time delay (mRo arms at non-min throttle after disarming < 4 sec)
// Add function in init.d file on mRo - Test
// Add position hold mode (built in function <uses GPS.?> __OR__ radio switch toggle.?)
// Estimator for position based on accelerometer readings
// PID for position hold - input position and/or velocity - output velocity ---- x and y linear velocity <directly proportional to> roll and pitch angles