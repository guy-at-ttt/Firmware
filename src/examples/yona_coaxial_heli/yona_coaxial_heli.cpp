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

#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

# define MAX_GYRO_COUNT 3

int init_parameters(struct param_handles *handle);
int update_parameters(const struct param_handles *handle, struct params *parameters);
void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[], bool rp_controller);
void control_thrust(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[], bool y_controller);
static void console_print(const char *reason);
int yona_coaxial_heli_main_thread(int argc, char *argv[]);

extern "C" __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);
// __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);


static int deamon_task;
static bool thread_should_exit = false;
static bool thread_running = false;
static struct params pp;
static struct param_handles ph;

int gyro_count = 0;
int gyro_selected = 0;
int gyro_sub[MAX_GYRO_COUNT];
float last_roll_err = 0.0f, last_pitch_err = 0.0f, last_yaw_err = 0.0f;

hrt_abstime rp_curr_time, rp_prev_time, y_curr_time, y_prev_time, st_time;


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
    return 0;
}

void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[], bool rp_controller) {
    // Control Roll and Pitch
    // Update PID gains
    if (rp_controller) {
        // Calculating (euler-quat) error and applying P-Gain
        // Roll, Pitch, Yaw -> phi, theta, psi
        // float tmp = matrix::Eulerf(matrix::Quatf(att->q)).phi();
        float roll_err = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi() - matrix::Eulerf(matrix::Quatf(att->q)).phi();
        float pitch_err = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta() - matrix::Eulerf(matrix::Quatf(att->q)).theta();

        rp_curr_time = hrt_absolute_time();
        float dt = (rp_curr_time - rp_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;
        // printf("roll err: %5.5f, tmp: %5.5f\n", (double)roll_err, (double)tmp);//, (double)pitch_err);
        // dt = dt;

        actuators->control[0] = (roll_err * pp.roll_p) + (((roll_err - last_roll_err) / dt) * pp.roll_d);          // ROLL
        actuators->control[1] = (pitch_err * pp.pitch_p) + (((pitch_err - last_pitch_err) / dt) * pp.pitch_d);        // PITCH

        // printf("roll err: %5.5f, tmp: %5.5f\n", (double)(yaw_err * pp.yaw_p), (double)(((yaw_err - last_yaw_err) / dt) * pp.yaw_d));//, (double)pitch_err);

        rp_prev_time = hrt_absolute_time();
        last_roll_err = roll_err;
        last_pitch_err = pitch_err;

        // printf("Att:\trollspeed: %3.5f\tpitchspeed:%3.5f\tyawspeed: %3.5f\n", (double)att->rollspeed, (double)att->pitchspeed, (double)att->yawspeed);
        // printf("Att_SP:\trollbody: %3.5f\tpitchbody:%3.5f\tyawbody: %3.5f\t, thrust: %3.5f\n", (double)att_sp->roll_body, (double)att_sp->pitch_body, (double)att_sp->yaw_body, (double)att_sp->thrust);
        // printf("Timestamps:\tatt: %d\tatt_sp: %d\n\n", att->timestamp, att_sp->timestamp);

        // printf("att q - %3.5f, %3.5f, %3.5f, %3.5f\n", (double)att->q[0], (double)att->q[1], (double)att->q[2], (double)att->q[3]);
        // printf("\t\tatt_sp q_d - %f, %f, %f, %f\n", (double)att_sp->q_d[0]*1000, (double)att_sp->q_d[1]*1000, (double)att_sp->q_d[2]*1000, (double)att_sp->q_d[3]*1000);
    }
    else {
        // Setting ROLL and PITCH to 0
        actuators->control[0] = rc_channel_values[1];    // ROLL
        actuators->control[1] = -1 * rc_channel_values[2];    // PITCH
    }
}

void control_thrust(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, float rc_channel_values[], bool y_controller) {
    // Control thrust
    // TODO: Add a controller to update PI gains on yaw and thrust
    actuators->control[3] = rc_channel_values[0];   // THRUST
    
    // YAW
    if (y_controller) {
        float yaw_err = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).psi() - matrix::Eulerf(matrix::Quatf(att->q)).psi();

        y_curr_time = hrt_absolute_time();
        float dt = (y_curr_time - y_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;
        
        actuators->control[2] = (yaw_err * pp.yaw_p) + (((yaw_err - last_yaw_err) / dt) * pp.yaw_d);            // YAW        
        y_prev_time = hrt_absolute_time();
        last_yaw_err = yaw_err;
    }
    else {
        actuators->control[2] = rc_channel_values[3];          // YAW
    }

    actuators->timestamp = hrt_absolute_time();

    // printf("control_thrust - %f, %f, %f, %f\n", (double)actuators->control[0]*1000, (double)actuators->control[1]*1000, (double)actuators->control[2]*1000, (double)actuators->control[3]*1000);
}

static void console_print(const char *reason) {
    if (reason)
        fprintf(stderr, "%s\n", reason);
    
    fprintf(stderr, "Usage: yona_coaxial_heli {start|stop|status}\n\n");
}

int yona_coaxial_heli_main_thread(int argc, char *argv[]) {
    // Main thread
    /* ------ Reading arguments ------ */
	bool verbose = false;
    bool rp_controller = false;
    bool y_controller = false;
    bool tune_params = false;
    // bool tune_flags[6] = {false, false, false, false, false, false};
    bool tune_flags[12] = {};
    for (int i = 0; i < 12; i++)
        tune_flags[i] = false;
    float tmp_roll[3] = {0.0f, 0.0f, 0.0f};
    float tmp_pitch[3] = {0.0f, 0.0f, 0.0f};
    float tmp_yaw[3] = {0.0f, 0.0f, 0.0f};
    
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
        if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--controller") == 0) {
            rp_controller = true;
        }
        
        if (strcmp(argv[i], "-yc") == 0 || strcmp(argv[i], "--yawcontroller") == 0) {
            rp_controller = true;
            y_controller = true;
        }
        if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tune") == 0) {
            if ((i + 2) < argc) {
                tune_params = true;
                if (strcmp(argv[i+1], "rp") == 0) {
                    tune_flags[0] = true;
                    tmp_roll[0] = atof(argv[i+2]);
                    printf("\tChanging Roll Proportional Gain (rp) value to: %1.2f\n", (double)tmp_roll[0]);
                }
                if (strcmp(argv[i+1], "ri") == 0) {
                    tune_flags[1] = true;
                    tmp_roll[1] = atof(argv[i+2]);
                    printf("\tChanging Roll Integral Gain (ri) value to: %s\n", argv[i+2]);
                }
                if (strcmp(argv[i+1], "rd") == 0) {
                    tune_flags[2] = true;
                    tmp_roll[2] = atof(argv[i+2]);
                    printf("\tChanging Roll Derivative Gain (rd) value to: %2.3f\n", (double)tmp_roll[2]);
                }
                
                if (strcmp(argv[i+1], "pp") == 0) {
                    tune_flags[3] = true;
                    tmp_pitch[0] = atof(argv[i+2]);
                    printf("\tChanging Pitch Proportional Gain (pp) value to: %s\n", argv[i+2]);
                }
                if (strcmp(argv[i+1], "pi") == 0) {
                    tune_flags[4] = true;
                    tmp_pitch[1] = atof(argv[i+2]);
                    printf("\tChanging Pitch Integral Gain (pi) value to: %s\n", argv[i+2]);
                }
                if (strcmp(argv[i+1], "pd") == 0) {
                    tune_flags[5] = true;
                    tmp_pitch[2] = atof(argv[i+2]);
                    printf("\tChanging Pitch Derivative Gain (pd) value to: %2.3f\n", (double)tmp_pitch[2]);
                }
                
                if (strcmp(argv[i+1], "yp") == 0) {
                    tune_flags[6] = true;
                    tmp_yaw[0] = atof(argv[i+2]);
                    printf("\tChanging Yaw Proportional Gain (yp) value to: %s\n", argv[i+2]);
                }
                if (strcmp(argv[i+1], "yi") == 0) {
                    tune_flags[7] = true;
                    tmp_yaw[1] = atof(argv[i+2]);
                    printf("\tChanging Yaw Integral Gain (yi) value to: %s\n", argv[i+2]);
                }
                if (strcmp(argv[i+1], "yd") == 0) {
                    tune_flags[8] = true;
                    tmp_yaw[2] = atof(argv[i+2]);
                    printf("\tChanging Yaw Derivative Gain (yd) value to: %s\n", argv[i+2]);
                }
            }
            else {
                // TODO: Edit
                console_print("Usage: yona_coaxial_heli start -t <parameter> <value>\n\n\tparameters:\n\t\trp\tRoll Proportional Gain\n\t\tpp\tPitch Proportional Gain\n\n\tvalues:\n\t\trp\tmin:0, max:12.00\n\t\tpp\tmin:0, max:12.00\n\n");
            }
        }
	}

    /* ------ Initializing the Input parameters ------ */
    
    init_parameters(&ph);
    if (tune_params) {
        if(tune_flags[0])
            param_set(ph.roll_p, (const void*)&tmp_roll[0]);
        if(tune_flags[1])
            param_set(ph.roll_i, (const void*)&tmp_roll[1]);
        if(tune_flags[2])
            param_set(ph.roll_d, (const void*)&tmp_roll[2]);
        
        if(tune_flags[3])
            param_set(ph.pitch_p, (const void*)&tmp_pitch[0]);
        if(tune_flags[4])
            param_set(ph.pitch_i, (const void*)&tmp_pitch[1]);
        if(tune_flags[5])
            param_set(ph.pitch_d, (const void*)&tmp_pitch[2]);
        
        if(tune_flags[6])
            param_set(ph.yaw_p, (const void*)&tmp_yaw[0]);
        if(tune_flags[7])
            param_set(ph.yaw_i, (const void*)&tmp_yaw[1]);
        if(tune_flags[8])
            param_set(ph.yaw_d, (const void*)&tmp_yaw[2]);
    }
    for (int i = 0; i < 12; i++) {
        // printf(tune_flags[i] ? "true\n" : "false\n");
        tune_flags[i] = false;
    }

    update_parameters(&ph, &pp);
    printf("GAINS:\n\troll p: %2.3f\n\troll i: %2.3f\n\troll d: %2.3f\n\tpitch p: %2.3f\n\tpitch i: %2.3f\n\tpitch d: %2.3f\n\tyaw p: %2.3f\n\tyaw i: %2.3f\n\tyaw d: %2.3f\n", (double)pp.roll_p, (double)pp.roll_i, (double)pp.roll_d, (double)pp.pitch_p, (double)pp.pitch_i, (double)pp.pitch_d, (double)pp.yaw_p, (double)pp.yaw_i, (double)pp.yaw_d);


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
    

    // /* ------ Arming ------ */
    // // TODO: Change to a button trigger
	// struct actuator_armed_s arm;
	// memset(&arm, 0, sizeof(arm));

	// arm.timestamp = hrt_absolute_time();
	// arm.ready_to_arm = true;
	// arm.armed = true;
	// orb_advert_t arm_pub_ptr = orb_advertise(ORB_ID(actuator_armed), &arm);
	// orb_publish(ORB_ID(actuator_armed), arm_pub_ptr, &arm);

	// /* read back values to validate */
	// int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	// orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

	// if (arm.ready_to_arm && arm.armed) {
	// 	warnx("Actuator armed");

	// } else {
	// 	errx(1, "Arming actuators failed");
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
                
                bool gyro_updated;
                orb_check(gyro_correction_sub, &gyro_updated);
                if (gyro_updated)
                    orb_copy(ORB_ID(sensor_correction), gyro_correction_sub, &gyro_correction);
                /* update the latest gyro selection */
                if (gyro_correction.selected_gyro_instance < gyro_count)
                    gyro_selected = gyro_correction.selected_gyro_instance;
                
                bool gyro_bias_updated;
                orb_check(gyro_bias_sub, &gyro_bias_updated);
                if (gyro_bias_updated)
                    orb_copy(ORB_ID(sensor_bias), gyro_bias_sub, &gyro_bias);
                // Creating a local copy
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
                if (att_sp_updated)
                    orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &att_sp);

                // Checking RC inputs
                orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels);
                // printf("Input RC - %f, %f, %f, %f\t\t", (double)rc_channels.channels[1]*1000, (double)rc_channels.channels[2]*1000, (double)rc_channels.channels[3]*1000, (double)rc_channels.channels[0]*1000);
                
                control_right_stick(&att, &att_sp, &actuators, rc_channels.channels, rp_controller);
                control_thrust(&att, &att_sp, &actuators, rc_channels.channels, y_controller);

                // TODO: Throttle limit check.?

                // Reading info on current vehicle status and flight mode
                orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                // Publishing rates
                orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

                // Sanity check and publishing actuator outputs
                if (PX4_ISFINITE(actuators.control[0]) && PX4_ISFINITE(actuators.control[1]) && PX4_ISFINITE(actuators.control[2]) && PX4_ISFINITE(actuators.control[3])) {
                    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                    // printf("Actuator outputs - %d, %d, %d, %d\n\n", (int)actuators.control[0]*100, (int)actuators.control[1]*100, (int)actuators.control[2]*100, (int)actuators.control[3]*100);

                    if (verbose) {
                        warnx("Actuator controls - Published");
                    }
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


// Check pre-flight checklist
// Check disarm time delay (mRo arms at non-min throttle after disarming < 4 sec)
// Change the controller for roll and pitch
// Add controller for yaw
// Add function in init.d file on mRo - Test
// Add position hold mode (built in function <uses GPS.?> __OR__ radio switch toggle.?)