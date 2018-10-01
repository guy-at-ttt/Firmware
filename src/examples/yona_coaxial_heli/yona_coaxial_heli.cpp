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

// Publisher and Subscriber includes
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/rc_channels.h>

#include <drivers/drv_rc_input.h>

int init_parameters(struct param_handles *handle);
int update_parameters(const struct param_handles *handle, struct params *parameters);
void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, struct actuator_outputs_s *rotor_outputs, float rc_channel_values[]);
void control_thrust(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, struct actuator_outputs_s *rotor_outputs, float rc_channel_values[]);
static void console_print(const char *reason);
int yona_coaxial_heli_main_thread(int argc, char *argv[]);

extern "C" __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);
// __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);


static bool thread_should_exit = false;
static bool thread_running = false;
static int deamon_task;
static struct params pp;
static struct param_handles ph;



int init_parameters(struct param_handles *handle) {
    handle->roll_p = param_find("YONA_ROLL_P");
    handle->pitch_p = param_find("YONA_PITCH_P");
    handle->yaw_p = param_find("YONA_YAW_P");
    handle->thrust_p = param_find("YONA_THRUST_P");
    return 0;
}

int update_parameters(const struct param_handles *handle, struct params *parameters) {
    // Copy values from parameters to the handles
    param_get(handle->roll_p, &(parameters->roll_p));
    param_get(handle->pitch_p, &(parameters->pitch_p));
    param_get(handle->yaw_p, &(parameters->yaw_p));
    param_get(handle->thrust_p, &(parameters->thrust_p));
    return 0;
}

void control_right_stick(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, struct actuator_outputs_s *rotor_outputs, float rc_channel_values[]) {
    // Control Roll and Pitch
    // Update PI gains

    // Setting ROLL and PITCH to 0
    // actuators->control[0] = rc_channel_values[1]    // ROLL
    // actuators->control[0] = rc_channel_values[2]    // PITCH

    // actuators->control[0] = 0.0f;
    // actuators->control[1] = 0.0f;

    // Calculating error and applying P-Gain
    float roll_err = matrix::Eulerf(matrix::Quatf(att->q)).phi() - matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi();
    actuators->control[0] = roll_err * pp.roll_p;
    rotor_outputs->output[0] = roll_err * pp.roll_p;
    
    float pitch_err = matrix::Eulerf(matrix::Quatf(att->q)).theta() - matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta();
    actuators->control[1] = pitch_err * pp.pitch_p;
    rotor_outputs->output[1] = pitch_err * pp.pitch_p;
}

void control_thrust(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators, struct actuator_outputs_s *rotor_outputs, float rc_channel_values[]) {
    // Control thrust
    // TODO: Add a controller to update PI gains on yaw and thrust

    actuators->control[2] = rc_channel_values[3];   // YAW
    actuators->control[3] = rc_channel_values[0];   // THRUST

    rotor_outputs->output[2] = rc_channel_values[3];   // YAW
    rotor_outputs->output[3] = rc_channel_values[0];   // THRUST
    printf("control_thrust - %f, %f, %f, %f\n", (double)actuators->control[0]*100, (double)actuators->control[1]*100, (double)actuators->control[2]*100, (double)actuators->control[3]*100);
}

static void console_print(const char *reason) {
    if (reason)
        fprintf(stderr, "%s\n", reason);
    
    fprintf(stderr, "err: yona_coaxial_heli {start|stop|status}\n\n");
}

int yona_coaxial_heli_main_thread(int argc, char *argv[]) {
    // Main thread
    /* ------ Reading arguments ------ */
	bool verbose = false;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

    /* ------ Initializing the Input parameters ------ */
    init_parameters(&ph);
    update_parameters(&ph, &pp);

    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_attitude_setpoint_s att_sp;
    memset(&att_sp, 0, sizeof(att_sp));
    struct manual_control_setpoint_s manual_sp;
    memset(&manual_sp, 0, sizeof(manual_sp));
    struct vehicle_status_s v_status;
    memset(&v_status, 0, sizeof(v_status));
    struct vehicle_rates_setpoint_s rates_sp;
    memset(&rates_sp, 0, sizeof(rates_sp));
    struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct position_setpoint_s global_sp;
	memset(&global_sp, 0, sizeof(global_sp));
    struct rc_channels_s rc_channels;
    memset(&rc_channels, 0, sizeof(rc_channels));
    

    /* ------ Arming ------ */
    // TODO: Change to a button trigger
	struct actuator_armed_s arm;
	memset(&arm, 0, sizeof(arm));

	arm.timestamp = hrt_absolute_time();
	arm.ready_to_arm = true;
	arm.armed = true;
	orb_advert_t arm_pub_ptr = orb_advertise(ORB_ID(actuator_armed), &arm);
	orb_publish(ORB_ID(actuator_armed), arm_pub_ptr, &arm);

	/* read back values to validate */
	int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);

	if (arm.ready_to_arm && arm.armed) {
		warnx("Actuator armed");

	} else {
		errx(1, "Arming actuators failed");
	}


    /* ------ Initializing the Output parameters ------ */
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));
    struct actuator_outputs_s rotor_outputs;
    memset(&rotor_outputs, 0, sizeof(rotor_outputs));

    // Publish 0 values to actuators
    for (unsigned i=0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
        actuators.control[i] = 0.0f;
    }
    for (unsigned i=0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
        rotor_outputs.output[i] = 0.0f;
    }

    struct vehicle_attitude_setpoint_s _att_sp = {};

    /* ------ Advertise these controllers (actuator_pub and rates_pub) as publishers of
              these topics (actuator_controls and rates_sp)                     ------ */
    // orb_advertise(topic) returns handles for the topics.
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
    orb_advert_t rotor_pub = orb_advertise(ORB_ID(actuator_outputs), &rotor_outputs);
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

                // Reading the updated values
                update_parameters(&ph, &pp);
            }

            // Run controller
            if (fds[1].revents & POLLIN) {
                // Checking for new position setpoint
                bool manual_sp_updated;
                orb_check(manual_sp_sub, &manual_sp_updated);
                bool pos_updated;
                orb_check(global_pos_sub, &pos_updated);
                bool global_sp_updated;
                orb_check(global_sp_sub, &global_sp_updated);
                bool att_updated;
                orb_check(att_sp_sub, &att_updated);

                // Creating a local copy
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
                if (att_updated) {
                    orb_copy(ORB_ID(vehicle_attitude_setpoint), att_sp_sub, &_att_sp);
                }

                // Checking RC inputs
                orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels);
                printf("Input RC - %f, %f, %f, %f\t\t", (double)rc_channels.channels[1]*1000, (double)rc_channels.channels[2]*1000, (double)rc_channels.channels[3]*1000, (double)rc_channels.channels[0]*1000);
                
                
                control_right_stick(&att, &_att_sp, &actuators, &rotor_outputs, rc_channels.channels);
                control_thrust(&att, &_att_sp, &actuators, &rotor_outputs, rc_channels.channels);

                // if (true) {
                if (manual_sp_updated) {
                    orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
                    // printf("manual_sp - %d, %d, %d, %d\n", (int)manual_sp.x*1000, (int)manual_sp.y*1000, (int)manual_sp.z*1000, (int)manual_sp.r*1000);
                }

                // TODO: Throttle limit check.?

                // Reading info on current vehicle status and flight mode
                orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                // Publishing rates
                orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

                // // Sanity check and publishing actuator outputs
                // if (PX4_ISFINITE(actuators.control[0]) && PX4_ISFINITE(actuators.control[1]) && PX4_ISFINITE(actuators.control[2]) && PX4_ISFINITE(actuators.control[3])) {
                //     orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                //     // printf("Actuator outputs - %d, %d, %d, %d\n\n", (int)actuators.control[0]*100, (int)actuators.control[1]*100, (int)actuators.control[2]*100, (int)actuators.control[3]*100);

                //     if (verbose) {
                //         warnx("Actuator controls - Published");
                //     }
                // }

                // Sanity check and publishing actuator outputs
                if (PX4_ISFINITE(rotor_outputs.output[0]) && PX4_ISFINITE(rotor_outputs.output[1]) && PX4_ISFINITE(rotor_outputs.output[2]) && PX4_ISFINITE(rotor_outputs.output[3])) {
                    orb_publish(ORB_ID(actuator_outputs), rotor_pub, &rotor_outputs);
                    // printf("rotor_outputs - %d, %d, %d, %d\n\n", (int)rotor_outputs.control[0]*100, (int)rotor_outputs.control[1]*100, (int)rotor_outputs.control[2]*100, (int)rotor_outputs.control[3]*100);

                    if (verbose) {
                        warnx("Actuator outputs - Published");
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
    
    for(unsigned i=0; i<actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
        rotor_outputs.output[i] = 0.0f;
    }
    orb_publish(ORB_ID(actuator_outputs), rotor_pub, &actuators);
    
    fflush(stdout);
    return 0;
}

int yona_coaxial_heli_main(int argc, char *argv[]) {
    PX4_INFO("__ YONA __");

    // Checking for arguments in the command {start|status|stop}        // TODO: Keep this.?
    if (argc < 2) {
        console_print("missing command");
        return 1;
    }

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
                                        (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
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



// Thread
    // Control roll & pitch -> automatically - read inputs and try to correct angles - controller
    // Control thrust & yaw from radio - callibrate radio - assign channels - map channels to mRo - try using QGC
// Run the thread in main()

// Add function in init.d file on mRo