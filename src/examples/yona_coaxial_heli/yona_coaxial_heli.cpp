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
#include <uORB/topics/parameter_update.h>

int parameters_init(struct param_handles *handle);
int parameters_update(const struct param_handles *handle, struct params *parameters);

extern "C" __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);
// __EXPORT int yona_coaxial_heli_main(int argc, char *argv[]);


static bool thread_should_exit = false;
static bool thread_running = false;
static int deamon_task;
static struct params pp;
static struct param_handles ph;



int parameters_init(struct param_handles *handle) {
    handle->roll_p = param_find("YONA_ROLL_P");
    handle->pitch_p = param_find("YONA_PITCH_P");
    handle->yaw_p = param_find("YONA_YAW_P");
    handle->thrust_p = param_find("YONA_THRUST_P");
    return 0;
}

int parameters_update(const struct param_handles *handle, struct params *parameters) {
    // Copy values from parameters to the handles
    param_get(handle->roll_p, &(parameters->roll_p));
    param_get(handle->pitch_p, &(parameters->pitch_p));
    param_get(handle->yaw_p, &(parameters->yaw_p));
    param_get(handle->thrust_p, &(parameters->thrust_p));
}

int control_right_stick(struct actuator_controls_s *actuators) {
    // Control Roll and Pitch
    // Update PI gains

    // Setting ROLL and PITCH to 0
    actuators->control[0] = 0.0f;
    actuators->control[1] = 0.0f;
}

int control_thrust(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, struct actuator_controls_s *actuators) {
    // Control thrust
    // TODO: Add a controller to update PI gains on yaw and thrust

    // control_right_stick(actuators);
    // Setting ROLL and PITCH to 0
    actuators->control[0] = 0.0f;               // ROLL
    actuators->control[1] = 0.0f;               // PITCH

    actuators->control[2] = att_sp->yaw;        // YAW
    actuators->control[3] = att_sp->thrust;     // THRUST
}

// int control_roll() {
//     // Combine in control_thrust().?
//     // Read channel 1
//     // Update PI gains
// }

static void console_print(const char *reason) {
    if (reason)
        fprintf(stderr, "%s\n", reason);
    
    fprintf(stderr, "console_print: rover-steering_control {start|stop|status}\n\n");
}

int yona_coaxial_heli_main_thread() {
    // Main thread
    /* ------ Initializing the Input parameters ------ */
    parameters_init(&ph);
    parameters_update(&ph, &pp);

    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    struct vehicle_attitude_setpoint_s att_sp;
    memset(&att_sp, 0, sizeof(att_sp));
    struct vehicle_rates_setpoint_s rates_sp;
    memset(&rates_sp, 0, sizeof(rates_sp));
    struct manual_control_setpoint_s manual_sp;
    memset(&manual_sp, 0, sezieof(manual_sp));
    struct vehicle_status_s v_status;
    memset(&v_status, 0, sizeof(v_status));


    /* ------ Initializing the Output parameters ------ */
    struct actuator_controls_s actuators;
    memset(&actuators, 0, sizeof(actuators));

    // Publish 0 values to actuators
    for (unsigned i=0; i < actuator_controls_s::NUM_ACTUATOR_CONTROLS; i++) {
        actuators.control[i] = 0.0f;
    }

    /* ------ Advertise these controllers (actuator_pub and rates_pub) as publishers of
              these topics (actuator_controls and rates_sp)                     ------ */
    orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
    orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

    /* ------ Subscribe to rest of the topics ------ */
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    int v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    int param_sub = orb_subscribe(ORB_ID(parameter_update));


    struct pollfd fds[2] = {};
    fds[0].fd = param_sub;          // Descriptor being polled
    fds[0].events = POLLIN;         // Data other than high-priority data may be read without blocking
    fds[1].fd = att_sub;
    fds[1].events = POLLIN;

    while (!thread_should_exit) {
        // Parameters : list of file descriptors (structures);
        //              number of list items (descriptors);
        //              timeout (in ms)
        // Returns : number of structures that have non-zero revents fields.
        //           0; if times out waiting for a "descriptor ready"
        int poll_ret_val = poll(fds, 2, 500);

        if (ret < 0) {
            warnx("Poll Error");
        }
        else if (ret == 0) {
            continue;
        }
        else {
            // Main Control Thread Logic
            // Failsafes, Sanity checks, Read RC inputs, Backup Values, Update att and att_sp, Publish actuator values

            // Check if parameters have changed
            if (fds[0].revents & POLLIN) {
                struct parameter_update_s value_updates;
                orb_copy(ORB_ID(parameter_update_s), param_sub, &value_updates);

                // Reading the updated values
                parameters_update(&ph, &pp);
            }
        }
    }
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