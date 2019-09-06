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
 *      CH 5: Mode Switch
 *      CH ?: Return Switch
 *      CH 7: Arm Switch
 *      CH 8: Thrust Controller Toggle
 * 
 * @author TreetownTech User <tttuser@example.com>
 */

#include "yona_controller.cpp"
#include <mathlib/math/Limits.hpp>

int yona_coaxial_heli_main_thread(int argc, char *argv[]) {
    // Main thread
    parse_arguments(argc, argv);

    struct vehicle_attitude_s           att;
    struct vehicle_attitude_setpoint_s  att_sp;
    struct manual_control_setpoint_s    manual_sp;
    struct vehicle_status_s             v_status;
    // struct vehicle_rates_setpoint_s     rates_sp;
    // struct vehicle_global_position_s    global_pos;
	// struct position_setpoint_s          global_sp;
    struct rc_channels_s                rc_channels;
	struct sensor_gyro_s			    gyro_sensor;           // gyro data before thermal correctons and ekf bias estimates are applied
	struct sensor_correction_s		    gyro_correction;    // sensor thermal corrections
	struct sensor_bias_s			    gyro_bias;          // sensor in-run bias corrections
    struct vehicle_magnetometer_s       mag;
    struct vehicle_air_data_s           air_data;
    struct vehicle_local_position_s     local_pos;

    
    memset(&att, 0, sizeof(att));
    memset(&att_sp, 0, sizeof(att_sp));
    memset(&manual_sp, 0, sizeof(manual_sp));
    memset(&v_status, 0, sizeof(v_status));
    // memset(&rates_sp, 0, sizeof(rates_sp));
	// memset(&global_pos, 0, sizeof(global_pos));
	// memset(&global_sp, 0, sizeof(global_sp));
    memset(&rc_channels, 0, sizeof(rc_channels));
    memset(&gyro_sensor, 0, sizeof(gyro_sensor));
    memset(&gyro_correction, 0, sizeof(gyro_correction));
    memset(&mag, 0, sizeof(mag));
    memset(&air_data, 0, sizeof(air_data));
    memset(&local_pos, 0, sizeof(local_pos));

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
    // orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

    /* ------ Subscribe to rest of the topics ------ */
    // orb_subscribe(topic) returns handles for the topics.
    int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
    int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    int v_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    int param_sub = orb_subscribe(ORB_ID(parameter_update));
    int rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    int arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
    int mag_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
    int air_data_sub = orb_subscribe(ORB_ID(vehicle_air_data));
    int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));

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
        orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);
        // first_iteration_flag = arm.armed ? true : false;
        if (arm.armed && yaw_sp_reset) {
            // warnx("Actuators Armed");
            printf("ARMED\n");

            first_iteration_flag = true;
            yaw_sp_reset = false;
        }
        if (!arm.armed) {
            // warnx("Actuators Not Armed");
            yaw_sp_reset = true;
        }
        
        int poll_ret_val = poll(fds, 2, 500);

        // Sanity checks
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
                bool att_sp_updated;
                orb_check(att_sp_sub, &att_sp_updated);
                
                // Creating a local copy
                orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
                orb_copy(ORB_ID(vehicle_magnetometer), mag_sub, &mag);
                orb_copy(ORB_ID(vehicle_air_data), air_data_sub, &air_data);
                orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
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
                control_thrust(&local_pos, &air_data, &actuators, rc_channels.channels);

                // printf("Actuators: %3.4f\t%3.4f\t%3.4f\t%3.4f\t\t\t", (double)actuators.control[0], (double)actuators.control[1], (double)actuators.control[2], (double)actuators.control[3]);

                // TODO: Throttle limit check.?
                actuators.control[0] = math::constrain(actuators.control[0], -1.0f, 1.0f) * pp.invert_roll;
                actuators.control[1] = math::constrain(actuators.control[1], -1.0f, 1.0f) * pp.invert_pitch;
                actuators.control[2] = math::constrain(actuators.control[2], -1.0f, 1.0f) * pp.invert_yaw;
                actuators.control[3] = math::constrain(actuators.control[3], 0.0f, 1.0f);

                // printf("Limited: %3.4f\t%3.4f\t%3.4f\t%3.4f\t\n", (double)actuators.control[0], (double)actuators.control[1], (double)actuators.control[2], (double)actuators.control[3]);

                // Reading info on current vehicle status and flight mode
                orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                // Publishing rates
                // orb_publish(ORB_ID(vehicle_rates_setpoint), rates_pub, &rates_sp);

                // Sanity check and publishing actuator outputs
                if (PX4_ISFINITE(actuators.control[0]) && PX4_ISFINITE(actuators.control[1]) && PX4_ISFINITE(actuators.control[2]) && PX4_ISFINITE(actuators.control[3])) {
                    orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
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