/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// #include <matrix/math.hpp>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <poll.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_air_data.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

// int _gyro_count = 1;
// int _sensor_gyro_sub[3];

hrt_abstime last_time, end_time;
float dt_this;

int px4_simple_app_main(int argc, char *argv[])
{
	// PX4_INFO("Hello Earth!");

	/* subscribe to sensor_combined topic */
	int sensor_acc_sub = orb_subscribe(ORB_ID(sensor_accel));
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int sensor_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	int v_mag_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	int air_data_sub = orb_subscribe(ORB_ID(vehicle_air_data));
	// for (int s = 0; s <= _gyro_count; s++)
	// 	_sensor_gyro_sub[s] = orb_subscribe_multi(ORB_ID(sensor_gyro), s);
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = att_sub,   .events = POLLIN },
		// { .fd = _sensor_gyro_sub[0], .events = POLLIN },
		// { .fd = _sensor_gyro_sub[1], .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;
	bool tmp = true;

	last_time = hrt_absolute_time();
	float prev_ax = 0.0f, prev_vx = 0.0f, prev_prev_vx = 0.0f, prev_prev_ax = 0.0f;
	double prev_x = 0.0;

	for (int i = 0; i < 100; i++) {
	// while (!tmp) {
	// 	if (!strcmp(argv[1], "stop")) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 500);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("px4_poll Timeout: > 1s");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			// if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_accel_s accel;
				struct sensor_combined_s raw;
				struct vehicle_attitude_s att;
				struct sensor_mag_s s_mag;
				struct vehicle_magnetometer_s v_mag;
				struct vehicle_local_position_s local_pos;
				struct vehicle_local_position_setpoint_s local_pos_sp;
				struct vehicle_air_data_s air_data;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_accel), sensor_acc_sub, &accel);
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);	
				orb_copy(ORB_ID(sensor_mag), sensor_mag_sub, &s_mag);
				orb_copy(ORB_ID(vehicle_magnetometer), v_mag_sub, &v_mag);
				orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
				orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);
				orb_copy(ORB_ID(vehicle_air_data), air_data_sub, &air_data);
				// struct sensor_gyro_s gyro_raw;
				// orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[0], &gyro_raw);
				// struct sensor_gyro_s gyro_rawone;
				// orb_copy(ORB_ID(sensor_gyro), _sensor_gyro_sub[1], &gyro_rawone);
				
				// roll (x-axis rotation)
				double sinr_cosp = +2.0 * (double)(att.q[0] * att.q[1] + att.q[2] * att.q[3]);
				double cosr_cosp = +1.0 - 2.0 * (double)(att.q[1] * att.q[1] + att.q[2] * att.q[2]);
				double roll = atan2(sinr_cosp, cosr_cosp), pitch;

				// pitch (y-axis rotation)
				double sinp = +2.0 * (double)(att.q[0] * att.q[2] - att.q[3] * att.q[1]);
				if (fabs(sinp) >= 1)
					pitch = copysign(3.14 / 2, sinp); // use 90 degrees if out of range
				else
					pitch = asin(sinp);

				// yaw (z-axis rotation)
				double siny_cosp = +2.0 * (double)(att.q[0] * att.q[3] + att.q[1] * att.q[2]);
				double cosy_cosp = +1.0 - 2.0 * (double)(att.q[2] * att.q[2] + att.q[3] * att.q[3]);  
				double yaw = atan2(siny_cosp, cosy_cosp);

				// Estimator for displacement
				end_time = hrt_absolute_time();
				dt_this = (float)(end_time - last_time) / 1000000.0f;
				if (dt_this < 0.002f)
					dt_this = 0.002f;
				if (dt_this > 0.02f)
					dt_this = 0.02f;
				float lin_acc = raw.accelerometer_m_s2[0] + 9.8f;
				
				prev_vx = ((prev_ax + prev_prev_ax) * dt_this) + prev_prev_vx;
				double dis_x = (double)(0.5f * (lin_acc + prev_ax) * dt_this * dt_this) + (double)(prev_vx * dt_this) + prev_x;
				prev_prev_vx = prev_vx;
				prev_prev_ax = prev_ax;
				prev_ax = lin_acc;
				prev_x = dis_x;
				sleep(0.5);
				last_time = hrt_absolute_time();

				if (!tmp) {
					printf("\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",//\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",
						roll,
						pitch,
						yaw,
						(double)att.rollspeed,
						(double)att.pitchspeed,
						(double)att.yawspeed,
						// (double)s_mag.x,
						// (double)s_mag.y,
						// (double)s_mag.z,
						// (double)v_mag.magnetometer_ga[0],
						// (double)v_mag.magnetometer_ga[1],
						// (double)v_mag.magnetometer_ga[2],
						(double)accel.x,
						(double)accel.y,
						(double)accel.z,
						(double)raw.accelerometer_m_s2[0],
						(double)raw.accelerometer_m_s2[1],
						(double)raw.accelerometer_m_s2[2]);
						//  (double)gyro_rawone.x,
						//  (double)gyro_rawone.y,
						//  (double)gyro_rawone.z,
						//  (double)gyro_rawone.x_raw,
						//  (double)gyro_rawone.y_raw,
						//  (double)gyro_rawone.z_raw);
				// }
				}
				else {
					printf("%8.19f\t%8.19f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.20f\t%8.20f\t%8.9f\n",
						(double)local_pos.x,
						(double)local_pos.y,
						(double)local_pos.z,
						(double)air_data.baro_alt_meter,
						(double)air_data.baro_temp_celcius,
						(double)air_data.baro_pressure_pa,
						(double)raw.accelerometer_m_s2[0],
						(double)raw.accelerometer_m_s2[1],
						(double)raw.accelerometer_m_s2[2],
						dis_x,
						(double)prev_vx,
						(double)air_data.rho);
				}
		}
		// }
		// else {
		// 	break;
		// }
	}

	// PX4_INFO("exiting");

	return 0;
}
