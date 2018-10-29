#include "yona_controller.hpp"

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
        // roll_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi() - matrix::Eulerf(matrix::Quatf(att->q)).phi();
        // pitch_err_acc = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta() - matrix::Eulerf(matrix::Quatf(att->q)).theta();
        roll_err_acc = (matrix::Eulerf(matrix::Quatf(att_sp->q_d)).phi() + (pp.roll_bias * rc_channel_values[1])) - matrix::Eulerf(matrix::Quatf(att->q)).phi();
        pitch_err_acc = (matrix::Eulerf(matrix::Quatf(att_sp->q_d)).theta() + (-1 * pp.pitch_bias * rc_channel_values[2])) - matrix::Eulerf(matrix::Quatf(att->q)).theta();

        roll_err_gyro = att_sp->roll_body - att->rollspeed;
        pitch_err_gyro = att_sp->pitch_body - att->pitchspeed;
        // if (verbose)
        //     printf("Pitch_gyro_err: %3.4f\t\t", (double)pitch_err_gyro);

        rp_curr_time = hrt_absolute_time();
        dt = (rp_curr_time - rp_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;

        p_err = (pp.alpha * roll_err_gyro * dt) + ((1.0f - pp.alpha) * roll_err_acc);
        d_err = (p_err - last_roll_err) / dt;
        actuators->control[0] = (p_err * pp.roll_p) + (d_err * pp.roll_d);// + (pp.roll_bias * rc_channel_values[1]);              // ROLL
        last_roll_err = p_err;
        
        p_err = (pp.alpha * pitch_err_gyro * dt) + ((1.0f - pp.alpha) * pitch_err_acc);
        d_err = (p_err - last_pitch_err) / dt;
        last_pitch_err = p_err;
        actuators->control[1] = (-1 * p_err * pp.pitch_p) + (-1 * d_err * pp.pitch_d);// + (pp.pitch_bias * rc_channel_values[2]);              // PITCH

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
}

void control_yaw(const struct vehicle_attitude_s *att, const struct vehicle_attitude_setpoint_s *att_sp, const struct vehicle_magnetometer_s *mag, struct actuator_controls_s *actuators, float rc_channel_values[]) {
    if (first_iteration_flag) {
        printf("Resetting YAW Setpoint\n");
        yaw_euler_sp = matrix::Eulerf(matrix::Quatf(att_sp->q_d)).psi();
        // yaw_euler_sp = matrix::Eulerf(matrix::Quatf(att->q)).psi() + (pp.yaw_bias * rc_channel_values[3]);
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
        yaw_err_mag = 0 - atan2f(-1 * mag->magnetometer_ga[1], mag->magnetometer_ga[0]);

        y_curr_time = hrt_absolute_time();
        dt = (y_curr_time - y_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;

        // p_err = (pp.alpha * yaw_err_gyro * dt) + ((1 - pp.alpha) * yaw_err_acc);
        p_err = (pp.alpha * yaw_err_gyro * dt) + ((1 - pp.alpha) * (((1 - pp.beta) * yaw_err_acc) + (pp.beta * yaw_err_mag)));
        d_err = (p_err - last_yaw_err) / dt;
        tmp_i_err = yaw_i_err + (p_err * dt);
        yaw_i_err =  math::constrain(tmp_i_err, pp.yaw_i_min, pp.yaw_i_max);

        actuators->control[2] = (p_err * pp.yaw_p) + (d_err * pp.yaw_d) + (yaw_i_err * pp.yaw_i);// + (pp.yaw_bias * rc_channel_values[3]);            // YAW
        if (verbose)
            printf("YAW: %3.9f\t%3.9f\t%3.9f\n", (double)tmp_i_err, (double)yaw_i_err, (double)actuators->control[2]);
        
        y_prev_time = hrt_absolute_time();
        last_yaw_err = yaw_err_acc;
    }

    else if (yaw_controller_select == 3) {
        // PD - accelerometer only
        yaw_euler_sp = pp.yaw_bias * rc_channel_values[3];
        yaw_err_acc = yaw_euler_sp - matrix::Eulerf(matrix::Quatf(att->q)).psi();

        y_curr_time = hrt_absolute_time();
        dt = (y_curr_time - y_prev_time)/1000000;         // dt in seconds        // TODO: Check
        if (dt < 0.002f)
            dt = 0.002f;
        if (dt > 0.02f)
            dt = 0.02f;
        
        p_err = yaw_err_acc;
        d_err = (yaw_err_acc - last_yaw_err) / dt;
        actuators->control[2] = (p_err * pp.yaw_p) + (d_err * pp.yaw_d);// + (pp.yaw_bias * rc_channel_values[3]);            // YAW
        
        y_prev_time = hrt_absolute_time();
        last_yaw_err = yaw_err_acc;
    }

    else {
        warnx("Invalid Yaw Controller");
    }
}

void control_thrust(const struct vehicle_air_data_s *air_data, struct actuator_controls_s *actuators, float rc_channel_values[]) {
    // THRUST
    // printf("%d\n", (int)rc_channel_values[7]);
    if (rc_input_val_reset)
       rc_input_val = rc_channel_values[0];
    baro_smooth_val = smoothen_baro(air_data);

    if (tmp_thrust_counter <= THRUST_MOVING_AVG_SPAN) {
        actuators->control[3] = rc_channel_values[0];
        rc_input_val_reset = true;
        tmp_thrust_counter++;
    }
    else {
        if ((int)rc_channel_values[7] < 0) {
            rc_input_val_reset = true;
            actuators->control[3] = rc_channel_values[0];       // Manual Control
            rc_input_val = rc_channel_values[0];
        }
        else if ((int)rc_channel_values[7] == 0) {
            // Hold the throttle percentage from RC input
            thrust_sp_baro = baro_smooth_val;
            rc_input_val_reset = false;
            actuators->control[3] = rc_input_val;
        }
        else {
            // Altitude hold
            rc_input_val_reset = false;
            thrust_err_baro = (thrust_sp_baro + (pp.thrust_bias * (rc_channel_values[0] - 0.5f))) - baro_smooth_val;

            th_curr_time = hrt_absolute_time();
            dt = (th_curr_time - th_prev_time)/10e6;
            if (dt < 0.002f)
                dt = 0.002f;
            if (dt > 0.02f)
                dt = 0.02f;

            p_err = thrust_err_baro;
            d_err = (thrust_err_baro - last_thrust_err) / dt;
            tmp_i_err = thrust_i_err + (p_err * dt);
            thrust_i_err = math::constrain(tmp_i_err, pp.thr_i_min, pp.thr_i_max);
            
            actuators->control[3] = rc_input_val + (p_err * pp.thrust_p) + (d_err * pp.thrust_d) + (thrust_i_err * pp.thrust_i);// + (pp.thrust_bias * (rc_channel_values[0] - 0.5f));            // THRUST
            if (verbose)
                printf("\t\t\t\t\t\t\tTHR: %3.9f\t%3.9f\t%3.9f\n", (double)tmp_i_err, (double)thrust_i_err, (double)actuators->control[3]);

            th_prev_time = hrt_absolute_time();
            last_thrust_err = thrust_err_baro;
        }
    }
    actuators->timestamp = hrt_absolute_time();
}