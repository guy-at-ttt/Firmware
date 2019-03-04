#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <poll.h>
#include <time.h>

#include "params.h"

static void console_print(const char *reason);
void parse_arguments(int argc, char *argv[]);
int init_parameters(struct param_handles *handle);
int update_parameters(const struct param_handles *handle, struct params *parameters);

bool verbose = false;
int rp_controller_select = 0, yaw_controller_select = 0;

static struct params pp;
static struct param_handles ph;

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

    handle->yaw_i_min = param_find("YONA_YAW_I_MIN");
    handle->yaw_i_max = param_find("YONA_YAW_I_MAX");
    handle->thr_i_min = param_find("YONA_THR_I_MIN");
    handle->thr_i_max = param_find("YONA_THR_I_MAX");

    handle->invert_roll = param_find("YONA_INV_ROLL");
    handle->invert_pitch = param_find("YONA_INV_PITCH");
    handle->invert_yaw = param_find("YONA_INV_YAW");
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

    param_get(handle->yaw_i_min, &(parameters->yaw_i_min));
    param_get(handle->yaw_i_max, &(parameters->yaw_i_max));
    param_get(handle->thr_i_min, &(parameters->yaw_i_min));
    param_get(handle->thr_i_max, &(parameters->yaw_i_max));

    param_get(handle->invert_roll, &(parameters->invert_roll));
    param_get(handle->invert_pitch, &(parameters->invert_pitch));
    param_get(handle->invert_yaw, &(parameters->invert_yaw));
    return 0;
}

static void console_print(const char *reason) {
    if (reason)
        fprintf(stderr, "%s\n", reason);
    
    fprintf(stderr, "Usage: yona_coaxial_heli {start|stop|status}\n\n");
}

void parse_arguments(int argc, char *argv[]) {
    /* ------ Reading arguments ------ */
	verbose = false;
    float alpha = 0.02f, beta = 0.0f, time_diff = 0.1f;
    bool tune_alpha = false, tune_beta = false;
    bool tune_timediff = false;
    bool tune_params = false;
    // bool tune_flags[6] = {false, false, false, false, false, false};
    bool tune_flags[23] = {};
    for (int i = 0; i < 23; i++)
        tune_flags[i] = false;
    float tmp_roll[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_pitch[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_yaw[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_thrust[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float tmp_i[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int tmp_invert[3] = {1, 1, 1};
    
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
                else if (strcmp(argv[i+1], "rb")) {
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
                
                else if (strcmp(argv[i+1], "yimin") == 0) {
                    tune_flags[16] = true;
                    tmp_i[0] = atof(argv[i+2]);
                    printf("\tChanging Yaw Integral Min value to: %2.3f\n", (double)tmp_i[0]);
                }
                else if (strcmp(argv[i+1], "yimax") == 0) {
                    tune_flags[17] = true;
                    tmp_i[1] = atof(argv[i+2]);
                    printf("\tChanging Yaw Integral Max value to: %2.3f\n", (double)tmp_i[1]);
                }
                else if (strcmp(argv[i+1], "timin") == 0) {
                    tune_flags[18] = true;
                    tmp_i[2] = atof(argv[i+2]);
                    printf("\tChanging Thrust Integral Min value to: %2.3f\n", (double)tmp_i[2]);
                }
                else if (strcmp(argv[i+1], "timax") == 0) {
                    tune_flags[19] = true;
                    tmp_i[3] = atof(argv[i+2]);
                    printf("\tChanging Thrust Integral Max value to: %2.3f\n", (double)tmp_i[3]);
                }
                
                else if (strcmp(argv[i+1], "invr") == 0) {
                    tune_flags[20] = true;
                    tmp_invert[0] = atoi(argv[i+2]);
                    if (tmp_invert[0] < 0)
                        tmp_invert[0] = -1;
                    else
                        tmp_invert[0] = 1;
                    printf("\tInverting Roll value: %d\n", tmp_invert[0]);
                }
                else if (strcmp(argv[i+1], "invp") == 0) {
                    tune_flags[21] = true;
                    tmp_invert[1] = atoi(argv[i+2]);
                    if (tmp_invert[1] < 0)
                        tmp_invert[1] = -1;
                    else
                        tmp_invert[1] = 1;
                    printf("\tInverting Pitch value: %d\n", tmp_invert[1]);
                }
                else if (strcmp(argv[i+1], "invy") == 0) {
                    tune_flags[22] = true;
                    tmp_invert[2] = atoi(argv[i+2]);
                    if (tmp_invert[2] < 0)
                        tmp_invert[2] = -1;
                    else
                        tmp_invert[2] = 1;
                    printf("\tInverting Yaw value: %d\n", tmp_invert[2]);
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
        
        if (tune_flags[16])
            param_set(ph.yaw_i_min, (const void*)&tmp_i[0]);
        if (tune_flags[17])
            param_set(ph.yaw_i_max, (const void*)&tmp_i[1]);
        if (tune_flags[18])
            param_set(ph.thr_i_min, (const void*)&tmp_i[2]);
        if (tune_flags[19])
            param_set(ph.thr_i_max, (const void*)&tmp_i[3]);

        if (tune_flags[20])
            param_set(ph.invert_roll, (const void*)&tmp_invert[0]);
        if (tune_flags[21])
            param_set(ph.invert_pitch, (const void*)&tmp_invert[1]);
        if (tune_flags[22])
            param_set(ph.invert_yaw, (const void*)&tmp_invert[2]);
    }

    if (tune_alpha)
        param_set(ph.alpha, (const void*)&alpha);
    if (tune_beta)
        param_set(ph.beta, (const void*)&beta);
    if (tune_timediff)
        param_set(ph.time_diff, (const void*)&time_diff);
    
    for (int i = 0; i < 20; i++) {
        // printf(tune_flags[i] ? "true\n" : "false\n");
        tune_flags[i] = false;
    }

    update_parameters(&ph, &pp);
    printf("GAINS:\n\troll p: %2.3f\n\troll i: %2.3f\n\troll d: %2.3f\n\troll bias: %2.3f\n\n\tpitch p: %2.3f\n\tpitch i: %2.3f\n\tpitch d: %2.3f\n\tpitch bias: %2.3f\n\n\tyaw p: %2.3f\n\tyaw i: %2.3f\n\tyaw d: %2.3f\n\tyaw bias: %2.3f\n\n\tthrust p: %2.3f\n\tthrust i: %2.3f\n\tthrust d: %2.3f\n\tthrust bias: %2.3f\n\n\talpha (complementary filter GYRO): %2.3f\n\tbeta (complementary filter MAGNETOMETER): %2.3f\n\ttime_diff: %2.3f\n", (double)pp.roll_p, (double)pp.roll_i, (double)pp.roll_d, (double)pp.roll_bias, (double)pp.pitch_p, (double)pp.pitch_i, (double)pp.pitch_d, (double)pp.pitch_bias, (double)pp.yaw_p, (double)pp.yaw_i, (double)pp.yaw_d, (double)pp.yaw_bias, (double)pp.thrust_p, (double)pp.thrust_i, (double)pp.thrust_d, (double)pp.thrust_bias, (double)pp.alpha, (double)pp.beta, (double)pp.time_diff);
}

