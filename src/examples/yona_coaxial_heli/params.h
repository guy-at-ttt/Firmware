#include <parameters/param.h>

struct params {
    float roll_p;
    float roll_i;
    float roll_d;
    float pitch_p;
    float pitch_i;
    float pitch_d;
    float yaw_p;
    float yaw_i;
    float yaw_d;
    float thrust_p;
    float thrust_i;
    float thrust_d;
    float alpha;
    float beta;
    float time_diff;
    float roll_bias;
    float pitch_bias;
    float yaw_bias;
    float thrust_bias;
    float yaw_i_min;
    float yaw_i_max;
    float thr_i_min;
    float thr_i_max;
    int invert_roll;
    int invert_pitch;
    int invert_yaw;
};

struct param_handles {
    param_t roll_p;
    param_t roll_i;
    param_t roll_d;
    param_t pitch_p;
    param_t pitch_i;
    param_t pitch_d;
    param_t yaw_p;
    param_t yaw_i;
    param_t yaw_d;
    param_t thrust_p;
    param_t thrust_i;
    param_t thrust_d;
    param_t alpha;
    param_t beta;
    param_t time_diff;
    param_t roll_bias;
    param_t pitch_bias;
    param_t yaw_bias;
    param_t thrust_bias;
    param_t yaw_i_min;
    param_t yaw_i_max;
    param_t thr_i_min;
    param_t thr_i_max;
    param_t invert_roll;
    param_t invert_pitch;
    param_t invert_yaw;
};