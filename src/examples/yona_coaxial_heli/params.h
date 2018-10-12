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
    float time_diff;
    float roll_bias;
    float pitch_bias;
    float yaw_bias;
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
    param_t time_diff;
    param_t roll_bias;
    param_t pitch_bias;
    param_t yaw_bias;
};