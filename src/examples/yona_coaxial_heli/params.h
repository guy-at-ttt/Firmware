#include <parameters/param.h>

struct params {
    float roll_p;
    float pitch_p;
    float yaw_p;
    float thrust_p;
};

struct param_handles {
    param_t roll_p;
    param_t pitch_p;
    param_t yaw_p;
    param_t thrust_p;
};