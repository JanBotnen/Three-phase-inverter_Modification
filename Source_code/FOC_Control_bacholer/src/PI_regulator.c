
#include "PI_regulator.h"  // ✅ Inkluderer header-filen

// Her trenger vi ikke en prototype, fordi den allerede finnes i headeren
float PI_reg(PI_regulator *pi, float setpoint, float actual, float dt) {
    float error = setpoint - actual;
    pi->integral += error * dt;

    float output = (pi->Kp * error) + (pi->Ki * pi->integral);

    if (output > pi->max_output) output = pi->max_output;
    if (output < pi->min_output) output = pi->min_output;

    return output;
}