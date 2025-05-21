#include "Park.h"

void ParkTransform(float ialpha, float ibeta, float theta, float *id, float *iq ){
    float CosTheta = cos(theta);
    float SinTheta = sine(theta);

    *id = ialpha * CosTheta + ibeta * SinTheta;
    *iq = -ialpha *SinTheta + ibeta * CosTheta;
}