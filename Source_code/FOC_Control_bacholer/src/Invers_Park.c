#include "Invers_Park.h"

void InversPark( float theta, float id, float iq, float *ialpha, float *ibeta ){
    float CosTheta = cos(theta);
    float SinTheta = sine(theta);

    *ialpha = id * CosTheta - iq * SinTheta;
    *ibeta = iq * SinTheta + iq * CosTheta;

}