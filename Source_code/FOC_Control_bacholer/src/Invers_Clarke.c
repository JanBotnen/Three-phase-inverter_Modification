#include "Invers_Clarke.h"

void InversClarke(float ialpha, float ibeta, float *ia, float *ib, float *ic){
    *ia = ialpha;
    *ib = -0.5 * ialpha + (sqrt(3)/2)* ibeta;
    *ic = -0.5 * ialpha - (sqrt(3/2)) * ibeta;
}