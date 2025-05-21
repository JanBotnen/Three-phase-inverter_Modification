#include "Park.h"

void ClarkeTransform(float ia, float ib, float *Ialpha, float *Ibeta){
  *Ialpha = ia; // setter verdien til 
  *Ibeta = (1.0/sqrt(3.0))*(ia+2*ib);

}
// prototype, initialserer funksjonen
void ClarkeTransform(float ia, float ib, float *Ialpha, float *Ibeta);
