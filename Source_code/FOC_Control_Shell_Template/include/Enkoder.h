#include "stdint.h" // må inkluderes for å kunne bruke int32_t
#include <math.h>    // For sin/cos-funksjoner

#define ENCODER_PPR 2048  
#define TWO_PI 6.28318530718  // 2 * pi for full rotasjon

// *** Globale variabler ***
extern volatile int32_t enkoder_teller;  // Enkoderens puls-teller

// *** Funksjonsprototyper ***
void Enkoder_Interrupt_Handler(int retning);
float beregn_vinkel_radianer(void);

// Beregner turtall (RPM) basert på pulser og tid
void beregn_rpm(uint32_t system_tid);

// Henter siste beregnede turtall (RPM)
float hent_rpm(void);
