#include "stdint.h" // må inkluderes for å kunne bruke int32_t
#include "Enkoder.h"

// *** DEFINERE GLOBAL TELLER ***
volatile int32_t enkoder_teller = 0;  // 32-bit teller
volatile int32_t siste_enkoder_teller = 0;  // For turtallsmåling
volatile float motor_rpm = 0;  // Beregnet turtall (RPM)
volatile uint32_t siste_tid = 0;  // Sist oppdateringstidspunkt for RPM


// *** ENKODER INTERRUPT HANDLER ***
// Oppdaterer enkoderens teller (1 = CW, -1 = CCW)
void Enkoder_Interrupt_Handler(int retning) {
    if (retning == 1) {  
        enkoder_teller++;  // Teller opp
    } else {  
        enkoder_teller--;  // Teller ned
    }
}

// *** BEREGNER VINKEL I RADIANER ***
// Gir vinkel mellom 0 og 2π basert på enkoderteller
float beregn_vinkel_radianer(void) {
    int32_t teller_mod = enkoder_teller % ENCODER_PPR;  // Holder seg innen én rotasjon
    if (teller_mod < 0) {
        teller_mod += ENCODER_PPR;  // Sikrer positiv vinkel
    }
    return (teller_mod / (float)ENCODER_PPR) * TWO_PI;  // Konverterer til radianer
}

// *** BEREGNER TURTALL I RPM ***
// Basert på antall pulser i en gitt tidsperiode
void beregn_rpm(uint32_t system_tid) {
    uint32_t tidsdifferanse = system_tid - siste_tid;  // Beregner tid siden siste oppdatering

    if (tidsdifferanse >= 100) {  // Oppdaterer RPM hver 100 ms
        int32_t puls_endring = enkoder_teller - siste_enkoder_teller;  // Antall nye pulser

        // Beregner RPM: (puls_endring / PPR) gir antall rotasjoner, ganger 600 for minutt
        motor_rpm = ((float)puls_endring / ENCODER_PPR) * (60000.0 / tidsdifferanse);

        // Oppdaterer siste verdier
        siste_enkoder_teller = enkoder_teller;
        siste_tid = system_tid;
    }
}

// *** HENTER TURTALL I RPM ***
float hent_rpm(void) {
    return motor_rpm;
}