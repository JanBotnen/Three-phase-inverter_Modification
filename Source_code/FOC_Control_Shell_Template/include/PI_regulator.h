// Definerer PI-regulator-struktur
typedef struct {
    float Kp;         // Proposjonalforsterkning
    float Ki;         // Integralforsterkning
    float integral;   // Akkumulert integralverdi
    float max_output; // Maks utgangsverdi (sikrer at regulatoren ikke mettes)
    float min_output; // Min utgangsverdi
} PI_regulator;

// Deklarerer PI-regulator funksjon
float PI_reg(PI_regulator *pi, float setpoint, float actual, float dt); 