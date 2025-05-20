//**********************************************************************************************************************//
//** Dene filen inneholder programvare utviklet av Fredrik Larsgård, Kristian Farnes og Jan Magnus Botnene våren 2025©***//
//**********************************************************************************************************************//

//*******************************************************************************************************************************//
//**Denne koden er ikke komplett. Ramme strukturen satt opp for field oriented controll******************************************//
//**Grunnet begrenset tid er ingen funksjoner i denne koden test. Strømmåling og enkoder er verifisert med separat programvare.**// 
//*******************************************************************************************************************************//

#include "stm32l4xx_hal.h"  // Inkluderer HAL-biblioteket for STM32L4-serien
#include <stdio.h>
#include <string.h>
//Egne header filer
#include <math.h> // Trengs for å utføre Clarke og Park transform
#include "Clarke.h" // inkluderer Clarke transform funksjon
#include "Park.h" //inkluder Park transform funksjonen
#include "Invers_Clarke.h" // Inkluderer invers Clarke transform funksjone
#include "Invers_Park.h" // Inlkuderer invers Park transform funksjonen
#include "PI_regulator.h" 
#include "Enkoder.h"


//globale variabler
 volatile float vinkel_rad = 0; //definerer rotor vinkel
 volatile uint32_t system_tid = 0; // variabel som brukes til systemtid
 uint32_t siste_oppdatering = 0; //Opdaterer systemtid

// Definerer hvilke GPIO-pinner ADC-en er koblet til for fase U, V, W
#define PHASE_L1_PIN GPIO_PIN_0  // ADC kanal for fase L1
#define PHASE_L2_PIN GPIO_PIN_1  // ADC kanal for fase L2
#define PHASE_L3_PIN GPIO_PIN_2  // ADC kanal for fase L3

// Definerer elektriske egenskaper for strømsensoren
#define VREF 3.3          // Referansespenning for ADC (STM32 bruker 3.3V)
#define V_OFFSET 1.65     // Quiescent voltage (midtpunkt på sensoren)
#define SENSITIVITY 0.01  // 10mV/A (må justeres basert på sensormodell)

UART_HandleTypeDef huart2;  // Håndterer UART2 (tilkoblet USB-seriell)
ADC_HandleTypeDef hadc1;    // Håndterer ADC-modulen
TIM_HandleTypeDef htim2;    // Håndterer timer 2 (TIM2)


// Buffers for å sende data over UART
char uart_buffer[100];

 PI_regulator pi_id = {.Kp = 0.1, .Ki = 0.1, .integral = 0, .max_output = 10, .min_output = -10};
 PI_regulator pi_iq = {.Kp = 0.2, .Ki = 0.2, .integral = 0, .max_output = 15, .min_output = -15};


// Funksjonsprototyper for systemkonfigurasjon
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
float readCurrent(uint16_t);
void HAL_SYSTICK_Callback(void);


// *** INTERRUPT CALLBACK FOR TIMER 2 ***
// Denne funksjonen kalles automatisk hver gang TIM2 genererer et interrupt
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {  // Sjekker om det er vår timer (TIM2) som utløste interruptet
        //oppdaterer vinkel theta for rotor magnetfelt
        vinkel_rad = beregn_vinkel_radianer();
        // Leser strømmen fra sensorene
        float ia = readCurrent(PHASE_L1_PIN);
        float ib = readCurrent(PHASE_L2_PIN);
        float ic = readCurrent(PHASE_L3_PIN);
        float ialpha, ibeta, id, iq;

        //utfører Clarke transformen
        ClarkeTransform(ia, ib, &ialpha, &ibeta);

        //Setter inn en vilkårlig verdi for Theta, her skal vi bruke verdien fra enkoder når denne delen av koden blir kla
        float theta = vinkel_rad; //30 grader

        //utfører park transform
        ParkTransform(ialpha, ibeta, theta, &id, &iq );

        //Regulerer id og iq med PI regulatorene pi_id og pi_iq

        float setpunkt_id = 0; // Setter ønsket magnetiseringsstrøm til 0 A
        float setpunkt_iq = 3; // setter ønsket på magnetiseringsstrømme ia til 3 A, kan brukes til test i regulering av iq om vi ikke ønsker å bruke rpm måling

        // Beregner faktisk RPM og bruker det i PI-kontrolleren
        float rpm = hent_rpm();
        float ønsket_rpm = 1000.0;  // Eksempelverdi - dette kan settes fra et overordnet kontrollsystem
        float regulert_turtall = PI_reg(&pi_iq, ønsket_rpm, rpm, 0.001);  // Regulerer Iq basert på RPM

        //reguler id og iq med PI regulatoren

        float dt = (system_tid - siste_oppdatering) / 1000.0;  // ✅ Beregner sanntid i sekunder
        float regulet_id = PI_reg(&pi_id, setpunkt_id, id, dt); 
        float regulet_iq = PI_reg(&pi_iq, regulert_turtall, iq, dt); 

        //transfomrer regulerte verdier tilbake til alpha betta verdier
        float regulert_ialpha, regulert_ibeta;
        InversPark(regulet_id, regulet_iq, theta, &regulert_ialpha, &regulert_ibeta);

        //transformerer regulerte verdier tilbake til a,b,c verdier
        float regulert_ia, regulert_ib, regulert_ic;
        InversClarke(regulert_ialpha,regulert_ibeta, &regulert_ia, &regulert_ib, &regulert_ic );

        //Sender de nye strømmene ia, ib, ic videre til SVPWM funksjonen

        // SVPWM(regulert_ia, regulert_ib, regulert_ic); 

        // Formatterer strømmålingene til en streng
        snprintf(uart_buffer, sizeof(uart_buffer), 
      
         "ia: %.2f | ib: %.2f | ic: %.2f | ialpha: %.2f | ibeta: %.2f | id: %.2f -> %.2f | iq: %.2f -> %.2f\r\n", 
         ia, ib, ialpha, ibeta, id, iq);
        // Sender dataene over UART2
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    }
}

// *** HOVEDPROGRAMMET ***
int main(void) {
    HAL_Init();  // Initialiserer HAL-biblioteket (nødvendig for STM32)

    SystemClock_Config();  // Konfigurerer systemklokken

    MX_GPIO_Init();  // Initialiserer GPIO (ikke brukt i denne koden, men god praksis)
    MX_ADC1_Init();  // Initialiserer ADC1
    MX_TIM2_Init();  // Initialiserer Timer 2
    MX_USART2_UART_Init();  // Initialiserer UART2 for seriell kommunikasjon

    HAL_TIM_Base_Start_IT(&htim2);  // Starter TIM2 med interrupts aktivert

    

    while (1) {

     if ((system_tid - siste_oppdatering) >= 100){ // 10 ms oppdateringstid, skriver kun ut verdier, må vurdere tiden her
        siste_oppdatering = system_tid;


        printf("Enkoder teller: %d | Vinkel: %.5f rad\n", enkoder_teller, vinkel_rad);
     }
        
    }
}

// *** INITIALISERING AV UART (SERIELL KOMMUNIKASJON) ***
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;  // Bruker USART2 (tilkoblet USB på STM32 Nucleo)
    huart2.Init.BaudRate = 115200;  // Baudrate = 115200 bps
    huart2.Init.WordLength = UART_WORDLENGTH_8B;  // 8-bit data
    huart2.Init.StopBits = UART_STOPBITS_1;  // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;  // Ingen paritet
    huart2.Init.Mode = UART_MODE_TX_RX;  // Både sending og mottak (her bruker vi kun TX)
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // Ingen hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;  // Standard oversampling
    HAL_UART_Init(&huart2);  // Initialiserer UART-en
}

// *** FUNKSJON FOR Å LESE STRØMVERDIER FRA EN SENSOR ***
// Denne funksjonen konfigurerer ADC, leser spenningen fra sensoren og konverterer den til strøm.
float readCurrent(uint16_t pin) {
    ADC_ChannelConfTypeDef sConfig = {0};  // Oppretter en ADC konfigurasjonsstruktur
    sConfig.Channel = pin;  // Velger riktig ADC-kanal basert på parameteren 'pin'
    sConfig.Rank = ADC_REGULAR_RANK_1;  // Setter kanalrangering (vi bruker bare én kanal om gangen)
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;  // Velger rask prøvetid
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);  // Konfigurerer ADC-en

    HAL_ADC_Start(&hadc1);  // Starter ADC-konvertering
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  // Venter til konverteringen er ferdig
    uint32_t adcValue = HAL_ADC_GetValue(&hadc1);  // Leser ADC-verdien
    HAL_ADC_Stop(&hadc1);  // Stopper ADC-en for å spare strøm

    // Konverterer ADC-verdi til spenning
    float voltage = (adcValue * VREF) / 4095.0;
    
    // Konverterer spenning til strøm basert på sensorens karakteristikk (I = (V - V_OFFSET) / S)
    return (voltage - V_OFFSET) / SENSITIVITY;
}

//*** FUNKSJON FOR Å TELLE HVERT MILLISEKUND ***
void HAL_SYSTICK_Callback(void) {
        system_tid++;  
}