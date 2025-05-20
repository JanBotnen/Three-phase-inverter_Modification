//**********************************************************************************************************************//
//** Dene filen inneholder programvare utviklet av Fredrik Larsgård, Kristian Farnes og Jan Magnus Botnene våren 2025©**//
//**********************************************************************************************************************//

#include <Arduino.h>
#include <stdio.h>
#include <string.h> 
#include "stm32l4xx_ll_adc.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_rcc.h"


#define ENCODER_PULSER_PER_OMDREINING 8192.0f // 2048 linjer × 4 for kvadratur
#define to_pi 6.283185307 //setter verdien til 2*pi

// Prototyepr
void MX_USART2_UART_Init(void);
void send_uart(const char *message);
void init_enkoder_pins();
float hent_enkoder_vinkel();
float hent_rotor_rpm();
void LL_systemklokke_setup();
uint32_t LL_systemklokke_tid();

//variabel
char buffer[50];

//float rotor_vinkel; //denne må byttes til float, men bruker int nå siden eg ikke får til å skrive ut desimaltall i serieporten
uint32_t enkoder_verdi; //midlertidig variabel for å leste enkoder verdi
uint32_t system_tid ;
uint32_t forige_system_tid;

int main(void){
  init();
  MX_USART2_UART_Init();
  LL_systemklokke_tid(); 
  LL_systemklokke_setup(); //aktiverer systemklokken
  init_enkoder_pins();

  while(1){

      //vinkel = hent_enkoder_vinkel()*1000;
      system_tid = LL_systemklokke_tid();

      if(system_tid-forige_system_tid>200){ //2000 for et sekund fordi dette går opp med frekvensen
        uint32_t rotor_vinkel = hent_enkoder_vinkel(); //bytt til float når du finner ut hvordan man skrive float til serieporten
        uint32_t rotor_rpm = hent_rotor_rpm(); //bytt til float
        //MOdlertidig variabel for å lese av teller
        LL_TIM_SetRepetitionCounter(TIM2,0);
        uint32_t enkoder_teller_verdi = LL_TIM_GetCounter(TIM2);
        
        sprintf(buffer,"1 sekund har passert \nTIM2 teller er: %lu\n Rotor rpm *1000 er %lu\n rotor vinkel *1000 er %lu\n" 
        ,enkoder_teller_verdi, (rotor_rpm*1000), (rotor_vinkel*1000));

        //Bruk funksjon under for å printe teller verdi direkte i serieporten
        //sprintf(buffer, "%lu " 
        //,enkoder_teller_verdi);

        send_uart(buffer);
        forige_system_tid = system_tid; 
      }   
  }
}

float hent_enkoder_vinkel(){
  
  float enkoder_verdi = LL_TIM_GetCounter(TIM2);//Henter verdi fra TIM2. Dette vil være verdien på hvor langt telleren til enkoderen har talt 2048, ganges med 2 eller 4 i noen moduser
  float rotor_vinkel = (enkoder_verdi/ENCODER_PULSER_PER_OMDREINING)*to_pi;

  return rotor_vinkel;
}

float hent_rotor_rpm(){
  float enkoder_vinkel = hent_enkoder_vinkel();
  uint32_t system_tid = LL_systemklokke_tid();

  static float forige_enkoder_vinkel = 0; //Settes til null bare en gang på grunn av static
  static uint32_t forige_system_tid = 0;  //Settes til nulø en gang på grunn av static 
  static float siste_motor_rpm = 0;
  //Filter variable
  float ny_motor_rpm =0;
  const float ALPHA = 0.2f; //filter parameter

  if(system_tid-forige_system_tid>2000){//måler hvert sekund
   float delta_tid = (system_tid-forige_system_tid)/20.f; //må dele på 2000 for å få tiden i sekund. 2000.f betyr at det er et flyttall
   float delta_vinkel= enkoder_vinkel-forige_enkoder_vinkel;

  //logikk for å håndtere vinkel overgang
  if (delta_vinkel > M_PI) {
    delta_vinkel -= to_pi;
  } else if (delta_vinkel < -M_PI) {
      delta_vinkel += to_pi;
  }

  ny_motor_rpm = (delta_vinkel/to_pi)*(60/delta_tid); //Regner ut momenten rpm
  siste_motor_rpm =ALPHA * ny_motor_rpm + (1.0f - ALPHA) * siste_motor_rpm; // Oppdaterer rpm med filterering
  //Oppdater vinkler og tid
   forige_enkoder_vinkel = enkoder_vinkel; //Resetter enkoder vinkel
   forige_system_tid = system_tid; //Oppdaterrer forige system tid til å bruke i neste iterasjon
  }
return siste_motor_rpm;
}
//Aktiverer en systemklokke. Bruker TIM6 til dette
void LL_systemklokke_setup(){
LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
// Forutsetter at systemklokke er 80 MHz – juster om din er annerledes
LL_TIM_SetPrescaler(TIM6, (SystemCoreClock));  // (80 MHz *2 = 2000 Hz) → 0,5 ms per tick
LL_TIM_SetAutoReload(TIM6, 0xFFFF); // Maks 65535 ms (kan også bruke 32-bit timer)
LL_TIM_SetCounterMode(TIM6, LL_TIM_COUNTERMODE_UP);
LL_TIM_EnableCounter(TIM6);
}
//henter verdi til system klokke
uint32_t LL_systemklokke_tid(){
return LL_TIM_GetCounter(TIM6);
}

void MX_USART2_UART_Init(void) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Sett baudrate til riktig frekvens
    LL_USART_SetBaudRate(USART2, HAL_RCC_GetHCLKFreq(), LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
    LL_USART_Enable(USART2);

    // Sørg for at TX-linjen er aktiv
    LL_USART_EnableDirectionTx(USART2);

    send_uart("USART2 Initialized\r\n");  // Testmelding
}
void send_uart(const char *message) {
    while (*message) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));  // Vent til TX er klar
        LL_USART_TransmitData8(USART2, *message++);
    }
    while (!LL_USART_IsActiveFlag_TC(USART2));  // Vent til siste byte er sendt
}

void init_enkoder_pins() {
  //Aktiverer klokker. Disse må aktiveres selv om vi skal bruke enkoder modus til disse inngangene

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA); //for PA0 og PA1
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);//For PB7
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2); //For TIM2, Tror ikke denne må vere aktivert om vi skla bruke enkoderen som inngang. 
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4); //For TIM4

  //Setter piner til enkoder modus
  LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);//Setter modus for å telle pulser i både A og B  både stigende og fallende flanke. Gir 4x oppløsning på enkoder pulser
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING); //Set ter trigger på rising edge til kanal 1 (A)
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING); //Setter trigger til rising edge på kanal 2 (B)
  LL_TIM_EnableCounter(TIM2);//aktiverer teller på TIM2
  LL_TIM_SetCounter(TIM2, 0);  //teller starter på null
  LL_TIM_SetAutoReload(TIM2, ENCODER_PULSER_PER_OMDREINING - 1);//Setter verdien til TIM2 til å passe til enkoder oppløsningen 8192-1
  
  //Pin for A, PA0
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE); //setter inngang til alternerende modus. Tror vi trenger dette for enkoder
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN); //Aktiverer intern pulldown motstand
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_0, 1); // PA0 = TIM2_CH1

  //Pin for B, PA1
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE); //setter inngang til alternerende modus. Tror vi trenger dette for enkoder
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN); //Aktiverer intern pulldown motstand
  LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_1, 1); // PA1 = TIM2_CH2

  // Pin for Z, PB7
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE); //setter inngang til alternerende modus. Tror vi trenger dette for enkoder
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN); //Aktiverer inern pulldown motstand 
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7, 2); // PB7 = TIM4_CH1

  // Konfigurer TIM4_CH2 som input capture på rising edge. Lager en puls når z blir aktivert
  LL_TIM_IC_SetActiveInput(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetPrescaler(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV32_N8); //Filterer ut støy, den mest nøyaltge filter komponenten

  LL_TIM_EnableIT_CC2(TIM4);
  NVIC_EnableIRQ(TIM4_IRQn); 
 
  //Aktiverer tellere
  NVIC_EnableIRQ(TIM2_IRQn);// Skrur på TIM2 interoupt, uten denne vil ikke introupt funksjonen fungere, 0 = høyest prioritet
  NVIC_EnableIRQ(TIM4_IRQn);// Skrur på TIM4 interoupt, uten denne vil ikke introupt funksjonen fungere, 0 = høyest prioritet
  NVIC_SetPriority(TIM2_IRQn,0); //Setter prioritet til høyeste
  NVIC_SetPriority(TIM4_IRQn,0); //Setter prioritet til høyeste

}
//Interroupt funksjon for z-pulsen til enkoderen som skal nullstille TIM2 telleren. Merk at interoupt funksjoner må ha dette navnet
void TIM4_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_CC2(TIM4)) {
    LL_TIM_ClearFlag_CC2(TIM4); // må alltid cleares!

    LL_TIM_SetCounter(TIM2, 0); // ← her nullstiller du encoder-telleren!
  }
}
