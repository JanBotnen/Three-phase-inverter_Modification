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
#include "stm32l4xx_ll_rcc.h" //Må inkluderes for å bruke  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);

// **Prototyper**
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void send_uart(const char *message);
void calibrate_adc();
void init_adc();
void LL_systemklokke_setup();
uint32_t LL_systemklokke_tid();
uint32_t filter_adc_fast_ema(int index, uint32_t ny_ADC_sample);
uint32_t read_ADC(void);
//Vaiabler
 uint32_t adc_values[3];  
 uint32_t voltage[3];       
 int current[3];
 uint32_t filtered_adc_value[3]; //variabel for filter. setter startverdi for ca 0 ampere. 
 uint32_t filtered_voltage[3]; 
 int filtered_current[3];
 float S_L1 =0.0537; //Orginalt 0,048
 float S_L2 =0.0548; //Orginalt 0,0537
 float S_L3 = 0.0544; //Sensitivitets koefisient. Orginalt 0.0557
 float U_reff = 1.6561; //referanse volt, måles før oppstart
 float S_justert = S_L3*(U_reff/1.65);
 uint32_t system_tid ;
 uint32_t forige_system_tid;
 char buffer[150];

int main(void) {
    init();  // Initialiserer serieporten for Arduino framework, viktig for å kunne sende noe vi Serial porten
    MX_USART2_UART_Init(); // initialisere funskjon
    read_ADC();// initialiserer funkso
    calibrate_adc();
    init_adc();
    LL_systemklokke_setup();
    LL_systemklokke_tid();
    // MX_GPIO_Init(); //initialiserer funksjon
    //MX_ADC1_Init(); //

    // **Sjekk systemklokke**
    char clock_buffer[50];
    sprintf(clock_buffer, "Clock: %lu\r\n", SystemCoreClock);
    send_uart(clock_buffer);
     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);

   

    while (1) { 
        system_tid = LL_systemklokke_tid();

        adc_values[0] = read_ADC();
        adc_values[1] = read_ADC();
        adc_values[2] = read_ADC();
        
        //Rgner ut strøm for L1
        filtered_adc_value[0] = filter_adc_fast_ema(0,adc_values[0]);
        voltage[0] = ((adc_values[0]*3.3 *1000)/4095)+50;//Legger til noen milivolt for at det skal samsvare med spenningen som chippen gir ut ved 0 A
        current[0] = ((voltage[0]-(U_reff*1000))/S_L1);
        filtered_voltage[0] = ((filtered_adc_value[0]*3.3 *1000)/4095)+58; //Brukte 40 mV på L3, 54 mv L2, 58 mV L1
        filtered_current[0] = ((filtered_voltage[0]-(U_reff*1000))/S_L1);
          //Rgner ut strøm for L2
        filtered_adc_value[1] = filter_adc_fast_ema(1, adc_values[1]);
        voltage[1] = ((adc_values[1]*3.3 *1000)/4095)+50;//Legger til noen milivolt for at det skal samsvare med spenningen som chippen gir ut ved 0 A
        current[1] = ((voltage[1]-(U_reff*1000))/S_L2);
        filtered_voltage[1] = ((filtered_adc_value[1]*3.3 *1000)/4095)+54; //Brukte 40 mV på L3, 54 mv L2, 58 mV L1
        filtered_current[1] = ((filtered_voltage[1]-(U_reff*1000))/S_L2);
          //Rgner ut strøm for L3
        filtered_adc_value[2] = filter_adc_fast_ema(2, adc_values[2]);
        voltage[2] = ((adc_values[2]*3.3 *1000)/4095)+50;//Legger til noen milivolt for at det skal samsvare med spenningen som chippen gir ut ved 0 A
        current[2] = ((voltage[2]-(U_reff*1000))/S_L3);
        filtered_voltage[2] = ((filtered_adc_value[2]*3.3 *1000)/4095)+44; 
        filtered_current[2] = ((filtered_voltage[2]-(U_reff*1000))/S_L3);
        
        if (system_tid-forige_system_tid>4000) { //2000 ser ut til å tilsvare et sekund
        //
        for (int i = 0; i < 3; i++) {
        sprintf(buffer,
        "ADC L%d: %lu, Spenning L%d: %lumV, Strøm L%d: %d mA, "
        "Filtrert ADC L%d: %lu, Filtrert spenning L%d: %lu mV, Filtrert strøm L%d: %d mA\r\n",
        i+1, adc_values[i],
        i+1, voltage[i],
        i+1, current[i],
        i+1, (uint32_t)filtered_adc_value[i],
        i+1, filtered_voltage[i],
        i+1, filtered_current[i]
         );
         send_uart(buffer);
        }
        send_uart("\r\n"); //Linjeskift for hver iterasjon
        forige_system_tid = LL_systemklokke_tid();   
        }   
    }
}
//Aktiverer en systemklokke. Bruker TIM6 til dette
void LL_systemklokke_setup(){
LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
// Forutsetter at systemklokke er 80 MHz – juster om din er annerledes
//LL_TIM_SetPrescaler(TIM6, (SystemCoreClock));  // (80 MHz *2 = 2000 Hz) → 0,5 ms per tick 
LL_TIM_SetPrescaler(TIM6, 80000 - 1);  // 80 MHz / 80000 = 1000 Hz = 1 ms per tick
LL_TIM_SetAutoReload(TIM6, 0xFFFF); // Maks 65535 ms (kan også bruke 32-bit timer)
LL_TIM_SetCounterMode(TIM6, LL_TIM_COUNTERMODE_UP);
LL_TIM_EnableCounter(TIM6);
}
//henter verdi til system klokke
uint32_t LL_systemklokke_tid(){
return LL_TIM_GetCounter(TIM6);
}


//Funksjon for software filter
float ema_internal[3] = {0.0f, 0.0f, 0.0f};  // tre forskjellige interne flytvallvaribler for tre mplinger

uint32_t filter_adc_fast_ema(int index, uint32_t ny_ADC_sample) {
    const float alpha = 0.0001f; //Juster denne for å gjre filteret mer nøyaktig, orginalt 0.1. Lavere verdi filtrerer mer støy
    ema_internal[index] = alpha * ny_ADC_sample + (1.0f - alpha) * ema_internal[index];
    return (uint32_t)(ema_internal[index] + 0.5f);  // avrund til nærmeste heltall
}
// **USART2-konfigurasjon** 

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

void init_adc() {

    
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC); //aktiverer klokke for alle analoge innganger GPIOC
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA); //Aktiverer innganger for GPIOA
    
    //Pin PA6 L1
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG); //setter inngang til analog modus
    LL_GPIO_EnablePinAnalogControl(GPIOA, LL_GPIO_PIN_6);  //Aktiverer analog kontroll funksjon på inngang
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO); //Deaktiverer pull up/down funksjon, slik at vi kan uten motstand 

    //Pin PC2 L2    
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_ANALOG); //setter inngang til analog modus
    LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_2);  //Aktiverer analog kontroll funksjon på inngang
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_2, LL_GPIO_PULL_NO); //Deaktiverer pull up/down funksjon, slik at vi kan uten motstand 

    //Pin PC3 L3
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_ANALOG); //setter inngang til analog modus
    LL_GPIO_EnablePinAnalogControl(GPIOC, LL_GPIO_PIN_3);  //Aktiverer analog kontroll funksjon på inngang
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_3, LL_GPIO_PULL_NO); //Deaktiverer pull up/down funksjon, slik at vi kan uten motstand 

    
 NVIC_SetPriority(ADC1_2_IRQn, 0); //setter adc til høyest prioritet i interoupt funksjonene
    NVIC_EnableIRQ(ADC1_2_IRQn); // Skrur på interoupt, uten denne vil ikke introupt funksjonen fungere
    
    /**
     * Configure the ADC clock
     *
     */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC); // aktiverer klokken for ADC, denne er av som standard og må skrus på
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4); //Deler den orginale frekvensen med 4 80MHz/4 = 20 MHz
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK); // Velger SYSCLC som kilde til ADC

    //LL_ADC_Disable(ADC1);
    while(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE()); // Block here if ADC is enabled (it should not be)

    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B); // Setter oppløsning til 12 bit
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT); //Høyresiller datane siden vi b
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE); // skrur av low power mode som er standard instilling i STM32

    /**
     * @brief
     *
     * Scan mode can only be used with DMA.
     */
    // Sequencer disabled is equivalent to sequencer of 1 rank, ADC conversion on only 1 channel.
  
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS); //Setter opp ADC1 for å kunne lese 3 inngangr sammtidig
    // LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE); // Setter opp ADC for å lese en inngang
    

    /**
     * @brief
     *
     * The sequencer discont mode can not be used with continous mode
     */
    LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_1RANK); // aktiverer diskontinuerlig måling, som tar målt ing baser på kun en trigger
    //PA6 L1

   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_11);
   LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_640CYCLES_5);
   LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_4, LL_ADC_OVS_SHIFT_NONE);
   LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);


    //PC2 L2
   LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_3);
   LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_640CYCLES_5);
   LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_8, LL_ADC_OVS_SHIFT_NONE);
   LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
 

    //PC3 L3
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_4);// kanal 5
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_640CYCLES_5); // kanal 5 (Skrudd opp til 247 for å prøve å få)
    LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_8, LL_ADC_OVS_SHIFT_NONE);  //ny
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED); // kanal 5

    LL_ADC_DisableDeepPowerDown(ADC1); // Skrur av deep power down funksjonen
    LL_ADC_EnableInternalRegulator(ADC1);

    calibrate_adc();

    LL_ADC_Enable(ADC1);
    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0){}

   // LL_ADC_REG_StartConversion(ADC1);
}

//Funksjon for kalibrering
void calibrate_adc() {
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0);
}

// **Send en tekststreng via UART (LL)**
void send_uart(const char *message) {
    while (*message) {
        while (!LL_USART_IsActiveFlag_TXE(USART2));  // Vent til TX er klar
        LL_USART_TransmitData8(USART2, *message++);
    }
    while (!LL_USART_IsActiveFlag_TC(USART2));  // Vent til siste byte er sendt
}

// **Les ADC-verdi**
uint32_t read_ADC(void) {
  
    //  Sjekk om ADC faktisk er aktivert
    if (!LL_ADC_IsEnabled(ADC1)) {
        send_uart("ADC not enabled!\r\n");
        return 99999;
    }
    //  Start ADC-konvertering
    LL_ADC_REG_StartConversion(ADC1);
    //  Sjekk om ADC-konvertering starter
    if (!LL_ADC_IsActiveFlag_ADRDY(ADC1)) {
        send_uart("ADC not ready!\r\n");
    }
    //  Vent til ADC-konvertering er ferdig
    while (!LL_ADC_IsActiveFlag_EOC(ADC1)) {
       // send_uart("Waiting for ADC conversion...\r\n");  
    }  
    return LL_ADC_REG_ReadConversionData12(ADC1);
}
