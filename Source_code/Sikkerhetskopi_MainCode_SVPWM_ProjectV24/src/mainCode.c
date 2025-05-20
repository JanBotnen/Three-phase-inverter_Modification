#include <math.h>
#include <stdbool.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_cortex.h>
#include <stm32l4xx_ll_rcc.h>
#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_ll_tim.h>
#include <stm32l4xx_ll_utils.h>
#include <stm32l4xx_ll_pwr.h>
#include <stm32l4xx_ll_system.h>
#include <stm32l4xx_ll_usart.h>


//_______________________________________________________________________________________________________________________
// Declarations of I/O:


// UART:

#define UART2_PORT            GPIOA             // UART2 Port
#define UART2_TX_PIN          LL_GPIO_PIN_2     // UART2 Transmitter Pin
#define UART2_RX_PIN          LL_GPIO_PIN_3     // UART2 Receiver Pin

// Bulit-in LED:
#define ONBOARD_LED_PORT      GPIOA
#define ONBOARD_LED_PIN       LL_GPIO_PIN_5

// Encoder SW-pin (D10 (Arduino pin) = PB6)
#define SW_PORT               GPIOB
#define SW_PIN                LL_GPIO_PIN_6  

// Direction button (PB14)
#define DIR_Button_PORT       GPIOB
#define DIR_Button_PIN        LL_GPIO_PIN_14
/*
// ON/OFF-button (PB13) 
#define ON_OFF_Button_PORT    GPIOB
#define ON_OFF_Button_PIN     LL_GPIO_PIN_13 */
#define ON_OFF_Button_PORT    GPIOC
#define ON_OFF_Button_PIN     LL_GPIO_PIN_13 //Blå intern knapp for å skru av og på

// Status LEDs:
#define RED_LED_PORT          GPIOB
#define RED_LED_PIN           LL_GPIO_PIN_15
#define GREEN_LED_PORT        GPIOB
#define GREEN_LED_PIN         LL_GPIO_PIN_11
#define YELLOW_LED_PORT       GPIOB
#define YELLOW_LED_PIN        LL_GPIO_PIN_12

/* Gate Driver 1 */
#define DRIVER1_RDY_PORT      GPIOA             // Gate Driver Leg A Ready Port
#define DRIVER1_RDY_PIN       LL_GPIO_PIN_11     // Gate Driver Leg A Ready Pin
#define DRIVER1_INHS_PORT     GPIOA             // Gate Driver Leg A High Side PWM Port
#define DRIVER1_INHS_PIN      LL_GPIO_PIN_8     // Gate Driver Leg A High Side PWM Pin
#define DRIVER1_INLS_PORT     GPIOA             // Gate Driver Leg A Low Side PWM Port
#define DRIVER1_INLS_PIN      LL_GPIO_PIN_7     // Gate Driver Leg A Low Side PWM Pin



/*
VIKTIG: for at UART skal virke må disse kommenteres ut, men de må
kommenteres inn igjen når koden er komplett
første spalte er V2(v3.0) andre er v3.1
*/
#define DRIVER1_RST_PORT      GPIOA             // Gate Driver Leg A Reset Port
#define DRIVER1_RST_PIN       LL_GPIO_PIN_3     // Gate Driver Leg A Reset Pin
#define DRIVER1_FLT_PORT      GPIOA             // Gate Driver Leg A Fault Port
#define DRIVER1_FLT_PIN       LL_GPIO_PIN_2     // Gate Driver Leg A Fault Pin
/*
#define DRIVER1_RST_PORT      GPIOC             // Gate Driver Leg A Reset Port
#define DRIVER1_RST_PIN       LL_GPIO_PIN_11     // Gate Driver Leg A Reset Pin
#define DRIVER1_FLT_PORT      GPIOA             // Gate Driver Leg A Fault Port
#define DRIVER1_FLT_PIN       LL_GPIO_PIN_12     // Gate Driver Leg A Fault Pin
*/



/* Gate Driver 2 */
#define DRIVER2_RDY_PORT      GPIOB             // Gate Driver Leg B Ready Port
#define DRIVER2_RDY_PIN       LL_GPIO_PIN_10    // Gate Driver Leg B Ready Pin
#define DRIVER2_INHS_PORT     GPIOA             // Gate Driver Leg B High Side PWM Port
#define DRIVER2_INHS_PIN      LL_GPIO_PIN_9     // Gate Driver Leg B High Side PWM Pin
#define DRIVER2_INLS_PORT     GPIOB             // Gate Driver Leg B Low Side PWM Port
#define DRIVER2_INLS_PIN      LL_GPIO_PIN_0     // Gate Driver Leg B Low Side PWM Pin
#define DRIVER2_RST_PORT      GPIOB             // Gate Driver Leg B Reset Port
#define DRIVER2_RST_PIN       LL_GPIO_PIN_5     // Gate Driver Leg B Reset Pin
#define DRIVER2_FLT_PORT      GPIOB             // Gate Driver Leg B Fault Port
#define DRIVER2_FLT_PIN       LL_GPIO_PIN_4     // Gate Driver Leg B Fault Pin

/* Gate Driver 3 */
#define DRIVER3_RDY_PORT      GPIOA             // Gate Driver Leg C Ready Port
#define DRIVER3_RDY_PIN       LL_GPIO_PIN_4     // Gate Driver Leg C Ready Pin
#define DRIVER3_INHS_PORT     GPIOA             // Gate Driver Leg C High Side PWM Port
#define DRIVER3_INHS_PIN      LL_GPIO_PIN_10    // Gate Driver Leg C High Side PWM Pin
#define DRIVER3_INLS_PORT     GPIOB             // Gate Driver Leg C Low Side PWM Port
#define DRIVER3_INLS_PIN      LL_GPIO_PIN_1     // Gate Driver Leg C Low Side PWM Pin
#define DRIVER3_RST_PORT      GPIOC             // Gate Driver Leg C Reset Port
#define DRIVER3_RST_PIN       LL_GPIO_PIN_0     // Gate Driver Leg C Reset Pin
#define DRIVER3_FLT_PORT      GPIOC             // Gate Driver Leg C Fault Port
#define DRIVER3_FLT_PIN       LL_GPIO_PIN_1     // Gate Driver Leg C Fault Pin

/*
Kommentert ut fordi det benyttes ikke til noe*/

#define TRIGGER_SIGNAL_PWM_PORT GPIOB
#define TRIGGER_SIGNAL_PWM_PIN  LL_GPIO_PIN_8




//_______________________________________________________________________________________________________________________
// Global variables:

uint32_t sine_res_g           = 0;              // No. of Compare Values for Each Period (max 1024).
float sine_table[] = {0,0.0627905195293134,0.125333233564304,0.187381314585725,0.248689887164855,0.309016994374947,0.368124552684678,0.425779291565073,0.481753674101715,0.535826794978997,0.587785252292473,0.63742398974869,0.684547105928689,0.728968627421412,0.770513242775789,0.809016994374947,0.844327925502015,0.876306680043864,0.904827052466019,0.929776485888251,0.951056516295154,0.968583161128631,0.982287250728689,0.992114701314478,0.998026728428272,1,0.998026728428272,0.992114701314478,0.982287250728689,0.968583161128631,0.951056516295154,0.929776485888251,0.904827052466019,0.876306680043864,0.844327925502015,0.809016994374947,0.770513242775789,0.728968627421412,0.684547105928689,0.63742398974869,0.587785252292473,0.535826794978997,0.481753674101715,0.425779291565072,0.368124552684678,0.309016994374947,0.248689887164855,0.187381314585725,0.125333233564304,0.0627905195293133,0,-0.0627905195293133,-0.125333233564304,-0.187381314585725,-0.248689887164855,-0.309016994374947,-0.368124552684678,-0.425779291565072,-0.481753674101715,-0.535826794978997,-0.587785252292473,-0.63742398974869,-0.684547105928689,-0.728968627421412,-0.770513242775789,-0.809016994374947,-0.844327925502015,-0.876306680043863,-0.90482705246602,-0.929776485888251,-0.951056516295154,-0.968583161128631,-0.982287250728689,-0.992114701314478,-0.998026728428272,-1,-0.998026728428272,-0.992114701314478,-0.982287250728689,-0.968583161128631,-0.951056516295154,-0.929776485888251,-0.90482705246602,-0.876306680043863,-0.844327925502015,-0.809016994374947,-0.770513242775789,-0.728968627421412,-0.684547105928689,-0.63742398974869,-0.587785252292473,-0.535826794978996,-0.481753674101715,-0.425779291565072,-0.368124552684678,-0.309016994374947,-0.248689887164854,-0.187381314585725,-0.125333233564304,-0.0627905195293138,0};

uint32_t      modulationFrequency = 0;               // Modulasjonsfrekvens
float         m_a                 = 0;               // Amplitudemodulasjonsindeks
uint32_t      sine_res            = 101;             // Antall verdier i sine_table
float         Vd                  = 0;
uint32_t      interrutCounterLim  = 0;     
bool          on_off              = false;           // Hvis true -> omformer er på , hvis false -> omformer er av (TIM1 teller ikke)          

// Defines valid modulation schemes for the inverter.
typedef enum {
  SPWM = 0,
  SVPWM
} scheme_t;

// Defines valid rotation directions for the rotor.
typedef enum {
  CW = 0,
  CCW
} direction_t;


// Defines valid operating states for the inverter.
typedef enum {
  NOT_READY = 0,
  READY,
  RUNNING,
  FAULT
} state_t;

typedef struct {
  uint32_t L1;
  uint32_t L2;
  uint32_t L3;
} compare_t;

direction_t  direction_g         = CW;               // Rotor Rotation Direction
scheme_t     algorithm_g          = SVPWM;           // Modulation Scheme

//_______________________________________________________________________________________________________________________
// SysTick handler:
void SysTick_Handler(void){

  static uint_fast32_t cnt = 0;
  cnt++;

  if ((cnt % 500) == 0){
   // LL_GPIO_TogglePin(ONBOARD_LED_PORT, ONBOARD_LED_PIN);
  }
}

//_______________________________________________________________________________________________________________________

// Initialize functions:
void enable_gpio_ports_clock();

void initialize_UART();
void initialize_SystemClock();
void initialize_IO();
void initialize_TIM1();
void initialize_TIM2();
void initialize_EncoderTimer();

void generate_SquareWave();

void update_TIM1_compareValue_SPWM_v1(float, uint32_t, uint32_t);
void update_TIM1_compareValue_SPWM_v2(float, uint32_t, uint32_t);
void update_TIM1_compareVale_SVPWM(float, float, uint32_t, uint32_t);

void set_ModulationFrequency_Manually(uint32_t);
void set_SwitchingFrequency_Manually(uint32_t);

uint32_t calculate_MaximunFrequency_SPWM(double, double, double);
double   calculate_AmplitudeModulationIndex(uint32_t, double, double, double);
uint32_t get_encoder_timer_count();

void     read_DirectionButton(void);
void     read_OnOffButton(void);

// Parametre for PWM-algoritmer:
void        set_ModulationFrequency(uint32_t f)
{
  modulationFrequency = f;
}
uint32_t    get_ModulationFrequency(void)
{
  return modulationFrequency;
}
void        set_ModulationAlgorithm(scheme_t algorithm) {

  algorithm_g = algorithm;
}
scheme_t    get_ModulationAlgorithm(void) {

  return algorithm_g;
}
void        set_AmplitudeModulationIndex(float ma)
{
  m_a = ma;
}
float       get_AmplitudeModulationIndex(void)
{
  return m_a;
}
void        set_Vd(float Ud)
{
  Vd = Ud;
}
float       get_Vd(void)
{
  return Vd;
}
void        set_InterruptCounterLimit(uint32_t f)
{
  // Denne funksjonen er dimensjonert for å virke med en klokkefrekvens på 10MHz 
  // med en AutoReload på 100. 

  long     f_tick    = SystemCoreClock / LL_TIM_GetPrescaler(TIM2);
  double   T_tick    = 1 / (double)f_tick;

  double   T_int     = T_tick * LL_TIM_GetAutoReload(TIM2);

  uint32_t f_count     = 101 * f;
  double   T_count     = 1 / (double)f_count;

  uint32_t lim       = (uint32_t)round(T_count / T_int);
  interrutCounterLim = lim;
}
uint32_t    get_InterruptCounterLimit(void)
{
  return interrutCounterLim;
}
void        set_On_Off_State(bool state)
{
  if (state)
  {
      on_off    = true;
  }
  else
  on_off        = false;
}
bool        get_On_Off_State(void)
{
  return on_off;
}
void        set_rotorDirection(direction_t direction) 
{
  direction_g = direction;
}
direction_t get_rotorDirection(void) 
{
  return direction_g;
}
void        set_RedLED(bool on)
{
  if (on) {LL_GPIO_SetOutputPin(RED_LED_PORT, RED_LED_PIN);}
  else    {LL_GPIO_SetOutputPin(RED_LED_PORT, RED_LED_PIN);}
}
void        set_GreenLED(bool on)
{
  if (on) {LL_GPIO_SetOutputPin(GREEN_LED_PORT, GREEN_LED_PIN);}
  else    {LL_GPIO_SetOutputPin(GREEN_LED_PORT, GREEN_LED_PIN);}
}
void        set_YellowLED(bool on)
{
  if (on) {LL_GPIO_SetOutputPin(YELLOW_LED_PORT, YELLOW_LED_PIN);}
  else    {LL_GPIO_SetOutputPin(RED_LED_PORT, RED_LED_PIN);}
}

// UART-functions:
char *intToStrDec(uint32_t x, char *s)
{
    *--s = 0; // Decrement pointer and NULL terminate string
    if (!x) *--s = '0'; // Add a zero in ASCII if x is zero
    for (; x; x/=10) *--s = '0'+x%10; // Loop as long as x is not equal to zero, divide by 10 and add the remainder as the current digit
    // (remember that x/=10 is performed after the body x%10)
    return s; // Return a pointer to the beginning of the string (this does not have to be the beginning of the buffer)
}
void print_uint32(uint32_t val) {

    char buf[3*sizeof(uint32_t)+1];

    char *str = intToStrDec(val, buf + sizeof(buf));

    while(*str){
        while(!LL_USART_IsActiveFlag_TC(USART2));
        LL_USART_TransmitData8(USART2, *str++);
    }
}
void print_str(char *str){
    while(*str){
        while(!LL_USART_IsActiveFlag_TC(USART2));
        LL_USART_TransmitData8(USART2, *str++);
    }
}

float min(float x, float y, float z) 
{
  // Returns the lowest floating-point number among the three inputs.
  return x < y ? (x < z ? x : z) : (y < z ? y : z);
}
float max(float x, float y, float z) 
{
  // Returns the highest floating-point number among the three inputs.
  return x > y ? (x > z ? x : z) : (y > z ? y : z);
}

uint32_t get_encoder_timer_count(){
    return LL_TIM_GetCounter(TIM3);
}


//_______________________________________________________________________________________________________________________
// ISR-function for handling of timer-interrupt on TIM2:

void TIM2_IRQHandler(void)
{

  //LL_GPIO_SetOutputPin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN);

  // Parametre for PWM-algoritmer:
  scheme_t        algo       = get_ModulationAlgorithm();
  float           m_a        = get_AmplitudeModulationIndex();
  float           Vd         = get_Vd();
  uint32_t        k_reload   = LL_TIM_GetAutoReload(TIM1);
  uint32_t        sine_res   = 101;
  uint32_t        counterLim = get_InterruptCounterLimit();
  static uint32_t counter    = 0;

  if(counter >= counterLim)
  {

    static uint32_t i1 = 0;
    static uint32_t i2 = 67; //120 grader
    static uint32_t i3 = 34; //240 grader

    compare_t compare;
    float UAo1, UBo1, UCo1, Uk;

    switch (algo)
    {
      case SPWM:
          compare.L1 = (uint32_t)((m_a*k_reload*sine_table[i1] + k_reload)/2.0);
          compare.L2 = (uint32_t)((m_a*k_reload*sine_table[i2] + k_reload)/2.0);
          compare.L3 = (uint32_t)((m_a*k_reload*sine_table[i3] + k_reload)/2.0);
        break;


      case SVPWM:
          UAo1 = m_a*(Vd/sqrt(3))*sine_table[i1];
          UBo1 = m_a*(Vd/sqrt(3))*sine_table[i2];
          UCo1 = m_a*(Vd/sqrt(3))*sine_table[i3];
    
          Uk = (1/2.0)*(max(UAo1, UBo1, UCo1) + min(UAo1, UBo1, UCo1));

          compare.L1 = (uint32_t)(k_reload*((1/sqrt(3))*m_a*sine_table[i1]-(Uk/Vd)) + k_reload/2.0);
          compare.L2 = (uint32_t)(k_reload*((1/sqrt(3))*m_a*sine_table[i2]-(Uk/Vd)) + k_reload/2.0);
          compare.L3 = (uint32_t)(k_reload*((1/sqrt(3))*m_a*sine_table[i3]-(Uk/Vd)) + k_reload/2.0);
        break;

    default:
      break;
    }

    if (get_rotorDirection() == CCW) 
    { 
      uint32_t temp = compare.L2;
      compare.L2    = compare.L3;
      compare.L3    = temp;
    }

    // Set compare values:
    LL_TIM_OC_SetCompareCH1(TIM1, compare.L1);
    LL_TIM_OC_SetCompareCH2(TIM1, compare.L2);
    LL_TIM_OC_SetCompareCH3(TIM1, compare.L3);

    i1++; i2++; i3++;
   
    /*
    if (i1 >= sine_res) {
      LL_GPIO_TogglePin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN);
      i1 = 1;
    }*/
    if (i1 >= sine_res) {
      /*
      // === Alternativ 1: Generer en puls HØY → LAV i én og samme interrupt ===
      LL_GPIO_SetOutputPin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN); // Trigger HØY
      __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); // ~60 ns delay @ 80 MHz
      LL_GPIO_ResetOutputPin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN); // Trigger LAV
  */
      // === Alternativ 2: Generer en puls over to interrupt-sykluser ===
      /*
      static bool pulse_high = false;
      if (!pulse_high) {
          LL_GPIO_SetOutputPin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN); // Sett HØY
          pulse_high = true;
      } else {
          LL_GPIO_ResetOutputPin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN); // Sett LAV
          pulse_high = false;
      }
      */
      
  //Eirik sin variant:
  
   LL_GPIO_TogglePin(TRIGGER_SIGNAL_PWM_PORT, TRIGGER_SIGNAL_PWM_PIN);


      i1 = 1;
  }
  
    if (i2 >= sine_res) i2 = 1;
    if (i3 >= sine_res){
      i3 = 1;
    }

    counter = 0;

  }

  counter++;

  if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) LL_TIM_ClearFlag_UPDATE(TIM2);


}



//_______________________________________________________________________________________________________________________
// MAIN FINCTION:


int main()
{
  // Initializations:
  initialize_SystemClock();
  initialize_TIM1();
  initialize_TIM2();
  initialize_EncoderTimer();
  enable_gpio_ports_clock();
  initialize_UART();
  initialize_IO();
  

  //Sett RST-pinnene høye for å aktivere gate driverene //V3
  LL_GPIO_SetOutputPin(DRIVER1_RST_PORT, DRIVER1_RST_PIN);  // PA3
  LL_GPIO_SetOutputPin(DRIVER2_RST_PORT, DRIVER2_RST_PIN);  // PB5
  LL_GPIO_SetOutputPin(DRIVER3_RST_PORT, DRIVER3_RST_PIN);  // PC0


  //__________________________________________________________________________________________________________________________________
  // SETTINGS 
  // (Må instilles korrekt av bruker):

  set_ModulationAlgorithm(SVPWM);               //       Ønsket modulasjonsalgoritme (SVPWM eller SPWM)
  set_SwitchingFrequency_Manually(5000);        // [Hz]  Ønsket svitsjefrekvens
  set_Vd(230);                                  // [V]   Påtrykt DC-link spenning
  double   nominal_MotorVoltage    = 230;       // [V]   Nominell linjespenning for motor (400V for stjerne, 230V for trekant)
  double   nominal_MotorFrequency  = 50;        // [Hz]  Nominell frekvens for motor
  uint32_t max_Frequency           = 100;       // [Hz]  Høyeste tillatte frekvens
  uint32_t min_Frequency           = 1;         // [Hz]  Laveste tillatte frekvens

  // Driftsmodus:
  uint32_t manualMode              = 1;      
  /*  manualMode avgjør hvilke driftsmodus omformeren opererer i:
  *     - Hvis manualMode = 1  ->  modulasjonsfrekens og ma settes konstant til de manuelle instullingene, dreieretning kan ikke endres
  *     - Hvis manualMode = 0  ->  modulasjonsfrekvens styres av encoder med U/f = konst, dreieretning på rotor styres av DIR-knapp
  */ 

  //__________________________________________________________________________________________________________________________________




  switch(manualMode)
  {
    case 1:
      set_AmplitudeModulationIndex(1.0);
      set_ModulationFrequency_Manually(50);
      break;

    case 0:
      LL_TIM_SetPrescaler(TIM2, 8);
      LL_TIM_SetAutoReload(TIM2, 100);
      break;

    default:
    break;
  }

  // Enable counters:
  LL_TIM_EnableCounter(TIM2);
  LL_TIM_EnableCounter(TIM1);

  uint16_t prev_encoder_Val   = 0;

  // Dynamiske variabler:
  float    V_DC;                                  // DC-link voltage
  float    curr_ma;                               // Modulation index
  uint32_t curr_f             = 1;                // Set current frequency to minimal value at first

// Infinite loop:
for(;;)
{
    read_OnOffButton();
    V_DC = get_Vd();               

    // Få inn modulasjonsfrekvens fra encoder:
    uint32_t encoder_Val    = get_encoder_timer_count();       // Får telle-verdien til TIM3
    if(encoder_Val > prev_encoder_Val) {curr_f--;}             // Encoder har rotert mot høyre
    if(encoder_Val < prev_encoder_Val) {curr_f++;}             // Encoder har rotert mot venstre¨
    prev_encoder_Val = encoder_Val;

    // Skaler modulasjonsfrekvens til å ligge i riktig intervall
    if(curr_f > max_Frequency)     {curr_f = max_Frequency;}
    if(curr_f < min_Frequency + 1) {curr_f = min_Frequency;}

    // Kontroller modulasjonsfrevkens fra encoder dersom driftsmodus ikke er i manuelt mode
    if(manualMode == 0)
    {
      // Beregn ma basert på ønsket modulasjonsfrekvens slik at U/f holdes konstant
      curr_ma = calculate_AmplitudeModulationIndex(curr_f, V_DC, nominal_MotorFrequency, nominal_MotorVoltage);
      set_AmplitudeModulationIndex(curr_ma);

      // Beregn interrupt-counter basert på øsnket modulasjonsfrekvens
      set_InterruptCounterLimit(curr_f);

      read_DirectionButton();
    }

    
    print_uint32(curr_f);
    print_str("\n");
    
  }
}



//______________________________________________________________________________________________________________________
// Calcultatios

// SPWM calculations:
uint32_t    calculate_MaximunFrequency_SPWM(double Vd, double f_n, double V_n)
{
  // Returns the maximum frequency before overmodulation (m_a > 1)
  double k          = 1*sqrt(3)/(2*sqrt(2));
  uint32_t f_max    = k*Vd*f_n/V_n;
  return f_max;
}
double calculate_AmplitudeModulationIndex(uint32_t curr_f, double curr_Vd, double f_n, double V_n)
{
  // Calculate linear modulation-index
  double m_a;
  double k    = 2*sqrt(2)/sqrt(3);                 // Constant
  m_a         = k*V_n*curr_f/(curr_Vd*f_n);        // Actual calcutation

  if(m_a > 0)
    {
      return m_a;
    }

    if (m_a < 0)
      {
        return 0;     
      }
}



//_______________________________________________________________________________________________________________________
// Configurations and initializations:

// Initialiseringsfunksjoner:
void initialize_SystemClock(){

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4){
  }

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is Ready */
  while(LL_RCC_HSI_IsReady() != 1){
  }

  LL_RCC_HSI_SetCalibTrimming(16);

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is Ready */
  while(LL_RCC_PLL_IsReady() != 1){
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System Clock is Ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL){
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(80000000); // 80 MHz

  LL_SetSystemCoreClock(80000000); // 80 MHz

  SysTick_Config(SystemCoreClock / 1000);
}
void initialize_EncoderTimer(){

  // This functions works to set TIM3 to count upwards if encoder is rotated clockwise, and
  // count downwards if encoder is rotared counter-clockwise

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    // Enable GPIO clocks
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);     

    /* PA9 is on D8 */
    /**/ //Var kommentert fra starten av
    /*
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    */

    /*
     * TIM3 has CH1 on PA6 and CH2 on PC7
     * PA6 -> D12
     * PC7 -> D9
     *
     */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_6, LL_GPIO_AF_2);

    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_7, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetAFPin_0_7(GPIOC, LL_GPIO_PIN_7, LL_GPIO_AF_2);

    //IS_TIM_ENCODER_INTERFACE_INSTANCE(TIM3);
    LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X2_TI1);
    //LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);

    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_SetPrescaler(TIM3, 0);
    LL_TIM_SetCounterMode(TIM3, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetAutoReload(TIM3, 65535);
    LL_TIM_SetClockDivision(TIM3, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetRepetitionCounter(TIM3, 0);

    LL_TIM_DisableARRPreload(TIM3);
    LL_TIM_SetTriggerInput(TIM3, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM3);

    LL_TIM_SetCounter(TIM3, 0);
    LL_TIM_EnableCounter(TIM3);
}
void initialize_UART(void) 
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(UART2_PORT, UART2_TX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(UART2_PORT, UART2_RX_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(UART2_PORT, UART2_TX_PIN, LL_GPIO_AF_7);
    LL_GPIO_SetAFPin_0_7(UART2_PORT, UART2_RX_PIN, LL_GPIO_AF_7);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

    LL_USART_SetOverSampling(USART2, LL_USART_OVERSAMPLING_8);

    LL_USART_SetBaudRate(USART2, SystemCoreClock, LL_USART_OVERSAMPLING_8, 9600);

    LL_USART_EnableDirectionRx(USART2);
    LL_USART_EnableDirectionTx(USART2);

    LL_USART_Enable(USART2);
}
void initialize_IO(void) 
{

  
  //_________________________________________________________________________________
  // Gate-drivers I/O:

  /* Configure Gate Driver 1 RDY */
  LL_GPIO_SetPinMode(DRIVER1_RDY_PORT, DRIVER1_RDY_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER1_RDY_PORT, DRIVER1_RDY_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER1_RDY_PORT, DRIVER1_RDY_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 1 INHS */
  LL_GPIO_SetPinMode(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(DRIVER1_INHS_PORT, DRIVER1_INHS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 1 INLS */
  LL_GPIO_SetPinMode(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(DRIVER1_INLS_PORT, DRIVER1_INLS_PIN, LL_GPIO_AF_1);

  /*
  Disse kommenteres ut for å gjøre plass til UART under testing av koden!!
  Må kommenters inn igjen når kode er komplett!*/
  // Configure Gate Driver 1 RST 
  LL_GPIO_SetPinMode(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER1_RST_PORT, DRIVER1_RST_PIN, LL_GPIO_PULL_NO);

  // Configure Gate Driver 1 FLT 
  LL_GPIO_SetPinMode(DRIVER1_FLT_PORT, DRIVER1_FLT_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER1_FLT_PORT, DRIVER1_FLT_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER1_FLT_PORT, DRIVER1_FLT_PIN, LL_GPIO_PULL_NO);
  

  /* Configure Gate Driver 2 RDY */
  LL_GPIO_SetPinMode(DRIVER2_RDY_PORT, DRIVER2_RDY_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER2_RDY_PORT, DRIVER2_RDY_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER2_RDY_PORT, DRIVER2_RDY_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 2 INHS */
  LL_GPIO_SetPinMode(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(DRIVER2_INHS_PORT, DRIVER2_INHS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 2 INLS */
  LL_GPIO_SetPinMode(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(DRIVER2_INLS_PORT, DRIVER2_INLS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 2 RST */
  LL_GPIO_SetPinMode(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER2_RST_PORT, DRIVER2_RST_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 2 FLT */
  LL_GPIO_SetPinMode(DRIVER2_FLT_PORT, DRIVER2_FLT_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER2_FLT_PORT, DRIVER2_FLT_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER2_FLT_PORT, DRIVER2_FLT_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 3 RDY */
  LL_GPIO_SetPinMode(DRIVER3_RDY_PORT, DRIVER3_RDY_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER3_RDY_PORT, DRIVER3_RDY_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER3_RDY_PORT, DRIVER3_RDY_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 3 INHS */
  LL_GPIO_SetPinMode(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_8_15(DRIVER3_INHS_PORT, DRIVER3_INHS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 3 INLS */
  LL_GPIO_SetPinMode(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinSpeed(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetAFPin_0_7(DRIVER3_INLS_PORT, DRIVER3_INLS_PIN, LL_GPIO_AF_1);

  /* Configure Gate Driver 3 RST */
  LL_GPIO_SetPinMode(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(DRIVER3_RST_PORT, DRIVER3_RST_PIN, LL_GPIO_PULL_NO);

  /* Configure Gate Driver 3 FLT */
  LL_GPIO_SetPinMode(DRIVER3_FLT_PORT, DRIVER3_FLT_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DRIVER3_FLT_PORT, DRIVER3_FLT_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DRIVER3_FLT_PORT, DRIVER3_FLT_PIN, LL_GPIO_PULL_NO);


  //_________________________________________________________________________________

/* Kommentert ut fordi denne benyttes ikke til noe*/
  // Trigger pin for PWM-signal meassurement (): 
  LL_GPIO_SetPinMode(TRIGGER_SIGNAL_PWM_PORT,TRIGGER_SIGNAL_PWM_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinSpeed(TRIGGER_SIGNAL_PWM_PORT,TRIGGER_SIGNAL_PWM_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinOutputType(TRIGGER_SIGNAL_PWM_PORT,TRIGGER_SIGNAL_PWM_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(TRIGGER_SIGNAL_PWM_PORT,TRIGGER_SIGNAL_PWM_PIN, LL_GPIO_PULL_NO);
  
  //_________________________________________________________________________________
  // PUSH BUTTONS:

  // Rotor-direction button:
  LL_GPIO_SetPinMode(DIR_Button_PORT, DIR_Button_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(DIR_Button_PORT, DIR_Button_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(DIR_Button_PORT, DIR_Button_PIN, LL_GPIO_PULL_NO);

  // On/off button:
  LL_GPIO_SetPinMode(ON_OFF_Button_PORT, ON_OFF_Button_PIN, LL_GPIO_MODE_INPUT);
  LL_GPIO_SetPinSpeed(ON_OFF_Button_PORT, ON_OFF_Button_PIN, LL_GPIO_SPEED_FREQ_LOW);
  LL_GPIO_SetPinPull(ON_OFF_Button_PORT, ON_OFF_Button_PIN, LL_GPIO_PULL_NO);

    //_________________________________________________________________________________
    // LED:

    LL_GPIO_SetPinMode(RED_LED_PORT, RED_LED_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(RED_LED_PORT, RED_LED_PIN, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(RED_LED_PORT, RED_LED_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(RED_LED_PORT, RED_LED_PIN, LL_GPIO_PULL_NO);

    LL_GPIO_SetPinMode(GREEN_LED_PORT, GREEN_LED_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(GREEN_LED_PORT, GREEN_LED_PIN, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(GREEN_LED_PORT, GREEN_LED_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(GREEN_LED_PORT, GREEN_LED_PIN, LL_GPIO_PULL_NO);

    LL_GPIO_SetPinMode(YELLOW_LED_PORT, YELLOW_LED_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinSpeed(YELLOW_LED_PORT, YELLOW_LED_PIN, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(YELLOW_LED_PORT, YELLOW_LED_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(YELLOW_LED_PORT, YELLOW_LED_PIN, LL_GPIO_PULL_NO);
}
void initialize_TIM1(void) {
// Initializes TIM1 for PWM generation on CH1, CH1N, CH2, CH2N, CH3 and CH3N.

  /* Enable Peripheral Clock */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* General Settings */
  LL_TIM_SetCounterMode(TIM1, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetClockDivision(TIM1, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_SetRepetitionCounter(TIM1, 0);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

  /* CH1 Settings */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1N, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1N, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH1);

  /* CH2 Settings */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH2N, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH2);

  /* CH3 Settings */
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH3N, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH3, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH3N, LL_TIM_OCIDLESTATE_LOW);
  LL_TIM_OC_EnableFast(TIM1, LL_TIM_CHANNEL_CH3);

  /* Other Settings */
  LL_TIM_SetOCRefClearInputSource(TIM1, LL_TIM_OCREF_CLR_INT_NC);
  LL_TIM_DisableExternalClock(TIM1);
  LL_TIM_ConfigETR(TIM1, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  LL_TIM_OC_SetDeadTime(TIM1, 32); //Originalt satt til 32.

  /* Enable PWM Outputs */
  //LL_TIM_EnableAllOutputs(TIM1);    // For å kunne skru av/på omformeren er denne funksjonen brukt i funksjonen 
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1N); 
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3); 
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3N);
}
void initialize_TIM2(void) 
{
  // Initialize TIM2 with the purpose of using it as a timer-interrupt 

  // Enable Peripheral Clock 
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* General Settings */
  LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  LL_TIM_EnableIT_UPDATE(TIM2);

  /* IRQ Settings */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);
}
void enable_gpio_ports_clock(void) 
{
  /* Enable GPIO Ports Clock */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
}

// Funksjoner for å leste trykknapper:
void read_OnOffButton(void)
{
      // Denne funksjonen 

      static bool ONOFF_last_state  = false;
      static bool ON_OFF            = false;
      bool        ONOFF_state       = LL_GPIO_IsInputPinSet(ON_OFF_Button_PORT, ON_OFF_Button_PIN);

      if(ONOFF_state != ONOFF_last_state) 
      {
        if(ONOFF_state) 
        { 
          ON_OFF = !ON_OFF;               // Toggle state
        }
        ONOFF_last_state = ONOFF_state;
      } 

      if (ON_OFF) {LL_TIM_DisableAllOutputs(TIM1);}
      else        {LL_TIM_EnableAllOutputs(TIM1);}

      // Håndtering av status-lys:
      switch (LL_TIM_IsEnabledAllOutputs(TIM1))
          {
              case 1:  // Omformer er på
                  LL_GPIO_SetOutputPin(GREEN_LED_PORT, GREEN_LED_PIN);
                  LL_GPIO_ResetOutputPin(RED_LED_PORT, RED_LED_PIN);
                  print_str("Omformer pa\n"); //kun for å verifisere i testing
                break;

              case 0:  // Omformer er av
                  LL_GPIO_ResetOutputPin(GREEN_LED_PORT, GREEN_LED_PIN);
                  LL_GPIO_SetOutputPin(RED_LED_PORT, RED_LED_PIN);
                  print_str("Omformer AV\n"); //Kun for å verifisere i testing
                break;
          
          default:
            break;
          }
}
void read_DirectionButton(void)
{
    // Funksjon som endrer dreieretning på rotor dersom 
    // DIR-knappen trykkes

    static bool DIR_Button_last_state  = false;
    static bool state_                 = false;
    bool        DIR_Button_state       = LL_GPIO_IsInputPinSet(DIR_Button_PORT, DIR_Button_PIN);

      if(DIR_Button_state != DIR_Button_last_state) 
      {
        if(DIR_Button_state) 
        { 
          state_ = !state_;               // Toggle state
        }
        DIR_Button_last_state = DIR_Button_state;
      } 

      // Endre dreieretning på rotor dersom knapp trykkes:
      if (state_) {set_rotorDirection(CCW);}
      else        {set_rotorDirection(CW);}
}

void generate_SquareWave(void)
{
  // Set prescaler to 80 for 1MHz tick frequency
  LL_TIM_SetPrescaler(TIM1, 1);

  // Get auto reload:
  uint32_t k_reload                     = 10000;
  LL_TIM_SetAutoReload(TIM1, k_reload); 

  // Set compare-value (for controlling duty-cycle):
  uint32_t comp                         = k_reload / 2;   // 50% duty cycle
  LL_TIM_OC_SetCompareCH1(TIM1, comp); 
}

// Funksjoner for å operere omformeren manuelt mode (fra koden)
void set_ModulationFrequency_Manually(uint32_t f_modulation)
{
  // Set prescaler til 80 konstant slik at klokkefrekvens på TIM1 blir 1MHz konstant:
  LL_TIM_SetPrescaler(TIM2, 80);

  double T_tick             = 1 / 1000000.0;         // Tick frequency is 1MHz
  uint32_t f_int            = 101 * f_modulation;    // Interrupt frequency (101 because sine_table consists of 101 values)
  double T_int              = 1 / (double)f_int;
  uint32_t k_reload         = (uint32_t)round(T_int / T_tick);
  LL_TIM_SetAutoReload(TIM2, k_reload);             // Set auto-reload on TIM2
}
void set_SwitchingFrequency_Manually(uint32_t f_sw)
{
  // Set prescaler to 80 so that the tick-frequency becomes 1MHz
  LL_TIM_SetPrescaler(TIM1, 80);

  // Since the tick-frequency is held constant 1MHz, the switching frequency is 
  // set by the auto-reload (the max value TIM1 counts up to)
  uint32_t  f_tick             = 1000000;     // 1MHz tick frequency (set by prescaler)
  uint32_t  k_reload           = (uint32_t)round(f_tick / (2 * f_sw));
  LL_TIM_SetAutoReload(TIM1, k_reload);
  
}

// Modulasjonsalgoritmer:
void update_TIM1_compareValue_SPWM_v1(float ma, uint32_t k_reload, uint32_t sine_res)
{
    // Sinus PWM algoritme (SPWM)

    // Denne metoden, til motsetning av v2, bruker en oppslagstabell med sinus-verdier til å beregne
    // sinuskurvene som skal sammenlignes med triangelkurven.

    // Målt tid for en iterasjon av denne metoden: 12.56µs

    static uint32_t i1 = 0;
    static uint32_t i2 = 67;
    static uint32_t i3 = 34;

    compare_t compare;

    float bias = k_reload;
    
    // Calcuate comare values:
    compare.L1 = (uint32_t)((ma*k_reload*sine_table[i1]+bias)/2.0);
    compare.L2 = (uint32_t)((ma*k_reload*sine_table[i2]+bias)/2.0);
    compare.L3 = (uint32_t)((ma*k_reload*sine_table[i3]+bias)/2.0);

    // Increment time values:
    i1++; i2++; i3++;
    if (i1 >= sine_res) i1 = 1;
    if (i2 >= sine_res) i2 = 1;
    if (i3 >= sine_res) i3 = 1;

    // Set compare values:
    LL_TIM_OC_SetCompareCH1(TIM1, compare.L1);
    LL_TIM_OC_SetCompareCH2(TIM1, compare.L2);
    LL_TIM_OC_SetCompareCH3(TIM1, compare.L3);
}
void update_TIM1_compareValue_SPWM_v2(float ma, uint32_t k_reload, uint32_t sine_res)
{
    // Sinus PWM algoritme (SPWM)

    // Denne metoden, til motsetning av v1, "sin(..)"-funksjonen til å beregne sinuskurvene
    // som sammenlignes med triangelkurven.

    // Målt tid for en iterasjon av denne metoden: 135µs

    static uint32_t i1 = 0;
    static uint32_t i2 = 67;
    static uint32_t i3 = 34;

    float f = 60;
    float T    = 1 / f;
    float step = T / (sine_res - 1);

    float t_L1 = step * i1;
    float t_L2 = step * i2;
    float t_L3 = step * i3;

    compare_t compare;

    float bias = k_reload;
    float PI   = 3.14159265359;
    // Calcuate comare values:
    compare.L1 = (uint32_t)((ma*k_reload*sin(2*PI*f*t_L1) + bias)/2.0);
    compare.L2 = (uint32_t)((ma*k_reload*sin(2*PI*f*t_L2) + bias)/2.0);
    compare.L3 = (uint32_t)((ma*k_reload*sin(2*PI*f*t_L3) + bias)/2.0);

    // Increment time values:
    i1++; i2++; i3++;
    if (i1 >= sine_res) i1 = 1;
    if (i2 >= sine_res) i2 = 1;
    if (i3 >= sine_res) i3 = 1;

    // Set compare values:
    LL_TIM_OC_SetCompareCH1(TIM1, compare.L1);
    LL_TIM_OC_SetCompareCH2(TIM1, compare.L2);
    LL_TIM_OC_SetCompareCH3(TIM1, compare.L3);
}
void update_TIM1_compareVale_SVPWM(float ma, float Ud, uint32_t k_reload, uint32_t sine_res)
{
    // Space Vector PWM algoritme (SVPWM)

    // Målt tid for èn iterasjon av denne algoritmen: 38µs

    static uint32_t i1 = 0;
    static uint32_t i2 = 67;
    static uint32_t i3 = 34;

    float UAo1 = ma*(Ud/sqrt(3))*sine_table[i1];
    float UBo1 = ma*(Ud/sqrt(3))*sine_table[i2];
    float UCo1 = ma*(Ud/sqrt(3))*sine_table[i3];
    
    float Uk = (1/2.0)*(max(UAo1, UBo1, UCo1)+min(UAo1, UBo1, UCo1));

    float bias = k_reload;

    compare_t compare;
    compare.L1 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sine_table[i1]-(Uk/Ud))+bias/2.0);
    compare.L2 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sine_table[i2]-(Uk/Ud))+bias/2.0);
    compare.L3 = (uint32_t)(k_reload*((1/sqrt(3))*ma*sine_table[i3]-(Uk/Ud))+bias/2.0);

    // Increment time values:
    i1++; i2++; i3++;
    if (i1 >= sine_res) i1 = 1;
    if (i2 >= sine_res) i2 = 1;
    if (i3 >= sine_res) i3 = 1;

    // Set compare values:
    LL_TIM_OC_SetCompareCH1(TIM1, compare.L1);
    LL_TIM_OC_SetCompareCH2(TIM1, compare.L2);
    LL_TIM_OC_SetCompareCH3(TIM1, compare.L3);
}
