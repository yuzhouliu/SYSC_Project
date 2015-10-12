#include "CU_TM4C123.h"

// General defines
#define SYS_FREQ 16000000UL
#define MILLISEC_IN_SEC 1000
#define PRESCALE 255UL

// Define for servo motor
#define CMPA_0_DEGREES 4846				// CMPA 4846 is the 0 degree of the servo
#define CMPA_180_DEGREES 4372			// CMPA 4372 is the 180 degree of the servo

// For PWM
static uint16_t pwm_load = 0;
static uint32_t g_frequency = 50;

// Miscellaneous functions
void turn_on_green_LED(void);
void turn_off_green_LED(void);
void turn_on_red_LED(void);
void turn_off_red_LED(void);
void init_gpio(void);
void init_timer_A(void);
void delay_timer(uint32_t delay_time_ms);

																	
//PWM functions
void set_frequency(uint16_t frequency);
void set_position(uint16_t position);
void sweep_counter_clockwise(void);
void sweep_clockwise(void);
void PWM_init(void);
																	
int main(void)
{
	init_gpio();
	init_timer_A();

	PWM_init();
	
	PWM0->_2_LOAD = 4999;
	//PWM0->_2_CMPA = 4846;
	
	while(1)
	{
		sweep_counter_clockwise();
		sweep_clockwise();
	}
}

void turn_on_green_LED(void)
{
	GPIOF->DATA |= (1UL << 3);
}

void turn_off_green_LED(void)
{
	GPIOF->DATA &= ~(1UL << 3);
}

void turn_on_red_LED(void)
{
	GPIOF->DATA |= (1UL << 1);
}

void turn_off_red_LED(void)
{
	GPIOF->DATA &= ~(1UL << 1);
}

void init_gpio(void)
{
  uint32_t dummy;

  // Enable GPIOE (Speaker) and GPIOF (LED for debug)
  SYSCTL->RCGCGPIO |= ((1UL << 4) | (1UL << 5));

  // Do a dummy read to insert a few cycles after enabling the peripheral.
  dummy = SYSCTL->RCGCGPIO;

	// Sets PE4 for Digital out
	GPIOE->DIR |= (1UL << 4);
	GPIOE->DEN |= (1UL << 4);
	
	// Sets GPIOF's direction for LED
	GPIOF->DIR |= ((1UL << 3) | (1UL << 1));	// Output for PF3 LED
	GPIOF->DEN |= ((1UL << 3) | (1UL << 1));	//Enabled PF3(LED)

}

/** Configures TIMER0->TIMERA
*   Configures TIMERA as two 16-bit independent timer config (A & B)
*   One-shot and counts down
**/
void init_timer_A(void)
{
   uint32_t dummy;

   SYSCTL->RCGCTIMER |= 1UL << 0;  //Enable TIMER0
   dummy = SYSCTL->RCGCTIMER;      //Dummy read to give TIMER time to respond
   TIMER0->CTL &= ~(1UL << 0);      //Disable TIMERA in TIMER0 during config
   TIMER0->CFG = 0x4UL;             //Independent 16-bit timers
   TIMER0->TAMR = 0x1UL;            //One-Shot Timer mode
   TIMER0->TAPR = PRESCALE;         //Prescale value
	
	TIMER0->CTL |= (1UL << 1);				//TASTALL
   return;
}

/** Sets TIMERA as delay timer that delays for the amount in ms specified in input
*   Returns only when timer has expired
**/
void delay_timer(uint32_t delay_time_ms)
{
	uint32_t timer_reload_val;
	uint32_t prescale_output;
	
    //The reload value for the timer is the delay period we would like divided by
    //prescaled clock period
	prescale_output = SYS_FREQ/(PRESCALE+1);
	// VERY IMPORTANT: delay_time_ms/MILLISEC_IN_SEC will give 0, THE ORDER MATTERS HERE.
  timer_reload_val = (delay_time_ms*prescale_output)/MILLISEC_IN_SEC;

    TIMER0->TAILR = timer_reload_val;
    TIMER0->CTL |= (1UL << 0);   //Enable TIMER0A

    //While timer has not expired yet, Loop forever
    while( TIMER0->CTL & 1UL )
    {
    }    
    return;
}

/** Sets the frequency output of PWM signal **/
void set_frequency(uint16_t frequency)
{
	pwm_load = (SYS_FREQ/frequency)/64 - 1;
	PWM0->_2_LOAD = pwm_load;
}

/** Sets the PWM CMPA in order to get the appropriate position **/
void set_position(uint16_t position)
{
	uint16_t pos_cmpa;
	pos_cmpa = CMPA_0_DEGREES - (((100*(CMPA_0_DEGREES-CMPA_180_DEGREES))/180)*position)/100;
	
	if (pos_cmpa > CMPA_0_DEGREES)
		pos_cmpa = CMPA_0_DEGREES;
	else if (pos_cmpa < CMPA_180_DEGREES)
		pos_cmpa = CMPA_180_DEGREES;
	
	PWM0->_2_CMPA = pos_cmpa;
}

/** Sweeps the servo from Right to Left by increments of 1 degree
*		Waits 50ms before incrementing
**/
void sweep_counter_clockwise(void)
{
	uint16_t position;
	
	set_position(180);
	delay_timer(1000);
	delay_timer(1000);
	
	for (position = 180; position > 0; position--)
	{
		set_position(position);
		delay_timer(50);
	}
}

/** Sweeps the servo from Left to Right by increments of 1 degree
*		Waits 50ms before incrementing
**/
void sweep_clockwise(void)
{
	uint16_t position;

	set_position(0);
	delay_timer(1000);
	
	for (position = 0; position < 180; position++)
	{
		set_position(position);
		delay_timer(50);
	}
}

/** Initializes PWM signal on PE4 for output to speaker **/
void PWM_init(void)
{
	SYSCTL->RCGC0 |= (1UL << 20);
	//SYSCTL->RCGC2 |= (1UL << 4);	// Cannot configure this AND RCGCGPIO
	
	GPIOE->AFSEL |= (1UL << 4);		// Alternate function
	GPIOE->PCTL |= (0x4 << 16);		// Select M0PWM4

	SYSCTL->RCC |= (1 << 20);		// Use PWM divider
	SYSCTL->RCC |= (0x7 << 17);		// Divider set to divide by 64
	PWM0->_2_CTL = 0x0UL;			// Immediate update to parameters, Count down starting from LOAD to 0
	PWM0->_2_GENA = 0x8CUL;			// Drive PWM high when counter matches LOAD, drive low when matches CMPA
	PWM0->_2_CTL = 0x1UL;			// enabled PWM module 0, generator 2
	PWM0->ENABLE |= (1UL << 4);		// enable PWM module 0	
}

