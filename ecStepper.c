#include "stm32f4xx.h"
#include "ecStepper.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
static uint32_t step_delay = 100; 

// Stepper Motor variable
volatile static Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_full_t;

static State_full_t FSM_full[4] = {  
	{0xC,{S1,S3}},		// S0 : 1100
	{0x6,{S2,S0}},		// S1 : 0110
	{0x3,{S3,S1}},		// S2 : 0011
	{0x9,{S0,S2}}			// S3 : 1001
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_half_t;

static State_half_t FSM_half[8] = { 
	{0x8,{S1,S7}},		// S0 : 1000
	{0xC,{S2,S0}},		// S1 : 1100
	{0x4,{S3,S1}},		// S2 : 0100
	{0x6,{S4,S2}},		// S3 : 0110
	{0x2,{S5,S3}},		// S4 : 0010
	{0x3,{S6,S4}},		// S5 : 0011
	{0x1,{S7,S5}},		// S6 : 0001
	{0x9,{S0,S6}},		// S7 : 1001
};


void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
//  GPIO Digital Out Initiation
	myStepper.port1 = port1;
	myStepper.pin1  = pin1;
	myStepper.port2 = port2;
	myStepper.pin2  = pin2;
	myStepper.port3 = port3;
	myStepper.pin3  = pin3;
	myStepper.port4 = port4;
  myStepper.pin4  = pin4;
	
	GPIO_init(port1, pin1, OUTPUT);
	GPIO_pupdr(port1, pin1, NO_PUPD);
	GPIO_otype(port1, pin1, PUSH_PULL);
	GPIO_ospeed(port1, pin1, FAST_SPEED);
	GPIO_init(port2, pin2, OUTPUT);
	GPIO_pupdr(port2, pin2, NO_PUPD);
	GPIO_otype(port2, pin2, PUSH_PULL);
	GPIO_ospeed(port2, pin2, FAST_SPEED);
	GPIO_init(port3, pin3, OUTPUT);
	GPIO_pupdr(port3, pin3, NO_PUPD);
	GPIO_otype(port3, pin3, PUSH_PULL);
	GPIO_ospeed(port3, pin3, FAST_SPEED);
	GPIO_init(port4, pin4, OUTPUT);
	GPIO_pupdr(port4, pin4, NO_PUPD);
	GPIO_otype(port4, pin4, PUSH_PULL);
	GPIO_ospeed(port4, pin4, FAST_SPEED);	
}

void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode == FULL){         // FULL mode
			GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0x08) >> 3);
  		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_full[state].out & 0x04) >> 2);
			GPIO_write(myStepper.port3, myStepper.pin3, (FSM_full[state].out & 0x02) >> 1);
			GPIO_write(myStepper.port4, myStepper.pin4, (FSM_full[state].out & 0x01) >> 0); 
			}	 
		 else if (mode ==HALF){    // HALF mode
			GPIO_write(myStepper.port1, myStepper.pin1, (FSM_half[state].out & 0x08) >> 3);
  		GPIO_write(myStepper.port2, myStepper.pin2, (FSM_half[state].out & 0x04) >> 2);
			GPIO_write(myStepper.port3, myStepper.pin3, (FSM_half[state].out & 0x02) >> 1);
			GPIO_write(myStepper.port4, myStepper.pin4, (FSM_half[state].out & 0x01) >> 0); 
			}
}


void Stepper_setSpeed (long whatSpeed){      // rpm
		step_delay = (uint32_t) 60000000/whatSpeed/2048;  				// Convert rpm to micro sec
}


void Stepper_step(int steps, int direction, int mode){
	myStepper._step_num = steps;
	uint32_t State_number = 0;
	if (mode == HALF) step_delay = step_delay/2;
	 
	for(;myStepper._step_num>0;myStepper._step_num--){ 	// run for step size
		delay_us(step_delay);                         		// delay (step_delay); 
				
		Stepper_pinOut(State_number, mode);
		 
		if (mode == FULL) 		 												
			State_number = FSM_full[State_number].next[direction];    // state_number = 0 to 3 for FULL step mode
		else if (mode == HALF) 
			State_number = FSM_half[State_number].next[direction];    // state_number = 0 to 7 for HALF step mode					
				
   }
}


void Stepper_stop (void){ 
     
	myStepper._step_num = 0;    
	// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
	GPIO_write(myStepper.port1, myStepper.pin1, 0);
	GPIO_write(myStepper.port2, myStepper.pin2, 0);
	GPIO_write(myStepper.port3, myStepper.pin3, 0);
	GPIO_write(myStepper.port4, myStepper.pin4, 0);
}


