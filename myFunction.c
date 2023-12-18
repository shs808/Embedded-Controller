#include "myFunction.h"

// LED 
void LED_Toggle(GPIO_TypeDef* Port, int Pin){
		Port->ODR ^= 1<<Pin; 
}

// 7 Segment Display ----------------------------------------
static int Seg7[8] 							= { 	0, 	 	 1, 	  0,  	 1, 
																			0, 		14, 	13, 		 4};
static GPIO_TypeDef* Seg7Pin[8] = { GPIOA, GPIOA, GPIOB, GPIOC,
																		GPIOC, GPIOA, GPIOA, GPIOA };

static int Seg7Table[11][8] =
{ {0, 0, 0, 0, 0, 0, 1, 1},		// 0
	{1, 0, 0, 1, 1, 1, 1, 1},		// 1
	{0, 0, 1, 0, 0, 1, 0, 1},		// 2
	{0, 0, 0, 0, 1, 1, 0, 1}, 	// 3
	{1, 0, 0, 1, 1, 0, 0, 1}, 	// 4
	{0, 1, 0, 0, 1, 0, 0, 1}, 	// 5
	{0, 1, 0, 0, 0, 0, 0, 1}, 	// 6
	{0, 0, 0, 1, 1, 1, 1, 1},		// 7
	{0, 0, 0, 0, 0, 0, 0, 1},		// 8
	{0, 0, 0, 0, 1, 0, 0, 1},		// 9
	{1, 1, 1, 1, 1, 1, 1, 1},		// dp
};	

void SevenSegment_init(void){
	RCC_GPIO_enable(GPIOA);
	RCC_GPIO_enable(GPIOB);
	RCC_GPIO_enable(GPIOC);
	for (int idx = 0 ; idx < 8 ; idx++){
		GPIO_mode(Seg7Pin[idx], Seg7[idx], OUTPUT);
		GPIO_otype(Seg7Pin[idx], Seg7[idx], PUSH_PULL);
		GPIO_pupdr(Seg7Pin[idx], Seg7[idx], NO_PUPD);
	}
}

void SevenSegment_Decode(unsigned int Num){
	for (int idx = 0; idx < 8; idx++)
		GPIO_write(Seg7Pin[idx], Seg7[idx], Seg7Table[Num][idx]);
}

int max(int val, int ref){
	if (val > ref) val = ref;
	return val;
}

int min(int val, int ref){
	if (val < ref) val = ref;
	return val;
}
