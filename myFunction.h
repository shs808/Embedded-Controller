#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"

#ifndef __MY_FUNCTION_H
#define __MY_FUNCTION_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

extern uint32_t cnt;
	 
void LED_Toggle(GPIO_TypeDef* Port, int Pin);
void SevenSegment_init(void);
void SevenSegment_Decode(unsigned int Num);
int max(int val, int ref);
int min(int val, int ref);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
