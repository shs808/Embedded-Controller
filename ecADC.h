#ifndef __MY_ADC_H
#define __MY_ADC_H
#include "stm32f411xe.h"

#ifdef __cplusplus
	extern "C" {
#endif	/* __cplusplus */

// ADC trigmode
#define SW 0
#define TRGO 1

// ADC groupmode
#define REGULAR 0
#define INJECT 	1

// ADC contmode
#define CONT 0
#define SINGLE 1

// Edge Type
#define RISE_ADC 1
#define FALL_ADC 2
#define BOTH_ADC 3

#define _DEFAULT 0

// ADC setting
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode, int groupmode, int priority);			// trigmode : SW , TRGO / groupmode : REGULAR, INJECT
void ADC_TRGO(TIM_TypeDef* TIMx, int msec, int edge, int groupmode);
void ADC_continue(int contmode); 													// contmode : CONT, SINGLE / Operate both ADC,JADC
void ADC_sequence(int length, int *seq);
void ADC_sequence_Inject(int length, int *seq);

void ADC_start(ADC_TypeDef *ADCx, int groupmode);

uint32_t Is_ADC_EOC(void);
uint32_t Is_ADC_JEOC(void);
void Clear_ADC_JEOC(void);

uint32_t Is_ADC_OVR(void);
void Clear_ADC_OVR(void);

uint32_t ADC_read(void);
uint32_t ADC_read_Inject(int JDRn);
uint32_t ADC_pinmap(GPIO_TypeDef *port, int pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif