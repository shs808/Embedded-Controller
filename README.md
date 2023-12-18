
# Embedded Controller HAL

Written by:   Hyeong Seok, Song

Program: 		  C/C++

IDE/Compiler: Keil uVision 5

OS: 					WIn10

MCU:  				STM32F411RE, Nucleo-64



## GPIO Digital In/Out 

### Header File

 `#include "ecGPIO.h"`


```c++
#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

// MODER
#define INPUT  		0x00
#define OUTPUT 		0x01
#define AF     		0x02
#define ANALOG 		0x03

// IDR & ODR
#define HIGH 		1
#define LOW  		0

// OSPEED
#define LOW_SPEED		0x00
#define MID_SPEED		0x01
#define FAST_SPEED		0x02
#define HIGH_SPEED		0x03

// OTYPER
#define PUSH_PULL 		0	// Push-pull
#define OPEN_DRAIN 		1 	// Open-Drain

// PUDR
#define NO_PUPD			0x00 	// No pull-up, pull-down
#define PULL_UP			0x01 	// Pull-up
#define PULL_DOWN 		0x02 	// Pull-down	
#define RESERVED 		0x03 	// Reserved

// PIN
#define LED_PIN 		5
#define BUTTON_PIN 		13

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### GPIO_init\(\)

Initializes GPIO pins with default setting and Enables GPIO Clock. Mode: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT(0), OUTPUT(1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```



### GPIO_mode\(\)

Configures  GPIO pin modes: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_mode(GPIOA, 5, OUTPUT);
```



### GPIO_write\(\)

Write the data to GPIO pin: High, Low

```c++
write(GPIO_TypeDef *Port, int pin, int output);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **output**:   LOW(0), HIGH(1)



**Example code**

```c++
GPIO_write(GPIOA, 5, 1);  // 1: High
```



### GPIO_read\(\)

Read the data from GPIO pin

```c++
int  GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



**Example code**

```c++
GPIO_read(GPIOC, 13);
```



### GPIO_ospeed\(\)

Configures  output speed of GPIO pin : Low, Mid, Fast, High

```c++
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **speed**:   LOW_SPEED(0), MID_SPEED(1), FAST_SPEED(2) , HIGH_SPEED(3)



**Example code**

```c++
GPIO_ospeed(GPIOA, 5, 2);  // 2: FAST_SPEED
```



### GPIO_otype\(\)

Configures  output type of GPIO pin: Push-Pull / Open-Drain

```c++
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **type**:   PUSH_PULL(0), OPEN_DRAIN(1)



**Example code**

```c++
GPIO_otype(GPIOA, 5, 0);  // 0: Push-Pull
```



### GPIO_pupdr\(\)

Configures  Pull-up/Pull-down mode of GPIO pin: No Pull-up, Pull-down/ Pull-up/ Pull-down/ Reserved

```c++
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupd);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **pupd**:   NO_PUPD(0), PULL_UP(1), PULL_DOWN(2), RESERVED(3)



**Example code**

```c++
GPIO_pupdr(GPIOA, 5, 0);  // 0: No Pull-up, Pull-down
```



## EXTI (External Interrupt)

### Header File

 `#include "ecEXTI.h"`


```c++
#include "stm32f411xe.h"

#ifndef __EC_EXTI_H
#define __EC_EXTI_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

#define FALLING 	0
#define RISING 		1
#define FALL_RISE 2

void EXTI_init(GPIO_TypeDef* Port, int Pin, int TrigType, int priority);
void EXTI_PinSetup(GPIO_TypeDef* Port, int Pin);
void EXTI_FallingTrigger(int Pin);
void EXTI_RisingTrigger(int Pin);
void EXTI_Enable(uint32_t Pin);
void EXTI_Disable(uint32_t Pin);
void EXTI_SetPriority(int Pin, int priority);
void NVIC_Enable(int Pin);
uint32_t Is_Pending_EXTI(uint32_t Pin);
void Clear_Pending_EXTI(uint32_t Pin);
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### EXTI_init\(\)

Sets GPIO pins to enable external interrupt.

```c++
void EXTI_init(GPIO_TypeDef* Port, int Pin, int TrigType, int priority);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **TrigType**: Trigger Type, FALLING(0), RISING(01), FALL_RISE(02)
* **prioirty**:   Priority of EXTIx_IRQ Handler



### EXTI_PinSetup(\)

Connect the corresponding external line to GPIO

```c++
void EXTI_PinSetup(GPIO_TypeDef* Port, int Pin);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



### EXTI_FallingTrigger\(\) / EXTI_RisingTrigger()

Configures the trigger edge for Pin number

```c++
void EXTI_FallingTrigger(int Pin);
void EXTI_RisingTrigger(int Pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



### EXTI_Enable\(\)

Sets interrupt mask register for Pin number

```c++
void EXTI_Enable(uint32_t Pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



### EXTI_Disable\(\)

Clears interrupt mask register for Pin number.

```c++
void EXTI_Disable(uint32_t Pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15



### EXTI_SetPriority\(\)

Configures the priority of EXTI interrupt request

```c++
void EXTI_SetPriority(int Pin, int priority);
```

**Parameters**

* **pin**:  pin number (int) 0~15
* **priority**:  Priority of EXTIx_IRQ Handler



### NVIC_Enable\(\)

Enables EXTI interrupt request

```c++
void NVIC_Enable(int Pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15




### Is_Pending_EXTI\(\)

Checks the pending register when selected trigger request occurred

```c++
uint32_t Is_Pending_EXTI(uint32_t Pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15




### Clear_Pending_EXTI\(\)

Clears the pending register

```c++
void Clear_Pending_EXTI(uint32_t Pin);
```

**Parameters**

* **pin**:  pin number (int) 0~15




## SysTick

### Header File

 `#include "ecSysTick.h"`


```c++
#include "stm32f411xe.h"

#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

// Clock
#define MCU_CLK_PLL 	84000000
#define MCU_CLK_HSI 	16000000
#define MCU_SYS_CLK 	MCU_CLK_PLL		// MCU_CLK_HSI	or MCU_CLK_PLL

// UNIT
#define UNIT_MILLI		1/1000
#define UNIT_MICRO		1/1000000

void SysTick_init(uint32_t Tick_Period_ms);
void SysTick_init_us(uint32_t Tick_Period_us);
void delay_ms(uint32_t msec);
void delay_ms(uint32_t usec);
uint32_t SysTick_val(void);
void SysTick_Enable(void);
void SysTick_Disable(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```




### SysTick_init\(\)

Initializes System Tick

```c++
void SysTick_init(uint32_t Tick_Period_ms);
```

**Parameters**

* **Tick_Period_ms**:  Tick period, [ms]



**Example code**

```c++
SysTick_init(1);  // Tick period : 1[ms]
```



### SysTick_init_us\(\)

Initializes System Tick with 1usec period.

```c++
void SysTick_init_us(uint32_t Tick_Period_us);
```

**Parameters**

* **Tick_Period_us**:  Tick period, [us]



**Example code**

```c++
SysTick_init_us(1);  // Tick period : 1[ms]
```



### delay_ms\(\)

Pauses the program for a while [msec]

```c++
void delay_ms(uint32_t msec);
```

**Parameters**

* **msec**:  Time, [ms]



**Example code**

```c++
delay_ms(1000);  // delay time : 1[sec]
```



### delay_us\(\)

Pauses the program for a while [usec]

```c++
void delay_us(uint32_t usec);
```

**Parameters**

* **usec**:  Time, [us]



**Example code**

```c++
delay_us(1000);  // delay time : 1[msec]
```



### SysTick_Enable\(\)

Enables System Ticker

```c++
void SysTick_Enable(void);
```



### SysTick_Disable\(\)

Disables System Ticker

```c++
void SysTick_Disable(void);
```





## Timer

### Header File

 `#include "ecTIM.h"`


```c++
#include "stm32f411xe.h"

#ifndef __EC_TIM_H
#define __EC_TIM_H

#ifdef __cplusplus
	extern "C" {
#endif	/* __cplusplus */

// Count Type
#define UP_CNT 		0
#define DOWN_CNT 	1

// Edge Type
#define RISE_TIM 0
#define FALL_TIM 1
#define BOTH_TIM 2

/* Timer Coonfiguration */
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_CNT_DIR(TIM_TypeDef*TIMx, uint32_t direction);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_Interrupt_init(TIM_TypeDef* TIMx, uint32_t msec, uint32_t priority);
void TIM_Interrupt_enable(TIM_TypeDef* TIMx);
void TIM_Interrupt_disable(TIM_TypeDef* TIMx);
uint32_t Is_UIF(TIM_TypeDef* TIMx);
void Clear_UIF(TIM_TypeDef* TIMx);

//Input Capture
typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		//int Timer Channel
	int ICnum;  	//int IC number
} IC_t;

void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin, uint32_t priority);
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);
void ICAP_counter_us(IC_t *ICx, int usec);
uint32_t Is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void Clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void ICAP_pinmap(IC_t *timer_pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### TIM_init\(\)

Initializes Timer with TIMx and Timer period.

```c++
void TIM_init(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)
* **msec**: Period of Timer



**Example code**

```c++
TIM_init(TIM2, 1);  // Select TIM2 and 1ms period
```



### TIM_CNT_DIR\(\)

Initializes Timer with TIMx and Timer period.

```c++
void TIM_CNT_DIR(TIM_TypeDef*TIMx, uint32_t direction);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)
* **direction**: Direction of Counter, UP_CNT or DOWN_CNT



**Example code**

```c++
TIM_CNT_DIR(TIM2, DOWN_CNT);  // Select TIM2 and 1ms period
```



### TIM_period_us(\)

Sets timer counter period of usec

```c++
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t usec);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)
* **usec**: Period of Timer (unit: micro second)
      Default : System Clock = 84 MHz     // PSC + 1 = 84
                      CNT = 1 MHz                       // ARR + 1 = usec * 1  
       => usec : 1 ~ 65000  (1us ~ 65ms)  // because ARR is 16-bit  (But, TIM2 and TIM5 have 32-bit ARR)
       

**Example code**

```c++
TIM_period_us(TIM2, 10);  // Select TIM2 and 10us period (100kHz)
```



### TIM_period_ms(\)

Sets timer counter period of msec

```c++
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)
* **msec**: Period of Timer (unit: milli second)
      Default : System Clock = 84 MHz     // PSC + 1 = 84
                     CNT = 1 MHz                        // ARR + 1 = msec * 1000 
      => msec : 1 ~ 65  (1ms ~ 65ms)       // because ARR is 16-bit (But, TIM2 and TIM5 have 32-bit ARR)



**Example code**

```c++
TIM_period_ms(TIM2, 1);  // Select TIM2 and 1ms period (1kHz)
```



### TIM_Interrupt_init\(\)

Initializes timer interrupt. (Timer is no output mode)

```c++
void TIM_Interrupt_init(TIM_TypeDef* TIMx, uint32_t msec, uint32_t priority);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)
* **msec**: Period of Timer (unit: milli second)
* **priority**: Priority of IRQ Handler



**Example code**

```c++
TIM_Interrupt_init(TIM2, 1, 5);  // TIM2 interrupt initializes, period: 1msec, priority number = 5
```



### TIM_Interrupt_enable\(\)

Enable update interrupt

```c++
void TIM_Interrupt_enable(TIM_TypeDef* TIMx);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)



**Example code**

```c++
TIM_Interrupt_enable(TIM2);  // Enable TIM2 update interrupt
```





### TIM_Interrupt_disable\(\)

Disable update interrupt

```c++
void TIM_Interrupt_disable(TIM_TypeDef* TIMx);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)



**Example code**

```c++
TIM_Interrupt_disable(TIM2);  // Disable TIM2 update interrupt
```



### Is_UIF\(\)

Returns UIF (Update Interrupt Flag)

```c++
uint32_t Is_UIF(TIM_TypeDef* TIMx);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)



**Example code**

```c++
if (Is_UIF(TIM2)){			// Check UIF 
    // something acts		// Whenever timer interrupts, this line is read.
    Clear_UIF(TIM2);
}
```



### Clear_UIF\(\)

Clears UIF (Update Interrupt Flag)

```c++
void Clear_UIF(TIM_TypeDef* TIMx);
```

**Parameters**

* **TIMx**:  Timer number(x = 1, 2, 3, 4, 5, 9, 10, 11)



**Example code**

```c++
if (Is_UIF(TIM2)){
    // something acts
    Clear_UIF(TIM2);	// It is essential to clear UIF after reading UIF = 1
}
```



### ICAP_init\(\)

Initializes input capture mode of timer.

```c++
void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin);
```

**Parameters**

* **ICx**:  IC_t structure variable.
* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15 



**Example code**

```c++
ICAP_init(&echo, GPIOB, 10);		// ICAP init as PB10 as input caputre
```



### ICAP_setup\(\)

Configure selecting TIx-ICy and edge type.

```c++
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type, uint32_t priority);
```

**Parameters**

* **ICx**:  IC_t structure variable.
* **Port:**  IC number,  1~4
* **edge_type**:   RISE, FALL, or BOTH
* **priority**:   Priority of IRQHandler



**Example code**

```c++
ICAP_setup(&echo, 3, RISE, 4);   	// TIM2_CH3 as IC3 , rising edge detect
```



### ICAP_counter_us(\)

Sets the counter clock period with micro sec.

```c++
void ICAP_counter_us(IC_t *ICx, int usec);
```

**Parameters**

* **ICx**:  IC_t structure variable.

* **usec**: micro sec.

  

**Example code**

```c++
ICAP_counter_us(&echo, 10);   	// ICAP counter step time as 10us
```



### Is_CCIF(\)

Checks whether capture/compare interrupt flag is set.

```c++
uint32_t Is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters**

* **TIMx**:  Timer number,(x = 1, 2, 3, 4, 5, 9, 10, 11)

* **ccNum**: IC number, 1 ~ 4

  

**Example code**

```c++
if(Is_CCIF(TIM2,3)) 	// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
```



### Clear_CCIF(\)

Clears capture/compare interrupt flag.

```c++
void Clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
```

**Parameters**

* **TIMx**:  Timer number,(x = 1, 2, 3, 4, 5, 9, 10, 11)

* **ccNum**: IC number, 1 ~ 4

  

**Example code**

```c++
Clear_CCIF(TIM2,3);                 // clear capture/compare interrupt flag 
```



## PWM

### Header File

 `#include "ecPWM.h"`


```c++
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecTIM.h"
#include "ecSysTick.h"

#ifndef __EC_PWM_H
#define __EC_PWM_H

#ifdef __cplusplus
	extern "C" {
#endif	/* __cplusplus */

typedef struct{
	GPIO_TypeDef *port;
	int pin;
	TIM_TypeDef *timer;
	int ch;
} PWM_t;

/* PWM Configuration */
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);
void PWM_period_ms(PWM_t *pwm, uint32_t msec);
void PWM_period_us(PWM_t *pwm, uint32_t usec);
void PWM_PulseWidth_ms(PWM_t *pwm, float pulse_width_ms);
void PWM_PulseWidth_us(PWM_t *pwm, float pulse_width_us);
void PWM_duty(PWM_t *pwm, float duty);
void PWM_PinMap(PWM_t *PWM_pin);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

```




### PWM_init\(\)

Initializes PWM with Timer according to GPIO pin

```c++
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin);
```

**Parameters**

* **pwm**:  Variable about pwm
* **port**: Port number, GPIO A~H
* **pin**: Pin number, 0~15



**Example code**

```c++
PWM_t pwm;					// define variable of PWM_t type
PWM_init(&pwm, GPIOA, 1);  	// Initialize pwm on PA1
```



### PWM_period_ms\(\)

Initializes timer counter period(=PWM period) of msec 

```c++
void PWM_period_ms(PWM_t *pwm, uint32_t msec);
```

**Parameters**

* **pwm**:  Variable about pwm

* **msec**: Period of PWM (unit: milli second) 

  ​            Default : System Clock = 84 MHz     // PSC + 1 = 84
  ​                           CNT = 1 MHz                        // ARR + 1 = msec * 1000 
  ​            => msec : 1 ~ 65  (1ms ~ 65ms)       // because ARR is 16-bit (But, TIM2 and TIM5 have 32-bit ARR) 

  

**Example code**

```c++
PWM_period_ms(&pwm, 20);  	// Period of PWM : 20ms
```



### PWM_period_us\(\)

Initializes timer counter period(=PWM period) of usec 

```c++
void PWM_period_us(PWM_t *pwm, uint32_t usec);
```

**Parameters**

* **pwm**:  Variable about pwm
* **usec**: Period of PWM (unit: micro second)
      Default : System Clock = 84 MHz     // PSC + 1 = 84
                      CNT = 1 MHz                       // ARR + 1 = usec * 1  
       => usec : 1 ~ 65000  (1us ~ 65ms)  // because ARR is 16-bit  (But, TIM2 and TIM5 have 32-bit ARR)



**Example code**

```c++
PWM_period_us(&pwm, 10);  	// Period of PWM : 10us
```



### PWM_PulseWidth_ms\(\)

Inputs pulse width of PWM

```c++
void PWM_PulseWidth_ms(PWM_t *pwm, float pulse_width_ms);
```

**Parameters**

* **pwm**:  Variable about pwm
* **pulse_width_ms**: Pulse width (unit: milli second), 
      0.0 ~ period of PWM, or range according to hardware condition



**Example code**

```c++
PWM_period_ms(&pwm, 20);  		// Period of PWM : 20ms
PWM_PulseWidth_ms(&pwm, 0.5);  	// Pulse width : 0.5ms 	=> duty ratio : 0.5/20
```



### PWM_duty\(\)

Inputs duty ratio of PWM

```c++
void PWM_duty(PWM_t *pwm, float duty);
```

**Parameters**

* **pwm**:  Variable about pwm
* **duty**: duty, 0.0 ~ 1.0



**Example code**

```c++
PWM_period_ms(&pwm, 20);  	// Period of PWM : 20ms
PWM_duty(&pwm, 0.1);  		// duty ratio : 0.1   => Pulse width : 20ms * 0.1 = 2ms
```



### PWM_PinMap\(\)

Matches output port and pin for TIMx

```c++
void PWM_PinMap(PWM_t *PWM_pin);
```

**Parameters**

* **pwm**:  Variable about pwm



**Example code**

```c++
PWM_PinMap(pwm);   // pwm->timer and pwm->ch is assigned according to output port and pin
```



## Stepper Motor

### Header File

 `#include "ecStepper.h"`


```c++
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	

// Direction
#define CW	1		// Clock-wise
#define CCW	0		// Counter Clock-wise
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
   GPIO_TypeDef *port1;
   int pin1;
	 GPIO_TypeDef *port2;
   int pin2;
	 GPIO_TypeDef *port3;
   int pin3;
	 GPIO_TypeDef *port4;
   int pin4;
	 int _step_num;
} Stepper_t;

void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
void Stepper_pinOut (uint32_t state, int mode);
void Stepper_setSpeed (long whatSpeed);
void Stepper_step(int steps, int direction, int mode); 
//void Stepper_run (int direction, int mode); 
void Stepper_stop (void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### Stepper_init\(\)

Initializes stepper motor (4 phases) by setting GPIO pins.

```c++
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
```

**Parameters**

* **port**: Port number, GPIO A~H
* **pin**: Pin number, 0~15



**Example code**

```c++
Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); 	// Stepper GPIO pin initialization
```



### Stepper_pinOut\(\)

Ouputs High or Low value to pins. 

```c++
void Stepper_pinOut (uint32_t state, int mode);
```

**Parameters**

* **state**: state number, Full-step (S0~S4), Half-step (S0~S7)
* **mode**: Step mode, FULL or HALF



**Example code**

```c++
Stepper_pinOut(S0, FULL);
```



### Stepper_setSpeed\(\)

Calculates the step delay

```c++
void Stepper_setSpeed (long whatSpeed);
```

**Parameters**

* **whatSpeed**: the speed of stepper motor, rpm



**Example code**

```c++
Stepper_setSpeed(2);	// set the speed to 2rpm
```



### Stepper_step\(\)

Moves as much as the desired steps by direction and mode.

```c++
void Stepper_step(int steps, int direction, int mode); 
```

**Parameters**

* **steps**: the amount of step you desired, (Full-step: 2048 steps - 1 revolution)
* **direction**: the direction of rotation, CW or CCW (clock-wise or counter clock-wise)
* **mode**: the step mode, FULL or HALF



**Example code**

```c++
Stepper_step(2048, CCW, FULL);	// 1 revolution by Full-step and CCW direction
```



### Stepper_stop\(\)

Stops the stepper motor. Normally, it is used with the button interrupt for emergency stop.

```c++
void Stepper_stop (void);
```



**Example code**

```c++
Stepper_stop();
```



## ADC

### Header File

 `#include "ecADC.h"`


```c++
#ifndef __MY_ADC_H
#define __MY_ADC_H
#include "stm32f411xe.h"

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
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode, int groupmode, int priority);
void ADC_TRGO(TIM_TypeDef* TIMx, int msec, int edge, int groupmode);
void ADC_continue(int contmode); 			
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

#endif
```




### ADC_init\(\)

Initializes ADC mode by setting GPIO pins.

```c++
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode, int groupmode, int priority);
```

**Parameters**

* **port**: Port number, GPIO A~H
* **pin**: Pin number, 0~15
* **trigmode**: Trigger mode, SW or TRGO
* **groupmode**: Group mode, REGULAR or INJECT
* **priority**: Priority of IRQHandler



**Example code**

```c++
// Initializes PB_0 pin to ADC mode with Software Trigger and regular channel
ADC_init(GPIOB, 0, SW, REGULAR); 
```



### ADC_TRGO\(\)

Initializes ADC mode with Hardware trigger.

```c++
void ADC_TRGO(TIM_TypeDef* TIMx, int msec, int edge, int groupmode);
```

**Parameters**

* **TIMx**: Timer number, TIM2 or TIM3
* **msec**: Period of the Timer
* **edge**: Trigger detection, RISE, FALL, or BOTH
* **groupmode**: Group mode, REGULAR or INJECT



**Example code**

```c++
// TRGO Initialize : TIM2, 1msec, Rising edge, Regular channel
ADC_TRGO(TIM2, 1, RISE, REGULAR); 
```



### ADC_continue\(\)

Set the conversion mode: Single or Continuous

```c++
void ADC_continue(int contmode);
```

**Parameters**

* **contmode**: Conversion mode, CONT or SINGLE



**Example code**

```c++
ADC_continue(CONT); 	// Enable Continuous conversion mode
```



### ADC_sequence\(\)

Set the conversion sequence of the regular channels

```c++
void ADC_sequence(int length, int *seq); 
```

**Parameters**

* **length**: The number of channels
* **seq**: Sequence array (list of channels)



**Example code**

```c++
int sequence[2] = { 8, 9 };
ADC_init(GPIOB, 0, TRGO, REGULAR);		// PB_0 : channel 8
ADC_init(GPIOB, 1, TRGO, REGULAR);		// PB_1 : channel 9
ADC_sequence(2, sequence);				// set the conversion sequcne of 2 channels
```



### ADC_sequence_Inject\(\)

Set the conversion sequence of the injected channels

```c++
void ADC_sequence_Inject(int length, int *seq);
```

**Parameters**

* **length**: The number of channels (*length must be 4)
* **seq**: Sequence array (list of channels)



**Example code**

```c++
int sequence[2] = { 8, 9 };
ADC_init(GPIOB, 0, TRGO, INJECT);		// PB_0 : channel 8
ADC_init(GPIOB, 1, TRGO, INJECT);		// PB_1 : channel 9
ADC_sequence(4, sequence);		// set the conversion sequcne of 2 channels
```



### ADC_start\(\)

Start ADC mode (Enable ADON, SW Trigger)

```c++
void ADC_start(ADC_TypeDef *ADCx, int groupmode);
```

**Parameters**

* **ADCx**: ADC number, ADC1
* **groupmode**: Group mode, REGULAR or INJECT



**Example code**

```c++
ADC_start(ADC1); 	// Start ADC
```



### Is_ADC_EOC\(\)

Check whether the end of conversion is triggered. When the ADCx->DR is read, EOC flag is cleared automatically.

```c++
uint32_t Is_ADC_EOC(void);
```

**Example code**

```c++
if (Is_ADC_EOC())
    value = ADC_read();
```



### Is_ADC_JEOC\(\)

Check whether the end of conversion at all injected channels is triggered. JEOC flag is cleared by software.

```c++
uint32_t Is_ADC_JEOC(void);
```

**Example code**

```c++
if (Is_ADC_JEOC()){
    value = ADC_read_Inject(1);
    Clear_ADC_JEOC();
}
```



### Clear_ADC_JEOC\(\)

EOC flag is cleared by software.

```c++
void Clear_ADC_JEOC(void);
```

**Example code**

```c++
if (Is_ADC_JEOC()){
    value = ADC_read_Inject(1);
    Clear_ADC_JEOC();
}
```



### Is_ADC_OVR\(\)

Check whether overrun is triggered. It is cleared by software.

```c++
uint32_t Is_ADC_OVR(void);
```

**Example code**

```c++
if(Is_ADC_OVR())
    Clear_ADC_OVR();
```



### Clear_ADC_OVR\(\)

clear the overrun register.

```c++
void Clear_ADC_OVR(void);
```

**Example code**

```c++
if(Is_ADC_OVR())
    Clear_ADC_OVR();
```



### ADC_read\(\)

read the data register(DR)

```c++
uint32_t ADC_read(void);
```

**Example code**

```c++
if (Is_ADC_EOC())
    value = ADC_read();
```



### ADC_read_Inject\(\)

read the data register(JDRn) on the injected channels

```c++
uint32_t ADC_read_Inject(int JDRn);
```

* **JDRn**: JDR number, 1~4



**Example code**

```c++
if (Is_ADC_JEOC()){
    value1 = ADC_read_Inject(1);
    value2 = ADC_read_Inject(2);
    Clear_ADC_JEOC();
}
```



### ADC_pinmap\(\)

Assign the channel number from GPIO pin

```c++
uint32_t ADC_pinmap(GPIO_TypeDef *Port, int Pin);
```

**Parameters**

* **Port**: GPIO pin number, GPIO A~H
* **Pin**: Pin number



**Example code**

```c++
CHn = ADC_pinmap(GPIOB, 0);		// ADC Channel <-> Port/Pin mapping
```



## USART

### Header File

 `#include "ecUSART.h"`


```c++
#ifndef __EC_USART_H
#define __EC_USART_H

#include "stm32f411xe.h"
#include <stdio.h>
#include "ecGPIO.h"
#include "ecRCC.h"

#define POL 0
#define INT 1

// You can modify this
#define BAUD_9600		0
#define BAUD_19200	1
#define BAUD_57600	2
#define BAUD_115200 3
#define BAUD_921600 4


// ********************** USART 2 (USB) ***************************
	// PA_2 = USART2_TX
	// PA_3 = USART2_RX
	// Alternate function(AF7), High Speed, Push pull, Pull up
	// APB1
// **********************************************************

// ********************** USART 1 ***************************
	// PB_6 = USART1_TX (default)	// PA_9  (option)
	// PB_3 = USART1_RX (default)	// PA_10 (option)
	// APB2
// **********************************************************

// ********************** USART 6 ***************************
	// PA_11 = USART6_TX (default)	// PC_6  (option)
	// PA_12 = USART6_RX (default)	// PC_7 (option)
	// APB2
// **********************************************************


/* Given to Students */ 
void UART2_init();
void USART_write(USART_TypeDef* USARTx, uint8_t* buffer, uint32_t nBytes);
void USART_delay(uint32_t us);  

/* Exercise*/
void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, int baud);
void USART_init(USART_TypeDef* USARTx, int baud);  															// default pins. 
uint8_t USART_getc(USART_TypeDef * USARTx);			
uint32_t Is_USART_RXNE(USART_TypeDef * USARTx);

										

#endif
```




### UART2_init\(\)

Initializes UART2 simply.

```c++
void UART2_init();
```



**Example code**

```c++
UART2_init(); 
```



### USART_write\(\)

Transmit the data(or message) to other communication module through USART

```c++
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes)
```

**Parameters**

* **USARTx**: USART number, USART1, 2, 6
* **buffer**: message sent by USARTx
* **nBytes**: size of message



**Example code**

```c++
char msg[11];
sprintf(msg, "%s", "Hello World");
USART_write(USART2, &msg, 11); 
```



### USART_delay\(\)

Delay the time

```c++
void USART_delay(uint32_t us)
```

**Parameters**

* **us**: usec



**Example code**

```c++
USART_delay(1000);	//delay 1msec.
```



### USART_init\(\)

Initializes USARTx with baud rate and default settings.

```c++
USART_init(USART_TypeDef * USARTx, int baud)
```

**Parameters**

* **USARTx**: USART number, USART1, 2, 6
* **baud**: baud rate, 9600, 19200, 57600, 115200,  ...



**Example code**

```c++
USART_init(USART6,9600);	//delay 1msec.
```



### USART_begin\(\)

Initializes USARTx with detailed settings.

```c++
USART_begin(USART_TypeDef * USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, int baud)
```

**Parameters**

* **USARTx**: USART number, USART1, 2, 6
* **GPIO_TX**: Port number, GPIOA~H
* **pinTX**:  Pin number for TX
* **GPIO_RX**: Port number, GPIOA~H
* **pinRX**:  Pin number for RX
* **baud**: baud rate, 9600, 19200, 57600, 115200, ...



**Example code**

```c++
USART_begin(USART6, GPIOA, 11, GPIOA, 12, 9600);
```

