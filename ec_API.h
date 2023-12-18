#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"

#ifndef __EC_API_H
#define __EC_API_H

#define EC_DOUT  	1
#define EC_DIN 		0

#define EC_PU 1
#define EC_PD 0
#define EC_NONE 0

#define EC_LOW 		0
#define EC_MEDIUM 1
#define EC_FAST 	2
#define EC_HIGH 	3


// SysTick ---------------------------------
class EC_Ticker
{
public:
	EC_Ticker(uint32_t Tick_period_ms);
	~EC_Ticker(){
	}
	uint32_t read();
	void Delay_ms(uint32_t msec);
	void reset();
};

// Interrupt --------------------------------
class EC_Interrupt
{
public:
	EC_Interrupt(GPIO_TypeDef *Port, int pin, int TrigType, int priority);
	~EC_Interrupt(){
	}
	int read();
	
	void pupdr(int _pupd);
	
private:
	GPIO_TypeDef *Port_t;
	int pin_t;
	int mode_t;
	int val_t;
};



// Digital In/Out -----------------------------------
class EC_DigitalIn
{
public:
    EC_DigitalIn(GPIO_TypeDef *Port, int pin);

    ~EC_DigitalIn()
    {
			 delete[] Port_t;
    }

    int read();
		
		void pupdr(int _pupd);
    
    operator int()
    {
        return read();
    }

	private:
			GPIO_TypeDef *Port_t;
			int	pin_t;
			int mode_t;	
			int val_t;	
};



class EC_DigitalOut
{
public:
		EC_DigitalOut(GPIO_TypeDef *Port, int pin);

    ~EC_DigitalOut()
    {
			 delete[] Port_t;
    }

    void write(int _outVal);

  	void pupdr(int _pupd);
		
		void otype(int _type);
		
		void ospeed(int _speed);
		
		void Toggle();
	
		EC_DigitalOut &operator= (int value)
    {
				 write(value);
				 return *this;
    }
		int read()
    {
				return GPIO_read(Port_t, pin_t);
    }
		operator int()
		{
		// Underlying call is thread safe
			return read();
		}

	private:
			GPIO_TypeDef *Port_t;
			int pin_t;
			int mode_t;	

};

#endif


