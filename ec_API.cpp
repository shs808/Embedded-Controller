#include "ec_API.h"

EC_Ticker::EC_Ticker(uint32_t Tick_period_ms){
	SysTick_init(Tick_period_ms);
}
	
uint32_t EC_Ticker::read(){
	return SysTick_val();
}
void EC_Ticker::Delay_ms(uint32_t msec){
	delay_ms(msec);
}

void EC_Ticker::reset(){
	SysTick_reset();
}	

EC_Interrupt::EC_Interrupt(GPIO_TypeDef* Port, int pin, int TrigType, int priority){
	
	mode_t = EC_DIN;
	EXTI_init(Port, pin, TrigType, priority);
	GPIO_init(Port, pin, mode_t);
	Port_t = Port;
	pin_t = pin;
}

int EC_Interrupt::read(){
	val_t = GPIO_read(Port_t, pin_t);
	return val_t;
}

void EC_Interrupt::pupdr(int _pupd){
	GPIO_pupdr(Port_t, pin_t, _pupd);
}

EC_DigitalIn::EC_DigitalIn(GPIO_TypeDef* Port, int pin){
	
	uint8_t mode = EC_DIN; // mode=0
	GPIO_init(Port, pin, mode);
	Port_t = Port;
	pin_t = pin;
	mode_t = mode;
}

int EC_DigitalIn::read(){
	val_t = GPIO_read(Port_t, pin_t);
	return val_t;
}

void EC_DigitalIn::pupdr(int _pupd){
	GPIO_pupdr(Port_t, pin_t, _pupd);
}

EC_DigitalOut::EC_DigitalOut(GPIO_TypeDef* Port, int pin){
	
	uint8_t mode = EC_DOUT;
	GPIO_init(Port, pin, mode);
	Port_t = Port;
	pin_t = pin;
	mode_t = mode;
}

void EC_DigitalOut::write(int _outVal){
	GPIO_write(Port_t, pin_t, _outVal);
}

void EC_DigitalOut::pupdr(int _pupd){
	GPIO_pupdr(Port_t, pin_t, _pupd);
}

void EC_DigitalOut::otype(int _type){
	GPIO_otype(Port_t, pin_t, _type);
}

void EC_DigitalOut::ospeed(int _speed){
	GPIO_ospeed(Port_t, pin_t, _speed);
}

void EC_DigitalOut::Toggle(){
	GPIOA->ODR ^= 1<<pin_t;
}