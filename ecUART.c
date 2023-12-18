#include "ecUART.h"
#include "ecGPIO.h"
#include <math.h>

// ********************** DO NOT MODIFY HERE ***************************
// 
// Implement a dummy __FILE struct, which is called with the FILE structure.
//#ifndef __stdio_h
struct __FILE {
    //int dummy;
		int handle;

};

FILE __stdout;
FILE __stdin;
//#endif

// Retarget printf() to USART2
int fputc(int ch, FILE *f) { 
  uint8_t c;
  c = ch & 0x00FF;
  USART_write(USART2, (uint8_t *)&c, 1);
  return(ch);
}

// Retarget getchar()/scanf() to USART2  
int fgetc(FILE *f) {  
  uint8_t rxByte;
  rxByte = USART_getc(USART2);
  return rxByte;
}


void UART2_init(){
	// Enable the clock of USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART 2 clock (APB1 clock: AHB clock / 2 = 42MHz)
	
	// Enable the peripheral clock of GPIO Port
	RCC->AHB1ENR |=   RCC_AHB1ENR_GPIOAEN;	
	
	// ********************** USART 2 ***************************
	// PA2 = USART2_TX
	// PA3 = USART2_RX
	// Alternate function(AF7), High Speed, Push pull, Pull up
	// **********************************************************
	int TX_pin = 2;
	
	GPIOA->MODER   &= ~(0xF << (2*TX_pin));	// Clear bits
	GPIOA->MODER   |=   0xA << (2*TX_pin);  // Alternate Function(10)    			
	GPIOA->AFR[0]  |=   0x77<< (4*TX_pin);  // AF7 - USART2     	
	GPIOA->OSPEEDR |=   0xF<<(2*TX_pin); 		// High speed (11)			 	
	GPIOA->PUPDR   &= ~(0xF<<(2*TX_pin));
	GPIOA->PUPDR   |=   0x5<<(2*TX_pin);    // Pull-up (01)			
	GPIOA->OTYPER  &=  ~(0x3<<TX_pin) ; 		// push-pull (0, reset)
	
	USART_TypeDef *USARTx = USART2;
	 // No hardware flow control, 8 data bits, no parity, 1 start bit and 1 stop bit      
   USARTx->CR1 &= ~USART_CR1_UE;  // Disable USART
   
   // Configure word length to 8 bit
   USARTx->CR1 &= ~USART_CR1_M;       // M: 0 = 8 data bits, 1 start bit   
   USARTx->CR1 &= ~USART_CR1_PCE;   // No parrity bit 
   USARTx->CR2 &= ~USART_CR2_STOP;  // 1 stop bit    
   
   // Configure oversampling mode (to reduce RF noise)
   USARTx->CR1 &= ~USART_CR1_OVER8;  // 0 = oversampling by 16   
                                 
   // CSet Baudrate to 9600 using APB frequency (42MHz)
   // If oversampling by 16, Tx/Rx baud = f_CK / (16*USARTDIV),  
   // If oversampling by 8,  Tx/Rx baud = f_CK / (8*USARTDIV)
   // USARTDIV = 42MHz/(16*9600) = 237.4375
   
   //USARTx->BRR  = 42000000/ baud_rate;
   float Hz = 42000000;

   float USARTDIV = (float)Hz/16/9600;
   uint32_t MNT = (uint32_t)USARTDIV;
   uint32_t FRC = round((USARTDIV - MNT) * 16);
   if (FRC > 15) {
      MNT += 1;
      FRC = 0;
   }
   USARTx->BRR  |= (MNT << 4) | FRC;

   USARTx->CR1  |= (USART_CR1_RE | USART_CR1_TE);   // Transmitter and Receiver enable
   USARTx->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
   USARTx->CR1 |= USART_CR1_RXNEIE;                 // Received Data Ready to be Read Interrupt

   USARTx->CR1  |= USART_CR1_UE; // USART enable

		NVIC_SetPriority(USART2_IRQn, 1);		// Set Priority to 1
		NVIC_EnableIRQ(USART2_IRQn);			 	// Enable interrupt of USART2 peripheral

}


void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	int i;
	for (i = 0; i < nBytes; i++) {
		// wait until TXE (TX empty) bit is set
		while (!(USARTx->SR & USART_SR_TXE));  
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->DR = buffer[i] & 0xFF;
		USART_delay(300);
	}
	
	// wait until TC bit is set
	while (!(USARTx->SR & USART_SR_TC));		
	// TC bit clear
	USARTx->SR &= ~USART_SR_TC;	
	
}  

void USART_delay(uint32_t us) {
   uint32_t time = 100*us/7;    
   while(--time);   
}
// **********************************************************


void USART_init(USART_TypeDef * USARTx, int baud){
	
	GPIO_TypeDef* GPIO_Tx;
	GPIO_TypeDef* GPIO_Rx;
	int pin_Tx;
	int pin_Rx;
	
	// set default pins for USART2, USART1, USART6
	if (USARTx == USART2){
		GPIO_Tx = GPIOA;	pin_Tx = 2;
		GPIO_Rx = GPIOA; 	pin_Rx = 3;
	}
	else if (USARTx == USART1){
		GPIO_Tx = GPIOA;	pin_Tx = 9;		// PB6 or PA9
		GPIO_Rx = GPIOA; 	pin_Rx = 10;		// PB3 or PA10
	}
	else if (USARTx == USART6){
		GPIO_Tx = GPIOA;	pin_Tx = 11;		// PA11 or PC6
		GPIO_Rx = GPIOA; 	pin_Rx = 12;		// PA12 or PC7
	}
	
		
	// USART_begin(USARTx, GPIO_TX, pinTX, ..., baud);
	USART_begin(USARTx, GPIO_Tx, pin_Tx, GPIO_Rx, pin_Rx, baud);
	
}

void USART_begin(USART_TypeDef * USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, int baud){
//1. GPIO Pin for TX and RX	
	// Initialize GPIO pins for Tx, Rx 
	// AF, High Speed, Push pull, Pull up
	GPIO_init(GPIO_TX, pinTX, AF);
	GPIO_ospeed(GPIO_TX, pinTX, HIGH_SPEED);
	GPIO_otype(GPIO_TX, pinTX, PUSH_PULL);
	GPIO_pupdr(GPIO_TX, pinTX, NO_PUPD);
	
	GPIO_init(GPIO_RX, pinRX, AF);
	GPIO_ospeed(GPIO_RX, pinRX, HIGH_SPEED);
	GPIO_otype(GPIO_RX, pinRX, PUSH_PULL);
	GPIO_pupdr(GPIO_RX, pinRX, NO_PUPD);
	
	// AF7: USART1, USART2 , AF8: USART6
	GPIO_TX->AFR[pinTX>>3] &= ~( 15UL << (4*(pinTX & 7)) );
	GPIO_RX->AFR[pinRX>>3] &= ~( 15UL << (4*(pinRX & 7)) );
	
	if (USARTx == USART2 || USARTx == USART1){
		GPIO_TX->AFR[pinTX>>3] |= 0x7UL << (4*(pinTX & 7));	// [pin/8] = [pin>>3], (pin%8) = (pin&7)
		GPIO_RX->AFR[pinRX>>3] |= 0x7UL << (4*(pinRX & 7));
	}
	else if (USARTx == USART6){
		GPIO_TX->AFR[pinTX>>3] |= 0x8UL << (4*(pinTX & 7));	// [pin/8] = [pin>>3], (pin%8) = (pin&7)
		GPIO_RX->AFR[pinRX>>3] |= 0x8UL << (4*(pinRX & 7));
	}

	// 2. USARTx (x = 2, 1, 6) configuration
	// Enable USART peripheral clock
	if (USARTx == USART2)
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART 2 clock (APB1 clock: AHB clock / 2 = 42MHz)
	else if (USARTx == USART1)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	else if (USARTx == USART6)
		RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	
	// Disable USART
	USARTx->CR1 &= ~USART_CR1_UE;  
	
	// No Parity / 8-bit word length / Oversampling x16 / 1 stop bit
	USARTx->CR1 &= ~USART_CR1_PCE;   // No parrity bit 
	USARTx->CR1 &= ~USART_CR1_M;     // M: 0 = 8 data bits, 1 start bit 
	USARTx->CR1 &= ~USART_CR1_OVER8;  // 0 = oversampling by 16     
	USARTx->CR2 &= ~USART_CR2_STOP;  // 1 stop bit    

															 
	// CSet Baudrate to 9600 using APB frequency (42MHz)
	// If oversampling by 16, Tx/Rx baud = f_CK / (16*USARTDIV),  
	// If oversampling by 8,  Tx/Rx baud = f_CK / (8*USARTDIV)
	// USARTDIV = 42MHz/(16*9600) = 237.4375

	//USARTx->BRR  = 42000000/ baud_rate;
	
	float Hz = 84000000;
	if (USARTx == USART2)	Hz = 42000000;

	float USARTDIV = (float)Hz/(8.0 * (2 - 0) * baud); // 0: USART_CR_OVER8
	uint32_t MNT = (uint32_t)USARTDIV;
	uint32_t FRC = round((USARTDIV - MNT) * 16);
	if (FRC > 15) {
		MNT += 1;
		FRC = 0;
	}
	USARTx->BRR  |= (MNT << 4) | FRC;
//	USARTx->BRR = 0x222E;
//	USARTx->BRR = 0x1117;
	
	// Enable TX, RX, and USARTx 
	USARTx->CR1  |= (USART_CR1_RE | USART_CR1_TE);   // Transmitter and Receiver enable

//	USARTx->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;	 // DMA Transmitter and Receiver enable
	
// 3. Read USARTx Data (Interrupt)
	// Set the priority and enable interrupt
	USARTx->CR1 |= USART_CR1_RXNEIE;  	// RXNE interrupt, Received Data Ready to be Read Interrupt

	USARTx->CR1  |= USART_CR1_UE; // USART enable

	int USARTx_IRQn;
	if (USARTx == USART1)
		USARTx_IRQn = USART1_IRQn;
	else if (USARTx == USART2)
		USARTx_IRQn = USART2_IRQn;
	else if (USARTx == USART6)
		USARTx_IRQn = USART6_IRQn;

	NVIC_SetPriority(USARTx_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(USARTx_IRQn);			 	// Enable interrupt of USART2 peripheral
}

uint8_t USART_getc(USART_TypeDef * USARTx){
	// Wait until RXNE (RX not empty) bit is set by HW -->Read to read
	while ( (USARTx->SR & USART_SR_RXNE) != USART_SR_RXNE );  
	// Reading USART_DR automatically clears the RXNE flag 
	return ((uint8_t)(USARTx->DR & 0xFF)); 	
}

uint32_t Is_USART_RXNE(USART_TypeDef * USARTx){
	return (USARTx->SR & USART_SR_RXNE);
}