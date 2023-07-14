

#include"stm32f10x.h"
#include"stdio.h"
#include"stdlib.h"
#include"stdarg.h"
#include"string.h"


#define DEBUG_USART USART1
#define CPU_FREQUENCY_IN_HZ 8000000


void USART1_Init (void);
void PRINT_fN(char *u,...);
void delay(int );
int main(void)
		{
				USART1_Init ();
				
						while(1)
						{
							PRINT_fN("HI,IM FROM USART %d ..... \r\n", 1);
							delay(500);
						}
						
		}

void USART1_Init (void)
		{
			RCC->APB2ENR |= (1<<14);  // To Enable the USART2 Clock
			RCC->APB2ENR |= (1<<2) ;  // To Enable the PORT-A Clock
			GPIOA->CRH = 0;
			//PA9 TX
			GPIOA->CRH |= (1<<4);     // 10MHz output TX PA9
			GPIOA->CRH |= (1<<7);    // Push/Pull output
			
			//PA10 RX
			GPIOA->CRH &= ~(3<<8);  // inputmode  PA10 RX
			GPIOA->CRH |= (1<<11);   //  push/pull input
			
			USART1->BRR |= (1<<0)|(39<<4);        //72MHZ  bus 115200 
//			USART1->BRR |= (12<<0)|(468<<4);     // 72MHZ  bus 9600 In HEX-> 0x1D4C;
			
	    USART1->CR1 |= (1<<5) | (1<<7); // USART1_Both RxIE_TxIE Enable
			
			USART1->CR1 |= (1<<2)|(1<<3)|(1<<13);// RX,TX,USART1 Enable
 
		}
 
 void PRINT_fN(char *user_input,...)
	 {
			 char storing_buff[80];
			 uint8_t loop = 0;
				 #ifdef DEBUG_USART
					 
							va_list arguments;
							va_start(arguments,user_input);
							vsprintf(storing_buff,user_input,arguments);
		 
										 for(loop = 0 ; loop < strlen(storing_buff) ; loop++)
										 {
											 while( !( USART1->SR & USART_SR_TXE ));
											 USART1->DR = storing_buff[loop];  //send it back out
										 }
									 
				#endif
	 }
	 
	 void delay(int milliseconds) {
    // Calculate the number of iterations based on the processor speed
    // and the desired delay time
		 int i;
    int iterations = milliseconds * (CPU_FREQUENCY_IN_HZ / 1000);
  
    for (i = 0; i < iterations; i++) {
        // Do nothing, just waste some CPU cycles
    }
}
