		#include"stm32f10x.h"
		#include"stdio.h"
		#include"stdlib.h"
		#include"stdarg.h"
		#include"string.h"


		#define DEBUG_USART USART3
		#define CPU_FREQUENCY_IN_HZ 8000000


		void USART3_Init (void);
		void PRINT_fN(char *u,...);
		void delay(int );
		
		void USART3_Init (void)
				{
					RCC->APB1ENR |= RCC_APB1ENR_USART3EN ;//(1<<18);  // To Enable the USART3 Clock
					RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;//(1<<3) ;  // To Enable the PORT-B Clock
					
					GPIOB->CRH = 0;
					//PB10 TX
					GPIOB->CRH |= (3<<8);     // 10MHz output TX PA9
					GPIOB->CRH |= (1<<11);    // AF - Push/Pull output
				 
					
				//USART2->BRR = (8<<0) | (19<<4);   // Baud rate of 115200, PCLK1 at 36MHz
					USART3->BRR = (6<<0) | (234<<4);    // Baud rate of 9600, PCLK1 at 36MHz
					
				//	USART3->CR1 |= (1<<5) | (1<<7); // USART1_Both RxIE_TxIE Enable
					
					USART3->CR1 |= (1<<3)|(1<<13);// TX,USART1 Enable
		 
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
													 while( !( USART3->SR & USART_SR_TXE ));
													 USART3->DR = storing_buff[loop];  //send it back out
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
