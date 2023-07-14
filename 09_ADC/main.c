/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*
* 1.To set the ADC Prescaler dont Exceds 14 Mhz
* 2.To Enable the ADC and GPIOA ,AFIO clocks
* 3.To set the GPIOA_5 as input and push pull
*	4.Enable the Intruppt Function
* 5.Enable the IT in the NVIC Fuction
* 6.Sampling the value per sec like mapping
* 7.To swt the ADC priority which port is first
* 8.Turn on and set on continious mode 
* 9.To start the ADC calibrattion
*/

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*																		               HEADER_INCLUDES																																		 */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
#include"stm32f10x.h"
#include"stdio.h"
#include"stdlib.h"
#include"stdarg.h"
#include"string.h"
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*																									  MACRO_DEFINE																																			 */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/

#define DEBUG_USART USART1
#define CPU_FREQUENCY_IN_HZ 8000000

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*                          											  FUNCTION_PROTOTYPE                                                                   */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/

void USART1_Init (void);
void ADC1_2_IRQHandler(void);
void PRINT_fN(char *u,...);
void delay(int );

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*        																				 VARIABLE_DECLARATION                                     														 */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/

int adc_val;
float voltage;

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/

int main()
			{
				USART1_Init ();
				//To set the ADC Prescaler dont Exceds 14 Mhz
				RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6 ;
				
				//To Enable the ADC and GPIOA ,AFIO clocks
				RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN ;
				
				//To set the GPIOA_5 as input and push pull
				GPIOA->CRL |= GPIO_CRL_CNF5 ;
				GPIOA->CRL &= ~(GPIO_CRL_CNF5_0) ;
				
				//Enable the IT
				ADC1->CR1 |= ADC_CR1_EOCIE ;
				//Enable the IT in the NVIC Fuction
				 NVIC_EnableIRQ(ADC1_2_IRQn);  
				
				//Sampling the value
				ADC1->SMPR2 |= ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_2 ;
				
				// Sequence its setting the priority
				ADC1->SQR3 |= ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2 ;
				
				// Turn on and set on continious mode 
				ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ADON ;
				//delay_ms(1);
				
				// Turn on ADC conversion
				ADC1->CR2 |= ADC_CR2_ADON ;
				//delay_ms(1);

				// To start the ADC calibrattion
				ADC1->CR2 |= ADC_CR2_CAL ;
				//delay_ms(2); 
				
													while(1)
													{				
												//				adc_val = ADC1->DR; // Normal Mode
												//				// raw data from adc
												//				PRINT_fN("ADC VAL: %d  \r\n", adc_val );
														
															 // Raw data to voltage conversion
																voltage = (float)adc_val * (5.0f / 4096.0f); // Assuming 12-bit ADC

															 // Print the voltage value
																PRINT_fN("ADC Voltage: %.2f V\r\n", voltage);
																delay(100);
													}
				
			}

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*																								   USART1_Init																																				 */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/

void USART1_Init (void)
		{
			RCC->APB2ENR |= (1<<14);  // To Enable the USART1 Clock
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

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*																										 PRINT_fN																																				   */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
		
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

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*																											delay																																						 */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
	 
	 void delay(int milliseconds) {
    // Calculate the number of iterations based on the processor speed
    // and the desired delay time
		 int i;
    int iterations = milliseconds * (CPU_FREQUENCY_IN_HZ / 1000);
  
    for (i = 0; i < iterations; i++) {
        // Do nothing, just waste some CPU cycles
    }
}

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*																												ADC1_2_IRQHandler																															 */
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/

  void ADC1_2_IRQHandler(void)
	{
		if(ADC1->SR & ADC_SR_EOC)
		{
			adc_val = ADC1->DR;  // In Inttrupt mode
		}
		
	}

/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/
/*```````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````````*/	