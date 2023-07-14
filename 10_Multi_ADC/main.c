



#include"stm32f10x.h"
#include"stdio.h"
#include"stdlib.h"
#include"stdarg.h"
#include"string.h"


#define DEBUG_USART USART1
#define CPU_FREQUENCY_IN_HZ 8000000


void USART1_Init (void);
//void ADC1_2_IRQHandler(void);
void PRINT_fN(char *u,...);
void delay(int );

uint16_t adc_val[4]={0};
float v_1,v_2,v_3,v_4;


int main(void)
{
	USART1_Init ();
	//To set the ADC Prescaler dont Exceds 14 Mhz
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6 ;
	
	//To Enable the ADC and GPIOA ,AFIO clocks
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_AFIOEN ;
	
	//To Enable DMA_1 
	RCC->AHBENR |= RCC_AHBENR_DMA1EN ;
	
	//To set the GPIOA_5 as input and push pull
	GPIOA->CRL |= GPIO_CRL_CNF5 ;
	GPIOA->CRL &= ~(GPIO_CRL_CNF5_0) ;
	
	//To set the GPIOA_7 as input and push pull
	GPIOA->CRL |= GPIO_CRL_CNF7 ;
	GPIOA->CRL &= ~(GPIO_CRL_CNF7_0) ;
	
	//To set the GPIOA_7 as input and push pull
	GPIOA->CRL |= GPIO_CRL_CNF4 ;
	GPIOA->CRL &= ~(GPIO_CRL_CNF4_0) ;
	
	//To set the GPIOA_7 as input and push pull
	GPIOA->CRL |= GPIO_CRL_CNF6 ;
	GPIOA->CRL &= ~(GPIO_CRL_CNF6_0) ;
//	//Enable the IT
//	ADC1->CR1 |= ADC_CR1_EOCIE ;
//  //Enable the IT in the NVIC Fuction
//   NVIC_EnableIRQ(ADC1_2_IRQn);  
	
	//Sampling the value
	ADC1->SMPR2 |= ADC_SMPR2_SMP5_0 | ADC_SMPR2_SMP5_1 | ADC_SMPR2_SMP5_2 ;
	ADC1->SMPR2 |= ADC_SMPR2_SMP7_0 | ADC_SMPR2_SMP7_1 | ADC_SMPR2_SMP7_2 ;
	ADC1->SMPR2 |= ADC_SMPR2_SMP4_0 | ADC_SMPR2_SMP4_1 | ADC_SMPR2_SMP4_2 ;
	ADC1->SMPR2 |= ADC_SMPR2_SMP6_0 | ADC_SMPR2_SMP6_1 | ADC_SMPR2_SMP6_2 ;
	
	// To SET how many adc
	ADC1->SQR1 |= (3<<20);//ADC_SQR1_L_1; length = 4
	
	// Sequence
	ADC1->SQR3 |= ADC_SQR3_SQ1_0 | ADC_SQR3_SQ1_2 ; // 1st conversion
	ADC1->SQR3 |= (7<<5); 													// 2nd coverision like priority
	ADC1->SQR3 |= (6<<10);
	ADC1->SQR3 |= (4<<15);
	
	//To Enable the scan mode 
	ADC1->CR1 |= (1<<8);// ADC_CR1_SCAN
	
	// To Enable the DMA
	ADC1->CR2 |= (1<<8);//ADC_CR2_DMA
	
	//To Give Pheripheral Add add of ADC1 to DMA1
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR));
	
	// Tell to DMA memory adress
	DMA1_Channel1->CMAR =(uint32_t)adc_val;
	
	// To assign how many data_in len
	DMA1_Channel1->CNDTR = 4;
	
	//To tell DMA1 CIRCULAR|PHER_SIZE|MEM_SIZE|MEM_INC
	DMA1_Channel1->CCR = DMA_CCR1_CIRC | DMA_CCR1_MINC | DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0 ;
	
	// To Enable the DMA only finaly not before ...
	 DMA1_Channel1->CCR |= DMA_CCR1_EN ;
	 
	// Turn on and set on continious mode 
	ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_ADON ;
	//delay_ms(1);
	
	// Turn on 
	ADC1->CR2 |= ADC_CR2_ADON ;
	//delay_ms(1);

	// To calibrattion
	ADC1->CR2 |= ADC_CR2_CAL ;
	//delay_ms(2);
	
	while(1)
	{				
//				adc_val = ADC1->DR; // Normal Mode
//				// raw data from adc
		    v_1 = ((float)adc_val[0]* (5.0f / 4096.0f));
				v_2 =	((float)adc_val[1]* (5.0f / 4096.0f));
		    v_3 = ((float)adc_val[2]* (5.0f / 4096.0f));
				v_4 =	((float)adc_val[3]* (5.0f / 4096.0f));
				PRINT_fN("VoL_PA5: %.2f, VoL_PA7: %.2f, VoL_PA6: %.2f, VoL_PA4: %.2f  \r\n",v_1 , v_2,v_3 ,v_4);
		
		   // Raw data to voltage conversion
		   // voltage = (float)adc_val * (5.0f / 4096.0f); // Assuming 12-bit ADC

			 // Print the voltage value
			 // PRINT_fN("ADC Voltage: %.2f V\r\n", voltage);
				delay(100);
	}
	
}

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

//  void ADC1_2_IRQHandler(void)
//	{
//		if(ADC1->SR & ADC_SR_EOC)
//		{
//			adc_val = ADC1->DR;  // In Inttrupt mode
//		}
//		
//	}
//	