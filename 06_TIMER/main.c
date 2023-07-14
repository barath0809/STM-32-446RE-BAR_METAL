/*
 * 1.To Enable the TIM_1 clock and GPIOA clock 
 * 2.Configure TIM1_CH1 -> PA8 as output 
 * 3.TIM1 prescaler
 * 4.TIM1 ARR
 * 5.TIM1 CR1
 * 6.TIM1 o/p
 */
  
 
 #include "stm32f10x.h"
 #include "HUART_3.h"
int main(void)
{
	USART3_Init();
	//To Enable the TIM_1 clock and GPIOA clock 
   RCC->APB2ENR |= (1<<2)|(1<<11) ;
	
	 //Configure TIM1_CH1 -> PA8 as output 50 Mhz| 
   GPIOA->CRH |= (3<<0);
	
	//TIM1 Prescaler
	 TIM1->PSC = 65535 ;
	
	//TIM1 ARR 
	 TIM1->ARR = 6000;
	
	//TIM1 CR1
//	 TIM1->CR1 = TIM_CR1_CEN ;  // UP_Counter
//	 TIM1->CR1 = TIM_CR1_CEN |TIM_CR1_DIR ;  // Down_Counter
	 TIM1->CR1 = TIM_CR1_CEN | TIM_CR1_CMS_0 ;  // UP_Down_Counter( Center Alignment Mode )

	
	
			while(1)
			{
				PRINT_fN("TIM1-> %d \r\n",TIM1->CNT);
				delay(100);
			}
			
}