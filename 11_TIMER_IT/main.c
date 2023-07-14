/*
 * 1.To Enable the TIM_1 clock and GPIOA clock 
 * 2.Configure TIM1_CH1 -> PA8 as output 
 * 3.TIM1 prescaler
 * 4.TIM1 ARR
 * 5.TIM1 CR1
 * 6.TIM1 o/p
 */
  
 
 #include "stm32f10x.h"
 #include "UART3.h"
 
void TIM4_IRQHandler(void);
void TIM1_Init(void);
void M_ms(int msec);
void U_ms(int U_s);
 
 
int my_cnt ;

int main(void)
{
	USART3_Init();
	TIM1_Init();
  NVIC_EnableIRQ(TIM4_IRQn);
  // TIM4->CR1 = TIM_CR1_CEN ;	
			while(1)
			{
				PRINT_fN("TIM4-> %d \r\n",my_cnt);
				
//				PRINT_fN("TIM1\r\n");
				U_ms(1000000);
			}
			
}
void TIM1_Init(void)
{
	//To Enable the TIM_1 clock and GPIOA clock 
   RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;
	//TIM1 Prescaler
	 TIM4->PSC = 0 ;
	//TIM1 ARR set the 1 Mhz TIM!
	 TIM4->ARR = 7200;
	//To set over flow condition 
	TIM4->CR1 |= TIM_CR1_URS ;
	 //To Enable the update enble
	 TIM4->DIER |= TIM_DIER_UIE ;
	 // To Enable the update the generation
	 TIM4->EGR |= TIM_EGR_UG ;
}
	
void TIM4_IRQHandler(void)
{
	 my_cnt++;
	 TIM4->SR &= ~(TIM_SR_UIF);
}

//void M_ms(int msec)
//{
//		 TIM2->CR1 = TIM_CR1_CEN ;  // To Enable the TIM1
//	   my_cnt = 0;
//	   while(my_cnt < msec)
//         TIM2->CR1 &= ~ TIM_CR1_CEN ;   // To Disable the TIM1
//}

void U_ms(int U_s)
{
    TIM4->CR1 |= TIM_CR1_CEN;  // Enable TIM4

    my_cnt = 0;
    while (my_cnt < (U_s * 1000))
    {
        // Wait for the desired delay
    }

    TIM4->CR1 &= ~TIM_CR1_CEN;  // Disable TIM4
}



