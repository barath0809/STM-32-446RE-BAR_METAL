

#include"stm32f10x.h"
#include"stdio.h"

void USART1_Init (void);
void USART1_IRQHandler(void);

int main(void)
{
	USART1_Init ();
	NVIC_EnableIRQ(USART1_IRQn);
	while(1)
	{

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


 void USART1_IRQHandler(void){
	 
		 //check if we are here because of RXNE interrupt
		 if (USART1->SR & USART_SR_RXNE) //if RX is not empty
		 {
			char temp = USART1->DR; //fetch the data received
			 while( !( USART1->SR & USART_SR_TXE ));
			USART1->DR = temp;  //send it back out
			//while (!(USART1->SR & USART_SR_TC));
  	 }

		 //check if we are here because of TXEIE interrupt
		 if (USART1->SR & USART_SR_TXE) //if RX is not empty
		 {
			//handle transmit completion here

		 }
	 }

