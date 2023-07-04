


#include "stm32f10x.h"
#include "string.h"

void UART_Init (void);								// Uart_init_function
void num_tx(uint8_t *,uint8_t );      // Transmit_function 
char _uart_rx(void);									// Rx_Function

char indx ;												    // Character_Variable

int main (void)
{											
	  UART_Init ();                      // UART_Initialization 
		while (1)
		{	
			indx = _uart_rx();
				if(indx == 'A')
				{
					GPIOC->ODR  = (1<<29);
					while( !( USART2->SR & USART_SR_TXE )){} //To transmit the single character 
					USART2->DR  = indx ;
				}
				else
				{
					GPIOC->BSRR  = (1<<13);
				}
		}
}

/*********************************************************************************************************************/
/*  														          		UART_Init_FUNCTION         																						 */
/*********************************************************************************************************************/

void UART_Init (void)
	{
		RCC->APB1ENR |= (1<<17);  	 // Enable USART2 Clock
		RCC->APB2ENR |= (1<<2);  		 // Enable GPIOA Clock
		RCC->APB2ENR |= (1<<4);      // FOR LED PC13_Clock
		GPIOC->CRH   |= (1<<20);     // For PC13_as_OUTPUT
		
		
		GPIOA->CRL = 0;
	//	GPIOA->CRL |= (1<<12);
		GPIOA->CRL |= (3<<8);   		 // output mode 50 MHz for PA2_TX
		GPIOA->CRL |= (2<<10);       // Alternate Func Push Pull For PA2
		
	//	GPIOA->CRL &= ~(3<<12);   // Intput Mode For PA3
	  GPIOA->CRL |= (2<<14);  // Input Pull Up/ Down For PA3	
	  
	
		USART2->CR1 = 0x00;   			 // Clear ALL
		USART2->CR1 |= (1<<13);  		 // UE = 1... Enable USART
		USART2->CR1 |= (1<<3);  		 // TE=1.. Enable Transmitter
		USART2->CR1 |= (1<<2);	   	 // RX_Enable
		/* 
			Tx/ Rx baud = PCLK /(16*Usr_BRR) 
			USARTDIV = 19.53125
			MENSTISSA = 19
			FRACTION = 16*0.53125 = 8.5
		*/

		 //USART2->BRR = (8<<0) | (19<<4);   // Baud rate of 115200, PCLK1 at 36MHz
		 USART2->BRR = (6<<0) | (234<<4);    // Baud rate of 9600, PCLK1 at 36MHz
	}
	
/*********************************************************************************************************************/
/*  														          		TRANSMIT_FUNCTION         																						 */
/*********************************************************************************************************************/
void num_tx(uint8_t *pData_Tx,uint8_t TxXferCount)
	{
		while (TxXferCount > 0U)
			{  
				 while( !( USART2->SR & USART_SR_TXE ));
				 USART2->DR = (uint8_t)(*pData_Tx & 0xFFU);
					pData_Tx++;
				 TxXferCount--;
			}
	}
	
/*********************************************************************************************************************/
/*  														          		         																															 */
/*********************************************************************************************************************/
	char _uart_rx(void)
	{
		while( !( USART2->SR & USART_SR_RXNE ));  	 //To transmit the single character 
		  return	USART2->DR ;
		
	}
/*********************************************************************************************************************/
/*  														          	  Delay_FUNCTION            																						 */
/*********************************************************************************************************************/	

void ms_delay(int delay)
	{
		int init;
		for( ;delay >0;delay --)
		{
			for(init=0;init <= 3195 ;init ++)
			{
			}
		}
	}
		
/*********************************************************************************************************************/
/*  														          		         																															 */
/*********************************************************************************************************************/
