


#include "stm32f10x.h"
#include "string.h"

void num_tx(uint8_t *,uint8_t );        // Transmit_function 
void ms_delay(int );										// Delay_Functin
void UART_Init (void);								// Uart_init_function

 char indx = 'A';												// Single_Character
 char string[20]="Hello World \r\n";	  // Multiple_Character
 char buff[20];													// Char_Buffer
 uint16_t num = 475;										// Intiger_Number
	
int main (void)
{											
	UART_Init ();                       // UART_Initialization 
	sprintf(buff," NUM: %d \r\n ", num);  // Intiger to string conversion
	while (1)
	{
		
			//while( !( USART2->SR & USART_SR_TXE ));  	 //To transmit the single character 
			//USART2->DR = 'A';
		  
		
		 //num_tx((uint8_t*)buff,sizeof(buff));				//To transmit the single character 
		
			 num_tx((uint8_t*)string,sizeof(string)); 	//To transmit the string
		   ms_delay(100);
	}
}

/*********************************************************************************************************************/
/*  														          		UART_Init_FUNCTION         																						 */
/*********************************************************************************************************************/

void UART_Init(void)
	{
		RCC->APB1ENR |= (1<<17);  	 // Enable USART2 Clock
		RCC->APB2ENR |= 1<<2;  			 // Enable GPIOA CLOCK
		GPIOA->CRL = 0;
		GPIOA->CRL |= (3<<8);   		 // output mode 50 MHz for PA2
		GPIOA->CRL |= (2<<10);       // Alternate Func Push Pull For PA2
		USART2->CR1 = 0x00;   			 // Clear ALL
		USART2->CR1 |= (1<<13);  		 // UE = 1... Enable USART
		USART2->CR1 |= (1<<3);  		 // TE=1.. Enable Transmitter
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
/*  														          		         																						 */
/*********************************************************************************************************************/
