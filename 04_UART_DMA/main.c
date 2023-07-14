

#include"stm32f10x.h"
#include"string.h"

#define RX_SIZE 20

/*--------------------------------------------- FUCN_PROTOTYPE-----------------------------------------------------------------------*/

	void DMA_Init(void);
	void UART_Init(void);
	void num_tx(uint8_t *,uint8_t);
	void DMA1_Channel6_IRQHandler(void);
	void DMA1_Config(uint32_t,uint32_t,uint16_t);
	
/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------VARIABLE_DECLARATION----------------------------------------------------------------*/
	
	uint8_t Rx_buff[20];
	uint8_t Main_buff[50];
	uint8_t iD_x = 0 ;
	uint8_t rx_tst=0;
	char deck[19]="HI_im_going_DMA\r\n";
	
/*-----------------------------------------------------------------------------------------------------------------------------------*/

int main()
{
	SystemInit();
	UART_Init();
	DMA_Init();
	NVIC_SetPriority(DMA1_Channel6_IRQn,0);
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	DMA1_Config((uint32_t)&USART2->DR,(uint32_t)Rx_buff,RX_SIZE);
	
	while(1)
	{
//		num_tx((uint8_t*)deck,19);
//		if(rx_tst==1)
//		{
//			num_tx(Main_buff,20);
//			rx_tst=0;
//		}
	}
}
/*=======================================================================================================================*/
/*                                                                                                                       */
/*=======================================================================================================================*/

void UART_Init(void)
	{
		RCC->APB1ENR |= (1<<17);  	 // Enable USART2 Clock
		RCC->APB2ENR |= 1<<2;  			 // Enable GPIOA CLOCK
		
		GPIOA->CRL = 0;
		GPIOA->CRL |= (3<<8);   		 // output mode 50 MHz for PA2
		GPIOA->CRL |= (2<<10);       // Alternate Func Push Pull For PA2
		
		GPIOA->CRL &=~(3<<12);   		 // PA3 50Hz INPUT Mode
		GPIOA->CRL |=(2<<14);				 // Input pull-up/push-down 
		GPIOA->ODR |=(1<<3);				 // pull-up
		
		USART2->CR1 = 0x00;   			 // Clear ALL
		USART2->CR1 |= (1<<13);  		 // UE = 1... Enable USART
//	USART2->CR3 |= (1<<7);  		 // TE=1.. Enable Transmitter DMA Mode
		USART2->CR3 |= (1<<6);       // TXE=1..Enable the Reciver DMA Mode
		
		/* 
			Tx/ Rx baud = PCLK /(16*Usr_BRR) 
			USARTDIV = 19.53125
			MENSTISSA = 19
			FRACTION = 16*0.53125 = 8.5
		*/

		 //USART2->BRR = (8<<0) | (19<<4);   // Baud rate of 115200, PCLK1 at 36MHz
		 USART2->BRR = (6<<0) | (234<<4);    // Baud rate of 9600, PCLK1 at 36MHz
		 USART2->CR1 |= (1<<3);  		 // Enable Transmitter
		 USART2->CR1 |= (1<<2);       // Enable the Reciver 
	}
	
/*=======================================================================================================================*/
/*                                                                                                                       */
/*=======================================================================================================================*/
	
	void DMA_Init(void)
{
	RCC->AHBENR |= (1<<0);                       //  DMA - Clock Enable
	
	DMA1_Channel6->CCR |= (1<<1)|(1<<2)|(1<<3);  //  To Set the TEIE,TCIE,HTIE
	
	DMA1_Channel6->CCR &=~(1<<4);								 //  Read from peripheral
	
	DMA1_Channel6->CCR |= (1<<5);								 //  Circular mode enabled
	
	DMA1_Channel6->CCR |= (1<<7);         			 //  Memory increment mode enabled
	
	DMA1_Channel6->CCR &=~(3<<8);                //  Peripheral size 8-bits
	
	DMA1_Channel6->CCR &=~(3<<10);               //  Memory size 8-bits
	
	DMA1_Channel6->CCR &=~(3<<12);								 //  Channel priority leve P=0...
}

/*=======================================================================================================================*/
/*                                                                                                                       */
/*=======================================================================================================================*/
void DMA1_Channel6_IRQHandler(void)
{
	
	if((DMA1->ISR)&(1<<22))    			// IF HALF_Transfer Completed  ....HTIF flag = 1
	{ 
		memset(&Main_buff[iD_x],Rx_buff[0],RX_SIZE/2);
		DMA1->IFCR |= (1<<22);        //  Clears the corresponding HTIF flag in the DMA_ISR registe
		iD_x = iD_x +(RX_SIZE/2);
		if(iD_x>49)  iD_x = 0;
	}
	
	if((DMA1->ISR)&(1<<21))    			// IF Transfer Completed  ....HTIF flag = 1
	{
		memset(&Main_buff[iD_x],Rx_buff[0],RX_SIZE/2);
		DMA1->IFCR |= (1<<21);        //  Clears the corresponding HTIF flag in the DMA_ISR registe
		iD_x = iD_x +(RX_SIZE/2);
		if(iD_x>49)  iD_x = 0;
	}
	
}

/*=======================================================================================================================*/
/*                                                                                                                       */
/*=======================================================================================================================*/
void DMA1_Config(uint32_t src_add,uint32_t m_add,uint16_t data_size)
{
	DMA1_Channel6->CNDTR = data_size ;					// To set CNDTR indicate the data_size
	
	DMA1_Channel6->CPAR  = src_add ;						// To set the CPAR too peripheral address
	
	DMA1_Channel6->CMAR  = m_add ;							// To set the CMAR help of memory address
	
	DMA1_Channel6->CCR  |=(1<<0) ;              // To Enable the DMA
}
/*=======================================================================================================================*/
/*                                                                                                                       */
/*=======================================================================================================================*/
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
/*=======================================================================================================================*/
/*                                                                                                                       */
/*=======================================================================================================================*/