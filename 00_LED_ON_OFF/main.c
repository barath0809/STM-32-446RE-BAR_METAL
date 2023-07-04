



#include"stm32f10x.h"

void ms_delay(int);

int main(void)
{
	RCC->APB2ENR |= 0x00000010;  // TO ENABLE_THE_CLCK
	GPIOC->CRH |= 0x00300000; // TO ENABLE THE PORT C 13(IT IS INBUILD LED)
	
	while(1)
	{
		
		GPIOC->BSRR |= 0x20000000; // TO SET
		ms_delay(100);
		GPIOC->BSRR |= 0x00002000; //TO RESET 
	   ms_delay(100);
	}
}

void ms_delay(int delay_num)
{
	int i ;
	for( ;delay_num>0;delay_num--)
	{
		   for(i=0;i<=31950;i++)	
		{
		}
	}
}