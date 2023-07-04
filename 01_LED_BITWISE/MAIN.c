


#include"stm32f10x.h"
void ms_delay(int);

int main(void)
{
	RCC->APB2ENR |= (1<<4);
	GPIOC->CRH |= (1<<21);
	while(1)
	{
		ms_delay(100);
	  GPIOC->BSRR |=  (1<<13); //   TO RESET
		ms_delay(100);
		GPIOC->BSRR |= (1<<(13+16)); // TO SET
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