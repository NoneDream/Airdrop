#include "r_cg_macrodriver.h"
#include "delay.h"

void delay_us(int us)
{
	int i;
	while (--us)
	{
		for(i=0;i<1;++i);
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
	}
}
void delay_ms(int ms)
{
	while(--ms)
	{
		delay_us(1000);
	}
}