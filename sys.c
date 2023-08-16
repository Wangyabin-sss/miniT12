#include "sys.h"


void delay_ms(u16 ms)
{
     u16 i;
     do{
          i = FOSC / 10000;
          while(--i);   //10T per loop
     }while(--ms);
}


void delay_us(u16 us)
{
	u16 i = us*6;
	while (--i);
}




