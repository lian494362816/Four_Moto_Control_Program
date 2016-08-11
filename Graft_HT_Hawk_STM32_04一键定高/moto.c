#include "include.h"

void moto_PwmRflash(int16 *Moter)
{		
	for(uint8 i=0;i<MOTOR_NUM;i++)
	{
        if(*(Moter+i) > Moto_PwmMax)  *(Moter+i) = Moto_PwmMax;
    }
	for(uint8 i=0;i<MOTOR_NUM;i++)
	{
        if(*(Moter+i) <= 0 )  *(Moter+i) = 0;
    }
	
    TBCCR2 = *(Moter++);
    TBCCR3 = *(Moter++);
    TBCCR4 = *(Moter++);
    TBCCR5 = *Moter;
    
	
}

void moto_STOP(void)
{
    TBCCR2 = 800;
    TBCCR3 = 800;
    TBCCR4 = 800;
    TBCCR5 = 800;
}
