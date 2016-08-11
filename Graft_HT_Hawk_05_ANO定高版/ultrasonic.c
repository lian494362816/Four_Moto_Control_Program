#include "include.h"
#include "delay.h"

//  ³¬Éù²¨
//  P2.1   Trig
//  P2.2   Echo

void Ultrasonic_Config(void)
{
    P2DIR |= BIT1;
    
}

void Ultrasonic_Pulsing(void)
{
    P2OUT |= BIT1;
    Delay_us(20);
    P2OUT &= ~BIT1;
}

