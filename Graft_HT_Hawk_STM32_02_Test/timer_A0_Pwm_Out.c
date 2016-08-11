#include "include.h"

/*

    P1.2  -->  YAW
    P1.3  -->  THROTTLR
    P1.4  -->  PIT
    P1.5  -->  ROL
  
    1MHZ
*/

extern unsigned int RC_Pwm_In[4];

void Tiemer_A0_Pwm_Out_Init(void)
{
    P1SEL |= BIT2 + BIT3 + BIT4 + BIT5;
    P1DIR |= BIT2 + BIT3 + BIT4 + BIT5;
    
    TA0CTL = TASSEL_2 + MC_1 + TACLR + ID_2;    // SMCLK = 12M, upmode conunts to TAxCCR0, clear TBR£¬4·ÖÆµ
    TA0EX0|= TAIDEX_2;//3·ÖÆµ
    
    TA0CCR0 = 2500;
    
    TA0CCTL1 = OUTMOD_7;        
    TA0CCR1 = 1000;
    
    TA0CCTL2 = OUTMOD_7;
    TA0CCR2 = 1000;
    
    TA0CCTL3 = OUTMOD_7;
    TA0CCR3 = 1000;
    
    TA0CCTL4 = OUTMOD_7;
    TA0CCR4 = 1000;
    
}