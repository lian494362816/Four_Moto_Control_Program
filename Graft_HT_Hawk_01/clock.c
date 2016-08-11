#include "clock.h"
#include "msp430f5438A.h"
#include "delay.h"
/* 
    配置时钟
    ACLK = 32.768
    SMCLK =  12M
    MCLK  =  24M
*/


void Clock_Init()
{

    uchar i;
    P5SEL |= BIT2 + BIT3;//XT2 输入引脚
    P7SEL |= BIT0 + BIT1;//XT1 输入引脚
    Set_Vcore(PMMCOREV_3);//不理解这条语句
    UCSCTL6 &= ~(XT1OFF + XT2OFF);
    //UCSCTL6 |= XCAP_3;
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
        SFRIFG1 &= ~OFIFG;
        for(i = 0xFF; i > 0; i--);
    }
    while(SFRIFG1 & OFIFG);
    Delay_ms(50);
    UCSCTL4 |= SELA__XT1CLK + SELS__XT2CLK + SELM__XT2CLK;
    UCSCTL5 |= DIVS_1;
}
/* 不理解这里的设置
    配置内核电压

 典型调用 Set_Vcore(PMMCOREV_3); 
*/
void Set_Vcore(uint level)
{  
    PMMCTL0_H = PMMPW_H;                    // Open PMM registers for write  
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;  // Set SVS/SVM high side new level  
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;   // Set SVM low side to new level  
    while ((PMMIFG & SVSMLDLYIFG) == 0);    // Wait till SVM is settled  
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);      // Clear already set flags  
    PMMCTL0_L = PMMCOREV0 * level;          // Set VCore to new level  
    if ((PMMIFG & SVMLIFG))                 // Wait till new level reached
        while ((PMMIFG & SVMLVLRIFG) == 0);
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;  // Set SVS/SVM low side to new level  
    PMMCTL0_H = 0x00;                       // Lock PMM registers for write access
}


/*   
        用来测试时钟的引脚   
        P11.0   ACLK
        P11.1   MCLK
        P11.2   SMCLK
*/    

void Clock_Test(void)
{
    P11DIR = BIT2 + BIT1 + BIT0;     //配置三个时钟的输出 
    P11SEL = BIT2 + BIT1 + BIT0;     
}