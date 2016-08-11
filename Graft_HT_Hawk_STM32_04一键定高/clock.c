#include "clock.h"
#include "msp430f5438A.h"
#include "delay.h"
/* 
    ����ʱ��
    ACLK = 32.768
    SMCLK =  12M
    MCLK  =  24M
*/


void Clock_Init()
{

    uchar i;
    P5SEL |= BIT2 + BIT3;//XT2 ��������
    P7SEL |= BIT0 + BIT1;//XT1 ��������
    Set_Vcore(PMMCOREV_3);//������������
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
/* ��������������
    �����ں˵�ѹ

 ���͵��� Set_Vcore(PMMCOREV_3); 
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
        ��������ʱ�ӵ�����   
        P11.0   ACLK
        P11.1   MCLK
        P11.2   SMCLK
*/    

void Clock_Test(void)
{
    P11DIR = BIT2 + BIT1 + BIT0;     //��������ʱ�ӵ���� 
    P11SEL = BIT2 + BIT1 + BIT0;     
}