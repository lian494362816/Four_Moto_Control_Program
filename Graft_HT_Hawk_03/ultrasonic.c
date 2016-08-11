#include "include.h"
#include "delay.h"
//float US100_Alt;
//float US100_Alt_delta,US100_Alt_old;
//unsigned int g_Hight=0,g_HightOld=0;
//float g_Alt_Hight=0,g_Alt_HightOld=0;
//float g_HightControl=0,g_HightControlold=0,hight_increment=0;
//unsigned char RcvIndex,GLengthHigh, GLengthLow; 
//float g_hight_Kp=0.8,g_hight_Ki=0.015,g_hight_Kd=10;  
//float hight_error=0,hight_errorold=0,hight_erroroldd,cao;
//  超声波
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

//void Measure()
//{ /******************************HC-sr04的初始化部分****************************/
//	
//    SET_Trig;//产生脉冲引脚
//	Delay_us(20);//延时20us，把Trig的引脚的电位拉高20us//之后，超声波内部自动发送8个40KHz的脉冲 
//    CLK_Trig;//再输出低电平 
//    _EINT();//中断使能,打开中断，上升沿捕获和下降沿捕获
//    /*********************************************************************
//    *********************************************************************/
//    while(Echo == 0);//判断echo是否变为高电平，上升沿
//      
//    TA1CTL = TASSEL_2 + MC_2 + TACLR + TAIE + ID_2;  // SMCLK, Continuous up, clear TAR                         
//    TA1CCTL1 = CAP + CM_1 + SCS + CCIE;     //捕获模式，通道B，上升沿捕获，同步，打开中断
//
//    
//}