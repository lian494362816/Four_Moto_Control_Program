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
//  ������
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
//{ /******************************HC-sr04�ĳ�ʼ������****************************/
//	
//    SET_Trig;//������������
//	Delay_us(20);//��ʱ20us����Trig�����ŵĵ�λ����20us//֮�󣬳������ڲ��Զ�����8��40KHz������ 
//    CLK_Trig;//������͵�ƽ 
//    _EINT();//�ж�ʹ��,���жϣ������ز�����½��ز���
//    /*********************************************************************
//    *********************************************************************/
//    while(Echo == 0);//�ж�echo�Ƿ��Ϊ�ߵ�ƽ��������
//      
//    TA1CTL = TASSEL_2 + MC_2 + TACLR + TAIE + ID_2;  // SMCLK, Continuous up, clear TAR                         
//    TA1CCTL1 = CAP + CM_1 + SCS + CCIE;     //����ģʽ��ͨ��B�������ز���ͬ�������ж�
//
//    
//}