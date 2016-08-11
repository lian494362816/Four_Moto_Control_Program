#include "include.h"

/* 
    1MHz
    3000 / 100000 =3ms 
    ��ʱ3����
*/

void Timer_A1_Init(void)
{
    TA1CTL |= TASSEL_2 + ID_2 + MC_1 + TACLR;//SMCLK=12M  4��Ƶ ����ģʽ 
    TA1CCTL0 |= CCIE; //���ж�
    TA1CCR0 = 13000;//13ms��ʱ
    TA1EX0 |= TAIDEX_2;//3��Ƶ
}

/* 
    ��һ��ִ��Ҫ13ms
    �Ժ�ִ��Ҫ9.4ms
*/

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIM1_Handle(void)
{
    //printf("13ms\r\n");//���ڲ����ж��Ƿ�ִ��
    //AHRS_Geteuler();//
    _EINT();
    TA1CCR0 += 13000;
    AHRS_Geteuler();	//
    Calculate_Target(); //
    CONTROL(Target);
    //HtoEs_Attitude_Data_Generate();//��ʾ��ǰ��̬
    // HtoEs_PID_Data_Generate();//��ʾ��PID�Ĳ���
}