#include "include.h"

/* 
    1MHz
    3000 / 100000 =3ms 
    定时3毫秒
*/

void Timer_A1_Init(void)
{
    TA1CTL |= TASSEL_2 + ID_2 + MC_1 + TACLR;//SMCLK=12M  4分频 向上模式 
    TA1CCTL0 |= CCIE; //打开中断
    TA1CCR0 = 13000;//13ms定时
    TA1EX0 |= TAIDEX_2;//3分频
}

/* 
    第一次执行要13ms
    以后执行要9.4ms
*/

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIM1_Handle(void)
{
    //printf("13ms\r\n");//用于测试中断是否执行
    //AHRS_Geteuler();//
    _EINT();
    TA1CCR0 += 13000;
    AHRS_Geteuler();	//
    Calculate_Target(); //
    CONTROL(Target);
    //HtoEs_Attitude_Data_Generate();//显示当前姿态
    // HtoEs_PID_Data_Generate();//显示各PID的参数
}