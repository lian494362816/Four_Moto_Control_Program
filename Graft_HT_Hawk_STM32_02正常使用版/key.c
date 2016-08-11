#include "include.h"


void Key_Init(void)
{
    P2DIR |= KEY_1;  //设置为输出
    P2OUT |= KEY_1;  //输出高电平
    
    P2REN |= KEY_1;  //使能 上下拉电阻
    P2OUT |= KEY_1;  //上拉电阻
    P2DIR &= ~KEY_1; //设置为输入
    P2IE |= KEY_1;   //中断使能
    P2IES &= KEY_1;  //下降沿中断
    P2IFG &= ~(KEY_1); //清除标志位
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~(KEY_1);
    P9OUT ^= (BIT0 + BIT1 + BIT2 + BIT3);
}