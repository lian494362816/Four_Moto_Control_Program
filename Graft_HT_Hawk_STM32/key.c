#include "include.h"

void Key_Init(void)
{
    P2DIR |= BIT0;  //设置为输出
    P2OUT |= BIT0;  //输出高电平
    
    P2REN |= BIT0;  //使能 上下拉电阻
    P2OUT |= BIT0;  //上拉电阻
    P2DIR &= ~BIT0; //设置为输入
    P2IE |= BIT0;   //中断使能
    P2IES &= BIT0;  //下降沿中断
    P2IFG &= ~(BIT0); //清除标志位
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~(BIT0);
    P9OUT ^= (BIT0 + BIT1 + BIT2 + BIT3);
}