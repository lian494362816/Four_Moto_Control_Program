#include "include.h"

void Key_Init(void)
{
    P2DIR |= BIT0;  //����Ϊ���
    P2OUT |= BIT0;  //����ߵ�ƽ
    
    P2REN |= BIT0;  //ʹ�� ����������
    P2OUT |= BIT0;  //��������
    P2DIR &= ~BIT0; //����Ϊ����
    P2IE |= BIT0;   //�ж�ʹ��
    P2IES &= BIT0;  //�½����ж�
    P2IFG &= ~(BIT0); //�����־λ
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~(BIT0);
    P9OUT ^= (BIT0 + BIT1 + BIT2 + BIT3);
}