#include "include.h"


void Key_Init(void)
{
    P2DIR |= KEY_1;  //����Ϊ���
    P2OUT |= KEY_1;  //����ߵ�ƽ
    
    P2REN |= KEY_1;  //ʹ�� ����������
    P2OUT |= KEY_1;  //��������
    P2DIR &= ~KEY_1; //����Ϊ����
    P2IE |= KEY_1;   //�ж�ʹ��
    P2IES &= KEY_1;  //�½����ж�
    P2IFG &= ~(KEY_1); //�����־λ
}

#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
    P2IFG &= ~(KEY_1);
    P9OUT ^= (BIT0 + BIT1 + BIT2 + BIT3);
}