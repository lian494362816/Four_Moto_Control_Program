#include "include.h"
#include <stdio.h>

// P5.6 Tx
// P5.7 Rx

unsigned char count_PID = 0;

unsigned char Temp_PID_buf[Temp_PID_count];
unsigned char Temp_PID[Temp_PID_count];
char Flag_PID_Change = 0;

/****************************************************************************
* ��    �ƣ�void Uart1_Init(void)
* ��    �ܣ�����1  115200��0��0           
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: �������Ӻ��ص���վ����λ��������ѭHTLink֡��ʽ˵��V1.0.2
* ��    ��:
****************************************************************************/

void Uart1_Init(void)
{
    P5SEL |= BIT6 + BIT7;
    UCA1CTL1 |= UCSWRST;//�����λ
    UCA1CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA1BR0 = 0x68;
    UCA1BR1 = 0x00;
    UCA1MCTL |= UCBRF_0 + UCBRS_1;
    UCA1CTL1 &= ~UCSWRST;//�����λ �ر�
    UCA1IE |= UCRXIE;//�����ж�
}

#pragma vector = USCI_A1_VECTOR
__interrupt void UART1_RX_Handle(void)
{
    switch(__even_in_range(UCA1IV, 4))
    {
        case 0:
            break;
        case 2:
            while(!(UCA1IFG & UCTXIFG)); 
          
            Temp_PID_buf[count_PID++] = UCA1RXBUF;
            
            if(Temp_PID_buf[0] != 0xFE)//�ж�֡ͷ�Ƿ�Ϊ 0xFE�����������㲢�˳�
            {
                count_PID = 0;  
                printf("Temp_PID_buf[0] = %x\r\n", Temp_PID_buf[0]);
                return;
            }
            if(3 == count_PID)
            { 
                //���������ݸ�ʽ�����㲢�˳�
                if(Temp_PID_buf[0] != 0xFE && Temp_PID_buf[1] !=0x1C && Temp_PID_buf[2] != 0xEB)
                {
                    for(char i = 0; i < 3; i++)
                    {
                        printf("Temp_PID_buf[%d] = %x\r\n", i, Temp_PID_buf[i]);
                    }
                    count_PID = 0;
                    return;
                }
            }
            
            if( 28 == count_PID)
            {
//                printf("����PID������\r\n");
//                for(char i = 0; i < 28; i++)
//                {
//                    printf("Temp_PID_buf[%d] = %x\r\n", i, Temp_PID_buf[i]);
//                }
                for(char i = 0; i < 28; i++)
                {
                    Temp_PID[i] = Temp_PID_buf[i];
                }
             
                Temp_PID_Change_To_PID();
                HtoEs_PID_Data_Generate();
                Printf_PID();//������ʾPID�Ĳ��� 
               
                return;
            }
            break;
            
        case 4:
            break;
            
        default:
            break;
    }
}

/****************************************************************************
* ��    �ƣ�void Temp_PID_Change_To_PID(void)
* ��    �ܣ�����λ�����յ���PID������ת��ΪPID����          
* ��ڲ�������
* ���ڲ�������
* ˵    ��: ��ѭHTLink֡��ʽ˵��V1.0.2
* ��    ��:
****************************************************************************/

void Temp_PID_Change_To_PID(void)
{
//    uint16 k;
//    printf("Temp_PID[3]=0x%x,Temp_PID[4] =0x%x,\r\n", Temp_PID[3], Temp_PID[4]);
//    k = ((Temp_PID[3] << 8) + Temp_PID[4]);
//    printf("k=%d\r\n", k);
    ctrl.pitch.core.kp = (float)((Temp_PID[3] << 8) + Temp_PID[4]) / 1000;
    //printf("Temp_PID[3]=0x%x\r\n", Temp_PID[3]);
//    printf("ctrl.pitch.core.kp = %f\r\n", ctrl.pitch.core.kp);
    ctrl.pitch.core.ki = (float)((Temp_PID[5] << 8) + Temp_PID[6]) / 1000;
    ctrl.pitch.core.kd = (float)((Temp_PID[7] << 8) + Temp_PID[8]) / 1000;
    
    ctrl.roll.core.kp = (float)((Temp_PID[9] << 8) + Temp_PID[10]) / 1000;
    ctrl.roll.core.ki = (float)((Temp_PID[11] << 8) + Temp_PID[12]) / 1000;
    ctrl.roll.core.kd = (float)((Temp_PID[13] << 8) + Temp_PID[14]) / 1000;
    
   #ifdef PITCH_YAW_ROLL
    ctrl.yaw.core.kp = (float)((Temp_PID[15] << 8) + Temp_PID[16]) / 1000;
    ctrl.yaw.core.ki = (float)((Temp_PID[17] << 8) + Temp_PID[18]) / 1000;
    ctrl.yaw.core.kd = (float)((Temp_PID[19] << 8) + Temp_PID[20]) / 1000;
    
   #elif PITCH_ROLL_SHELL
    ctrl.pitch.shell.kp = (float)((Temp_PID[15] << 8) + Temp_PID[16]) / 1000;
    ctrl.pitch.shell.ki = (float)((Temp_PID[17] << 8) + Temp_PID[18]) / 1000;
    
    ctrl.roll.shell.kp = (float)((Temp_PID[21] << 8) + Temp_PID[22]) / 1000;
    ctrl.roll.shell.ki = (float)((Temp_PID[23] << 8) + Temp_PID[24]) / 1000;
  
   #endif
    
    Flag_PID_Change = 1;
}
///* 
//    �ض���C����
//*/
//int putchar(int ch)
//{
//    while(!(UCA1IFG & UCTXIFG));
//    UCA1TXBUF = (char) ch;
//    return ch;
//}

