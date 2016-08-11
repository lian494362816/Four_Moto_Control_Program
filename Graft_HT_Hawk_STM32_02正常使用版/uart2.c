#include "include.h"
#include "msp430F5438A.h"
#include "stdio.h"

unsigned char count_PID = 0;

unsigned char Temp_PID_buf[Temp_PID_count];
unsigned char Temp_PID[Temp_PID_count];
char Flag_PID_Change = 0;
unsigned char data_to_send_uart1[10];//����������У���
unsigned char check_sum = 0;//����������У���
char Flag_PID_1 = 0;//�����ж��������յ�PID��
char Flag_PID_2 = 0;
char Flag_PID_3 = 0;
extern char Flag_ANO_DT_Send_Status;
extern struct _PID PID_US100;//����������
/****************************************************************************
* ��    �ƣ�void Uart1_Init(void)
* ��    �ܣ�����2  115200��0��0           
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: �������Ӻ��ص���վ����λ��������ѭHTLink֡��ʽ˵��V1.0.2
* ��    ��:
****************************************************************************/
void Uart2_Init(void)
{
    P9SEL |= BIT4 + BIT5;
    UCA2CTL1 |= UCSWRST;//�����λ
    UCA2CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA2BR0 = 0x68;
    UCA2BR1 = 0x00;
    UCA2MCTL |= UCBRF_0 + UCBRS_1;
    UCA2CTL1 &= ~UCSWRST;//�����λ �ر�
    UCA2IE |= UCRXIE;//�����ж�
}

#
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

/****************************************************************************
* ��    �ƣ�UART1�����ж�
* ��    �ܣ�����������λ��������          
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: �ܹ�����22Byte ����վ�汾4.03 Э��4.01
* ��    ��:
****************************************************************************/
#pragma vector = USCI_A2_VECTOR
__interrupt void UART2_RX_Handle(void)
{
    switch(__even_in_range(UCA2IV, 4))
    {
        case 0:
            break;
        case 2:
            while(!(UCA2IFG & UCTXIFG)); 
          
            Temp_PID_buf[count_PID++] = UCA2RXBUF;
            
            if(Temp_PID_buf[0] != 0xAA)//�ж�֡ͷ�Ƿ�Ϊ 0xFE�����������㲢�˳�
            {
                count_PID = 0;  
//                printf("Temp_PID_buf[0] = %x\r\n", Temp_PID_buf[0]);
                return;
            }
            if(3 == count_PID)
            { 
                //���������ݸ�ʽ�����㲢�˳�
                if(Temp_PID_buf[0] == 0xAA && Temp_PID_buf[1] ==0xAF && Temp_PID_buf[2] == 0x10)
                {
                    Flag_PID_1 = 1;
                }
                else if(Temp_PID_buf[0] == 0xAA && Temp_PID_buf[1] ==0xAF && Temp_PID_buf[2] == 0x11)
                { 
                    Flag_PID_2 = 1;  
                }
                
                 else if(Temp_PID_buf[0] == 0xAA && Temp_PID_buf[1] ==0xAF && Temp_PID_buf[2] == 0x12)
                { 
                    Flag_PID_3 = 1;  
                }
                else
                {
                    count_PID = 0;
                    Flag_PID_1 = 0;
                    Flag_PID_2 = 0;
                    Flag_PID_3 = 0;
                    return ;
                }
            }
            
            if(22 == count_PID)
            {
//                printf("����PID������\r\n");
                check_sum = 0;
//                for(char i = 0; i < 22; i++)
//                {
//                    printf("Temp_PID_buf[%d] = %x\r\n", i, Temp_PID_buf[i]);
//                }
                for(char i = 0; i < 22; i++)
                {
                    Temp_PID[i] = Temp_PID_buf[i];
                    check_sum += Temp_PID[i];
                }
                
                if(Flag_PID_1)
                {
                    Temp_PID_Change_To_PID_ANO_1();
//                    printf("\r\n 1\r\n");
                    ANO_DT_Send_PID_1(1, ctrl.pitch.core.kp, ctrl.pitch.core.ki, ctrl.pitch.core.kd,
                                   ctrl.roll.core.kp, ctrl.roll.core.ki, ctrl.roll.core.kd,
                                   ctrl.yaw.core.kp, ctrl.yaw.core.ki, ctrl.yaw.core.kd);
                    Flag_PID_1 = 0;
                }
                else if(Flag_PID_2)
                {
                    Temp_PID_Change_To_PID_ANO_2();
                    
                     ANO_DT_Send_PID_2(2, ctrl.pitch.shell.kp, ctrl.pitch.shell.ki, Flag_ANO_DT_Send_Status,
                                    ctrl.roll.shell.kp, ctrl.roll.shell.ki, 0,
                                   ctrl.yaw.shell.kp, 0, 0);
//                     printf("\r\n 2\r\n");
                    Flag_PID_2 = 0;
                }    
                else if(Flag_PID_3)
                {
                    Temp_PID_Change_To_PID_ANO_3();
                    
                    ANO_DT_Send_PID_2(3, PID_US100.P, PID_US100.I, PID_US100.D,
                                       0, 0, 0,
                                       0, 0, 0);
//                     printf("\r\n 2\r\n");
                    Flag_PID_3 = 0;
                }    
                
                
                //HtoEs_PID_Data_Generate();           
                Printf_PID_ANO();//������ʾPID�Ĳ��� 
                count_PID = 0;
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
* ��    �ƣ�void Temp_PID_Change_To_PID_ANO_1(void)
* ��    �ܣ�����λ�����յ�������ת��ΪPID����         
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ����վ�汾4.03 Э��4.01
* ��    ��:
****************************************************************************/

void Temp_PID_Change_To_PID_ANO_1(void)
{
    ctrl.pitch.core.kp = 0.001*( (int16)(*(Temp_PID+4)<<8)|*(Temp_PID+5) );
    ctrl.pitch.core.ki  = 0.001*( (int16)(*(Temp_PID+6)<<8)|*(Temp_PID+7) );
    ctrl.pitch.core.kd  = 0.001*( (int16)(*(Temp_PID+8)<<8)|*(Temp_PID+9) );
    ctrl.roll.core.kp = 0.001*( (int16)(*(Temp_PID+10)<<8)|*(Temp_PID+11) );
    ctrl.roll.core.ki = 0.001*( (int16)(*(Temp_PID+12)<<8)|*(Temp_PID+13) );
    ctrl.roll.core.kd = 0.001*( (int16)(*(Temp_PID+14)<<8)|*(Temp_PID+15) );
    ctrl.yaw.core.kp   = 0.001*( (int16)(*(Temp_PID+16)<<8)|*(Temp_PID+17) );
    ctrl.yaw.core.ki   = 0.001*( (int16)(*(Temp_PID+18)<<8)|*(Temp_PID+19) );
    ctrl.yaw.core.kd   = 0.001*( (int16)(*(Temp_PID+20)<<8)|*(Temp_PID+21) );
    ANO_DT_Send_Check(*(Temp_PID+2), check_sum);
}

/****************************************************************************
* ��    �ƣ�void Temp_PID_Change_To_PID_ANO_2(void)
* ��    �ܣ�����λ�����յ�������ת��ΪPID����             
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ����վ�汾4.03 Э��4.01
* ��    ��:
****************************************************************************/

void Temp_PID_Change_To_PID_ANO_2(void)
{
    ctrl.pitch.shell.kp = 0.001*( (int16)(*(Temp_PID+4)<<8)|*(Temp_PID+5) );
    ctrl.pitch.shell.ki  = 0.001*( (int16)(*(Temp_PID+6)<<8)|*(Temp_PID+7) );
    Flag_ANO_DT_Send_Status = (char)(0.001*( (int16)(*(Temp_PID+8)<<8)|*(Temp_PID+9) ));//�Ƿ����ݵ�ǰ��̬
    ctrl.roll.shell.kp = 0.001*( (int16)(*(Temp_PID+10)<<8)|*(Temp_PID+11) );
    ctrl.roll.shell.ki = 0.001*( (int16)(*(Temp_PID+12)<<8)|*(Temp_PID+13) );
    
    ctrl.yaw.shell.kp   = 0.001*( (int16)(*(Temp_PID+16)<<8)|*(Temp_PID+17) );

    ANO_DT_Send_Check(*(Temp_PID+2), check_sum);
}

/****************************************************************************
* ��    �ƣ�void Temp_PID_Change_To_PID_ANO_3(void)
* ��    �ܣ�����λ�����յ�������ת��ΪPID����             
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ����վ�汾4.03 Э��4.01
* ��    ��:
****************************************************************************/

void Temp_PID_Change_To_PID_ANO_3(void)
{

    PID_US100.P = 0.001*( (int16)(*(Temp_PID+4)<<8)|*(Temp_PID+5) );
    PID_US100.I = 0.001*( (int16)(*(Temp_PID+6)<<8)|*(Temp_PID+7) );
    PID_US100.D = 0.001*( (int16)(*(Temp_PID+8)<<8)|*(Temp_PID+9) );

    ANO_DT_Send_Check(*(Temp_PID+2), check_sum);
}



/****************************************************************************
* ��    �ƣ� void ANO_DT_Send_Check(unsigned head, unsigned check_sum)
* ��    �ܣ�          
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ����λ����������ʱ,����������  ����վ�汾4.03 Э��4.01
* ��    ��:
****************************************************************************/
 void ANO_DT_Send_Check(unsigned head, unsigned check_sum)
{
    data_to_send_uart1[0]=0xAA;
    data_to_send_uart1[1]=0xAA;
    data_to_send_uart1[2]=0xEF;
    data_to_send_uart1[3]=2;
    data_to_send_uart1[4]=head;
    data_to_send_uart1[5]=check_sum;
    
    
    unsigned char sum = 0;
    for(unsigned char i=0;i<6;i++)
        sum += data_to_send_uart1[i];
    data_to_send_uart1[6]=sum;

    for(unsigned char i = 0; i < 7; i ++)//���ڽ��������
    {
        while(!(UCA2IFG & UCTXIFG));
        UCA2TXBUF = data_to_send_uart1[i];
    }
}