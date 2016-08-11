#include "include.h"
#include <stdio.h>

// P5.6 Tx
// P5.7 Rx

unsigned char count_PID = 0;

unsigned char Temp_PID_buf[Temp_PID_count];
unsigned char Temp_PID[Temp_PID_count];
char Flag_PID_Change = 0;

/****************************************************************************
* 名    称：void Uart1_Init(void)
* 功    能：串口1  115200，0，0           
* 入口参数： 
* 出口参数：
* 说    明: 用于连接恒拓地面站（上位机），遵循HTLink帧格式说明V1.0.2
* 范    例:
****************************************************************************/

void Uart1_Init(void)
{
    P5SEL |= BIT6 + BIT7;
    UCA1CTL1 |= UCSWRST;//软件复位
    UCA1CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA1BR0 = 0x68;
    UCA1BR1 = 0x00;
    UCA1MCTL |= UCBRF_0 + UCBRS_1;
    UCA1CTL1 &= ~UCSWRST;//软件复位 关闭
    UCA1IE |= UCRXIE;//接收中断
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
            
            if(Temp_PID_buf[0] != 0xFE)//判断帧头是否为 0xFE，不是则清零并退出
            {
                count_PID = 0;  
                printf("Temp_PID_buf[0] = %x\r\n", Temp_PID_buf[0]);
                return;
            }
            if(3 == count_PID)
            { 
                //不符合数据格式则清零并退出
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
//                printf("接收PID数据流\r\n");
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
                Printf_PID();//串口显示PID的参数 
               
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
* 名    称：void Temp_PID_Change_To_PID(void)
* 功    能：将上位机接收到的PID数据流转化为PID参数          
* 入口参数：无
* 出口参数：无
* 说    明: 遵循HTLink帧格式说明V1.0.2
* 范    例:
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
//    重定向C函数
//*/
//int putchar(int ch)
//{
//    while(!(UCA1IFG & UCTXIFG));
//    UCA1TXBUF = (char) ch;
//    return ch;
//}

