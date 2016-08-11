#include "include.h"
#include <stdio.h>

// P5.6 Tx
// P5.7 Rx

unsigned char count_PID = 0;

unsigned char Temp_PID_buf[Temp_PID_count];
unsigned char Temp_PID[Temp_PID_count];
char Flag_PID_Change = 0;
unsigned char data_to_send_uart1[10];//用于匿名的校验和
unsigned char check_sum = 0;//用于匿名的校验和
char Flag_PID_1 = 0;//用于判断匿名接收的PID组
char Flag_PID_2 = 0;
char Flag_PID_3 = 0;
extern char Flag_ANO_DT_Send_Status;
extern struct _PID PID_US100;//超声波参数
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

/* 
     接收恒拓上位机使用
     28个byte
*/
//#pragma vector = USCI_A1_VECTOR
//__interrupt void UART1_RX_Handle(void)
//{
//    switch(__even_in_range(UCA1IV, 4))
//    {
//        case 0:
//            break;
//        case 2:
//            while(!(UCA1IFG & UCTXIFG)); 
//          
//            Temp_PID_buf[count_PID++] = UCA1RXBUF;
//            
//            if(Temp_PID_buf[0] != 0xFE)//判断帧头是否为 0xFE，不是则清零并退出
//            {
//                count_PID = 0;  
//                printf("Temp_PID_buf[0] = %x\r\n", Temp_PID_buf[0]);
//                return;
//            }
//            if(3 == count_PID)
//            { 
//                //不符合数据格式则清零并退出
//                if(Temp_PID_buf[0] != 0xFE && Temp_PID_buf[1] !=0x1C && Temp_PID_buf[2] != 0xEB)
//                {
//                    for(char i = 0; i < 3; i++)
//                    {
//                        printf("Temp_PID_buf[%d] = %x\r\n", i, Temp_PID_buf[i]);
//                    }
//                    count_PID = 0;
//                    return;
//                }
//            }
//            
//            if( 28 == count_PID)
//            {
////                printf("接收PID数据流\r\n");
////                for(char i = 0; i < 28; i++)
////                {
////                    printf("Temp_PID_buf[%d] = %x\r\n", i, Temp_PID_buf[i]);
////                }
//                for(char i = 0; i < 28; i++)
//                {
//                    Temp_PID[i] = Temp_PID_buf[i];
//                }
//             
//                Temp_PID_Change_To_PID();
//                HtoEs_PID_Data_Generate();
//                Printf_PID();//串口显示PID的参数 
//               
//                return;
//            }
//            break;
//            
//        case 4:
//            break;
//            
//        default:
//            break;
//    }
//}

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

/****************************************************************************
* 名    称：UART1串口中断
* 功    能：接收匿名上位机的数据          
* 入口参数： 
* 出口参数：
* 说    明: 总共接收22Byte 地面站版本4.03 协议4.01
* 范    例:
****************************************************************************/
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
            
            if(Temp_PID_buf[0] != 0xAA)//判断帧头是否为 0xFE，不是则清零并退出
            {
                count_PID = 0;  
//                printf("Temp_PID_buf[0] = %x\r\n", Temp_PID_buf[0]);
                return;
            }
            if(3 == count_PID)
            { 
                //不符合数据格式则清零并退出
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
//                printf("接收PID数据流\r\n");
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
                Printf_PID_ANO();//串口显示PID的参数 
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
* 名    称：void Temp_PID_Change_To_PID_ANO_1(void)
* 功    能：从上位机接收到的数据转化为PID数据         
* 入口参数： 
* 出口参数：
* 说    明: 地面站版本4.03 协议4.01
* 范    例:
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
* 名    称：void Temp_PID_Change_To_PID_ANO_2(void)
* 功    能：从上位机接收到的数据转化为PID数据             
* 入口参数： 
* 出口参数：
* 说    明: 地面站版本4.03 协议4.01
* 范    例:
****************************************************************************/

void Temp_PID_Change_To_PID_ANO_2(void)
{
    ctrl.pitch.shell.kp = 0.001*( (int16)(*(Temp_PID+4)<<8)|*(Temp_PID+5) );
    ctrl.pitch.shell.ki  = 0.001*( (int16)(*(Temp_PID+6)<<8)|*(Temp_PID+7) );
    Flag_ANO_DT_Send_Status = (char)(0.001*( (int16)(*(Temp_PID+8)<<8)|*(Temp_PID+9) ));//是否数据当前姿态
    ctrl.roll.shell.kp = 0.001*( (int16)(*(Temp_PID+10)<<8)|*(Temp_PID+11) );
    ctrl.roll.shell.ki = 0.001*( (int16)(*(Temp_PID+12)<<8)|*(Temp_PID+13) );
    
    ctrl.yaw.shell.kp   = 0.001*( (int16)(*(Temp_PID+16)<<8)|*(Temp_PID+17) );

    ANO_DT_Send_Check(*(Temp_PID+2), check_sum);
}

/****************************************************************************
* 名    称：void Temp_PID_Change_To_PID_ANO_3(void)
* 功    能：从上位机接收到的数据转化为PID数据             
* 入口参数： 
* 出口参数：
* 说    明: 地面站版本4.03 协议4.01
* 范    例:
****************************************************************************/

void Temp_PID_Change_To_PID_ANO_3(void)
{

    PID_US100.P = 0.001*( (int16)(*(Temp_PID+4)<<8)|*(Temp_PID+5) );
    PID_US100.I = 0.001*( (int16)(*(Temp_PID+6)<<8)|*(Temp_PID+7) );
    PID_US100.D = 0.001*( (int16)(*(Temp_PID+8)<<8)|*(Temp_PID+9) );

    ANO_DT_Send_Check(*(Temp_PID+2), check_sum);
}



/****************************************************************************
* 名    称： void ANO_DT_Send_Check(unsigned head, unsigned check_sum)
* 功    能：          
* 入口参数： 
* 出口参数：
* 说    明: 向上位机发送数据时,在最后面调用  地面站版本4.03 协议4.01
* 范    例:
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

    for(unsigned char i = 0; i < 7; i ++)//用于将数据输出
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = data_to_send_uart1[i];
    }
}