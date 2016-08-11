#include "include.h"
#include <stdio.h>


unsigned char Rx_Temp[3];
unsigned char Rx_Temp_count = 0;

static int US100_Alt_Temp=0; 
float 
        US100_Alt_V,Alt_CuntTmep1=0,Alt_CuntTmep2=0,
        Alt_Last=0,US100_Alt_Last=0,Alt_V_CuntTmep1=0,
        Alt_V_CuntTmep2=0,US100_Alt; 

//与超声波模块相连
// 9600
// P5.6  Tx  与超声波Tx相连
// P5.7  Rx  与超声波Rx相连
void Uart1_Init(void)
{
    P5SEL |= BIT6 + BIT7;
    UCA1CTL1 |= UCSWRST;//软件复位
    UCA1CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA1BR0 = 0xe2;
    UCA1BR1 = 0x04;
    UCA1MCTL |= UCBRF_0 + UCBRS_0;
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
            Rx_Temp[Rx_Temp_count++]= UCA1RXBUF;
            if(2 == Rx_Temp_count)
            {
                Rx_Temp_count = 0;         
                US100_Alt_Temp = (Rx_Temp[0] << 8) + Rx_Temp[1];
                if(US100_Alt_Temp > 5000 || US100_Alt_Temp < 0)
                {           
                     Send_US100_Start();
                     return;
                }
             
                if(US100_Alt_Temp > 4000)     //最大值 4000
                { 
                    US100_Alt_Temp = Alt_Last; 
                } 
                
                else   
                { 
                    Alt_Last=US100_Alt_Temp; 
                }
//                printf("US100_Alt_Temp=%d\r\n", US100_Alt_Temp);
                Alt_CuntTmep2=Alt_CuntTmep1;        //滑动平均滤波
                Alt_CuntTmep1=US100_Alt_Temp*((float)COS(IMU.Pitch/57.295779f))*((float)COS(IMU.Roll/57.295779f));//姿态补偿
                
                US100_Alt=((Alt_CuntTmep1+Alt_CuntTmep2)/2)/1000;  //除以1000转化为m
                
                Alt_V_CuntTmep2=Alt_V_CuntTmep1;//滑动平均滤波
                Alt_V_CuntTmep1=(US100_Alt-US100_Alt_Last)/ 0.05f;   //除以0.05s获得速度单位 ：m/s
                
                US100_Alt_V= (Alt_V_CuntTmep1+Alt_V_CuntTmep2)/2;   
                US100_Alt_Last=US100_Alt;     
            }

            break;
            
        case 4:
            break;
            
        default:
            break;
    }
}

void Send_US100_Start(void)
{
    while(!(UCA1IFG & UCTXIFG));
    UCA1TXBUF = 0x55;
}