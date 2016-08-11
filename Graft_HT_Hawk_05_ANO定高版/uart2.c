#include "include.h"
#include "msp430F5438A.h"
#include "stdio.h"
//  Tx P9.4
//  Rx P9.5
//  9600

unsigned char Rx_Temp[3];
unsigned char Rx_Temp_count = 0;


unsigned char ultra_start_f;
unsigned char ultra_ok = 0;
uint16 ultra_distance,ultra_distance_old;
int16 ultra_delta;


void Uart2_Init(void)
{
    P9SEL |= BIT4 + BIT5;
    UCA2CTL1 |= UCSWRST;//软件复位
    UCA2CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA2BR0 = 0xe2;
    UCA2BR1 = 0x04;
    UCA2MCTL |= UCBRF_0 + UCBRS_0;
    UCA2CTL1 &= ~UCSWRST;//软件复位 关闭
    UCA2IE |= UCRXIE;//接收中断
}

#pragma vector = USCI_A2_VECTOR
__interrupt void UART2_RX_Handle(void)
{
    switch(__even_in_range(UCA2IV, 4))
    {
        case 0:
            break;
        case 2:
            while(!(UCA2IFG & UCTXIFG));
            Rx_Temp[Rx_Temp_count++]= UCA2RXBUF;
            if(2 == Rx_Temp_count)
            {
                Rx_Temp_count = 0;
                ultra_start_f = 0;
		        ultra_ok = 1;
                ultra_distance = (Rx_Temp[0] << 8) + Rx_Temp[1];
//                printf("ultra_distance = %d\r\n", ultra_distance);  
                
//                if(ultra_distance > 0x1194)     //最大值 4500 
//                { 
//                    ultra_distance = ultra_distance_old; 
//                } 
//                else   
//                { 
//                    ultra_distance_old = ultra_distance; 
//                }
                
                ultra_delta = ultra_distance - ultra_distance_old;
                ultra_distance_old = ultra_distance;
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
    while(!(UCA2IFG & UCTXIFG));
    UCA2TXBUF = 0x55;
    ultra_start_f = 1;
}