#include "include.h"
#include "msp430F5438A.h"
#include "stdio.h"
//  Tx P9.4
//  Rx P9.5
//  9600

unsigned char Rx_Temp[3];
unsigned char Rx_Temp_count = 0;

uint16 US100_Alt_Temp=0,Alt_Last=0; 
float US100_Alt_Last=0;
extern float US100_Alt;
unsigned char ultra_start_f;
    
unsigned char  ultra_ok ;


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
                ultra_start_f = 1;
		        ultra_ok = 1;
                US100_Alt = (Rx_Temp[0] << 8) + Rx_Temp[1];
//                printf("US100_Alt=%f\r\n", US100_Alt);
                US100_Alt_delta=US100_Alt-US100_Alt_Last;
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
    while(!(UCA2IFG & UCTXIFG));
    UCA2TXBUF = 0x55;
    ultra_start_f = 1;
}