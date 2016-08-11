#include "include.h"
#include "msp430F5438A.h"
#include "stdio.h"
//  Tx P9.4
//  Rx P9.5
//  9600

unsigned char Rx_Temp[3];
unsigned char Rx_Temp_count = 0;
unsigned int Distance = 0;

static int US100_Alt_Temp=0; 
float 
        US100_Alt_V,Alt_CuntTmep1=0,Alt_CuntTmep2=0,
        Alt_Last=0,US100_Alt_Last=0,Alt_V_CuntTmep1=0,
        Alt_V_CuntTmep2=0,US100_Alt; 


void Uart2_Init(void)
{
    P9SEL |= BIT4 + BIT5;
    UCA2CTL1 |= UCSWRST;//�����λ
    UCA2CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA2BR0 = 0xe2;
    UCA2BR1 = 0x04;
    UCA2MCTL |= UCBRF_0 + UCBRS_0;
    UCA2CTL1 &= ~UCSWRST;//�����λ �ر�
    UCA2IE |= UCRXIE;//�����ж�
}

//int putchar(int ch)
//{
//    while(!(UCA2IFG & UCTXIFG));
//    UCA2TXBUF = (char) ch;
//    return ch;
//}


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
                US100_Alt_Temp = (Rx_Temp[0] << 8) + Rx_Temp[1];
//                printf("US100_Alt_Temp=%d\r\n", US100_Alt_Temp);
                
                if(US100_Alt_Temp>0x1194)     //���ֵ 4500 
                { 
                    US100_Alt_Temp = Alt_Last; 
                } 
                else   
                { 
                    Alt_Last=US100_Alt_Temp; 
                }
                Alt_CuntTmep2=Alt_CuntTmep1;        //����ƽ���˲�
                Alt_CuntTmep1=US100_Alt_Temp*((float)COS(IMU.Pitch/57.295779f))*((float)COS(IMU.Roll/57.295779f));//��̬����
                
                US100_Alt=((Alt_CuntTmep1+Alt_CuntTmep2)/2)/1000;  //����1000ת��Ϊm
                
                Alt_V_CuntTmep2=Alt_V_CuntTmep1;//����ƽ���˲�
                Alt_V_CuntTmep1=(US100_Alt-US100_Alt_Last)/ 0.05f;   //����0.05s����ٶȵ�λ ��m/s
                
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
    while(!(UCA2IFG & UCTXIFG));
    UCA2TXBUF = 0x55;
}