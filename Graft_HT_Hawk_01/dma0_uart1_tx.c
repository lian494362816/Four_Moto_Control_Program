#include "include.h"

void DMA0_Uart1_Tx_Init(void)
{
    DMACTL0 |= DMA0TSEL_0;//DMA_REQ 
    DMA0CTL |= DMASRCINCR_3 + DMADSTINCR_0; //源地址增加，目的地址不变
    DMA0CTL |= DMADSTBYTE + DMASRCBYTE;//源 位传送 目标 位传送
    DMA0CTL |= DMALEVEL + DMADT_4  ;//电平触发, 连续单次传送
//    DMA0DA = &UCA1TXBUF;//目标地址
    DMA0DA = 0x060Eu;//目标地址
    
}

void DMA0_Uart1_Transtmit(unsigned short source_addr, int len)
{
    DMA0SA = source_addr;//源地址
    DMA0SZ = len;
    DMA0CTL |= DMAEN;
    for(int j = 0; j < len; j++)
    {
        while(!(DMA0CTL & DMAREQ));
        {       
            //DMA0CTL |= DMAREQ; 
            while(!(UCA1IFG & UCTXIFG));
        }
    }
//    DMA0_START;
}