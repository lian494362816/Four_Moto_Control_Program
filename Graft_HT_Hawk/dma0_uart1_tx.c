#include "include.h"

void DMA0_Uart1_Tx_Init(void)
{
    DMACTL0 |= DMA0TSEL_21;//USCIA1 transmit
    DMA0CTL |= DMASRCINCR_3 + DMADSTINCR_0; //源地址增加，目的地址不变
    DMA0CTL |= DMADSTBYTE + DMASRCBYTE;//源 位传送 目标 位传送
    DMA0CTL |= DMALEVEL + DMADT_0;//电平触发, 单次传送
}

void DMA0_Transtmit(unsigned short source_addr, unsigned short destination_addr, int len)
{
    DMA0SA = source_addr;//源地址
    DMA0DA = destination_addr;//目标地址
    DMA0SZ = len;
    DMA0_START;
}