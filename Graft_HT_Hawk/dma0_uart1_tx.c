#include "include.h"

void DMA0_Uart1_Tx_Init(void)
{
    DMACTL0 |= DMA0TSEL_21;//USCIA1 transmit
    DMA0CTL |= DMASRCINCR_3 + DMADSTINCR_0; //Դ��ַ���ӣ�Ŀ�ĵ�ַ����
    DMA0CTL |= DMADSTBYTE + DMASRCBYTE;//Դ λ���� Ŀ�� λ����
    DMA0CTL |= DMALEVEL + DMADT_0;//��ƽ����, ���δ���
}

void DMA0_Transtmit(unsigned short source_addr, unsigned short destination_addr, int len)
{
    DMA0SA = source_addr;//Դ��ַ
    DMA0DA = destination_addr;//Ŀ���ַ
    DMA0SZ = len;
    DMA0_START;
}