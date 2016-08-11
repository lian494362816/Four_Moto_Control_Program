#include "include.h"

void DMA0_Uart1_Tx_Init(void)
{
    DMACTL0 |= DMA0TSEL_0;//DMA_REQ 
    DMA0CTL |= DMASRCINCR_3 + DMADSTINCR_0; //Դ��ַ���ӣ�Ŀ�ĵ�ַ����
    DMA0CTL |= DMADSTBYTE + DMASRCBYTE;//Դ λ���� Ŀ�� λ����
    DMA0CTL |= DMALEVEL + DMADT_4  ;//��ƽ����, �������δ���
//    DMA0DA = &UCA1TXBUF;//Ŀ���ַ
    DMA0DA = 0x060Eu;//Ŀ���ַ
    
}

void DMA0_Uart1_Transtmit(unsigned short source_addr, int len)
{
    DMA0SA = source_addr;//Դ��ַ
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