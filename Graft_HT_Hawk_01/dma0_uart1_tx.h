#ifndef __DMA0_UART1_TX_H
#define __DMA0_UART1_TX_H

#define DMA0_START   DMA0CTL |= DMAEN;


void DMA0_Uart1_Tx_Init(void);
void DMA0_Uart1_Transtmit(unsigned short source_addr, int len);
#endif 