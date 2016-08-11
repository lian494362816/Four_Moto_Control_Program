#ifndef __UART1_H
#define __UART1_H

void Uart1_Init(void);
#define Temp_PID_count     30 //用于接收上位机PID数据的长度
void Temp_PID_Change_To_PID(void);
void Temp_PID_Change_To_PID_ANO_1(void);
void Temp_PID_Change_To_PID_ANO_2(void);
 void ANO_DT_Send_Check(unsigned head, unsigned check_sum);
 void Printf_PID_ANO(void);
//int putchar(int ch);
#endif