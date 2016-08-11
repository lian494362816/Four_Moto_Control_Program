#ifndef __UART1_H
#define __UART1_H

void Uart1_Init(void);
#define Temp_PID_count     30 //用于接收上位机PID数据的长度
void Temp_PID_Change_To_PID(void);
void Temp_PID_Change_To_PID_ANO_1(void);
void Temp_PID_Change_To_PID_ANO_2(void);
void Temp_PID_Change_To_PID_ANO_3(void);
void ANO_DT_Send_Check(unsigned head, unsigned check_sum);
void Printf_PID_ANO(void);
void ANO_DT_Send_PID_1(unsigned char group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_PID_2(unsigned char group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
//int putchar(int ch);
#endif