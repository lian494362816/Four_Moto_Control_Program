#ifndef __UART0_H
#define __UART0_H
#define uchar  unsigned char
#define Compare_Long 5//�Ƚ���ĸ�ĳ��ȣ��޸����������ĸ�ĳ���Ӧ����Ӧ�ĸı�
void Uart0_Init(void);
int putchar(int ch);
uchar Compare(uchar *buffer1, uchar *buffer2);
void Compare_All(void);
void Change(void);
void Printf_PID_Uart0(void);
void Printf_PID_Uart0_ANO(void);
typedef struct {
                   uchar ROL_SP;    //ȷ���޸�ROL�����ı�־λ
                   uchar ROL_SI;
                   uchar ROL_CP;
                   uchar ROL_CI; 
                   uchar ROL_CD;
                   
                   uchar PIT_SP;    //ȷ���޸�PIT�����ı�־λ
                   uchar PIT_SI;
                   uchar PIT_CP;
                   uchar PIT_CI;
                   uchar PIT_CD;

                   uchar YAW_SP;    //ȷ���޸�YAW�����ı�־λ
                   uchar YAW_SI;
                   uchar YAW_CP;
                   uchar YAW_CI;
                   uchar YAW_CD;
                   
                   uchar command;
                 }_Flag;

#endif