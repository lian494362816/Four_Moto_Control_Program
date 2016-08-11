#include "msp430F5438A.h"
#include "clock.h"
#include "delay.h"
#include "uart0.h"
#include "stdio.h"
#include "include.h"
extern char Flag_PID_Change ;
void INIT(void);
extern char Flag_ANO_DT_Send_Status;
/*   IIC  */
//SDA P10.1
//SCL P10.2

/*  PWM����*/
//  P1.2/TA0.1|--> YAW
//  P1.3/TA0.2|-->THROTTLE
//  P1.4/TA0.3|-->PITCH
//  P1.5/TA0.4|-->ROLL


/*  PWM ��� */
//  P4.2/TB2|--> CCR2       
//  P4.3/TB3|--> CCR3 
//  P4.4/TB4|--> CCR4 
//  P4.5/TB5|--> CCR5
  
/* UART0 */
//    �����봮����������
//    Tx P3.4
//    Rx P3.5

/* UART1 */
//    ��������ص���վ����λ��������
//    Tx P5.6
//    Rx P5.7

/*
    1,���ݱ������� MPU6050 HMC5883L
    2,��һ��LED����������������У׼
     ��һ��LED��������У׼������У׼�ɹ�
    3,������һ��ʾ�����⣬����ȥ��

    4,���������������ŵ�����ͣ� ���ֱߵ�ң��Ҳ�����
     Ȼ�󿴼�4��LED����0.5��

    5,����1��������λ�������
     ����0�����봮����������

    6,LED4�����ɻ�����
    
    
    7,������У׼��ʹ�ɻ��������������Ŵ���ͣ���ҡ�˴����
      ��ʱ4��LED��2��
      LED1��ʱ���ɿذ�ƽ�Ų���ת��LED1��
      LED2��ʱ���ɿذ�X�����²���ת��LED2��
      LED3��ʱ���ɿذ�Y�����²���ת��LED3��
*/
// 1,3 ����P��
// 2,4 ����R��
void main( void )
{
    
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    INIT();
    _EINT();
    while(1) 
    {   
    } 
}

void INIT(void)
{
    Clock_Init();
    //LED_Init();
    Uart0_Init();
    Uart1_Init();
    I2C_Init();
    printf("\r\n printf test!\r\n");//����printf����
    MPU6050_Init();
    MPU6050_WHO_AM_I();//���MPU6050 �Ƿ����ӳɹ� 
    Init_HMC5883L();
    Identify_HMC5883L();
    paramLoad();
    flag.calibratingA = 0; //���ٶȼƲ���
    TimerA_PWM_In_Init();
    TimerB_PWM_Out_Init();
    Printf_PID_ANO();
    flag.ARMED = 0;//�ɻ�����  
//    DMA0_Uart1_Tx_Init();
//    LED4_ON; //��ʾ�ɻ�������
//    DMA0_Uart1_Tx_Init();
    //flag.ARMED = 1; //�������
    
    /* ��ȡ֮ǰ�����PID*/
//    flash_to_PID();
//    Printf_PID();
    Flag_ANO_DT_Send_Status  = 0;

    
}