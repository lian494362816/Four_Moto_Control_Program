/* STM32_04 һ������*/

#include "msp430F5438A.h"
#include "clock.h"
#include "delay.h"
#include "uart0.h"
#include "stdio.h"
#include "include.h"
extern char Flag_PID_Change ;
void INIT(void);
extern char Flag_ANO_DT_Send_Status;
extern struct _PID PID_US100;//����������
extern struct ADNS_PID PID_ADNS3080;//��������
extern float US100_Alt;//�߶�
extern float THR_Lock;//�߶ȵ�����
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
//    �����봮���������
//    Tx P3.4
//    Rx P3.5

/* UART1 */
//    ������ģ��
//    Tx P5.6   �ӳ�������Tx
//    Rx P5.7   �ӳ�������Rx
 
/* ��������λ������*/
//   Tx  P9.4  ������ģ�� Rx   
//   Rx  P9.5  ������ģ�� Tx

/* ADNS3080����ģ��*/
//   NCS    P6.1    Chip Select
//   MISO   P3.2
//   MOSI   P3.1
//   CLK    P3.3
//   Rset   P6.2   
//   NPD    P6.3    Power Down





/* 
    ��ǰ  Y-
    ����  Y+
    ����  X+
    ����  X-
*/

/*
    1,���ݱ������� MPU6050 HMC5883L
    2,��һ��LED����������������У׼
     ��һ��LED��������У׼������У׼�ɹ�
    3,������һ��ʾ�����⣬����ȥ��

    4,���������������ŵ�����ͣ� ���ֱߵ�ң��Ҳ�����
     Ȼ�󿴼�4��LED����0.5��

    5,����1��������λ�������
     ����0�����봮���������

    6,LED4����ɻ�����
    
    
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
    P9DIR |= BIT0 + BIT1 + BIT2 + BIT3;
    P9OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);
    P6DIR |= BIT7;
    P6OUT &= ~BIT7;
    while(1) 
    {        
//        ADNS3080_Read_Position();
//        Delay_ms(10);
//        ADNS3080_CONTROL();		
    } 
}

void INIT(void)
{
    Clock_Init();
    //LED_Init();
    Uart0_Init();
    Uart1_Init();
    Uart2_Init();
    I2C_Init();
    SPI_init();
    ADNS3080_Init();
    printf("\r\n printf test!\r\n");//����printf����
    MPU6050_Init();
    MPU6050_WHO_AM_I();//���MPU6050 �Ƿ����ӳɹ� 
    Init_HMC5883L();
    Identify_HMC5883L();
    paramLoad();
    flag.calibratingA = 0; //���ٶȼƲ���
    TimerA_PWM_In_Init();
    TimerB_PWM_Out_Init();
    TimerA1_PWM_In_Init();
    Key_Init();
    Printf_PID_ANO();
    flag.ARMED = 0;//�ɻ�����  
   
//    DMA0_Uart1_Tx_Init();
//    LED4_ON; //��ʾ�ɻ�������
//    DMA0_Uart1_Tx_Init();
    //flag.ARMED = 1; //�������
    
    /* ��ȡ֮ǰ�����PID*/
//    flash_to_PID();
//    Printf_PID();
    Flag_ANO_DT_Send_Status  = 1;//���͵�ǰ״̬
    ANO_DT_Send_PID_1(1, ctrl.pitch.core.kp, ctrl.pitch.core.ki, ctrl.pitch.core.kd,
                                   ctrl.roll.core.kp, ctrl.roll.core.ki, ctrl.roll.core.kd,
                                   ctrl.yaw.core.kp, ctrl.yaw.core.ki, ctrl.yaw.core.kd);
    ANO_DT_Send_PID_2(2, ctrl.pitch.shell.kp, ctrl.pitch.shell.ki, Flag_ANO_DT_Send_Status,
                                    ctrl.roll.shell.kp, ctrl.roll.shell.ki, 0,
                                    ctrl.yaw.shell.kp, 0, 0);
    ANO_DT_Send_PID_2(3, PID_US100.P, PID_US100.I, PID_US100.D,
                      PID_ADNS3080.P, 0, PID_ADNS3080.D,
                      0, 0, 0);
}