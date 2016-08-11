#include "msp430F5438A.h"
#include "clock.h"
#include "delay.h"
#include "uart0.h"
#include "stdio.h"
#include "include.h"
extern char Flag_PID_Change ;
void INIT(void);
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
     ����0�����봮���������

    6,LED4����ɻ�����
    
    
    7,������У׼��ʹ�ɻ��������������Ŵ���ͣ���ҡ�˴����
      ��ʱ4��LED��2��
      LED1��ʱ���ɿذ�ƽ�Ų���ת��LED1��
      LED2��ʱ���ɿذ�X�����²���ת��LED2��
      LED3��ʱ���ɿذ�Y�����²���ת��LED3��
*/

void main( void )
{
    
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    _EINT();

    INIT();
//    TA0CCTL0 &= ~CCIE;
//    TA0CCTL1 &= ~CCIE;
//    TA0CCTL2 &= ~CCIE;
//    TA0CCTL3 &= ~CCIE;
//    TA0CCTL4 &= ~CCIE;
    while(1) 
    {   
            
        //AHRS_getValues();
//        if(Flag_PID_Change)
//        {
////            PID_to_flash();
////            float_write_flash();
////            flash_to_PID();
//            //Delay_ms(5);      
//             
//            HtoEs_PID_Data_Generate();//��ʾ��PID�Ĳ��� ,��ʾ2�α�֤�ܹ�����ˢ��
//            HtoEs_PID_Data_Generate();
//            HtoEs_PID_Data_Generate();
//            HtoEs_PID_Data_Generate();
//            Printf_PID();//������ʾPID�Ĳ��� 
//            Flag_PID_Change = 0;
//        }
//        TA0CCR1 = 2000;
//        TA0CCR2 = 2000;
//        TA0CCR3 = 2000;
//        TA0CCR4 = 2000;
//        Delay_ms(5000);
//        TA0CCR1 = 1200;
//        TA0CCR2 = 1200;
//        TA0CCR3 = 1200;
//        TA0CCR4 = 1200;
//        Delay_ms(5000);
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
    MPU6050_Dataanl();
    Init_HMC5883L();
    Identify_HMC5883L();
    paramLoad();
    HtoEs_PID_Data_Generate();  
    flag.calibratingA = 1; //���ٶȼƲ���
    TimerA_PWM_In_Init();
    TimerB_PWM_Out_Init();
    Printf_PID();
    flag.ARMED = 0;//�ɻ�����                                                                                        
//    LED4_ON; //��ʾ�ɻ�������
    //DMA0_Uart1_Tx_Init();
    //flag.ARMED = 1; //�������
    
    /* ��ȡ֮ǰ�����PID*/
    flash_to_PID();
    Printf_PID();
//    HtoEs_PID_Data_Generate();
//    HtoEs_PID_Data_Generate();
    
}