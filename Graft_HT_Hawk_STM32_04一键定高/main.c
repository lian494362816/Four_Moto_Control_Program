/* STM32_04 一键定高*/

#include "msp430F5438A.h"
#include "clock.h"
#include "delay.h"
#include "uart0.h"
#include "stdio.h"
#include "include.h"
extern char Flag_PID_Change ;
void INIT(void);
extern char Flag_ANO_DT_Send_Status;
extern struct _PID PID_US100;//超声波参数
extern struct ADNS_PID PID_ADNS3080;//光流参数
extern float US100_Alt;//高度
extern float THR_Lock;//高度的油门
/*   IIC  */ 
//SDA P10.1
//SCL P10.2

/*  PWM输入*/
//  P1.2/TA0.1|--> YAW
//  P1.3/TA0.2|-->THROTTLE
//  P1.4/TA0.3|-->PITCH
//  P1.5/TA0.4|-->ROLL


/*  PWM 输出 */
//  P4.2/TB2|--> CCR2       
//  P4.3/TB3|--> CCR3 
//  P4.4/TB4|--> CCR4 
//  P4.5/TB5|--> CCR5
  
/* UART0 */
//    用来与串口软件相连
//    Tx P3.4
//    Rx P3.5

/* UART1 */
//    超声波模块
//    Tx P5.6   接超声波的Tx
//    Rx P5.7   接超声波的Rx
 
/* 与匿名上位机相连*/
//   Tx  P9.4  接蓝牙模块 Rx   
//   Rx  P9.5  接蓝牙模块 Tx

/* ADNS3080光流模块*/
//   NCS    P6.1    Chip Select
//   MISO   P3.2
//   MOSI   P3.1
//   CLK    P3.3
//   Rset   P6.2   
//   NPD    P6.3    Power Down





/* 
    往前  Y-
    往后  Y+
    往左  X+
    往右  X-
*/

/*
    1,数据保存问题 MPU6050 HMC5883L
    2,第一个LED亮代表正在陀螺仪校准
     第一个LED灭亮代表校准陀螺仪校准成功
    3,磁力计一显示有问题，不用去管

    4,行器解锁，把油门调到最低， 右手边的遥杆也打到最低
     然后看见4个LED灯亮0.5秒

    5,串口1用来与上位相机器连
     串口0用来与串口软件相连

    6,LED4代表飞机上锁
    
    
    7,磁力计校准，使飞机处于上锁，油门打到最低，右摇杆打到最高
      此时4个LED闪2次
      LED1亮时，飞控板平放并旋转至LED1灭
      LED2亮时，飞控板X轴向下并旋转至LED2灭
      LED3亮时，飞控板Y轴向下并旋转至LED3灭
*/
// 1,3 连接P桨
// 2,4 连接R桨
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
    printf("\r\n printf test!\r\n");//测试printf函数
    MPU6050_Init();
    MPU6050_WHO_AM_I();//检测MPU6050 是否连接成功 
    Init_HMC5883L();
    Identify_HMC5883L();
    paramLoad();
    flag.calibratingA = 0; //加速度计补偿
    TimerA_PWM_In_Init();
    TimerB_PWM_Out_Init();
    TimerA1_PWM_In_Init();
    Key_Init();
    Printf_PID_ANO();
    flag.ARMED = 0;//飞机上锁  
   
//    DMA0_Uart1_Tx_Init();
//    LED4_ON; //表示飞机已上锁
//    DMA0_Uart1_Tx_Init();
    //flag.ARMED = 1; //启动电机
    
    /* 读取之前保存的PID*/
//    flash_to_PID();
//    Printf_PID();
    Flag_ANO_DT_Send_Status  = 1;//发送当前状态
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