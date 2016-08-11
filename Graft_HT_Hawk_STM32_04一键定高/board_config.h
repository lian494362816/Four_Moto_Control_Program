#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H


/*----------------陀螺仪采集限幅--------------------*/
//#define GYRO_GATHER   30 

#define GYRO_GATHER   250

//#define TT                    0.0025 //控制周期2.5ms，与定时器5中断时间对应
#define TT                    0.0065 //控制周期13ms，与定时器5中断时间对应
#define MAX_PWM				  100		 //最大PWM输出为100%油门
#define MAX_THR               80 		 //油门通道最大占比80%，留20%给控制量

/*--------------------电机怠速----------------------*/
//#define IDLING  200
#define IDLING  800
#define MOTOR_NUM 4 
#endif