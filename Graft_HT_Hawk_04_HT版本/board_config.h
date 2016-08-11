#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H


/*----------------�����ǲɼ��޷�--------------------*/
//#define GYRO_GATHER   30 

#define GYRO_GATHER   250

//#define TT                    0.0025 //��������2.5ms���붨ʱ��5�ж�ʱ���Ӧ
#define TT                    0.0065 //��������13ms���붨ʱ��5�ж�ʱ���Ӧ
#define MAX_PWM				  100		 //���PWM���Ϊ100%����
#define MAX_THR               80 		 //����ͨ�����ռ��80%����20%��������

/*--------------------�������----------------------*/
//#define IDLING  200
#define IDLING  800
#define MOTOR_NUM 4 



/*----------------------�ظ�ģʽ--------------------*/
#define MANUAL_High      1
#define ULTRASONIC_High  2
#define ATMOSPHERE_High  3
#define AUTO_High        4
#define ERROR_High       5
#define ACC_High         6

#endif