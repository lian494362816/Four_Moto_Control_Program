#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_height_pid;



/*-------------------定高模式参数-------------------*/
#define SecondHigh_Factor 0.02       //与下面的参数一起用于高度控制互补滤波
#define MainHigh_Factor   0.98

#define Ultrasonic_MAX_Height 2500   //超声波最高有效高度，单位是mm
#define Ultrasonic_MIN_Height 200    //超声波最低有效高度，单位是mm
#define Baro_MAX_Height       8000   //气压计最高有效高度，单位是mm,可以根据实际修改
#define Baro_MIN_Height       1000   //气压计最低有效高度，单位是mm,再低其实不太可靠，可以根据实际修改
//#define TT                    0.0025 //控制周期2.5ms，与定时器5中断时间对应
#define CTRL_HEIGHT           1      //0失能，1使能控高功能
#define TAKE_OFF_THR          550    //根据个人飞机实际情况，设置起飞离地油门，用于定高模式下稍推油起飞
#define MAX_PWM				        100		 //最大PWM输出为100%油门
#define MAX_THR               80 		 //油门通道最大占比80%，留20%给控制量

void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed);
void Baro_dataporcess(float T);
void Baro_Ctrl(float T,float thr);
void Ultra_dataporcess(float T);
void Ultra_Ctrl(float T,float thr);
void LockForKeepHigh(float THROTTLE);
#endif