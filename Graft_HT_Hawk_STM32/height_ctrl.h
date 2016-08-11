#ifndef __HEIGHT_CTRL_H
#define __HEIGTH_CTRL_H

#define ACC_SPEED_NUM 50

#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_MAX_HEIGHT 1500   // mm
#define ULTRA_INT        300    // »ý·Ö·ù¶È

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


void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed);
void Ultra_Ctrl(float T,float thr);
void Height_Ctrl(float T,float thr);
void Ultra_PID_Init();
void WZ_Speed_PID_Init();
#endif