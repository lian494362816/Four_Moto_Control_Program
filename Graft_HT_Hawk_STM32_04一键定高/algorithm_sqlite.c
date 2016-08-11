#include "include.h"

//**************************************************************************
//加载参数
//**************************************************************************
extern struct _PID PID_US100;//超声波参数
extern struct ADNS_PID PID_ADNS3080;//光流参数
void paramLoad(void)
{
//	//The data of pitch
//	ctrl.pitch.shell.kp = 4;
//	ctrl.pitch.shell.ki = 0.02;
//	
//	ctrl.pitch.core.kp = 1.4;  
//	ctrl.pitch.core.ki = 0.45; 
//	ctrl.pitch.core.kd = 0.70;
//	
//	//The data of roll
//	ctrl.roll.shell.kp = 4;
//	ctrl.roll.shell.ki = 0.02;
//    
//	ctrl.roll.core.kp = 1.4;
//	ctrl.roll.core.ki = 0.45;
//	ctrl.roll.core.kd = 0.70;
//	
//	//The data of yaw
//	ctrl.yaw.shell.kp = 5;
//	ctrl.yaw.shell.kd = 0;
//	
//	ctrl.yaw.core.kp = 1.8;
//	ctrl.yaw.core.ki = 0;
//	ctrl.yaw.core.kd = 0.1;
	
//    	//The data of pitch
	ctrl.pitch.shell.kp = 3.67;
	ctrl.pitch.shell.ki = 0;
	
	ctrl.pitch.core.kp = 1.0;  
	ctrl.pitch.core.ki = 0; 
	ctrl.pitch.core.kd = 0.09;
	
	//The data of roll
	ctrl.roll.shell.kp = 3.65;
	ctrl.roll.shell.ki = 0;
    
	ctrl.roll.core.kp = 1.0;
	ctrl.roll.core.ki = 0;
	ctrl.roll.core.kd = 0.09;
	
	//The data of yaw
	ctrl.yaw.shell.kp = 5.0;
	ctrl.yaw.shell.kd = 0;
	
	ctrl.yaw.core.kp = 1.0;
	ctrl.yaw.core.ki = 0;
	ctrl.yaw.core.kd = 0.09;
    
//    超声波PID参数
    
//  PID_US100.P=250;
//	PID_US100.D=80;
//	PID_US100.I=0;
    
   /* 所有参数在使用时效果均扩大了10倍*/ 
	PID_US100.P=22.0;//20000~24000
	PID_US100.D=4.4;//4400
	PID_US100.I=0;
    
    
//    PID_ADNS3080.P=20;
//	PID_ADNS3080.D=60;
    PID_ADNS3080.P = 0;
    PID_ADNS3080.D = 0;
       
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;
	ctrl.roll.shell.increment_max = 20;
	
	ctrl.ctrlRate = 0;

	Gyro_OFFSET();          //采集陀螺仪零偏，上电时静止
    

}