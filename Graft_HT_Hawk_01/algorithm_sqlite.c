#include "include.h"

//**************************************************************************
//���ز���
//**************************************************************************
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
	ctrl.pitch.shell.kp = 0;
	ctrl.pitch.shell.ki = 0;
	
	ctrl.pitch.core.kp = 0;  
	ctrl.pitch.core.ki = 0; 
	ctrl.pitch.core.kd = 0;
	
	//The data of roll
	ctrl.roll.shell.kp = 0;
	ctrl.roll.shell.ki = 0;
    
	ctrl.roll.core.kp = 0;
	ctrl.roll.core.ki = 0;
	ctrl.roll.core.kd = 0;
	
	//The data of yaw
	ctrl.yaw.shell.kp = 0;
	ctrl.yaw.shell.kd = 0;
	
	ctrl.yaw.core.kp = 0;
	ctrl.yaw.core.ki = 0;
	ctrl.yaw.core.kd = 0;
    
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 20;
	ctrl.roll.shell.increment_max = 20;
	
	ctrl.ctrlRate = 0;
	
//  WZ_Speed_PID_Init();    //�߶ȿ���PID��ʼ��
//	Ultra_PID_Init();       //������PID
//	Baro_PID_Init();        //��ѹ��PID
//  EE_READ_ACC_OFFSET();   //��ȡ���ٶ���ƫ
//	EE_READ_MAG_OFFSET();   //��ȡ��������ƫ
//	EE_READ_RC_OFFSET();    //��ȡRCУ׼ֵ
//	EE_READ_Attitude_PID(); //��ȡ�ڻ�PID����
	Gyro_OFFSET();          //�ɼ���������ƫ���ϵ�ʱ��ֹ
    
}