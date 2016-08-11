#ifndef __Multirotor_Control_H
#define __Multirotor_Control_H

#define RC_Data_Division   38
#define RC_Data_Roll_Offset  1491
#define RC_Data_Pitch_Offset 1511
#define HEIGHT 1.1f
#define INCREASE_NUM 3 //定高模式下 每次变化的油门值
#define THR_NUM  3    //定高模式下 油门变化的周期: 定时周期 * THR_NUM


struct _pid{
          float kp;
		  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
		  float ki_out;
	      float kd_out;
	      float pid_out;
          };

struct _tache{
    struct _pid shell;
    struct _pid core;	
          };
	
struct _ctrl{
		      uint8  ctrlRate;
      struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;   
            };

struct _target{
      float Pitch;    
	    float Roll;  
	    float Yaw;   
      float Altiude; 
            };

struct _keephigh_point{
      float current_THR;    
	    float current_ALT;  
	    float current_YAW;
			float current_PRESSURE;
            };						


extern struct _ctrl ctrl;
extern struct _target Target;

void Attitude_RatePID(void);
void Thr_Ctrl(float T);
void Motor_Conter(void);
void Reset_Integral(void);
void Calculate_Target(void);
void CONTROL(struct _target Goal);  
void US100_CONTROL(float US100_Alt_Target_1);
void Height_Control_Thr_Up(void);
void Height_Control_Thr_Down(void);


#endif