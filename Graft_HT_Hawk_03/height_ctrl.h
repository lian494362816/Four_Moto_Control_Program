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



/*-------------------����ģʽ����-------------------*/
#define SecondHigh_Factor 0.02       //������Ĳ���һ�����ڸ߶ȿ��ƻ����˲�
#define MainHigh_Factor   0.98

#define Ultrasonic_MAX_Height 2500   //�����������Ч�߶ȣ���λ��mm
#define Ultrasonic_MIN_Height 200    //�����������Ч�߶ȣ���λ��mm
#define Baro_MAX_Height       8000   //��ѹ�������Ч�߶ȣ���λ��mm,���Ը���ʵ���޸�
#define Baro_MIN_Height       1000   //��ѹ�������Ч�߶ȣ���λ��mm,�ٵ���ʵ��̫�ɿ������Ը���ʵ���޸�
//#define TT                    0.0025 //��������2.5ms���붨ʱ��5�ж�ʱ���Ӧ
#define CTRL_HEIGHT           1      //0ʧ�ܣ�1ʹ�ܿظ߹���
#define TAKE_OFF_THR          550    //���ݸ��˷ɻ�ʵ��������������������ţ����ڶ���ģʽ�����������
#define MAX_PWM				        100		 //���PWM���Ϊ100%����
#define MAX_THR               80 		 //����ͨ�����ռ��80%����20%��������

void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed);
void Baro_dataporcess(float T);
void Baro_Ctrl(float T,float thr);
void Ultra_dataporcess(float T);
void Ultra_Ctrl(float T,float thr);
void LockForKeepHigh(float THROTTLE);
#endif