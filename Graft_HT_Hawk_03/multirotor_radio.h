#ifndef __Multirotor_Radio_H
#define __MUltirotor_Radio_H


unsigned char HtoEs_PID_Data_Generate(void);
unsigned char HtoEs_Attitude_Data_Generate(void);
void ANO_DT_Send_PID_1(unsigned char group,float p1_p,float p1_i,float p1_d,
                                           float p2_p,float p2_i,float p2_d,
                                           float p3_p,float p3_i,float p3_d);//匿名上位机发送PID数据

void ANO_DT_Send_PID_2(unsigned char group,float p1_p,float p1_i,float p1_d,
                                           float p2_p,float p2_i,float p2_d,
                                           float p3_p,float p3_i,float p3_d);

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw,
                        int32 alt, unsigned char fly_model, unsigned char armed);
void Printf_PID(void);
#define PITCH_YAW_ROLL   1//上位机调参  pitch,yaw,roll的内环PID

//#define PITCH_ROLL_SHELL  2//上位机调参  pitch,roll,的内环PID 和 外环PI
                          //其中航向的PI为pitch的PI, 高度的PI为roll的PI

#endif  