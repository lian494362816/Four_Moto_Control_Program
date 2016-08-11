#ifndef __Multirotor_Radio_H
#define __MUltirotor_Radio_H


unsigned char HtoEs_PID_Data_Generate(void);
unsigned char HtoEs_Attitude_Data_Generate(void);
void ANO_DT_Send_PID_1(unsigned char group,float p1_p,float p1_i,float p1_d,
                                           float p2_p,float p2_i,float p2_d,
                                           float p3_p,float p3_i,float p3_d);//������λ������PID����

void ANO_DT_Send_PID_2(unsigned char group,float p1_p,float p1_i,float p1_d,
                                           float p2_p,float p2_i,float p2_d,
                                           float p3_p,float p3_i,float p3_d);

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw,
                        int32 alt, unsigned char fly_model, unsigned char armed);
void Printf_PID(void);
#define PITCH_YAW_ROLL   1//��λ������  pitch,yaw,roll���ڻ�PID

//#define PITCH_ROLL_SHELL  2//��λ������  pitch,roll,���ڻ�PID �� �⻷PI
                          //���к����PIΪpitch��PI, �߶ȵ�PIΪroll��PI

#endif  