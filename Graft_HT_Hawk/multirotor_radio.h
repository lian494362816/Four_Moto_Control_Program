#ifndef __Multirotor_Radio_H
#define __MUltirotor_Radio_H


unsigned char HtoEs_PID_Data_Generate(void);
unsigned char HtoEs_Attitude_Data_Generate(void);
void Printf_PID(void);
#define PITCH_YAW_ROLL   1//��λ������  pitch,yaw,roll���ڻ�PID

//#define PITCH_ROLL_SHELL  2//��λ������  pitch,roll,���ڻ�PID �� �⻷PI
                          //���к����PIΪpitch��PI, �߶ȵ�PIΪroll��PI

#endif  