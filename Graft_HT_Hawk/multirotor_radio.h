#ifndef __Multirotor_Radio_H
#define __MUltirotor_Radio_H


unsigned char HtoEs_PID_Data_Generate(void);
unsigned char HtoEs_Attitude_Data_Generate(void);
void Printf_PID(void);
#define PITCH_YAW_ROLL   1//上位机调参  pitch,yaw,roll的内环PID

//#define PITCH_ROLL_SHELL  2//上位机调参  pitch,roll,的内环PID 和 外环PI
                          //其中航向的PI为pitch的PI, 高度的PI为roll的PI

#endif  