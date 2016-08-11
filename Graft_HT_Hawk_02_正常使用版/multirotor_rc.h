#ifndef __Multirotor_rc_H
#define __Multiroror_rc_H


typedef struct {
                
                int16 ROLL;
                int16 PITCH;
                int16 THROTTLE;
                int16 YAW;
                int16 SENSITIVITY;
                int16 KEEPHIGH;}RC_GETDATA;

typedef struct {
                int16 ROLL_MAX;
                int16 ROLL_MIN;
                int16 PITCH_MAX;
                int16 PITCH_MIN;
                int16 THROTTLE_MAX;
                int16 THROTTLE_MIN;
                int16 YAW_MAX;
                int16 YAW_MIN;
                int16 CH5_MAX;
                int16 CH5_MIN;}RC_CAL;   

extern  RC_GETDATA RC_Data;

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#endif