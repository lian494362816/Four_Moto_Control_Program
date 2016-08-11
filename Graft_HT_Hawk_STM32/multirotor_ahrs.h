#ifndef __Multirotor_ahrs_h
#define __Multirotor_ahrs_h

#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426
void AHRS_getValues(void);

void AHRS_Geteuler(void);
#endif