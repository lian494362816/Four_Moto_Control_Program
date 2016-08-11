#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/



typedef struct {
	      unsigned char MpuExist;      // MPU存在
	      unsigned char MagExist;      // MAG存在
	      unsigned char NrfExist;      // NRF存在
	      unsigned char MagIssue;      // MAG有问题
          unsigned char MsExist;
        unsigned char ARMED;         // 电机解锁
	      unsigned char LockYaw;       // 航向锁定  
        unsigned char calibratingR;	// 遥控器校准
        unsigned char calibratingA;  // 加速度采集
	      unsigned char calibratingM;  // 磁力计采集
	      unsigned char calibratingM_pre; //磁力计预采集
	      unsigned char calibratingG;
	      unsigned char ParamSave;     // 参数保存标志
	
	      unsigned char Loop_200Hz;
	      unsigned char Loop_100Hz;
				unsigned char Loop_40Hz;
				unsigned char Loop_27Hz;
				unsigned char Loop_20Hz;
	      unsigned char Loop_10Hz;
        unsigned char fortest;//加来临时用
				unsigned char FlightMode;
				unsigned char HUDMode;
				unsigned char ControlMode;
				unsigned char Special_Mode;
         }Flag_t;


extern Flag_t flag;

#endif