#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/



typedef struct {
	      unsigned char MpuExist;      // MPU����
	      unsigned char MagExist;      // MAG����
	      unsigned char NrfExist;      // NRF����
	      unsigned char MagIssue;      // MAG������
          unsigned char MsExist;
        unsigned char ARMED;         // �������
	      unsigned char LockYaw;       // ��������  
        unsigned char calibratingR;	// ң����У׼
        unsigned char calibratingA;  // ���ٶȲɼ�
	      unsigned char calibratingM;  // �����Ʋɼ�
	      unsigned char calibratingM_pre; //������Ԥ�ɼ�
	      unsigned char calibratingG;
	      unsigned char ParamSave;     // ���������־
	
	      unsigned char Loop_200Hz;
	      unsigned char Loop_100Hz;
				unsigned char Loop_40Hz;
				unsigned char Loop_27Hz;
				unsigned char Loop_20Hz;
	      unsigned char Loop_10Hz;
        unsigned char fortest;//������ʱ��
				unsigned char FlightMode;
				unsigned char HUDMode;
				unsigned char ControlMode;
				unsigned char Special_Mode;
         }Flag_t;


extern Flag_t flag;

#endif