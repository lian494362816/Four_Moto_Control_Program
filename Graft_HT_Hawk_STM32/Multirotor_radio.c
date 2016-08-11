#include "include.h"

int F,sw1,sw2; 

unsigned char data_to_send[50];	//�������ݻ���

unsigned char HtoEs_OutPut_Buffer[64] = {0};	   //���ڷ��ͻ�����
unsigned char HtoEs_test[64]={1,2,3,4,5,6,7,8,9,10};
unsigned int CHK_SUM;  //У���
extern int16 Moto_duty[MOTOR_NUM];
extern int16 magdata[6];
extern fp32 yawangle;
//extern vu16 US100_Alt_Temp;

unsigned char Flag_PID_Changed=0;

///////////////////////////////////////////////////////////////////
// PID ����

int Pitch_PID_P;
int Pitch_PID_I;
int Pitch_PID_D;
int Pitch_PID_P_S;
int Pitch_PID_I_S;


int Roll_PID_P;
int Roll_PID_I;
int Roll_PID_D;
int Roll_PID_P_S;
int Roll_PID_I_S;

int Yaw_PID_P;
int Yaw_PID_I;
int Yaw_PID_D;

int Alt_PID_P;
int Alt_PID_I;
int Alt_PID_D;

extern struct _PID PID_US100;//����������

//������̬����֡
unsigned char HtoEs_Attitude_Data_Generate(void)
{
    unsigned char i,Count=0;
    int16 _temp;
	
    HtoEs_OutPut_Buffer[Count++] = 0xFE; //��ʼ֡
    HtoEs_OutPut_Buffer[Count++] = 0x0C; //֡����
    HtoEs_OutPut_Buffer[Count++] = 0x01; //������
    
    _temp = (int)(IMU.Roll*100);
    HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
    HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
    
    _temp = (int)(-IMU.Pitch*100);
    HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
    HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
    
    _temp = (int)((IMU.Yaw+180)*100);
    HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
    HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
    
    
    _temp = (int)(0/10);//ultra_dis_lpf/10*100);
    HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
    HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
	//============================================================================	
    
    CHK_SUM =0; 
	
    for(i = 0 ; i < Count; i++)  //�����
        CHK_SUM += HtoEs_OutPut_Buffer[i];
    
    HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //����У��ֵ
 
      /* UART1*/
    for(i = 0; i < Count; i ++)//���ڽ��������
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = HtoEs_OutPut_Buffer[i];
    }
    
     /* UART0*/
//      for(i = 0; i < Count; i ++)//���ڽ��������
//    {
//        while(!(UCA0IFG & UCTXIFG));
//        UCA0TXBUF = HtoEs_OutPut_Buffer[i];
//    }
    //DMA0_Transtmit((unsigned short) &HtoEs_OutPut_Buffer, (unsigned short) &UCA1TXBUF, Count);
    return Count; 
}


//����PID����֡
unsigned char HtoEs_PID_Data_Generate(void)
{
    unsigned char i,Count=0;
	
    Pitch_PID_P = (int)(ctrl.pitch.core.kp   * 1000);
    Pitch_PID_I = (int)(ctrl.pitch.core.ki   * 1000);
    Pitch_PID_D = (int)(ctrl.pitch.core.kd   * 1000);
    Roll_PID_P  = (int)(ctrl.roll.core.kp    * 1000);
    Roll_PID_I  = (int)(ctrl.roll.core.ki    * 1000);
    Roll_PID_D  = (int)(ctrl.roll.core.kd    * 1000);
    
   #ifdef PITCH_YAW_ROLL   
    Yaw_PID_P   = (int)(ctrl.yaw.core.kp     * 1000);
    Yaw_PID_I   = (int)(ctrl.yaw.core.ki     * 1000);
    Yaw_PID_D   = (int)(ctrl.yaw.core.kd     * 1000);
    Alt_PID_P   = 0;//��ʱδʹ�ö�Ϊ0
    Alt_PID_I   = 0;
    Alt_PID_D   = 0;
//    Alt_PID_P   = ultra_wz_speed_pid.kp * 1000;
//	Alt_PID_I   = ultra_wz_speed_pid.ki * 1000;
//	Alt_PID_D   = ultra_wz_speed_pid.kd * 1000;
    
   #elif PITCH_ROLL_SHELL
    Pitch_PID_P_S = (int) ctrl.pitch.shell.kp  * 1000;//yaw�� PI ��Ϊpitch���⻷PI
	Pitch_PID_I_S = (int) ctrl.pitch.shell.ki  * 1000;
	Yaw_PID_D   = 0;//��δʹ�ã���Ϊ0
	Roll_PID_P_S = (int) ctrl.roll.shell.kp * 1000;//���ߵ� PI ��Ϊyaw���⻷PI
    Roll_PID_I_S = (int) ctrl.roll.shell.ki * 1000;
	Alt_PID_D   = 0;//��δʹ�ã���λ0
   
   #endif
    
    
    //printf("pit.core.kp= %f\r\n",ctrl.pitch.core.kp);
    //printf("%d\r\n",Pitch_PID_P);
	  
    HtoEs_OutPut_Buffer[Count++] = 0xFE; //��ʼ֡
    HtoEs_OutPut_Buffer[Count++] = 0x1C; //֡����
    HtoEs_OutPut_Buffer[Count++] = 0xEB; //������
    
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Pitch_PID_P); //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Pitch_PID_P); //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Pitch_PID_I); //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Pitch_PID_I); //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Pitch_PID_D); //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Pitch_PID_D); //ȡ��8λ
	
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Roll_PID_P);  //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Roll_PID_P);  //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Roll_PID_I);  //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Roll_PID_I);  //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Roll_PID_D);  //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Roll_PID_D);  //ȡ��8λ
    
   #ifdef PITCH_YAW_ROLL 
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Yaw_PID_P);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Yaw_PID_P);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Yaw_PID_I);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Yaw_PID_I);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Yaw_PID_D);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Yaw_PID_D);   //ȡ��8λ
    
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Alt_PID_P);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Alt_PID_P);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Alt_PID_I);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Alt_PID_I);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Alt_PID_D);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Alt_PID_D);   //ȡ��8λ
    
   #elif PITCH_ROLL_SHELL
         
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Pitch_PID_P_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Pitch_PID_P_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Pitch_PID_I_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Pitch_PID_I_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Yaw_PID_D);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Yaw_PID_D);   //ȡ��8λ
    
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Roll_PID_P_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Roll_PID_P_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Roll_PID_I_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Roll_PID_I_S);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE1(Alt_PID_D);   //ȡ��8λ
    HtoEs_OutPut_Buffer[Count++] = BYTE0(Alt_PID_D);   //ȡ��8λ
    
   #endif
    
    CHK_SUM =0; 
	
    for(i = 0 ; i < Count; i++)  //�����
        CHK_SUM += HtoEs_OutPut_Buffer[i];
    
    HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //����У��ֵ
	
    /* UART1*/
    for(i = 0; i < Count; i ++)//���ڽ��������
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = HtoEs_OutPut_Buffer[i];
    }
    
    
        /* UART0*/
//    for(i = 0; i < Count; i ++)//���ڽ��������
//    {
//        while(!(UCA0IFG & UCTXIFG));
//        UCA0TXBUF = HtoEs_OutPut_Buffer[i];
//    }
    return Count; 
}

/****************************************************************************
* ��    �ƣ�void Printf_PID(void)
* ��    �ܣ��ڴ������PID�Ĳ���         
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ���ں�����λ��
* ��    ��:
****************************************************************************/
void Printf_PID(void)
{
   #ifdef PITCH_YAW_ROLL
   
    printf("PITCH����\r\n");
    printf("ctrl.pitch.core.kp=%f\r\n", ctrl.pitch.core.kp);
    printf("ctrl.pitch.core.ki=%f\r\n", ctrl.pitch.core.ki);
    printf("ctrl.pitch.core.kd=%f\r\n", ctrl.pitch.core.kd);

    printf("ROLL����\r\n");
    printf("ctrl.roll.core.kp=%f\r\n", ctrl.roll.core.kp);
    printf("ctrl.roll.core.ki=%f\r\n", ctrl.roll.core.ki);
    printf("ctrl.roll.core.kd=%f\r\n", ctrl.roll.core.kd);
    
    printf("YAW����\r\n");
    printf("ctrl.yaw.core.kp=%f\r\n", ctrl.yaw.core.kp);
    printf("ctrl.yaw.core.ki=%f\r\n", ctrl.yaw.core.ki);
    printf("ctrl.yaw.core.kd=%f\r\n", ctrl.yaw.core.kd);
    
   #elif PITCH_ROLL_SHELL
    printf("PITCH����\r\n");
    printf("ctrl.pitch.core.kp=%f\r\n", ctrl.pitch.core.kp);
    printf("ctrl.pitch.core.ki=%f\r\n", ctrl.pitch.core.ki);
    printf("ctrl.pitch.core.kd=%f\r\n", ctrl.pitch.core.kd);
    printf("ctrl.pitch.shell.kp=%f\r\n", ctrl.pitch.shell.kp);
    printf("ctrl.pitch.shell.ki=%f\r\n", ctrl.pitch.shell.ki);
    
    printf("ROLL����\r\n");
    printf("ctrl.roll.core.kp=%f\r\n", ctrl.roll.core.kp);
    printf("ctrl.roll.core.ki=%f\r\n", ctrl.roll.core.ki);
    printf("ctrl.roll.core.kd=%f\r\n", ctrl.roll.core.kd);
    printf("ctrl.roll.shell.kp=%f\r\n", ctrl.roll.shell.kp);
    printf("ctrl.roll.shell.ki=%f\r\n", ctrl.roll.shell.ki);
    
   #endif
    
}


/****************************************************************************
* ��    �ƣ�void Printf_PID(void)
* ��    �ܣ��ڴ������PID�Ĳ���         
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ����������λ��
* ��    ��:
****************************************************************************/
void Printf_PID_ANO(void)
{
    printf("\r\n \r\n");
    printf("PITCH�ڻ�����\r\n");
    printf("ctrl.pitch.core.kp=%f\r\n", ctrl.pitch.core.kp);
    printf("ctrl.pitch.core.ki=%f\r\n", ctrl.pitch.core.ki);
    printf("ctrl.pitch.core.kd=%f\r\n", ctrl.pitch.core.kd);

    printf("ROLL�ڻ�����\r\n");
    printf("ctrl.roll.core.kp=%f\r\n", ctrl.roll.core.kp);
    printf("ctrl.roll.core.ki=%f\r\n", ctrl.roll.core.ki);
    printf("ctrl.roll.core.kd=%f\r\n", ctrl.roll.core.kd);
    
    printf("YAW�ڻ�����\r\n");
    printf("ctrl.yaw.core.kp=%f\r\n", ctrl.yaw.core.kp);
    printf("ctrl.yaw.core.ki=%f\r\n", ctrl.yaw.core.ki);
    printf("ctrl.yaw.core.kd=%f\r\n", ctrl.yaw.core.kd);
    
    printf("PITCH�⻷����\r\n");
    printf("ctrl.pitch.shell.kp=%f\r\n", ctrl.pitch.shell.kp);
    printf("ctrl.pitch.shell.ki=%f\r\n", ctrl.pitch.shell.ki);
    
    printf("ROLL�⻷����\r\n");
    printf("ctrl.roll.shell.kp=%f\r\n", ctrl.roll.shell.kp);
    printf("ctrl.roll.shell.ki=%f\r\n", ctrl.roll.shell.ki);
    
    printf("YAW����\r\n");
    printf("ctrl.yaw.shell.kp=%f\r\n", ctrl.yaw.shell.kp);
   
    printf("����������\r\n");
    printf("PID_US100.P=%f\r\n", PID_US100.P);
    printf("PID_US100.I=%f\r\n", PID_US100.I);
    printf("PID_US100.D=%f\r\n", PID_US100.D);
}

/****************************************************************************
* ��    �ƣ�void ANO_DT_Send_PID_1(unsigned char group,float p1_p,float p1_i,float p1_d,
                                                       float p2_p,float p2_i,float p2_d,
                                                       float p3_p,float p3_i,float p3_d)
* ��    �ܣ���������λ������PID����          
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ��������վ4.06  Э��汾4.01
* ��    ��:
****************************************************************************/
void ANO_DT_Send_PID_1(unsigned char group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	unsigned char _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = (int16)(p1_p * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p1_i  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p1_d  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p2_p  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p2_i  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p2_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p3_p  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p3_i  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p3_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	
	data_to_send[3] = _cnt-4;
	
	unsigned char sum = 0;
	for(unsigned char i=0;i<_cnt;i++)
    {
		sum += data_to_send[i];
    }
	
	data_to_send[_cnt++]=sum;


    for(unsigned char i = 0; i < _cnt; i ++)//���ڽ��������
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = data_to_send[i];
    }
    
   
}

/****************************************************************************
* ��    �ƣ�void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32 alt, unsigned char fly_model, unsigned char armed)
* ��    �ܣ���������λ�����͵�ǰ��̬         
* ��ڲ����� angle_pit  ǰ��Ӧ�üӸ�����
* ���ڲ�����
* ˵    ��:        ��������վ4.06  Э��汾4.01
* ��    ��:
****************************************************************************/
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32 alt, unsigned char fly_model, unsigned char armed)
{
	unsigned char _cnt=0;
	int16 _temp;
	int32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	unsigned char sum = 0;
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	  for(unsigned char i = 0; i < _cnt; i ++)//���ڽ��������
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = data_to_send[i];
    }
    
//     DMA0_Uart1_Transtmit((unsigned short) data_to_send, _cnt);
	
}

/****************************************************************************
* ��    �ƣ�void ANO_DT_Send_PID_2(unsigned char group,float p1_p,float p1_i,float p1_d,
                                                       float p2_p,float p2_i,float p2_d,
                                                       float p3_p,float p3_i,float p3_d)
* ��    �ܣ���������λ������PID����          
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: ��������վ4.06  Э��汾4.01
* ��    ��:
****************************************************************************/
void ANO_DT_Send_PID_2(unsigned char group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	unsigned char _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = (int16)(p1_p * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p1_i  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p1_d  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p2_p  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p2_i  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p2_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p3_p  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p3_i  * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int16)(p3_d * 1000);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	unsigned char sum = 0;
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

//	ANO_DT_Send_Data(data_to_send, _cnt);
    for(unsigned char i = 0; i < _cnt; i ++)//���ڽ��������
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = data_to_send[i];
    }
}



void ANO_DT_Send_Senser(int16 a_x,int16 a_y,int16 a_z,
                        int16 g_x,int16 g_y,int16 g_z,
                        int16 m_x,int16 m_y,int16 m_z)
{
	unsigned char _cnt=0;
	int16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	unsigned char sum = 0;
	for(unsigned char i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

//	ANO_DT_Send_Data(data_to_send, _cnt);
    for(unsigned char i = 0; i < _cnt; i ++)//���ڽ��������
    {
        while(!(UCA1IFG & UCTXIFG));
        UCA1TXBUF = data_to_send[i];
    }
}