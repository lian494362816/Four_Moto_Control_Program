#include "include.h"
#include "math.h"
#include "delay.h"
struct _ctrl ctrl;
struct _target Target;
float Thr_Weight;
unsigned char Thr_Low;
long float thr_value;//����ģʽ�µ�����ֵ
int16 Moto[4];
int16 Moto_duty[4];
int date_throttle;
/*====================================================================================================*/
/*====================================================================================================*
**���� : Calculate_target
**���� : ����Ŀ����
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/

//    YAW       930~2078 ��ֵ1510
//    THROTTLE  936~2083  
//    PITCH     957~2104 ��ֵ1584
//    ROLL      963~2111 ��ֵ1546

void Calculate_Target(void) 
{
	int16 ftemp=0;
    if(RC_Data.ROLL <1446 || RC_Data.ROLL > 1646)
    {
	    Target.Roll = (RC_Data.ROLL-1546)/(30 );
//       printf("Target.Roll=%f\r\n", Target.Roll);
    }
    else
    {
        Target.Roll = 0;
//        printf("Target.Roll=%f\r\n", Target.Roll);
    }
    
//    Target.Roll = (RC_Data.ROLL-1546)/(30 );
//    printf("Target.Roll=%f\r\n", Target.Roll);
    
    if(RC_Data.PITCH < 1484 || RC_Data.PITCH > 1684)
    {
        Target.Pitch = (RC_Data.PITCH - 1584)/(30 );
//        printf("Target.Pitch=%f\r\n", Target.Pitch);
    }
    else
    {
        Target.Pitch = 0;
//        printf("Target.Pitch=%f\r\n", Target.Pitch);
    }
	//Target.Roll = (1560 - RC_Data.ROLL)/(30 );
    
    //printf("RC_Data.PITCH=%d\r\n", RC_Data.PITCH);
  //  printf("RC_Data.ROLL=%d\r\n", RC_Data.ROLL);
    
    //printf("Target.Pitch=%f\r\n", Target.Pitch);
    //printf("Targer.Roll=%f\r\n", Target.Roll);
    //printf("RC_Data.YAW=%d\r\n", RC_Data.YAW);
    //Ŀ�꺽����ơ������Ŵ�����С���ֵʱ����Ϊ�û�ϣ����ɡ���ô��ʱ�ĺ�����ΪĿ�꺽��
    if(RC_Data.THROTTLE > 1100 ) 
    {
        if(flag.LockYaw != 1)
        {  
            flag.LockYaw = 1;
            Target.Yaw = IMU.Yaw; //����ǰ�ĺ�����ΪĿ�꺽��
        }
    }
    else 
    {
        flag.LockYaw = 0;	
        Target.Yaw = IMU.Yaw;
        
    } 
    
	//�������е�����һ���������ô����ֲ���ʱ����С����
	if((RC_Data.YAW > 1550)||(RC_Data.YAW < 1450))
    {
		ftemp = 1510 - RC_Data.YAW; 
        Target.Yaw += (ftemp / 200.0f)*0.1f; 
		//printf("Target.Yaw=%f\r\n", Target.Yaw);
		//ת[-180.0,+180.0]
        if(Target.Yaw >180.0f) 
        {
            Target.Yaw -= 360.0f;	
        }
        else if(Target.Yaw <-180.0f)
        {
            Target.Yaw += 360.0f;
        }
	}
   // printf("Target.Yaw=%f\r\n", Target.Yaw);
          

    
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : CONTROL(struct _target Goal) 
**���� : ����PID����
**���� : Goal
**ݔ�� : None
**��ע : YAW ֻ�����⻷P
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	
	if(ctrl.ctrlRate >= 2)
	{//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//*****************�⻷(�ǶȻ�)PID**************************//
		//�������///////////////
        deviation_pitch = Goal.Pitch - IMU.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;
		
		//limit for the max increment
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
        
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//��������//////////////
		deviation_roll = Goal.Roll - IMU.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);
        
		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//�������////////////
        if((Goal.Yaw - IMU.Yaw)>180 || (Goal.Yaw - IMU.Yaw)<-180)
        {
            if(Goal.Yaw>0 && IMU.Yaw<0)  deviation_yaw= (-180 - IMU.Yaw) +(Goal.Yaw - 180);
            if(Goal.Yaw<0 && IMU.Yaw>0)  deviation_yaw= (180 - IMU.Yaw) +(Goal.Yaw + 180);
        }
        else 
        {
            deviation_yaw = Goal.Yaw - IMU.Yaw;
		}
        ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * deviation_yaw;
        ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
    Attitude_RatePID();
	Thr_Ctrl(TT);// ���ſ���
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Attitude_RatePID
**���� : �����ʿ���PID
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
    fp32 E_pitch,E_roll,E_yaw;
	
	// ����ƫ��  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;
	
	// ����
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// �����޷�
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
    ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// ΢��
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y)*33;
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor.gyro.histor.x - sensor.gyro.averag.x)*33;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor.gyro.histor.z - sensor.gyro.averag.z)*33;	
	
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.x = sensor.gyro.averag.x; 
    sensor.gyro.histor.z = sensor.gyro.averag.z;	
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;
	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;
    
}


void Thr_Ctrl(float T)
{
///////////////////////////////////////////////////////////////////////////		
	static float thr;
	static float Thr_tmp;
	//thr = RC_Data.THROTTLE - RC_Cal.THROTTLE_MIN; //����ֵthr 0 ~ 1000
    thr = RC_Data.THROTTLE;
	Thr_tmp += 10 *3.14f *T *(thr/250.0f - Thr_tmp); //��ͨ�˲�
	Thr_Weight = LIMIT(Thr_tmp,0,1); //��߶ദ�������ݻ��õ����ֵ
//	printf("Thr_Weight=%f\r\n", Thr_Weight);
           
///////////////////////////////////////////////////////////////////////////////	

	if( thr < 100 )
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
	
//	#if(CTRL_HEIGHT)
        
		Height_Ctrl(T,thr);
        
		thr_value = Thr_Weight * height_ctrl_out;   //ʵ��ʹ��ֵ
//        printf("height_ctrl_out=%f\r\n", height_ctrl_out);//

//	#else
//	  thr_value = thr;   //ʵ��ʹ��ֵ

//	#endif

//	thr_value = LIMIT(thr_value,0,10 *80 *100/100);//�����������Ϊ800����200��ظ���̬����
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : Motor_Conter(void)
**���� : �������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
// Moto[0] 1263~2000  ,1308
// Moto[1] 1273~2000  ,
// Moto[2] 1107~2000  ,1300
// Moto[3] 1470~2000

void Motor_Conter(void)
{
	signed short pitch,roll,yaw;
	
    pitch = (int)ctrl.pitch.core.pid_out;
    roll  = (int)ctrl.roll.core.pid_out;    
    yaw   = (int)-ctrl.yaw.core.pid_out;
	
  	//if(RC_Data.THROTTLE > (RC_Cal.THROTTLE_MIN+100) && (flag.FlightMode==ULTRASONIC_High || flag.FlightMode==AUTO_High || flag.FlightMode==ACC_High  || flag.FlightMode==ATMOSPHERE_High))
    //     if(RC_Data.THROTTLE > (1000) && (flag.FlightMode==ULTRASONIC_High || flag.FlightMode==AUTO_High || flag.FlightMode==ACC_High  || flag.FlightMode==ATMOSPHERE_High))
    if((RC_Data.THROTTLE > 1100) && (flag.FlightMode==ULTRASONIC_High))
    {
        Moto[0] = (int)(thr_value - pitch - roll + yaw + 200);
        Moto[1] = (int)(thr_value - pitch + roll - yaw + 200);
        Moto[2] = (int)(thr_value + pitch + roll + yaw + 200);
        Moto[3] = (int)(thr_value + pitch - roll - yaw + 200);
//        printf("thr_value=%f\r\n", thr_value);
    //    else	if(RC_Data.THROTTLE > (RC_Cal.THROTTLE_MIN+100)) 
    
//    if(RC_Data.THROTTLE > (1100)) 
//    {
//        date_throttle	= (RC_Data.THROTTLE);///cos(IMU.Roll/RtA)/cos(IMU.Pitch/RtA);
//		
//        Moto[0] = date_throttle - pitch - roll + yaw + 200;
//        Moto[1] = date_throttle - pitch + roll - yaw + 200;
//        Moto[2] = date_throttle + pitch + roll + yaw + 200;
//        Moto[3] = date_throttle + pitch - roll - yaw + 200;
        
        
        //printf("date_throttle=%d\r\n", date_throttle);
        //printf("pitch=%d\r\n", pitch);
        //printf("roll=%d\r\n", roll);
//        printf("yaw=%d\r\n", yaw);
        for(char i = 0; i < 4; i++)
        {
            if(Moto[i] > 2000)
            {
                Moto[i] = 2000;
            }
        }
        
        for(char i = 0; i < 4; i++)
        {
            if(Moto[i] < 800)
            {
                Moto[i] = 800;
            }
        }
    }
    else
    {	
        array_assign(&Moto[0],IDLING,MOTOR_NUM);//������800
        Reset_Integral();//�ڻ�pidȫ�������0		
    }
    //    printf("0=%d\r\n", Moto[0]);
    //     IMU.Roll = Degree(AngE.Roll);  // roll
    //	IMU.Pitch = Degree(AngE.Pitch); // pitch
    
        if(IMU.Roll > -80 && IMU.Roll <80 && IMU.Pitch  > -80 && IMU.Pitch  <80)
        {
            if(RC_Data.PITCH < 1000 && RC_Data.THROTTLE < 970  && (!flag.ARMED))//�ɻ������� ����ң�˴���ͣ�
            {                                                                   //����ң��Ҳ�����
                flag.ARMED = 1;                                                 //4��LED��0.5���ʾ�����ɹ�
    //            LEDALL_ON;
    //            Delay_ms(600);
    //            LEDALL_OFF;
            }
            
            if( !flag.calibratingM)
            { 
                if(RC_Data.PITCH > 1800 && RC_Data.THROTTLE < 970  && (!flag.ARMED))//������У׼�� ����ң�˴���ͣ�
                {                                                                   //����ң��Ҳ�����
                    flag.calibratingM = 1;                                          //4��LED����˸���α�ʾ��ʼ������У׼
                    
    //                LEDALL_ON;
    //                Delay_ms(300);
    //                LEDALL_OFF;   
    //                Delay_ms(300);
    //                LEDALL_ON;
    //                Delay_ms(300);
    //                LEDALL_OFF;   
                }
            }
            if(flag.ARMED)
            {		      
                for(char i=0;i<4;i++)
                {
                    if(Moto[i]<0)
                    {
                        Moto_duty[i]=0;
                    }
                    else
                    {
                        Moto_duty[i]=Moto[i];
                    }
                }		
                moto_PwmRflash(&Moto_duty[0]);//������ˢ�£�ֱ��дPWM����Ĵ���	
            }	
        }
        else 
        {
            //array_assign(&Moto_duty[0],0,MOTOR_NUM);//������0
            //array_assign(&Moto_duty[0],0,MOTOR_NUM);
            moto_STOP();//ǿ�����800
            flag.ARMED = 0;
    //        LED4_ON;
        }	
    
    
//    for(i=0;i<4;i++)
//    {
//        if(Moto[i]<0)
//        {
//            Moto_duty[i]=0;
//        }
//        else
//        {
//            Moto_duty[i]=Moto[i];
//        }
//    }		
//    moto_PwmRflash(&Moto_duty[0]);//������ˢ�£�ֱ��дPWM����Ĵ���
    
}


/*====================================================================================================*/
/*====================================================================================================*
**���� : Reset_Integral
**���� : ��������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
    ctrl.pitch.core.increment = 0;		
    ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}