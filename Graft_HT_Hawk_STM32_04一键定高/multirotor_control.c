#include "include.h"
#include "math.h"
#include "delay.h"
struct _ctrl ctrl;
struct _target Target;
float Thr_Weight;
unsigned char Thr_Low;
long float thr_value;//定高模式下的油门值
int16 Moto[4];
int16 Moto_duty[4];
int date_throttle1, date_throttle2;
unsigned char Control_Thr_Up_F = 0;
unsigned char Control_Thr_Down_F = 0;
unsigned char Control_Thr_Up_Count = 0;
unsigned char Control_Thr_Down_Count = 0;
/*   超声波的数据 */
extern float US100_Alt;
extern float US100_Alt_V;
float alt_us100_1;

int Land;//强制降落标志位
 float tr_tmep1,thr_last,thr_tmep1,thr_tmep,THR_Lock=0;
 struct _PID PID_US100;
int date_THROTTLE_US_100=0;//加了超声波的油门

extern struct ADNS_PID PID_ADNS3080;

extern int SumX, SumY, dx, dy;//ADNS3080 Data
extern unsigned char Lock_Down ;
extern unsigned char Control_Mode ;
extern unsigned char Fly_Mode ;
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Calculate_target
**功能 : 计算目标量
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/

//    YAW       930~2078 中值1510
//    THROTTLE  936~2083  
//    PITCH     957~2104 中值1584
//    ROLL      963~2111 中值1546

void Calculate_Target(void) 
{
	int16 ftemp=0;
    if(RC_Data.ROLL <1446 || RC_Data.ROLL > 1646)
    {
	    Target.Roll = (RC_Data.ROLL-RC_Data_Roll_Offset)/(RC_Data_Division);
//       printf("Target.Roll=%f\r\n", Target.Roll);
    }
    else
    {
        Target.Roll = 0;
//        printf("Target.Roll=%f\r\n", Target.Roll);
    }
    
//    Target.Roll = (RC_Data.ROLL-1546)/(50 );
//    printf("Target.Roll=%f\r\n", Target.Roll);
    
    if(RC_Data.PITCH < 1484 || RC_Data.PITCH > 1684)
    {
        Target.Pitch = (RC_Data.PITCH - RC_Data_Pitch_Offset)/(RC_Data_Division);
//        printf("Target.Pitch=%f\r\n", Target.Pitch);
    }
    else
    {
        Target.Pitch = 0;
//        printf("Target.Pitch=%f\r\n", Target.Pitch);
    }

//    Target.Roll = (RC_Data.ROLL-RC_Data_Roll_Offset)/(RC_Data_Division );
//    printf("Targer.Roll=%f\r\n", Target.Roll);
    
//    Target.Pitch = (RC_Data.PITCH - RC_Data_Pitch_Offset)/(RC_Data_Division);
//    printf("Target.Pitch=%f\r\n", Target.Pitch);
   
    //printf("RC_Data.YAW=%d\r\n", RC_Data.YAW);
    //目标航向控制。当油门大于最小检查值时，认为用户希望起飞。那么此时的航向做为目标航向
//    if(RC_Data.THROTTLE > 1050  ) 
//    {
//        if(flag.LockYaw != 1)
//        {  
//            flag.LockYaw = 1;
//            Target.Yaw = IMU.Yaw; //将当前的航向做为目标航向
//        }
//    }
//    else 
//    {
//        flag.LockYaw = 0;	
//        Target.Yaw = IMU.Yaw;
//        
//    } 
    
	//航向在中点设置一个死区，好处是手操作时忽略小动作
	if((RC_Data.YAW > 1550)||(RC_Data.YAW < 1450))
    {
		ftemp = 1510 - RC_Data.YAW; 
        Target.Yaw += (ftemp / 50.0f)*0.1f; 
		//printf("Target.Yaw=%f\r\n", Target.Yaw);
		//转[-180.0,+180.0]
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
**函数 : CONTROL(struct _target Goal) 
**功能 : 串级PID控制
**输入 : Goal
**輸出 : None
**备注 : YAW 只用了外环P
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	
	if(ctrl.ctrlRate >= 2)
	{//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//*****************外环(角度环)PID**************************//
		//横滚计算///////////////
        deviation_pitch = Goal.Pitch - IMU.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;
		
		//limit for the max increment
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);
        
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//俯仰计算//////////////
		deviation_roll = Goal.Roll - IMU.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);
        
		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//航向计算////////////
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
	Thr_Ctrl(TT);// 油门控制
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Attitude_RatePID
**功能 : 角速率控制PID
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
    fp32 E_pitch,E_roll,E_yaw;
	
	// 计算偏差  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;
	
	// 积分
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// 积分限幅
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
    ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// 微分
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
	//thr = RC_Data.THROTTLE - RC_Cal.THROTTLE_MIN; //油门值thr 0 ~ 1000
    thr = RC_Data.THROTTLE;
	Thr_tmp += 10 *3.14f *T *(thr/  250.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1); //后边多处分离数据会用到这个值
	
///////////////////////////////////////////////////////////////////////////////	

	if( thr < 100 )
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
	
	  thr_value = thr;   //实际使用值

}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Motor_Conter(void)
**功能 : 电机控制
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
// Moto[0] 1263~2000  ,1308
// Moto[1] 1273~2000  ,
// Moto[2] 1107~2000  ,1300
// Moto[3] 1470~2000

void Motor_Conter(void)
{
	signed short pitch,roll,yaw;
	unsigned char i=0;
	
    pitch = (int)ctrl.pitch.core.pid_out;
    roll  = (int)ctrl.roll.core.pid_out;    
    yaw   = (int)-ctrl.yaw.core.pid_out;
	

    if(0 == Lock_Down)//解锁
    {
        if(Normal_Mode == Control_Mode)
        {
            if(RC_Data.THROTTLE > 1050  ) 
            {
                if(flag.LockYaw != 1)
                {  
                    flag.LockYaw = 1;
                    Target.Yaw = IMU.Yaw; //将当前的航向做为目标航向
                }
            }
            else 
            {
                flag.LockYaw = 0;	
                Target.Yaw = IMU.Yaw;       
            } 
            
            if(RC_Data.THROTTLE > (1100) && IMU.Roll > -60 && IMU.Roll <60 && IMU.Pitch  > -60 && IMU.Pitch  <60) 
            {
                date_throttle1	= (RC_Data.THROTTLE);///cos(IMU.Roll/RtA)/cos(IMU.Pitch/RtA);
                
                Moto[0] = date_throttle1 - pitch - roll + yaw + 200;
                Moto[1] = date_throttle1 - pitch + roll - yaw + 200;
                Moto[2] = date_throttle1 + pitch + roll + yaw + 200;
                Moto[3] = date_throttle1 + pitch - roll - yaw + 200;
                
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
                
                for(i=0;i<4;i++)
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
                
                moto_PwmRflash(&Moto_duty[0]);//马达输出刷新，直接写PWM输出寄存器	
            }
            else
            {	
                moto_STOP();//强制输出800
                Reset_Integral();//内环pid全部输出置0	
            }
        }
        
        if(Control_Mode == Height_Mode)
        {
            //            printf("date_throttle2=%d\r\n", date_throttle2);
            if(flag.LockYaw != 1)
            {  
                flag.LockYaw = 1;
                Target.Yaw = IMU.Yaw; //将当前的航向做为目标航向
            }
            
            if(Fly_Mode == One_Key_Up)
            {
                if(0 == Control_Thr_Up_F)//油门递增
                {
                    Height_Control_Thr_Up();
                }
            }
            
            if(Fly_Mode == One_Key_Down)
            {   
                Height_Control_Thr_Down();
            }
            
            if(IMU.Roll > -60 && IMU.Roll <60 && IMU.Pitch  > -60 && IMU.Pitch  <60) 
            {
                
                US100_CONTROL(HEIGHT);//定高1.1m  
                
                date_THROTTLE_US_100 = (int)(date_throttle2 + THR_Lock);
                
                //        printf("THR_Lock=%f\r\n", THR_Lock);
                //        PID_ADNS3080.Xout
                //        PID_ADNS3080.Yout
                /* 
                往前  Y-
                往后  Y+
                往左  X+
                往右  X-
                */    
                
                Moto[0] = date_THROTTLE_US_100 - pitch - roll + yaw + 200 + (int)PID_ADNS3080.Yout + (int)PID_ADNS3080.Xout;
                Moto[1] = date_THROTTLE_US_100 - pitch + roll - yaw + 200 + (int)PID_ADNS3080.Yout - (int)PID_ADNS3080.Xout;
                Moto[2] = date_THROTTLE_US_100 + pitch + roll + yaw + 200 - (int)PID_ADNS3080.Yout - (int)PID_ADNS3080.Xout; 
                Moto[3] = date_THROTTLE_US_100 + pitch - roll - yaw + 200 - (int)PID_ADNS3080.Yout + (int)PID_ADNS3080.Xout;
                //        printf("pitch=%d\r\n", pitch);// -71 ~ 71 
                //        printf("PID_ADNS3080.Yout=%f\r\n", PID_ADNS3080.Yout);
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
                
                for(i=0;i<4;i++)
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
                moto_PwmRflash(&Moto_duty[0]);//马达输出刷新，直接写PWM输出寄存器	
            }
            else
            {
                moto_STOP();
            }
        }
    }
    else
    {
         moto_STOP();
         Reset_Integral();//内环pid全部输出置0	
         Control_Thr_Up_F = 0;
         Control_Thr_Down_F = 0;
         date_throttle2 = 950;
         flag.LockYaw = 0;	
         Target.Yaw = IMU.Yaw;       
    }

}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Reset_Integral
**功能 : 积分清零
**输入 : None
**輸出 : None
**备注 : None
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


  /*
    超声波定高 PD算法
 */

void US100_CONTROL(float US100_Alt_Target_1)
{
	
    alt_us100_1 =(US100_Alt - US100_Alt_Target_1);   //误差 = 实际 - 高度
//    printf("alt_us100_1=%f\r\n", alt_us100_1);
    PID_US100.pout=-((PID_US100.P)*alt_us100_1 * 10);
//    printf("PID_US100.pout=%f\r\n", PID_US100.pout);
    PID_US100.dout=-((PID_US100.D)*US100_Alt_V * 10);
//    printf("PID_US100.dout=%f\r\n", PID_US100.dout);
    tr_tmep1=thr_last+PID_US100.I*(PID_US100.pout+PID_US100.dout);
    thr_last=thr_tmep1;
    
    PID_US100.out=tr_tmep1+PID_US100.pout+PID_US100.dout;
    thr_tmep=PID_US100.out;
    if(thr_tmep>150)   //油门输出限制幅度
    {
        thr_tmep=150;
    }
    if(thr_tmep<-200) 
    {
        thr_tmep=-200; 
    }
    THR_Lock=thr_tmep; //锁定油门大小
//    printf("THR_Lock=%f\r\n", THR_Lock);

}

void Height_Control_Thr_Up(void)
{
    if(US100_Alt < HEIGHT)
    {
        Control_Thr_Up_Count ++;
        if(Control_Thr_Up_Count >= THR_NUM)
        {
            date_throttle2 += INCREASE_NUM;
            Control_Thr_Up_Count = 0;
        }
//        printf("date_throttle2=%d\r\n", date_throttle2);
    }
    else
    { 
        Control_Thr_Down_F = 0;//置0， 下次可以启动一键降落
        Control_Thr_Up_F = 1;
    }
}

void Height_Control_Thr_Down(void)
{
    if(US100_Alt > 0.08)
    {
        Control_Thr_Down_Count ++;
        if(Control_Thr_Down_Count >= THR_NUM)
        {
            date_throttle2 -= INCREASE_NUM;
            Control_Thr_Down_Count = 0;
        }

        if(date_throttle2 <= 950)
        {
            date_throttle2 = 950;
        }
//        printf("date_throttle2=%d\r\n", date_throttle2);
    }
    else
    {
        moto_STOP();
        Control_Thr_Up_F = 0;//置0，下次可以启动一键起飞
        date_throttle2 = 950;
        SumX = 0; 
        SumY = 0;
        dx = 0;
        dy = 0;
        Reset_Integral();//内环pid全部输出置0	
        Control_Thr_Down_F = 1;
    }
       
}
