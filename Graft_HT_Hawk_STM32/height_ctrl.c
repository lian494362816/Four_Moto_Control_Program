#include "include.h"

_st_height_pid_v wz_speed_pid_v;
_st_height_pid wz_speed_pid;

float baro_speed;

float height_ctrl_out;
float wz_acc;
unsigned char height_ctrl_mode ;
extern unsigned char ultra_start_f;


float ultra_speed;
float ultra_ctrl_out;
float wz_speed;
float wz_acc_mms2;
extern float Thr_Weight;

float exp_height_speed,exp_height;
float ultra_speed;
float ultra_dis_lpf;
float ultra_ctrl_out;

extern uint16 ultra_distance;
extern int16 ultra_delta;
extern Gravity v;//重力分量

_st_height_pid_v ultra_ctrl;
_st_height_pid ultra_pid;

void Height_Ctrl(float T,float thr)
{	
	static float wz_speed_t;
	static unsigned char height_ctrl_start_f;
	static uint16 hc_start_delay;
    static unsigned char hs_ctrl_cnt;
//	printf("thr=%f\r\n", thr);
	switch( height_ctrl_start_f )
	{
		case 0:
        
            if( sensor.acc.averag.z > 4000 )
            {
                height_ctrl_start_f = 1;
//                printf("sensor.acc.averag.z=%f\r\n", sensor.acc.averag.z);
            }	
            else if( ++hc_start_delay > 500 )
            {
                height_ctrl_start_f = 1;
            }
            
            break;
		
		case 1:
		
		wz_acc += ( 1 / ( 1 + 1 / ( 20 *3.14f *T ) ) ) *( (v.z *sensor.acc.averag.z + v.y *sensor.acc.averag.y 
		                                                   + v.x *sensor.acc.averag.x - 4096 ) - wz_acc );
//	    wz_acc = my_sqrt(ABS(sensor.acc.averag.x*sensor.acc.averag.x)
//                       + ABS(sensor.acc.averag.y*sensor.acc.averag.y) 
//                       + ABS(sensor.acc.averag.z*sensor.acc.averag.z)) - sensor.acc.quiet.z;
//        printf("wz_acc = %f\r\n", wz_acc);//取值在4000左右
        /* 这里的 500 需要修改*/
//		wz_speed_t += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *(0.4f*(thr-500) - wz_speed_t);
        
        wz_speed_t += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *(0.4f*(thr-1400) - wz_speed_t);
        
//		printf("wz_speed_t=%f\r\n", wz_speed_t);//这个值没问题
//		Moving_Average( (float)( baro_alt_speed *10),baro_speed_arr,BARO_SPEED_NUM, baro_cnt ,&baro_speed ); //单位mm/s
        
//        Moving_Average( wz_acc, acc_speed_arr, ACC_SPEED_NUM,acc_cnt ,&wz_acc1 );
        
        // 		if( baro_alt_speed > 2000 )
        // 		{
        // 			while(1);
        // 		}
		
        //		if( height_ctrl_mode == 1)
        //		{
        //			//height_speed_ctrl(T,thr,0.8f*(thr-500),wz_speed_t);
        //			
        //			
        //			if(baro_ctrl_start==1)//20ms
        //			{
        //				height_speed_ctrl(0.02f,thr,( EXP_Z_SPEED ),baro_speed);//baro_alt_speed *10);///
        //				baro_ctrl_start = 0;
        //				Baro_Ctrl(0.02f,thr);
        //			}		
        //		}
		
		if( height_ctrl_mode == 2)//超声波定高
		{
			hs_ctrl_cnt++;
			hs_ctrl_cnt = hs_ctrl_cnt % 10;
			if(hs_ctrl_cnt == 0)
			{
//				height_speed_ctrl(0.02f,thr,0.4f*ultra_ctrl_out,ultra_speed);
                height_speed_ctrl(T,thr,0.4f*ultra_ctrl_out,ultra_speed);
//                printf("ultra_ctrl_out=%f\r\n", (0.4f * ultra_ctrl_out));
//                printf("height_speed_ctrl Done!\r\n");
			}
			
			if( ultra_start_f == 0 )
			{	
				
				Ultra_Ctrl(0.045f,thr);//超声波周期100ms，修改ultra_ctrl_out
//                printf("Ultra_Ctrl Done!\r\n");
				ultra_start_f = -1;
			}
		}
        
		
        
		if(height_ctrl_mode)
		{		
			height_ctrl_out = wz_speed_pid_v.pid_out;//最后的输出
//            printf("heigh_ctrl_out=%f\r\n", height_ctrl_out); 
		}
		else
		{
			height_ctrl_out = thr;
		}
		
		break; //case 1
		
		default: break;
		
	} //switch
}


void height_speed_ctrl(float T,float thr,float exp_z_speed,float h_speed)
{
	static float thr_lpf;
	float height_thr;
	static float lpf_tmp,hc_speed_i,hc_speed_i_2,wz_speed_0,wz_speed_1,wz_speed_2;
	
    /* 这里 600 需要修改*/
//	height_thr = LIMIT( 2 * thr , 0, 600 );
    height_thr = LIMIT( 2 * thr , 0, 1500 );
//    printf("thr%f\r\n", thr);这个值没问题
	
	thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );
//	printf("thr_lpf=%f\r\n", thr_lpf);//这个值没问题
//	wz_acc_mms2 = (wz_acc/4096.0f) *10000 ;//9800 *T;
    wz_acc_mms2 = (wz_acc/8192.0f) *9800 ;//9800 *T;
//	printf("wz_acc_mms2 = %f\r\n", wz_acc_mms2);//这个值没问题
	wz_speed_0 += my_deathzoom( wz_acc_mms2 ,100) *T;
//	printf("wz_speed_0=%f\r\n", wz_speed_0);//3159
	hc_speed_i += 0.4f *T *( h_speed - wz_speed_1 );
	hc_speed_i = LIMIT( hc_speed_i, -1500, 1500 );	
	
	wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.4f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;
//    printf("wz_speed_0=%f\r\n", wz_speed_0);//3000+
//    printf("h_speed = wz_speed_0 = %f \r\n", h_speed - wz_speed_0);
	wz_speed_1 = wz_speed_0 + hc_speed_i;
//	 printf("wz_speed_1=%f\r\n", wz_speed_1);//No problem
	if( ABS( wz_speed_1 ) < 50 )
	{
		wz_speed_1 = 0;
	}
	
	wz_speed_2 += my_deathzoom( wz_acc_mms2 ,100) *T;
	
    
    lpf_tmp += 0.4f *T *( wz_speed_1 - wz_speed ); 
    lpf_tmp = LIMIT( lpf_tmp, -1500, 1500 ); 
    
	hc_speed_i_2 += 0.01f *T *( wz_speed_1 - wz_speed_2 ); 
	hc_speed_i_2 = LIMIT( hc_speed_i_2, -500, 500 );	
	
	wz_speed_2 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( wz_speed_1 - wz_speed_2 + hc_speed_i_2 ) ;//*(baro_speed - wz_speed);
	wz_speed = wz_speed_2 + lpf_tmp;
	printf("wz_speed=%f\r\n", wz_speed);//12142
    // 	if( wz_speed_0 > 2000 )
    // 	{
    // 		while(1);
    // 	}
	
    /*     
        wz_speed_pid.kp = 0.3; 
        wz_speed_pid.kd= 1.4; 
        wz_speed_pid.ki = 0.12; 
    */
    
	wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );
//    printf("wz_speed_pid_v.err=%f\r\n", wz_speed_pid_v.err);//  -3628
	wz_speed_pid_v.err_d = 0.002f/T *10*wz_speed_pid.kd * (-wz_acc_mms2) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
//	printf("wz_speed_pid_v.err_d=%f\r\n", wz_speed_pid_v.err_d);// -112
	
	wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid.kp *( exp_z_speed - h_speed ) *T;
	wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *300,Thr_Weight *300);
//	printf("wz_speed_pid_v.err_i=%f\r\n", wz_speed_pid_v.err_i);//I 从负值慢慢累加上去
    //wz_speed_pid_v.pid_out 这是最后给油门的值
	wz_speed_pid_v.pid_out = thr_lpf + Thr_Weight *LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);
//    printf("wz_speed_pid_v.pid_out=%f\r\n", wz_speed_pid_v.pid_out);

	wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
}

void Ultra_Ctrl(float T,float thr)
{
	float ultra_sp_tmp,ultra_dis_tmp;	
	
    /*  500需要修改 ， 我的油门到起飞时 ，值为1457左右*/
//	exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,50)/200.0f;//在 -550 到 550内期望为0
    exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 1350,50)/200.0f;//
//    printf("thr=%f\r\n", thr);
//    printf("exp_height_speed=%f\r\n", exp_height_speed);
	exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);// 限幅3cm
	
	if( exp_height > ULTRA_MAX_HEIGHT )//1.5m
	{
		if( exp_height_speed > 0 )
		{
			exp_height_speed = 0;
		}
	}
	else if( exp_height < 20 )//2cm
	{
		if( exp_height_speed < 0 )
		{
			exp_height_speed = 0;
		}
	}
	
	exp_height += exp_height_speed *T;//对期望高度慢慢积分
//    printf("exp_height=%f\r\n", exp_height);
//    printf("thr=%f\r\n", thr);

    /* 100 需要修改
       油门45%开始增加
           35%开始减少
    
    */
//	if( thr < 100 )
//	{
//		exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);//慢慢减少
//	}
	
    if( thr < 1250 )
	{
		exp_height += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *( -exp_height);//慢慢减少
	}
//    printf("exp_height=%f\r\n", exp_height);
	ultra_sp_tmp = Moving_Median(0,5,ultra_delta/T); //ultra_delta/T;
	ultra_dis_tmp = Moving_Median(1,5,ultra_distance);
//	printf("ultra_sp_tmp=%f\r\n", ultra_sp_tmp);
	if( ultra_dis_tmp < 2000 )
	{
		if( ABS(ultra_sp_tmp) < 100 )
		{
			ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
		}
		else
		{
			ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
		}
	}
//	printf("ultra_speed=%f\r\n", ultra_speed);//这个值没问题
	if( ultra_dis_tmp < 2000 )
	{
		
		if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
		{
			
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
		{
			
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
		else
		{
			ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.6f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
		}
	}
	else
	{
		
	}

	ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );//这里用到了P
//	printf("ultra_ctrl.err=%f\r\n", ultra_ctrl.err); //这个值没错
    
	ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;//这里用到了I, I为0
	ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);//限幅300
//	printf("ultar_ctrl.err_i=%f\r\n", ultra_ctrl.err_i);
    
	ultra_ctrl.err_d = ultra_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(ultra_ctrl.err - ultra_ctrl.err_old) );//这里用到了D
  
//	printf("ultra_ctrl.err_d=%f\r\n", ultra_ctrl.err_d);//This value is very large
    
	ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;
	
    /* 这里 500可能需要修改*/
//	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);
    	ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-1000, 1000);
//	printf("ultra_ctrl.pid_out=%f\r\n", ultra_ctrl.pid_out);	
	ultra_ctrl_out = ultra_ctrl.pid_out;
	
	ultra_ctrl.err_old = ultra_ctrl.err;
}



void Ultra_PID_Init()//输出给的值给height_speed_ctrl
{
	ultra_pid.kp = 2.0;//油门上 加 油门下 减   高度加  减  高度减 加  以高度的大小来输出     
	ultra_pid.kd = 0.05;//高度加负值增加， 高度减负值减小
	ultra_pid.ki = 0;//油门上 到300 油门下 到 -300
}


void WZ_Speed_PID_Init()
{
	wz_speed_pid.kp = 0.01; 
	wz_speed_pid.kd = 1.4; //高度增加 负值减小， 高度减小 负值增大 
	wz_speed_pid.ki = 0.12; 
}