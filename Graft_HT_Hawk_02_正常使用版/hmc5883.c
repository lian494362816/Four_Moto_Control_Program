#include "include.h"


//当前磁场的最大值和最小值
int16_t  HMC58X3_limit[6]={0};
int16_t  *mag_limt = HMC58X3_limit;
int16_t magdata[6]={0};


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Init_HMC5883L
**功能 : 指南针初始化
**输入 : None
**輸出 : 状态
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
unsigned char Init_HMC5883L(void)
{
	unsigned char ack; 
	
	ack = Single_Read(MAG_ADDRESS, 0x0A);
	
	if (!ack)
    {
        return FALSE;
    }
	// leave test mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFA, 0x78);   // 0x70Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
	Single_Write(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   //0x20 Configuration Register B  -- 001 00000    configuration gain 1.33Ga
	Single_Write(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
	//delay(100);
	return TRUE;	 
}


/************************************************************   
* 函数名:Identify_HMC5883L  
* 描述：设备识别
* 输入：无
* 输出：无
*/
void Identify_HMC5883L(void)
{
	unsigned char ID_A,ID_B,ID_C;
	ID_A=Single_Read(MAG_ADDRESS,HMC5883L_ID_A);
	ID_B=Single_Read(MAG_ADDRESS,HMC5883L_ID_B);
	ID_C=Single_Read(MAG_ADDRESS,HMC5883L_ID_C);
	if(ID_A=='H'&&ID_B=='4'&&ID_C=='3')
	{
		printf("\r\n HMC5773L 识别成功\r\n");
	}else
	{
		printf("\r\n 无法识别 HMC5773L\r\n");
	}
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : hmc5883lRead
**功能 : 度取地磁数据
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void HMC5883lRead(int16_t *magData)
{
	unsigned char buf[6],cy,con=0;
	int16 mag[3];
	static unsigned char onc=1;
	static int32 An[3] = {0,0,0};
	
	// 读取寄存器数据
	I2C_Read(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    
	// 十位深度滤波
	An[0] -= An[0]/10;
	An[0] += (int16_t)(buf[0] << 8 | buf[1]);//x
	mag[0] = An[0]/10;
    
	An[1] -= An[1]/10;
	An[1] += (int16_t)(buf[4] << 8 | buf[5]);//y
	mag[1] = An[1]/10;
    
	An[2] -= An[2]/10;
	An[2] += (int16_t)(buf[2] << 8 | buf[3]);//z
	mag[2] = An[2]/10;
    magdata[0]=mag[2];
    magdata[1]=buf[2];
	magdata[2]=buf[3];
    

    
 //   printf("\r\n magdata[0]=%d,magdata[1]=%d,magdata[2]=%d \r\n", magdata[0], magdata[1], magdata[2]);
    
	//需要校准
    //printf("\r\n flag.calibratingM = %d\r\n",flag.calibratingM);
	if(flag.calibratingM) 
    {
        //printf("进行磁力计校准\r\n");
		onc=1;
		flag.MagIssue = 0;
		Mag_Calibration(mag);
	}
    
	if(onc)
    {
		onc=0;
		// 三个轴的最值都偏小 说明地磁有问题，停用地磁  
		for(cy=0;cy<6;cy++)	
        {
			if(absu16(*(mag_limt+cy))<20)	
            {
                con++;
            }  
            //printf("\r\n%d = %d\r\n", cy, *(mag_limt+cy));
		}
		if(con>=5)
        {
            flag.MagIssue = 1;
//            printf("磁力计存在问题\r\n");
//            for(cy=0;cy<6;cy++)	
//            {     
//                printf("\r\n%d = %d\r\n", cy, *(mag_limt+cy));
//            }
        }
    }
	
	// 修正
	for(cy=0;cy<3;cy++)
    {
		*(magData+cy) = (int)(fp32)(mag[cy] -(*(mag_limt+cy+3) + *(mag_limt+cy))/2);
    }
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Mag_Calibration
**功能 : 地磁校准
**输入 : None
**輸出 : None
**备注 : 这里是需要手动来旋转飞控板，达到磁力计校准
        保存数据没写
**====================================================================================================*/
/*====================================================================================================*/
void Mag_Calibration(int16 *array)
{
	unsigned char cy;
	static unsigned char  clen_flag=1; 
	static fp32 x,y,z; 
	//printf("\r\n x=%d, y=%d, z=%d\r\n", *(array), *(array + 1), *(array +2));
	//校准之前先把之前数据清零
	if(clen_flag)
    {
        //printf("磁力计校准数据清零\r\n");
		clen_flag = 0;
		x=y=z=0;
		for(cy=0;cy<6;cy++)
        {
			*(mag_limt+cy)=0;
        }
	}
    
	// 开始采集 寻找三个轴的最大和最小值
	for(cy=0;cy<3;cy++)
    {
		if(*(mag_limt+cy)> *(array+cy)) 
        {
            *(mag_limt+cy) = *(array+cy);  //找最小
           // printf("磁力计数据最小值 %d\r\n", *(mag_limt+cy));
        }
        
		else if(*(mag_limt+cy+3)<*(array+cy)) 
        {    
            *(mag_limt+cy+3) = *(array+cy);  //找最大
            //printf("磁力计数据最大值 %d\r\n", *(mag_limt+cy));
        }
        //printf("\r\n array1=%d, array2=%d, array3=%d\r\n", *(array), *(array + 1), *(array +2));
	}
	//下面就是判断进行地磁校准的动作利用加速度计判断是否垂直，利用陀螺仪判断是否转满了360度
    
//    printf("z = %f \r\n", absu16(sensor.acc.averag.z)); 
//    printf("sensor.acc.averag.x = %f \r\n", absu16(sensor.acc.averag.x)); 
//    printf("y = %f \r\n", absu16(sensor.acc.averag.y)); 
    
	if(flag.calibratingM == 1 && (absu16(sensor.acc.averag.z) > 5000))   
    {
        z += sensor.gyro.radian.z * Gyro_G * 0.002f;
//        LED1_ON;
//        printf("z = %f \r\n", z);
//		if(absFloat(z)>360) 
        if(absFloat(z)>150) 
        {
            flag.calibratingM = 2;
            printf("\r\n 地磁Z校准OK\r\n");
//            LED1_OFF;
        }
	}
	
	if(flag.calibratingM == 2 && (absu16(sensor.acc.averag.x) > 5000))  
    {
        x += sensor.gyro.radian.x * Gyro_G * 0.002f;
        //printf("x = %f \r\n", x);
//        LED2_ON;
//		if(absFloat(x)>360) 
        if(absFloat(x)>150) 
        {
            flag.calibratingM = 3;
            printf("\r\n 地磁X校准OK\r\n");
//            LED2_OFF;
        }
    }
	
	if(flag.calibratingM == 3 && (absu16(sensor.acc.averag.y) > 5000))  
    {
        y += sensor.gyro.radian.y * Gyro_G * 0.002f;
//        LED3_ON;
        //printf("y = %f \r\n", y);
//		if(absFloat(y)>360) 
        if(absFloat(y)>150)
        {
			clen_flag = 1;
			flag.calibratingM = 0;
            printf("\r\n 地磁Y校准OK\r\n");
//            LED3_OFF;
			//EE_SAVE_MAG_OFFSET();
		}
	}	
}
