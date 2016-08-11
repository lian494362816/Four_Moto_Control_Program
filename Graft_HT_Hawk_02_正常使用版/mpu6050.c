#include "include.h"
#include "delay.h"
unsigned char mpu6050_buffer[14];
struct _sensor sensor;
char ACC_OFFSET_OK = 0;
unsigned char MPU6050_Init(void)
{
    unsigned char ack;
	ack = Single_Read(MPU6050_ADDRESS, WHO_AM_I);
	if (!ack)
    {
        return FALSE;
	}
	Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	//解除休眠状态
	Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	Single_Write(MPU6050_ADDRESS, CONFIGL, MPU6050_DLPF);              //低通滤波
	Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, MPU6050_GYRO_FS_1000);  //陀螺仪量程 +-1000
	Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, MPU6050_ACCEL_FS_4);   //加速度量程 +-4G
    Single_Write(MPU6050_ADDRESS, INT_PIN_CFG, 0x42);//使能旁路I2C
    Single_Write(MPU6050_ADDRESS, USER_CTRL, 0x40);//使能旁路I2C  
	return TRUE;
}

void MPU6050_WHO_AM_I(void)
{
    unsigned char dev = 0;

    if(dev=Single_Read(MPU6050_ADDRESS, WHO_AM_I), dev==0x68)
    { 
    	printf("\r设备MP6050识别成功，id=0x%x\r\n\r",dev);
    }
	else
    {
        printf("\r错误!无法设别设备MP6050   id=0x%d\r\n",dev);
    }
}


//**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
//******************************************************************************
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, 0x3B);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, 0x3C);
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, 0x3D);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, 0x3E);
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, 0x3F);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, 0x40);
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, 0x43);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, 0x44);
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, 0x45);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, 0x46);
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, 0x47);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, 0x48);
}

/**************************实现函数********************************************
//将iic读取到加速度计和陀螺仪得数据分拆,放入相应寄存器
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	MPU6050_Read();
	
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);
//    printf("\r\n ax= %d, ay= %d , az= %d", sensor.acc.origin.x, sensor.acc.origin.y, sensor.acc.origin.z);
	
    sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
//    printf("\r\n gx= %d, gy= %d , gz= %d", sensor.gyro.origin.x, sensor.gyro.origin.x, sensor.gyro.origin.x);
	//printf("\r\n gx=%d \r\n",sensor.gyro.origin.x);
    sensor.gyro.radian.x = sensor.gyro.origin.x - sensor.gyro.quiet.x;
	sensor.gyro.radian.y = sensor.gyro.origin.y - sensor.gyro.quiet.y;
	sensor.gyro.radian.z = sensor.gyro.origin.z - sensor.gyro.quiet.z;	 
    //printf("\r\n grx= %f, gry= %f , grz= %f", sensor.gyro.radian.x, sensor.gyro.radian.y, sensor.gyro.radian.z);
   
     if(!flag.calibratingA)
	 {
		 static int32	tempax=0,tempay=0,tempaz=0;
		 static unsigned char cnt_a=0;
//         printf("ACC校准\r\n");
		 if(cnt_a==0)
		 {
             sensor.acc.quiet.x = 0;
             sensor.acc.quiet.y = 0;
             sensor.acc.quiet.z = 0;
             tempax = 0;
             tempay = 0;
             tempaz = 0;
             cnt_a = 1;
		 }
         tempax+= sensor.acc.origin.x;
         tempay+= sensor.acc.origin.y;
         tempaz+= sensor.acc.origin.z;
         if(cnt_a==200)
         {
             sensor.acc.quiet.x = tempax/cnt_a;
             sensor.acc.quiet.y = tempay/cnt_a;
             sensor.acc.quiet.z = tempaz/cnt_a;
             cnt_a = 0;
             flag.calibratingA = 1;
             
             return;
         }
         cnt_a++;		
     }	
    
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_Calculateoffest
**功能 : 计算陀螺仪零偏
**输入 : 
**输出 : None
**使用 : Hto_Gyro_Calculateoffest();
**====================================================================================================*/
/*====================================================================================================*/
void Gyro_Caloffest(float x,float y,float z,unsigned int amount)
{
     sensor.gyro.quiet.x = (int16)x/amount;
	 sensor.gyro.quiet.y = (int16)y/amount;
	 sensor.gyro.quiet.z = (int16)z/amount;
//     printf("x=%d, y=%d, z=%d\r\n", sensor.gyro.quiet.x, sensor.gyro.quiet.y, sensor.gyro.quiet.z);
}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_OFFSET
**功能 : 陀螺仪静态采集
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Gyro_OFFSET(void)
{
	 int cnt_g=200;
    int32  tempgx=0,tempgy=0,tempgz=0;
    sensor.gyro.averag.x=0;    //á?μ???ò???á?
    sensor.gyro.averag.y=0;  
    sensor.gyro.averag.z=0;
    while(cnt_g--)       //?-?·2é?ˉ2000′?   ?ó???ù
    {
        MPU6050_Dataanl(); 
        sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
        sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
        sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
        tempgx+= sensor.gyro.origin.x;
        tempgy+= sensor.gyro.origin.y;
        tempgz+= sensor.gyro.origin.z;
    }
    sensor.gyro.quiet.x=tempgx/200;
    sensor.gyro.quiet.y=tempgy/200;
    sensor.gyro.quiet.z=tempgz/200;
//    printf("gyro.quiet.x=%d, gyro.quiet.y=%d, gyro.quiet.z=%d\r\n", sensor.gyro.quiet.x, sensor.gyro.quiet.y, sensor.gyro.quiet.z);
	if(flag.calibratingA)
    {
        Accl_OFFSET();
    }
}


/*====================================================================================================*
**函数 : Accl_OFFSET
**功能 : 加速度计补偿
**输入 : None
**輸出 : None
**备注 : 解决偏移值保存的问题
**====================================================================================================*/
/*====================================================================================================*/
void Accl_OFFSET(void)
{
	int32	tempax=0,tempay=0,tempaz=0;
	uint8_t cnt_a=0;
	sensor.acc.quiet.x = 0;
	sensor.acc.quiet.y = 0;
	sensor.acc.quiet.z = 0;
	
	for(cnt_a=0;cnt_a<200;cnt_a++)
	{
		tempax+= sensor.acc.origin.x;
		tempay+= sensor.acc.origin.y;
		tempaz+= sensor.acc.origin.z;
	}
	
	sensor.acc.quiet.x = tempax/cnt_a;
	sensor.acc.quiet.y = tempay/cnt_a;
	sensor.acc.quiet.z = tempaz/cnt_a;

	flag.calibratingA = 0;
	//EE_SAVE_ACC_OFFSET();//保存数据				
}
