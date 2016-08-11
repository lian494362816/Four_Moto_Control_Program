#include "include.h"
#include "delay.h"
/*变量声明----------------------------------------------------------------*/
unsigned char Temp[14];
uint16_t Cal_C[7];  //用于存放PROM中的6组数据	
//uint32_t D1_Pres,D2_Temp; // 存放数字压力和温度
float Pressure;				//温度补偿大气压
float dT,Temperature,Temperature2;//实际和参考温度之间的差异,实际温度,中间值
double OFF,SENS;  //实际温度抵消,实际温度灵敏度
float Aux,OFF2,SENS2;  //温度校验值

//uint32_t ex_Pressure;			//串口读数转换值
uint8_t  exchange_num[8];

//void MS5611_Reset(void)
//{
//    I2C_NoAddr_WriteByte(MS561101BA_ADDR, MS561101BA_RESET);
//}


int32 baroAlt,baroAltOld;
float baro_alt_speed;
int32 baro_Offset;
uint32 ms5611_ut;  // static result of temperature measurement
uint32 ms5611_up;  // static result of pressure measurement
uint16 ms5611_prom[PROM_NB];  // on-chip ROM
unsigned char  t_rxbuf[3],p_rxbuf[3];//temperature buffer, pressure buffer




/************************************************************   
* 函数名:MS561101BA_readPROM   
* 描述 : 从PROM读取出厂校准数据
* 输入  :无   
* 输出  :无    
*/ 
void MS5611_readPROM(void)
{  

    for(char i = 0; i <= MS561101BA_PROM_REG_COUNT; i ++)
    {
        Cal_C[i] = I2C_Read_2Bytes(MS561101BA_ADDR, MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));
    }
    printf("\rC1=%d,C2=%d,C3=%d,C4=%d,C5=%d,C6=%d\r\n\r",Cal_C[1],Cal_C[2],Cal_C[3],Cal_C[4],Cal_C[5],Cal_C[6]);
}

void MS5611_Reset(void)
{
    Single_Write(MS5611_ADDR, CMD_RESET, 1);
}

void MS5611_Read_Prom(void)
{
    unsigned char rxbuf[2] = {0, 0};
//    char i ;
    for(char i = 0; i < PROM_NB; i ++)
    {
        I2C_Read(MS5611_ADDR, CMD_PROM_RD + (i * 2), 2, rxbuf);
        ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
//        printf("ms5611_prom[%d] = %d\r\n", i, ms5611_prom[i]);
    }
          
}

void MS5611_Read_Adc_T(void)
{
    I2C_Read(MS5611_ADDR, CMD_ADC_READ, 3,t_rxbuf);//read  ADC
}

void MS5611_Read_Adc_P(void)
{
    I2C_Read(MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf);//read ADC
}

void MS5611_Start_T(void)
{
    Single_Write(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1);//D2 temperature conversion start! 
}

void MS5611_Start_P(void)
{
    Single_Write(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1);// D1 pressure conversion start!
}


void MS5611_Init(void)
{
    MS5611_Reset();// Sensor Reset
    Delay_ms(3);
    MS5611_Read_Prom();
    MS5611_Start_T();// Start read temperature
}

int MS5611_Update(void)
{
	static int state = 0;
	
    //	I2C_FastMode = 0;
	
	if (state) 
	{
        MS5611_Read_Adc_P();
        MS5611_Start_T();
        MS5611_BaroAltCalculate();
        state = 0;
	} 
	else 
	{
        MS5611_Read_Adc_T();
        MS5611_Start_P();
        state = 1;
	}
	return (state);
}

float temperature_5611;
void MS5611_BaroAltCalculate(void)
{
	static unsigned char baro_start;
	
    int32 temperature, off2 = 0, sens2 = 0, delt;
    int32 pressure;
	float alt_3;
	
	int32 dT;
	long int off;
	long  long sens;
	
    static unsigned char sum_cnt = BARO_CAL_CNT + 10;
	
    ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
    ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
    
    dT = ms5611_ut - ((uint32)ms5611_prom[5] << 8);
    off = ((uint32)ms5611_prom[2] << 16) + (((long long)dT * ms5611_prom[4]) >> 7);
    sens = ((uint32)ms5611_prom[1] << 15) + (((long long )dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((long long)dT * ms5611_prom[6]) >> 23);
    
    if (temperature < 2000) 
    { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) 
        { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
    
    alt_3 = (101000 - pressure)/1000.0f;
    pressure = 0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
//    printf("\r\n pressure=%d", pressure);
    baroAlt = 10 *(int32)( 0.1f *( pressure - baro_Offset) ) ; //cm
    baro_alt_speed += 5 *0.02 *3.14 *( 50 *( baroAlt - baroAltOld ) - baro_alt_speed ); // 20ms 一次 /0.02 = *50 单位cm/s
//    printf("\r\n baroAlt=%d\r\n", baroAlt);
//    printf("baro_alt_speed=%f\r\n", baro_alt_speed);
    baroAltOld = baroAlt;
    
    if( baro_start < 100 )
    {
        baro_start++;
        baro_alt_speed = 0;
        baroAlt = 0;
    }	
    
    if(sum_cnt)
    {
        sum_cnt--;
    }
    
    temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
//    printf("\r temperature=%f\r\n", temperature_5611);
}

int32 MS5611_Get_BaroAlt(void)
{
	return baroAlt;
}