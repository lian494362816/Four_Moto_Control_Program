#ifndef __MS5611_H
#define __MS5611_H


/*宏定义------------------------------------------------------------------*/
//定义器件在IIC总线中的从地址,根据CSB引脚不同修改
//#define MS561101BA_ADDR  0xec   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_ADDR   0xee   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// 定义MS561101BA内部地址
// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08
//#define  MS561101BA_D1_OSR_256 0x40 
//#define  MS561101BA_D1_OSR_512 0x42 
//#define  MS561101BA_D1_OSR_1024 0x44 
//#define  MS561101BA_D1_OSR_2048 0x46 
#define  MS561101BA_D1_OSR_4096 0x48 

//#define  MS561101BA_D2_OSR_256 0x50 
//#define  MS561101BA_D2_OSR_512 0x52 
//#define  MS561101BA_D2_OSR_1024 0x54 
//#define  MS561101BA_D2_OSR_2048 0x56 
#define  MS561101BA_D2_OSR_4096 0x58 

#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry. 



#define MS5611_ADDR             0xee   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096

#define BARO_CAL_CNT 200
void MS5611_Reset(void);
void MS5611_readPROM(void);

void MS5611_Reset(void);
void MS5611_Read_Prom(void);
void MS5611_BaroAltCalculate(void);


void MS5611_Init(void);
int MS5611_Update(void);

#endif