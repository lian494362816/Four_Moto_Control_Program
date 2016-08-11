#ifndef __ADNS_3080_H
#define __ADNS_3080_H
#define uint unsigned int
#define uchar unsigned char
#define Product_ID   0x00
#define Motion 	     0x02
#define  DX		     0X03
#define  DY		     0X04
#define  SQUAL		 0x05
#define  Pixel_Sum	 0x06
#define Configuration_bits	 0x0a
#define Extended_Config      0x0b
#define Frame_Period_Lower	 0x10
#define Frame_Period_Uppe	 0x11
#define	Motion_Clear	     0x12
#define	Frame_Capture	     0x13
#define	SROM_Enable		     0x14
#define Frame_Period_Max_Bound_Lower	0x19
#define Frame_Period_Max_Bound_Upper    0x1a
#define Frame_Period_Min_Bound_Lower    0x1b
#define Frame_Period_Min_Bound_Upper    0x1c
#define Shutter_Max_Bound_Lower         0x1d
#define Shutter_Max_Bound_Upper         0x1e
#define  SROM_ID	         0x1f
#define	Pixel_Burst	         0x40
#define	Motion_Burst         0x50
#define	SROM_Load	         0x60
#define Cs_1 P6OUT |= BIT1
#define Cs_0 P6OUT &= ~BIT1
#define Reset_1 P6OUT |= BIT2
#define Reset_0 P6OUT &= ~BIT2
#define NPD_1 P6OUT |= BIT3

struct ADNS_PID
{
    float P;
    float I;
    float D;
    float pxout;
    float pyout;
    float dxout;
    float dyout;
    float Xout;
    float Yout;
};





uint SPI_SendReceive(uint) ; //SPI的收发
void Write_srom(void);
void ADNS3080_Init(void);
void SPI_init(void);
void ADNS_Configuration(void);
void ADNS3080_reset(void);
void write_register(uint adress,uint vlue);
uint read_register(uint adress);
uint read_busy(void);//写帧率的判别
void clear_motion(void);
long read_zhenlv(void); //读帧率
float read_average_pixel(void);	  //读平均像素
void read_pixel(void);//读像素
void read_pixel_burst(void);//爆发读像素
void Read_Data_burst(void);
void ADNS3080_Read_Position(void);
void ADNS3080_CONTROL(void);
#endif
