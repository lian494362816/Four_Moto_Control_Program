#include <msp430f5438A.h>
#include "i2c.h"
#include "delay.h"
#include "include.h"


void I2C_Delay(void)
{
    _NOP();
}

//I2C初始化
void I2C_Init(void)
{
    SCL_DIR_OUT;//P10.2 OUT
    SDA_DIR_OUT;//P10.1 OUT
    SDA_1;      //SDA=1
    SCL_1;      //SCL=1
}
//I2C起始信号
void I2C_Start(void)
{  
    SDA_1;  
    I2C_Delay();
 
    SCL_1; 
    I2C_Delay();

    SDA_0;  
    I2C_Delay();
    SCL_0;  

}
//I2C停止信号
void I2C_Stop(void)
{
    SCL_0;  
    I2C_Delay();
    SDA_0;  
    I2C_Delay();
    SCL_1;  
    I2C_Delay();
    SDA_1;  
}
//发送一个字节的数据
void I2C_SendByte(unsigned char data)
{   
    
    unsigned int i, tmep, tmep1;
    SDA_DIR_OUT;//P10.1 OUT
    tmep = data;
    for(i = 0; i < 8; i ++)
    {
        tmep1 = tmep & 0x80;
        tmep = tmep << 1;
        SCL_0;
        I2C_Delay();
        
        if(0x80 == tmep1)
        {
            SDA_1;
        }
        else
        {
            SDA_0;
        }
        I2C_Delay();
        SCL_1;
        I2C_Delay();
    }
    
    SCL_0;
    I2C_Delay();
    SDA_1;
    I2C_Delay();
    
    
}
//读取一个字节的数据，数据从高位到低位
unsigned char I2C_ReadByte(void)  
{ 
    unsigned char i, j ,k = 0;
    
    SCL_0;
    I2C_Delay();
    SDA_1;
    I2C_Delay();
    SDA_DIR_IN;//SDA设置成输入
    for(i = 0; i < 8; i ++)
    {
        I2C_Delay();
        SCL_1;
        I2C_Delay();
        if(1 == SDA_read)
        {
            j = 1;
        }
        else
        {
            j = 0;
        }
        k= (k << 1) | j;
        SCL_0;
        I2C_Delay();
        
    }
    SDA_DIR_OUT;//SDA设置成输出
    I2C_Delay();
    return (k);
} 
    
//主机发送应答信号
void I2C_Ack(void)
{	
	SCL_0;
	I2C_Delay();
	SDA_0;
	I2C_Delay();
	SCL_1;
	I2C_Delay();
	SCL_1;
	I2C_Delay();
}

//主机发送不应答信号
void I2C_NoAck(void)
{	
	SCL_0;
	I2C_Delay();
	SDA_1;
	I2C_Delay();
	SCL_1;
	I2C_Delay();
	SCL_0;
	I2C_Delay();
} 

//主机等待从机的应答信号
//返回为:=1有ACK,=0无ACK
unsigned char  I2C_WaitAck(void) 	 
{
    SCL_1;
    I2C_Delay();
    SDA_DIR_IN;
    if(SDA_read)
    {
        SCL_0;
        I2C_Delay();
        return 0;
    }
    SDA_DIR_OUT;//P3.1 OUT
    SCL_0;
    I2C_Delay();
    return 1;
}




//主机向从机写入一个字节的数据
//SlaveAddress：从机地址
//REG_Address：要写的寄存器地址
//REG_data：要向寄存器写的数据
//返回值：0-写入不成功 1-写入成功
unsigned char  Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)	
{
    I2C_Start();
    I2C_SendByte(SlaveAddress+0);   //发送设备地址+写信号
    if(!I2C_WaitAck())
    {
        I2C_Stop(); 
        return 0;
    }
    I2C_SendByte(REG_Address);   //设置低起始地址      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    return 1;
}

//主机从从机读出一个字节的数据
//SlaveAddress：从机地址
//REG_Address：要写的寄存器地址
//返回值：读出的数据

unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{  
    unsigned char REG_data;     	
    I2C_Start();
    I2C_SendByte(SlaveAddress+0); //发送设备地址+写信号
    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return 0;
    }
    I2C_SendByte(REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);//Read
    I2C_WaitAck();

    REG_data= I2C_ReadByte();
    I2C_NoAck();
    
    I2C_Stop();
    return REG_data;
}


void I2C_Read(unsigned char addr_, unsigned char  reg_, unsigned char len, unsigned char  *buf)
{
	unsigned char i;
	 // 起始信号
	I2C_Start();
	
	// 发送设备地址
	I2C_SendByte(addr_);   
	
	I2C_WaitAck();
	
	//发送存储单元地址
	I2C_SendByte(reg_);                   
	I2C_WaitAck();
	
	// 起始信号
	I2C_Start();

	//发送设备地址+读信号
	I2C_SendByte(addr_+1);     
	I2C_WaitAck();
	for (i=0; i<len; i++)                   //连续读取6个地址数据，存储中BUF
	{
		*(buf+i) = I2C_ReadByte();          //BUF[0]存储数据
		if (i == len-1)		I2C_NoAck();                   //最后一个数据需要回NOACK
		else		I2C_Ack();                     //回应ACK
	}
	I2C_Stop();                           //停止信号
	I2C_Delay();
}


void I2C_NoAddr_WriteByte(unsigned char DeviceAddr, unsigned char info)
{
    I2C_Start();
    I2C_SendByte(DeviceAddr);
    I2C_WaitAck();
    I2C_SendByte(info);
    I2C_WaitAck();
    I2C_Stop();
    I2C_Delay();
}

unsigned int I2C_Read_2Bytes(unsigned char DeviceAddr, unsigned char address)
{
    unsigned char data_temp1, data_temp2;
    unsigned int data16;
    I2C_Start();
    I2C_SendByte(DeviceAddr);
    I2C_WaitAck();
    I2C_SendByte(address);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(DeviceAddr + 1);
    I2C_WaitAck();
    
    data_temp1 = I2C_ReadByte();
    I2C_Ack();
    data_temp2 = I2C_ReadByte();
    I2C_NoAck();
    
    I2C_Stop();
    I2C_Delay();
    data16 = (data_temp1 * 256) + data_temp2;
    return data16;
}