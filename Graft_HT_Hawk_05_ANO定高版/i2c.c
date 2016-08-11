#include <msp430f5438A.h>
#include "i2c.h"
#include "delay.h"
#include "include.h"


void I2C_Delay(void)
{
    _NOP();
}

//I2C��ʼ��
void I2C_Init(void)
{
    SCL_DIR_OUT;//P10.2 OUT
    SDA_DIR_OUT;//P10.1 OUT
    SDA_1;      //SDA=1
    SCL_1;      //SCL=1
}
//I2C��ʼ�ź�
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
//I2Cֹͣ�ź�
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
//����һ���ֽڵ�����
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
//��ȡһ���ֽڵ����ݣ����ݴӸ�λ����λ
unsigned char I2C_ReadByte(void)  
{ 
    unsigned char i, j ,k = 0;
    
    SCL_0;
    I2C_Delay();
    SDA_1;
    I2C_Delay();
    SDA_DIR_IN;//SDA���ó�����
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
    SDA_DIR_OUT;//SDA���ó����
    I2C_Delay();
    return (k);
} 
    
//��������Ӧ���ź�
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

//�������Ͳ�Ӧ���ź�
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

//�����ȴ��ӻ���Ӧ���ź�
//����Ϊ:=1��ACK,=0��ACK
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




//������ӻ�д��һ���ֽڵ�����
//SlaveAddress���ӻ���ַ
//REG_Address��Ҫд�ļĴ�����ַ
//REG_data��Ҫ��Ĵ���д������
//����ֵ��0-д�벻�ɹ� 1-д��ɹ�
unsigned char  Single_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)	
{
    I2C_Start();
    I2C_SendByte(SlaveAddress+0);   //�����豸��ַ+д�ź�
    if(!I2C_WaitAck())
    {
        I2C_Stop(); 
        return 0;
    }
    I2C_SendByte(REG_Address);   //���õ���ʼ��ַ      
    I2C_WaitAck();	
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    return 1;
}

//�����Ӵӻ�����һ���ֽڵ�����
//SlaveAddress���ӻ���ַ
//REG_Address��Ҫд�ļĴ�����ַ
//����ֵ������������

unsigned char Single_Read(unsigned char SlaveAddress,unsigned char REG_Address)
{  
    unsigned char REG_data;     	
    I2C_Start();
    I2C_SendByte(SlaveAddress+0); //�����豸��ַ+д�ź�
    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return 0;
    }
    I2C_SendByte(REG_Address);   //���õ���ʼ��ַ      
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
	 // ��ʼ�ź�
	I2C_Start();
	
	// �����豸��ַ
	I2C_SendByte(addr_);   
	
	I2C_WaitAck();
	
	//���ʹ洢��Ԫ��ַ
	I2C_SendByte(reg_);                   
	I2C_WaitAck();
	
	// ��ʼ�ź�
	I2C_Start();

	//�����豸��ַ+���ź�
	I2C_SendByte(addr_+1);     
	I2C_WaitAck();
	for (i=0; i<len; i++)                   //������ȡ6����ַ���ݣ��洢��BUF
	{
		*(buf+i) = I2C_ReadByte();          //BUF[0]�洢����
		if (i == len-1)		I2C_NoAck();                   //���һ��������Ҫ��NOACK
		else		I2C_Ack();                     //��ӦACK
	}
	I2C_Stop();                           //ֹͣ�ź�
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