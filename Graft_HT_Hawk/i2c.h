#ifndef __I2C_H
#define __I2C_H

#include "type.h"
#include "include.h"
//SDA P10.1
//SCL P10.2


#define SDA_1 P10OUT |= BIT1
#define SDA_0 P10OUT &= ~BIT1
#define SCL_1 P10OUT |= BIT2
#define SCL_0 P10OUT &= ~BIT2

#define SDA_DIR_IN      P10DIR &= ~BIT1
#define SDA_DIR_OUT     P10DIR |= BIT1
#define SCL_DIR_IN      P10DIR &= ~BIT2
#define SCL_DIR_OUT     P10DIR |= BIT2

#define SDA_read        ((P10IN >> 1) & 0x01)
void delay5ms(void);
void I2C_Delay(void);
void I2C_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
void I2C_SendByte(unsigned char ucWRData);
unsigned char I2C_ReadByte(void);
unsigned char  I2C_WaitAck(void); 
unsigned char  Single_Write(unsigned char SlaveAddress,
                            unsigned char REG_Address,
                            unsigned char REG_data);

unsigned char Single_Read(unsigned char SlaveAddress,
                          unsigned char REG_Address);

void I2C_Read(unsigned char addr_, unsigned char  reg_, unsigned char len, unsigned char  *buf);

#endif