#include "include.h"


/* 
名称：     void flash_writebuf(uint8_t *flash_ptr, uint8_t *buff, uint8_t len)
作用：     向flash写入数据
 
例：       uint8_t test_buffer[8] = {1, 2, 3, 4, 5, 6, 7, 8};    
           flash_writebuf((uint8_t *) 0x1800, test_buffer, 8);
*/

float write_flash_buffer[write_flash_count]; // 存储要写入的参数

void flash_writebuf(uint8 *flash_ptr, uint8 *buff, uint8 len)
{
    __disable_interrupt();
    FCTL3 = FWKEY;
    FCTL1 = FWKEY + ERASE;
    *(unsigned int *) flash_ptr = 0;
    while(FCTL3 & BUSY);
    FCTL1 = FWKEY + WRT;
    
    for(uint8_t i = 0; i < len; i++)
    {
        *flash_ptr++ = *buff++;
    }
    while(FCTL3 & BUSY);
    FCTL1 = FWKEY;
    FCTL3 = FWKEY + LOCK;
    __enable_interrupt();
}


/* 
    uint8_t new_flash[8] = {0, };
    flash_readbuf((uint8_t *) 0x1800, new_flash, 8);
*/
void flash_readbuf(uint8 *flash_ptr, uint8 *buffer, uint8 len)
{
    __disable_interrupt();
    for(uint8_t i = 0; i < len; i++)
    {
        *buffer++ = *flash_ptr++ ;
    }
    __enable_interrupt();
}



                                                   
/*
名称:   void float_write_flash(void)
作用：  写数据进flash
说明：
    一，必须一次性将所有要使用的数据写入
        因为在写入之前需要做擦除操作
        如果分开写入，先写入的数据会被删除
        所以要一次性写入
    二，写入操作
    1,将小数值扩大1000倍
    2,扩大后的值取出万，千，百，十，个分别放入缓存中
    3,将缓存的值依次放入flash中

    三，x1804由来 
    0x1804 = Flash_addr - 1
    因为buffer是从buffer[1]开始使用，所以Flash_addr + 4的地址是给buffer[0]
    使用的，为无效数据.
    实际有效的数据从Flash - 1开始

    四，所保存的数据为 XX.xxx
    如 保存12.345，最大不能超过63.000

    五，应先调用void PID_to_flash(void)
        再使用该函数
*/

void float_write_flash(void)
{
    uint8_t buffer[write_flash_adder_count];
    uint16_t num;
    for(uint8_t j = 0; j < write_flash_count; j++)//当输入的参数数量改变时，这里也要修改
    {     
        num = (int16_t)(write_flash_buffer[j] * 1000);//要写入的数据分别存入buffer
        buffer[1 + (j * 5)] = num / 10000;
        buffer[2 + (j * 5)] = num / 1000 % 10;
        buffer[3 + (j * 5)] = num / 100 % 10;
        buffer[4 + (j * 5)] = num / 10 % 10;
        buffer[5 + (j * 5)] = num % 10;        
    }
    flash_writebuf((uint8_t *) (0x1804 ), buffer, write_flash_adder_count);
}                                                                  

/****************************************************************************
* 名    称：float float_read_flash( uint16_t read_addr)
* 功    能：读取存储在flash中的数据          
* 入口参数：read_addr 地址值
* 出口参数：f 地址中对应的数数据  
* 说    明: 一次性连续读取5个地址，这个函数是配合读取PID数值用的
* 范    例:
****************************************************************************/
float float_read_flash( uint16_t read_addr)
{
    uint8_t buffer[5];
    float f;
    flash_readbuf((uint8_t *)read_addr, buffer, 5);
    f = (buffer[0] * 10000 + buffer[1] * 1000 + buffer[2] * 100 + buffer[3] * 10 + buffer[4]) / 1000.0;
    return f;
}

/****************************************************************************
* 名    称：void PID_to_flash(void)
* 功    能：将PID参数写入write_flash_buffer 以便存储在Flash中         
* 入口参数： 
* 出口参数：
* 说    明: 
* 范    例: 
****************************************************************************/


void PID_to_flash(void)
{
    write_flash_buffer[ROL_SP_num] = ctrl.roll.shell.kp;//存储roll的数值
    write_flash_buffer[ROL_SI_num] = ctrl.roll.shell.ki;
    write_flash_buffer[ROL_CP_num] = ctrl.roll.core.kp;
    write_flash_buffer[ROL_CI_num] = ctrl.roll.core.ki;
    write_flash_buffer[ROL_CD_num] = ctrl.roll.core.kd;
    
    write_flash_buffer[PIT_SP_num] = ctrl.pitch.shell.kp;//存储pitch的数值
    write_flash_buffer[PIT_SI_num] = ctrl.pitch.shell.ki;
    write_flash_buffer[PIT_CP_num] = ctrl.pitch.core.kp;
    write_flash_buffer[PIT_CI_num] = ctrl.pitch.core.ki;
    write_flash_buffer[PIT_CD_num] = ctrl.pitch.core.kd;
    
    write_flash_buffer[YAW_SP_num] = ctrl.yaw.shell.kp;//存储yaw的数值
    write_flash_buffer[YAW_SI_num] = ctrl.yaw.shell.ki;
    write_flash_buffer[YAW_CP_num] = ctrl.yaw.core.kp;
    write_flash_buffer[YAW_CI_num] = ctrl.yaw.core.ki;
    write_flash_buffer[YAW_CD_num] = ctrl.yaw.core.kd;
    
//    for(char i = 0; i < 15 ; i++)
//    {
//        printf("write_flash_buffer[%d] = %f\r\n", i, write_flash_buffer[i]);
//    }
}


void flash_to_PID(void)
{
    float read_flash_buffer[write_flash_count];
//    ctrl.roll.shell.kp = float_read_flash(ROL_SP_addr);
//    ctrl.roll.shell.ki = float_read_flash(ROL_SI_addr);
//    ctrl.roll.core.kp = float_read_flash(ROL_CP_addr);
//    ctrl.roll.core.ki = float_read_flash(ROL_CI_addr);
//    ctrl.roll.core.kd = float_read_flash(ROL_CD_addr);
//    
//    ctrl.pitch.shell.kp = float_read_flash(PIT_SP_addr);
//    ctrl.pitch.shell.ki = float_read_flash(PIT_SI_addr);
//    ctrl.pitch.core.kp = float_read_flash(PIT_CP_addr);
//    ctrl.pitch.core.ki = float_read_flash(PIT_CI_addr);
//    ctrl.pitch.core.kd = float_read_flash(PIT_CD_addr);
//    
//    ctrl.yaw.shell.kp = float_read_flash(YAW_SP_addr);
//    ctrl.yaw.shell.ki = float_read_flash(YAW_SI_addr);
//    ctrl.yaw.core.kp = float_read_flash(YAW_CP_addr);
//    ctrl.yaw.core.ki = float_read_flash(YAW_CI_addr);
//    ctrl.yaw.core.kd = float_read_flash(YAW_CD_addr);
    
    
    read_flash_buffer[0] = float_read_flash(ROL_SP_addr);
    read_flash_buffer[1] = float_read_flash(ROL_SI_addr);
    read_flash_buffer[2] = float_read_flash(ROL_CP_addr);
    read_flash_buffer[3] = float_read_flash(ROL_CI_addr);
    read_flash_buffer[4] = float_read_flash(ROL_CD_addr);
    
    read_flash_buffer[5] = float_read_flash(PIT_SP_addr);
    read_flash_buffer[6] = float_read_flash(PIT_SI_addr);
    read_flash_buffer[7] = float_read_flash(PIT_CP_addr);
    read_flash_buffer[8] = float_read_flash(PIT_CI_addr);
    read_flash_buffer[9] = float_read_flash(PIT_CD_addr);
    
    read_flash_buffer[10] = float_read_flash(YAW_SP_addr);
    read_flash_buffer[11] = float_read_flash(YAW_SI_addr);
    read_flash_buffer[12] = float_read_flash(YAW_CP_addr);
    read_flash_buffer[13] = float_read_flash(YAW_CI_addr);
    read_flash_buffer[14] = float_read_flash(YAW_CD_addr);
    
//    ctrl.roll.shell.kp = read_flash_buffer[0];
//    ctrl.roll.shell.ki = read_flash_buffer[1];
    ctrl.roll.core.kp = read_flash_buffer[2];
    ctrl.roll.core.ki = read_flash_buffer[3];
    ctrl.roll.core.kd = read_flash_buffer[4];
    
//    ctrl.pitch.shell.kp = read_flash_buffer[5];
//    ctrl.pitch.shell.ki = read_flash_buffer[6];
    ctrl.pitch.core.kp = read_flash_buffer[7];
    ctrl.pitch.core.ki = read_flash_buffer[8];
    ctrl.pitch.core.kd = read_flash_buffer[9];
    
//    ctrl.yaw.shell.kp = read_flash_buffer[10];
//    ctrl.yaw.shell.ki = read_flash_buffer[11];
    ctrl.yaw.core.kp = read_flash_buffer[12];
    ctrl.yaw.core.ki = read_flash_buffer[13];
    ctrl.yaw.core.kd = read_flash_buffer[14];
//    for(char i = 0; i < 15 ; i++)
//    {
//        printf("read_flash_buffer[%d] = %f\r\n", i, read_flash_buffer[i]);
//    }
}