#include "include.h"


/* 
���ƣ�     void flash_writebuf(uint8_t *flash_ptr, uint8_t *buff, uint8_t len)
���ã�     ��flashд������
 
����       uint8_t test_buffer[8] = {1, 2, 3, 4, 5, 6, 7, 8};    
           flash_writebuf((uint8_t *) 0x1800, test_buffer, 8);
*/

float write_flash_buffer[write_flash_count]; // �洢Ҫд��Ĳ���

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
����:   void float_write_flash(void)
���ã�  д���ݽ�flash
˵����
    һ������һ���Խ�����Ҫʹ�õ�����д��
        ��Ϊ��д��֮ǰ��Ҫ����������
        ����ֿ�д�룬��д������ݻᱻɾ��
        ����Ҫһ����д��
    ����д�����
    1,��С��ֵ����1000��
    2,������ֵȡ����ǧ���٣�ʮ�����ֱ���뻺����
    3,�������ֵ���η���flash��

    ����x1804���� 
    0x1804 = Flash_addr - 1
    ��Ϊbuffer�Ǵ�buffer[1]��ʼʹ�ã�����Flash_addr + 4�ĵ�ַ�Ǹ�buffer[0]
    ʹ�õģ�Ϊ��Ч����.
    ʵ����Ч�����ݴ�Flash - 1��ʼ

    �ģ������������Ϊ XX.xxx
    �� ����12.345������ܳ���63.000

    �壬Ӧ�ȵ���void PID_to_flash(void)
        ��ʹ�øú���
*/

void float_write_flash(void)
{
    uint8_t buffer[write_flash_adder_count];
    uint16_t num;
    for(uint8_t j = 0; j < write_flash_count; j++)//������Ĳ��������ı�ʱ������ҲҪ�޸�
    {     
        num = (int16_t)(write_flash_buffer[j] * 1000);//Ҫд������ݷֱ����buffer
        buffer[1 + (j * 5)] = num / 10000;
        buffer[2 + (j * 5)] = num / 1000 % 10;
        buffer[3 + (j * 5)] = num / 100 % 10;
        buffer[4 + (j * 5)] = num / 10 % 10;
        buffer[5 + (j * 5)] = num % 10;        
    }
    flash_writebuf((uint8_t *) (0x1804 ), buffer, write_flash_adder_count);
}                                                                  

/****************************************************************************
* ��    �ƣ�float float_read_flash( uint16_t read_addr)
* ��    �ܣ���ȡ�洢��flash�е�����          
* ��ڲ�����read_addr ��ֵַ
* ���ڲ�����f ��ַ�ж�Ӧ��������  
* ˵    ��: һ����������ȡ5����ַ�������������϶�ȡPID��ֵ�õ�
* ��    ��:
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
* ��    �ƣ�void PID_to_flash(void)
* ��    �ܣ���PID����д��write_flash_buffer �Ա�洢��Flash��         
* ��ڲ����� 
* ���ڲ�����
* ˵    ��: 
* ��    ��: 
****************************************************************************/


void PID_to_flash(void)
{
    write_flash_buffer[ROL_SP_num] = ctrl.roll.shell.kp;//�洢roll����ֵ
    write_flash_buffer[ROL_SI_num] = ctrl.roll.shell.ki;
    write_flash_buffer[ROL_CP_num] = ctrl.roll.core.kp;
    write_flash_buffer[ROL_CI_num] = ctrl.roll.core.ki;
    write_flash_buffer[ROL_CD_num] = ctrl.roll.core.kd;
    
    write_flash_buffer[PIT_SP_num] = ctrl.pitch.shell.kp;//�洢pitch����ֵ
    write_flash_buffer[PIT_SI_num] = ctrl.pitch.shell.ki;
    write_flash_buffer[PIT_CP_num] = ctrl.pitch.core.kp;
    write_flash_buffer[PIT_CI_num] = ctrl.pitch.core.ki;
    write_flash_buffer[PIT_CD_num] = ctrl.pitch.core.kd;
    
    write_flash_buffer[YAW_SP_num] = ctrl.yaw.shell.kp;//�洢yaw����ֵ
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