#ifndef __FLASH_H
#define __FLASH_H


//void Display_Last_PID(void);
#define Flash_addr 0x1805//起始地址，请勿修改
#define write_flash_count (15  + 1)//数值为 参数数目 
#define write_flash_adder_count (write_flash_count * 5)//数值为 参数数目X5 目前参数有15个 所以为 75


#define ROL_SP_num 0    //ROL 参数标号 用来确定在Flash中的地址
#define ROL_SI_num 1
#define ROL_CP_num 2
#define ROL_CI_num 3
#define ROL_CD_num 4

#define PIT_SP_num 5     //PIT 参数标号 用来确定在Flash中的地址
#define PIT_SI_num 6
#define PIT_CP_num 7
#define PIT_CI_num 8
#define PIT_CD_num 9

#define YAW_SP_num 10    //YAW 参数标号 用来确定在Flash中的地址
#define YAW_SI_num 11
#define YAW_CP_num 12
#define YAW_CI_num 13
#define YAW_CD_num 14


#define ROL_SP_addr (ROL_SP_num * 5 + Flash_addr)    //ROL 在Flash 中的地址
#define ROL_SI_addr (ROL_SI_num * 5 + Flash_addr)
#define ROL_CP_addr (ROL_CP_num * 5 + Flash_addr)
#define ROL_CI_addr (ROL_CI_num * 5 + Flash_addr)
#define ROL_CD_addr (ROL_CD_num * 5 + Flash_addr)

#define PIT_SP_addr (PIT_SP_num * 5 + Flash_addr)    //PIT 在Flash 中的地址
#define PIT_SI_addr (PIT_SI_num * 5 + Flash_addr)
#define PIT_CP_addr (PIT_CP_num * 5 + Flash_addr)
#define PIT_CI_addr (PIT_CI_num * 5 + Flash_addr)
#define PIT_CD_addr (PIT_CD_num * 5 + Flash_addr)

#define YAW_SP_addr (YAW_SP_num * 5 + Flash_addr)    //YAW 在Flash 中的地址
#define YAW_SI_addr (YAW_SI_num * 5 + Flash_addr)
#define YAW_CP_addr (YAW_CP_num * 5 + Flash_addr)
#define YAW_CI_addr (YAW_CI_num * 5 + Flash_addr)
#define YAW_CD_addr (YAW_CD_num * 5 + Flash_addr)

void flash_writebuf(uint8 *flash_ptr, uint8 *buffer, uint8 len);
void flash_readbuf(uint8 *flash_ptr, uint8 *buffer, uint8 len);

float float_read_flash( uint16_t read_addr);

void float_write_flash(void);
void PID_to_flash(void);
void flash_to_PID(void);
#endif