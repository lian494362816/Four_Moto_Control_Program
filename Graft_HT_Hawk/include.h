#ifndef __INCLUDE_H
#define __INCLUDE_H

typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		  有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         fp64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */
typedef signed short     int int16_t;

#include "mpu6050.h"
#include "type.h"
#include "i2c.h"
#include "uart0.h"
#include "stdio.h"
//#include "delay.h"
#include "board_config.h"
//#include "stdint.h"
#include "msp430F5438A.h"
#include "led.h"
#include "Flash.h"
#include "hmc5883.h"
//#include "timer_A1.h"
//#include "timer_B_Pwm_In.h"
//#include "timer_A0_Pwm_Out.h"
#include "moto.h"
#include "flash.h"
#include "uart1.h"
#include "dma0_uart1_tx.h"
#include "timer.h"

#include "algorithm_quaternion.h"
#include "algorithm_filter.h"
#include "algorithm_sqlite.h"
#include "algorithm_math.h"


#include "multirotor_app.h"
#include "multirotor_ahrs.h"
#include "multirotor_rc.h"
#include "multirotor_control.h"
#include "multirotor_radio.h"
#endif