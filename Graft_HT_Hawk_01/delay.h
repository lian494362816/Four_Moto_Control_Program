#ifndef __DELAY_H
#define __DELAY_H



#define CPU_F ((double)24000000UL)            //XT2 --> 24MHZ
#define Delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) //—” ±1us
#define Delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))    //—” ±1ms

#endif