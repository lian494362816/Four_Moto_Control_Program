#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

//  ³¬Éù²¨
//  P2.1   Trig
//  P2.2   Echo
#define CLK_Trig  P2OUT &= ~BIT1 
#define SET_Trig  P2OUT |=  BIT1 
#define Echo (P2IN & BIT2) //»Ø²¨Òý½Å

extern float US100_Alt;
extern float US100_Alt_V;

extern float g_HightControl;
extern float US100_Alt;
extern float US100_Alt_delta,US100_Alt_old;
void Ultrasonic_Config(void);
void Ultrasonic_Pulsing(void);
#endif 