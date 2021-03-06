#include "include.h"

//输入
//  P1.2/TA0.1|--> YAW
//  P1.3/TA0.2|-->THROTTLE
//  P1.4/TA0.3|-->PITCH
//  P1.5/TA0.4|-->ROLL


//输出
//  P4.2/TB2|--> CCR2     
//  P4.3/TB3|--> CCR3 
//  P4.4/TB4|--> CCR4 
//  P4.5/TB5|--> CCR5


/* 

*/
RC_GETDATA RC_Data;
uint16 Rise[6], Drop[6];
uint16 Channel5, Channel6; 
char Flag_ANO_DT_Send_Status = 0;
char time_count = 0;
char time_count_2 = 0;

extern float alt_us100_1; //高度差距
extern float US100_Alt;//高度
extern float THR_Lock;//高度的油门

float   US100_Alt_V,Alt_CuntTmep1=0,Alt_CuntTmep2=0,
        US100_Alt_Last=0,Alt_V_CuntTmep1=0,
        Alt_V_CuntTmep2=0,US100_Alt; 
extern float  Alt_Last;
extern int US100_Alt_Temp;
extern unsigned char ultra_ok;
unsigned char Lock_Down = 0;
unsigned char Control_Mode = 0;
unsigned char Fly_Mode = 0;

//    YAW       930~2078 中值1510
//    THROTTLE  936~2083  
//    PITCH     957~2104 中值1584   受影响1410~1678
//    ROLL      963~2111 中值1546   受影响1411~1605     
 

//  2016.7.27
//  PITCH   936 ~ 2087     理论中值 1511.5
//  ROLL    917 ~ 2067     理论中值 1492

/*  TA
    PWM 输入
    1MHz
    定时13ms
*/

void TimerA_PWM_In_Init(void)
{
    P1SEL |= BIT2 + BIT3 + BIT4 + BIT5;               //设置为第二功能，作为捕获源
    P1DIR &= ~( BIT2 + BIT3 + BIT4 + BIT5);          //TA0.x设置为输入
    
    TA0CTL = TASSEL_2 + MC_2 + TACLR + TAIE + ID_2;  // SMCLK, Continuous up, clear TAR
    TA0CCTL0 = CCIE;   
    TA0EX0 |= TAIDEX_2;
    TA0CCR0 = 13000;                                
    TA0CCTL1 = CAP + CM_1 + SCS + CCIE;     //捕获模式，通道B，上升沿捕获，同步，打开中断
    TA0CCTL2 = CAP + CM_1 + SCS + CCIE;
    TA0CCTL3 = CAP + CM_1 + SCS + CCIE;
    TA0CCTL4 = CAP + CM_1 + SCS + CCIE;
}



/*  TB 
    PWM输出
    1MHz / 2500 = 400Hz
*/
void TimerB_PWM_Out_Init(void)
{
    P4SEL |= BIT2 + BIT3 + BIT4 + BIT5;
    P4DIR |= BIT2 + BIT3 + BIT4 + BIT5;  
    
    TBCCR0  = 2500;      
    TBCCTL2 = OUTMOD_7;
    TBCCR2 = 1000;
    
    TBCCTL3 = OUTMOD_7;
    TBCCR3 = 1000;
    
    TBCCTL4 = OUTMOD_7;
    TBCCR4 = 1000;
    
    TBCCTL5 = OUTMOD_7;
    TBCCR5 = 1000;
    
    TBCTL = TBSSEL_2 + MC__UP + TBCLR + ID_2;    // SMCLK, upmode, clear TBR，四分频
    TBEX0|=TBIDEX_2;//3分频

}


/* 
    Timer0 定时中断
*/
#pragma vector = TIMER0_A0_VECTOR//13ms一次定时中断
__interrupt void TIMER0_A0_ISR(void)
{
    _EINT();                    //开总中断（因为msp430默认进中断就关总中断）
//    printf("13ms中断\r\n");
    time_count ++;
    time_count_2 ++;
    ADNS3080_Read_Position();
    if(2 == time_count_2)
    {
        ADNS3080_CONTROL();
        time_count_2 = 0;
    }
    if(time_count > 5)
    {     
        Send_US100_Start(); 
        if(1 == ultra_ok)
        {
            Alt_CuntTmep2=Alt_CuntTmep1;        //滑动平均滤波
            Alt_CuntTmep1=US100_Alt_Temp*((float)COS(IMU.Pitch/57.295779f))*((float)COS(IMU.Roll/57.295779f));//姿态补偿
            
            US100_Alt=((Alt_CuntTmep1+Alt_CuntTmep2)/2)/1000;  //除以1000转化为m
//            printf("%f\r\n", US100_Alt);
            Alt_V_CuntTmep2=Alt_V_CuntTmep1;//滑动平均滤波
            Alt_V_CuntTmep1=(US100_Alt-US100_Alt_Last)/ 0.05f;   //除以0.05s获得速度单位 ：m/s
            
            US100_Alt_V= (Alt_V_CuntTmep1+Alt_V_CuntTmep2)/2;   
            US100_Alt_Last=US100_Alt;     
            time_count = 0;
            ultra_ok = 0;
        }
    }
    TA0CCR0 += 13000;
    AHRS_Geteuler();	//
    Calculate_Target(); //
    CONTROL(Target);
    if(Flag_ANO_DT_Send_Status)
    {
        ANO_DT_Send_Status(US100_Alt ,0, THR_Lock, 0, 0,0);
//        ANO_DT_Send_Status(IMU.Roll, -IMU.Pitch, IMU.Yaw, 0, 0,0);
    }
}
/* 
    Timer0 捕获中断
*/
#pragma vector=TIMER0_A1_VECTOR //CCR1~CCR4中断
__interrupt void TIMER0_A1_ISR(void)//外部电平捕获中断
{
    switch(__even_in_range(TA0IV,14))//与switch配合使用,用来选择偶数
    {                                //可生成效率比较高的代码
        case  0: break;                          // No interrupt
       
        case  2: 
            if(TA0CCTL1 & CM0)                        //捕获到上升沿
            {
                TA0CCTL1 = (TA0CCTL1&(~CM0))| CM1;    //改为下降沿捕获:CM0置零,CM1置一
                Rise[0]=TA0R;
            }
           
            else if(TA0CCTL1 & CM1)                   //捕获到下降沿
            {
                TA0CCTL1 = (TA0CCTL1&(~CM1))| CM0;   //改为上升沿捕获：CM1置零,CM0置一   
                Drop[0] = TA0R;
                if(Rise[0] > Drop[0])
                {
                    RC_Data.YAW = 65535 - Rise[0] + Drop[0];
//                   printf("RC_Data.YAW_1=%d\r\n", RC_Data.YAW);
                }
                else
                {
                    RC_Data.YAW = Drop[0] - Rise[0];
//                   printf("RC_Data.YAW_2=%d\r\n", RC_Data.YAW);
                }
            }
//             printf("Rc_Data.YAW = %d\r\n", RC_Data.YAW);
            break;  

        case  4:       
            if(TA0CCTL2 & CM0)                        //捕获到上升沿
            {
                TA0CCTL2 = (TA0CCTL2&(~CM0))| CM1;    //改为下降沿捕获:CM0置零,CM1置一
                Rise[1]= TA0R;
            }
            else if(TA0CCTL2 & CM1)                   //捕获到下降沿
            {
                TA0CCTL2 = (TA0CCTL2&(~CM1))| CM0;   //改为上升沿捕获：CM1置零,CM0置一          
                Drop[1] = TA0R;
                if(Rise[1] > Drop[1])
                {
                    RC_Data.THROTTLE = 65536 - Rise[1] + Drop[1];
                }
                else
                {
                    RC_Data.THROTTLE = Drop[1] - Rise[1];
//                    printf("RC_Data.THROTTLE=%d\r\n", RC_Data.THROTTLE);
                }
            }
            break;    

        case  6:
            if(TA0CCTL3 & CM0)                        //捕获到上升沿
            {
                TA0CCTL3 = (TA0CCTL3&(~CM0))| CM1;    //改为下降沿捕获:CM0置零,CM1置一
                Rise[2] = TA0R;
            }

            else if(TA0CCTL3 & CM1)                   //捕获到下降沿
            {
                TA0CCTL3 = (TA0CCTL3&(~CM1))| CM0;   //改为上升沿捕获：CM1置零,CM0置一          
                Drop[2] = TA0R;
                if(Rise[2] > Drop[2])
                {
                    RC_Data.PITCH = 65535 - Rise[2] + Drop[2];
                    
                }
                else
                {
                    RC_Data.PITCH = Drop[2] - Rise[2];
//                    printf("RC_Data.PITCH=%d\r\n", RC_Data.PITCH);
                }
            }

            break;  
            
        case  8: 
            if(TA0CCTL4 & CM0)                        //捕获到上升沿
            {                 
                TA0CCTL4    = (TA0CCTL4&(~CM0))| CM1;    //改为下降沿捕获:CM0置零,CM1置一
                Rise[3] = TA0R;
            }

            else if(TA0CCTL4 & CM1)                   //捕获到下降沿
            {
                TA0CCTL4 = (TA0CCTL4&(~CM1))| CM0;   //改为上升沿捕获：CM1置零,CM0置一          
                Drop[3] = TA0R;
                if(Rise[3] > Drop[3])
                {
                    RC_Data.ROLL = 65535 - Rise[3] + Drop[3];                     
                }
                else
                {
                    RC_Data.ROLL = Drop[3] - Rise[3];
//                     printf("RC_Data.ROLL=%d\r\n", RC_Data.ROLL);
                }
            }
            break;                          // reserved
            
        case 10: break;                          // reserved
      
        case 12: break;                          // reserved
        
        case 14: 
      
            break;
        
        default: break; 
    }
}
/* 
    TA1.1-->  P2.2
    TA1.2-->  P2.3
    12MHz / 12 = 1MHz
*/
void TimerA1_PWM_In_Init(void)
{
    P2SEL |= BIT2 + BIT3;            //设置为第二功能，作为捕获源
    P2DIR &= ~( BIT2 + BIT3);          //TA1.x设置为输入
    
    TA1CTL = TASSEL_2 + MC_2 + TACLR + TAIE + ID_2;  // SMCLK = 12M , Continuous up, clear TAR, 4分频
    TA1EX0 |= TAIDEX_2;//3分频
                               
    TA1CCTL1 = CAP + CM_1 + SCS + CCIE;     //捕获模式，通道B，上升沿捕获，同步，打开中断
    TA1CCTL2 = CAP + CM_1 + SCS + CCIE;

}

/* 
    Timer1 捕获中断
*/
#pragma vector=TIMER1_A1_VECTOR //CCR1~CCR4中断
__interrupt void TIMER1_A1_ISR(void)//外部电平捕获中断
{
    switch(__even_in_range(TA1IV,14))//与switch配合使用,用来选择偶数
    {                                //可生成效率比较高的代码
        case  0: break;                          // No interrupt
       
        case  2: 
            if(TA1CCTL1 & CM0)                        //捕获到上升沿
            {
                TA1CCTL1 = (TA1CCTL1&(~CM0))| CM1;    //改为下降沿捕获:CM0置零,CM1置一
                Rise[4]=TA1R;
            }
           
            else if(TA1CCTL1 & CM1)                   //捕获到下降沿
            {
                TA1CCTL1 = (TA1CCTL1&(~CM1))| CM0;   //改为上升沿捕获：CM1置零,CM0置一   
                Drop[4] = TA1R;
                if(Rise[4] > Drop[4])
                {
                    Channel5 = 65535 - Rise[4] + Drop[4];
//                    printf("Channel5=%d\r\n", Channel5);
                }
                else
                {
                    Channel5 = Drop[4] - Rise[4];
//                    printf("Channel5=%d\r\n", Channel5);
                }
            }
            
            if(Channel5 < 1100)
            {
                Lock_Down = 1;
                Control_Mode = 0;
//                printf("Lock_Down=%d\r\n", Lock_Down);
            }
            else if(Channel5 > 1100 && Channel5 < 1800)
            {
                Control_Mode = Height_Mode;
                Lock_Down = 0; 
            }
            else if(Channel5 > 1900)
            {
                Control_Mode = Normal_Mode;
                Lock_Down = 0;
            }
//            printf("Lock_Down=%d\r\n", Lock_Down);
            break;  

        case  4:       
            if(TA1CCTL2 & CM0)                        //捕获到上升沿
            {
                TA1CCTL2 = (TA1CCTL2&(~CM0))| CM1;    //改为下降沿捕获:CM0置零,CM1置一
                Rise[5]=TA1R;
            }
            else if(TA1CCTL2 & CM1)                   //捕获到下降沿
            {
                TA1CCTL2 = (TA1CCTL2&(~CM1))| CM0;   //改为上升沿捕获：CM1置零,CM0置一   
                Drop[5] = TA1R;
                if(Rise[5] > Drop[5])
                {
                    Channel6 = 65535 - Rise[5] + Drop[5];
//                    printf("Channel6=%d\r\n", Channel6);
                }
                else
                {
                    Channel6 = Drop[5] - Rise[5];
//                    printf("Channel6=%d\r\n", Channel6);
                }
            }
            
            if(Channel6 < 1900)
            {
                Fly_Mode = One_Key_Down;
//                printf("3\r\n");
            }
            else 
            {
                Fly_Mode = One_Key_Up;
            }
            
            break;    
            
        default: break; 
    }
}