#include "include.h"

//����
//  P1.2/TA0.1|--> YAW
//  P1.3/TA0.2|-->THROTTLE
//  P1.4/TA0.3|-->PITCH
//  P1.5/TA0.4|-->ROLL


//���
//  P4.2/TB2|--> CCR2     
//  P4.3/TB3|--> CCR3 
//  P4.4/TB4|--> CCR4 
//  P4.5/TB5|--> CCR5


/* 

*/
RC_GETDATA RC_Data;
uint16 Rise[4], Drop[4], RC_Pwm_In[4];
char Flag_ANO_DT_Send_Status = 0;
char time_count = 0;

extern float alt_us100_1; //�߶Ȳ��
extern float US100_Alt;//�߶�
extern float THR_Lock;//�߶ȵ�����

//    YAW       930~2078 ��ֵ1510
//    THROTTLE  936~2083  
//    PITCH     957~2104 ��ֵ1584   ��Ӱ��1410~1678
//    ROLL      963~2111 ��ֵ1546   ��Ӱ��1411~1605     
 

//  2016.7.27
//  PITCH   936 ~ 2087     ������ֵ 1511.5
//  ROLL    917 ~ 2067     ������ֵ 1492

/*  TA
    PWM ����
    1MHz
    ��ʱ13ms
*/

void TimerA_PWM_In_Init(void)
{
    P1SEL |= BIT2 + BIT3 + BIT4 + BIT5;               //����Ϊ�ڶ����ܣ���Ϊ����Դ
    P1DIR &= ~( BIT2 + BIT3 + BIT4 + BIT5);          //TA0.x����Ϊ����
    
    TA0CTL = TASSEL_2 + MC_2 + TACLR + TAIE + ID_2;  // SMCLK, Continuous up, clear TAR
    TA0CCTL0 = CCIE;   
    TA0EX0 |= TAIDEX_2;
    TA0CCR0 = 13000;                                
    TA0CCTL1 = CAP + CM_1 + SCS + CCIE;     //����ģʽ��ͨ��B�������ز���ͬ�������ж�
    TA0CCTL2 = CAP + CM_1 + SCS + CCIE;
    TA0CCTL3 = CAP + CM_1 + SCS + CCIE;
    TA0CCTL4 = CAP + CM_1 + SCS + CCIE;
}



/*  TB 
    PWM���
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
    
    TBCTL = TBSSEL_2 + MC__UP + TBCLR + ID_2;    // SMCLK, upmode, clear TBR���ķ�Ƶ
    TBEX0|=TBIDEX_2;//3��Ƶ

}


/* 
    Timer0 ��ʱ�ж�
*/
#pragma vector = TIMER0_A0_VECTOR//13msһ�ζ�ʱ�ж�
__interrupt void TIMER0_A0_ISR(void)
{
    _EINT();                    //�����жϣ���Ϊmsp430Ĭ�Ͻ��жϾ͹����жϣ�
//    printf("13ms�ж�\r\n");
    time_count ++;
    if(time_count > 5)
    {     
        Send_US100_Start(); 
        time_count = 0;
    }
    TA0CCR0 += 13000;
    AHRS_Geteuler();	//
    Calculate_Target(); //
    CONTROL(Target);
//    HtoEs_Attitude_Data_Generate();
//    ANO_DT_Send_PID(1, ctrl.pitch.core.kp, ctrl.pitch.core.ki, ctrl.pitch.core.kd,
//                       ctrl.roll.core.kp, ctrl.roll.core.ki, ctrl.roll.core.kd,
//                       ctrl.yaw.core.kp, ctrl.yaw.core.ki, ctrl.yaw.core.kd);
    if(Flag_ANO_DT_Send_Status)
    {
        ANO_DT_Send_Status(US100_Alt ,0, THR_Lock, 0, 0,0);
//        ANO_DT_Send_Status(IMU.Roll, -IMU.Pitch, IMU.Yaw, 0, 0,0);
    }
}
/* 
    Timer0 �����ж�
*/
#pragma vector=TIMER0_A1_VECTOR //CCR1~CCR4�ж�
__interrupt void TIMER0_A1_ISR(void)//�ⲿ��ƽ�����ж�
{
    switch(__even_in_range(TA0IV,14))//��switch���ʹ��,����ѡ��ż��
    {                                //������Ч�ʱȽϸߵĴ���
        case  0: break;                          // No interrupt
       
        case  2: 
            if(TA0CCTL1 & CM0)                        //����������
            {
                TA0CCTL1 = (TA0CCTL1&(~CM0))| CM1;    //��Ϊ�½��ز���:CM0����,CM1��һ
                Rise[0]=TA0R;
            }
           
            else if(TA0CCTL1 & CM1)                   //�����½���
            {
                TA0CCTL1 = (TA0CCTL1&(~CM1))| CM0;   //��Ϊ�����ز���CM1����,CM0��һ   
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
            if(TA0CCTL2 & CM0)                        //����������
            {
                TA0CCTL2 = (TA0CCTL2&(~CM0))| CM1;    //��Ϊ�½��ز���:CM0����,CM1��һ
                Rise[1]= TA0R;
            }
            else if(TA0CCTL2 & CM1)                   //�����½���
            {
                TA0CCTL2 = (TA0CCTL2&(~CM1))| CM0;   //��Ϊ�����ز���CM1����,CM0��һ          
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
            if(TA0CCTL3 & CM0)                        //����������
            {
                TA0CCTL3 = (TA0CCTL3&(~CM0))| CM1;    //��Ϊ�½��ز���:CM0����,CM1��һ
                Rise[2] = TA0R;
            }

            else if(TA0CCTL3 & CM1)                   //�����½���
            {
                TA0CCTL3 = (TA0CCTL3&(~CM1))| CM0;   //��Ϊ�����ز���CM1����,CM0��һ          
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
            if(TA0CCTL4 & CM0)                        //����������
            {                 
                TA0CCTL4    = (TA0CCTL4&(~CM0))| CM1;    //��Ϊ�½��ز���:CM0����,CM1��һ
                Rise[3] = TA0R;
            }

            else if(TA0CCTL4 & CM1)                   //�����½���
            {
                TA0CCTL4 = (TA0CCTL4&(~CM1))| CM0;   //��Ϊ�����ز���CM1����,CM0��һ          
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

