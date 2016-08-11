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

//  ������
//  P2.1   Trig
//  P2.2   Echo
/* 

*/
RC_GETDATA RC_Data;
uint16 Rise[5], Drop[5], RC_Pwm_In[4];
char Flag_ANO_DT_Send_Status = 0;
extern unsigned int Distance;
char time_count = 0;

//    YAW       930~2078 ��ֵ1510
//    THROTTLE  936~2083  
//    PITCH     957~2104 ��ֵ1584   ��Ӱ��1410~1678
//    ROLL      963~2111 ��ֵ1546   ��Ӱ��1411~1605     
 

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
    TA0CCR0 += 13000;
    time_count ++;
    AHRS_Geteuler();	//
    Calculate_Target(); //
    CONTROL(Target);
//    Ultrasonic_Pulsing();
//    US100_CONTROL(0.40);
    if(time_count > 5)
    {
        Send_US100_Start();
        time_count = 0;
    }
//    HtoEs_Attitude_Data_Generate();
//    ANO_DT_Send_PID(1, ctrl.pitch.core.kp, ctrl.pitch.core.ki, ctrl.pitch.core.kd,
//                       ctrl.roll.core.kp, ctrl.roll.core.ki, ctrl.roll.core.kd,
//                       ctrl.yaw.core.kp, ctrl.yaw.core.ki, ctrl.yaw.core.kd);
    if(Flag_ANO_DT_Send_Status)
    {
        ANO_DT_Send_Status(IMU.Roll, -IMU.Pitch, IMU.Yaw, 0, 0, 0);
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
                    //printf("RC_Data.THROTTLE=%d\r\n", RC_Data.THROTTLE);
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


//uint16 US100_Alt_Temp=0,Alt_Last=0; 
//float US100_Alt_Last=0;
//extern float US100_Alt;
//unsigned char ultra_start_f;
//unsigned char  Ultrasonic_OK;


/*  TA1

*/

void TimerA1_Init(void)
{
    P2SEL |= BIT2 ;               //����Ϊ�ڶ����ܣ���Ϊ����Դ
    P2DIR &= ~BIT2;               //TA0.x����Ϊ����
    
    TA1CTL = TASSEL_2 + MC_2 + TACLR + TAIE + ID_2;  // SMCLK, Continuous up, clear TAR    
    TA1EX0|=TAIDEX_2;//3��Ƶ
    TA1CCTL1 = CAP + CM_1 + SCS + CCIE;     //����ģʽ��ͨ��B�������ز���ͬ�������ж�

}

#pragma vector=TIMER1_A1_VECTOR //CCR1~CCR4�ж�
__interrupt void TIMER1_A1_ISR(void)//�ⲿ��ƽ�����ж�
{
    switch(__even_in_range(TA1IV,14))//��switch���ʹ��,����ѡ��ż��
    {                                //������Ч�ʱȽϸߵĴ���
        case  0: break;                          // No interrupt
       
        case  2: 
//            
//            if(TA1CCTL1 & CM0)                        //����������
//            {                 
//                TA1CCTL1    = (TA1CCTL1&(~CM0))| CM1;    //��Ϊ�½��ز���:CM0����,CM1��һ
//                Rise[4]=TA1R;  
//            }
//
//            else if(TA1CCTL1 & CM1)                   //�����½���
//            {
//                TA1CCTL1 = (TA1CCTL1&(~CM1))| CM0;   //��Ϊ�����ز���CM1����,CM0��һ          
//                Drop[4]=TA1R;
//                if(Rise[4]>Drop[4])  
//				{
//				    US100_Alt_Temp = 65535-Rise[4] + Drop[4];
//				}
//				else 	     
//                {	
//    				US100_Alt_Temp = Drop[4] - Rise[4];
//	            }		
//                
//                if(US100_Alt_Temp>20000)
//				{
//				    US100_Alt_Temp=Alt_Last; 
//				}
//				else    			
//                {
//				    Alt_Last=US100_Alt_Temp; 
//                }    
//				
//				US100_Alt = US100_Alt_Temp * 1.7 / 10;//����ֻ�ǻ�ø߶ȣ�����������,�Ż�ʱ���Լ�����ǲ���
//                printf("US100_Alt=%f\r\n", US100_Alt);
//				ultra_start_f=1; 
//
//				//�����Ƿ�ֹ�쳣���������ʱ��������Ϊ������������෶Χ�����ݻ�ͻȻ��ɺ�С������11.084�����������ֲ���
//				if(absu16(US100_Alt-US100_Alt_Last)>=500)
//				{
//					US100_Alt=US100_Alt_Last;
//					Ultrasonic_OK=0;
//				}
//				else 
//				{
//				    Ultrasonic_OK=1;
//				}
//				
//			    US100_Alt_delta=US100_Alt-US100_Alt_Last;
//				US100_Alt_Last=US100_Alt; 
//            }
//            
            break;
        case  4:
             break;
        
        default: break; 
    }
}