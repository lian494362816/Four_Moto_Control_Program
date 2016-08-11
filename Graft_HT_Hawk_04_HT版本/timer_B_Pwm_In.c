#include "include.h"

/*
    P4.1    TB1  --> YAW
    P4.2    TB2  --> THROTTLR
    P4.3    TB3  --> PIT
    P4.4    TB4  --> ROL

    频率为 1MHz

    YAW        取值范围 936~2085  中间值1518 操作THR对YAW无影响 
    THROTTLR   取值范围 936~2083   不好计算
    PIT        取值范围 936~2087  中间值1553 操作ROL对PIT的影响  1520 ~ 1650
    ROL        取值范围 930~2078  中间值1536 操作PIT对ROL的影响  1420 ~ 1570

    理论中间值为1510 
    实际捕获中间值 为 1518
*/
uint16 Rise[4], Drop[4], RC_Pwm_In[4];
RC_GETDATA RC_Data;

void Timer_B_Pwm_In_Init(void)
{
    P4SEL |= BIT1 + BIT2 + BIT3 + BIT4;//设置第二功能
    P4DIR |= ~(BIT1 + BIT2 + BIT3 + BIT4);//输入
    
    TB0CTL |= TBSSEL_2 + ID_2 + MC_2 + TBCLR + TBIE;//SMCLK = 12M 4分频  连续模式 清零TBxR 开中断
    TB0EX0 |= TBIDEX_2;//3分频 
    
    TB0CCTL1 |= CM_1 + SCS + CAP + CCIE + CCIS_1;//上升沿捕获  异步捕获 捕获模式 开中断
    TB0CCTL2 |= CM_1 + SCS + CAP + CCIE + CCIS_1;
    TB0CCTL3 |= CM_1 + SCS + CAP + CCIE + CCIS_1;
    TB0CCTL4 |= CM_1 + SCS + CAP + CCIE + CCIS_1;
}

#pragma vector = TIMER0_B1_VECTOR
__interrupt void TIMER0_B_Handle(void)
{
    switch(__even_in_range(TB0IV, 14))
    {
        case 0:
            break;  
        
        case 2:          
            if(TB0CCTL1 & CM0)
            {
                TB0CCTL1 = (TB0CCTL1 & (~CM0)) | CM1;//改为下降沿捕获
                Rise[0] = TB0R;
                
            }
            else if (TB0CCTL1 & CM1)
            {
                TB0CCTL1 = (TB0CCTL1 & (~CM1)) | CM0;//改为上升沿捕获
                Drop[0] = TB0R;              
                if(Rise[0] > Drop[0])
                {     
                    RC_Data.YAW = 65535 - Rise[0] + Drop[0];
                    //RC_Pwm_In[0] = 65535 - Rise[0] + Drop[0];
                    //TA0CCR1 = RC_Pwm_In[0];
                }
                else
                {
                    RC_Data.YAW = Drop[0] - Rise[0];
                    //RC_Pwm_In[0] = Drop[0] - Rise[0];
                    //printf("RC_Pwm_In[0]=%d\r\n", RC_Pwm_In[0]);
                    //TA0CCR1 = RC_Pwm_In[0];
                }
            }
            break;       
        
        case 4:          
            if(TB0CCTL2 & CM0)
            {
                TB0CCTL2 = (TB0CCTL2 & (~CM0)) | CM1;//改为下降沿捕获
                Rise[1] = TB0R; 
            }
            else if (TB0CCTL2 & CM1)
            {
                TB0CCTL2 = (TB0CCTL2 & (~CM1)) | CM0;//改为上升沿捕获
                Drop[1] = TB0R;              
                if(Rise[1] > Drop[1])
                {   
                    RC_Data.THROTTLE = 65535 - Rise[1] + Drop[1];
                    //RC_Pwm_In[1] = 65535 - Rise[1] + Drop[1];
                }
                else
                {
                     RC_Data.THROTTLE = Drop[1] - Rise[1];
                    //RC_Pwm_In[1] = Drop[1] - Rise[1];
                    //printf("RC_Pwm_In[1]=%d\r\n", RC_Pwm_In[1]);
                    //printf("RC_Data.THROTTLE=%d\r\n", RC_Data.THROTTLE);
                }
            }
            break;    
        
        case 6:          
            if(TB0CCTL3 & CM0)
            {
                TB0CCTL3 = (TB0CCTL3 & (~CM0)) | CM1;//改为下降沿捕获
                Rise[2] = TB0R;
                
            }
            else if (TB0CCTL3 & CM1)
            {
                TB0CCTL3 = (TB0CCTL3 & (~CM1)) | CM0;//改为上升沿捕获
                Drop[2] = TB0R;              
                if(Rise[2] > Drop[2])
                {
                    RC_Data.PITCH = 65535 - Rise[2] + Drop[2];
                    //RC_Pwm_In[2] = 65535 - Rise[2] + Drop[2];
                }
                else
                {
                    RC_Data.PITCH = Drop[2] - Rise[2];
                    //RC_Pwm_In[2] = Drop[2] - Rise[2];
                    //printf("P=%d\r\n", RC_Pwm_In[2]);
                }
            }
            break;
        
        case 8:          
            if(TB0CCTL4 & CM0)
            {
                TB0CCTL4 = (TB0CCTL4 & (~CM0)) | CM1;//改为下降沿捕获
                Rise[3] = TB0R;
                
            }
            else if (TB0CCTL4 & CM1)
            {
                TB0CCTL4 = (TB0CCTL4 & (~CM1)) | CM0;//改为上升沿捕获
                Drop[3] = TB0R;              
                if(Rise[3] > Drop[3])
                {      
                    RC_Data.ROLL = 65535 - Rise[3] + Drop[3];
                    //RC_Pwm_In[3] = 65535 - Rise[3] + Drop[3];
                }
                else
                {
                    RC_Data.ROLL = Drop[3] - Rise[3];
                    //RC_Pwm_In[3] = Drop[3] - Rise[3];
                    //printf("R=%d\r\n", RC_Pwm_In[3]);
                }
            }
            break;
        
        default:
            break;
    }
}


