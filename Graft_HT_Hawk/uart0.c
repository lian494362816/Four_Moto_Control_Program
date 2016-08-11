#include "include.h"
#include <msp430f5438A.h>
#include <stdio.h>
unsigned char float_count_i = 0, float_count_j = 0;
unsigned char buffer[10];//�洢��ĸ
unsigned char num_buffer[10];//�洢���յ�ASC��
unsigned long int float_num1 ;//С���������Ĺ��̱���ֵ
float float_num2;//ʵ��ʹ�õ�С��ֵ
//float float_buffer[write_flash_count];//����д��flash�����ݴ滺����
extern struct _ctrl ctrl;
extern float write_flash_buffer[write_flash_count];


uchar ROL_SP[Compare_Long] = {"ROLSP"};    //ROL �Ƚ��ַ��� �봮��������бȽ�
uchar ROL_SI[Compare_Long] = {"ROLSI"};
uchar ROL_CP[Compare_Long] = {"ROLCP"};
uchar ROL_CI[Compare_Long] = {"ROLCI"};
uchar ROL_CD[Compare_Long] = {"ROLCD"};

uchar PIT_SP[Compare_Long] = {"PITSP"};    //PIT �Ƚ��ַ��� �봮��������бȽ�
uchar PIT_SI[Compare_Long] = {"PITSI"};
uchar PIT_CP[Compare_Long] = {"PITCP"};
uchar PIT_CI[Compare_Long] = {"PITCI"};
uchar PIT_CD[Compare_Long] = {"PITCD"};

uchar YAW_SP[Compare_Long] = {"YAWSP"};    //YAW �Ƚ��ַ��� �봮��������бȽ�
uchar YAW_SI[Compare_Long] = {"YAWSI"};
uchar YAW_CP[Compare_Long] = {"YAWCP"};
uchar YAW_CI[Compare_Long] = {"YAWCI"};
uchar YAW_CD[Compare_Long] = {"YAWCD"};



_Flag Flag;
/*
    ����0  115200
    P3.4 Tx
    P3.5 Rx
*/


void Uart0_Init(void)
{
    P3SEL |= BIT4 + BIT5;
    UCA0CTL1 |= UCSWRST;//�����λ
    UCA0CTL1 |= UCSSEL_2; //SMCLK = 12M
    UCA0BR0 = 0x68;
    UCA0BR1 = 0x00;
    UCA0MCTL |= UCBRF_0 + UCBRS_1;
    UCA0CTL1 &= ~UCSWRST;//�����λ �ر�
    UCA0IE |= UCRXIE;//�����ж�
}

/* 
    UART0 �����жϴ���
    
*/
#pragma vector = USCI_A0_VECTOR
__interrupt void UART0_RX_Handle(void)
{
    switch(__even_in_range(UCA0IV, 4))
    {
        case 0:
            break;
        case 2:
            while(!(UCA0IFG & UCTXIFG));
            //printf("EP: %d, ID: %d",Flag.EP, Flag.ID);
            
            if(float_count_i < Compare_Long)//ǰ��������ĸ�� buffer �洢
            {
                buffer[float_count_i] = UCA0RXBUF;
                if(buffer[0] == 'S')//�յ�S ��ʾ��ǰ������ֵ
                {
                    printf("\n \n");

                    printf("\r\n PIT ����\r\n"); //�����ǰPIT ��������
                    printf("ctrl.pitch.shell.kp=%f\r\n", ctrl.pitch.shell.kp);
                    printf("ctrl.pitch.shell.ki=%f\r\n", ctrl.pitch.shell.ki);
                    printf("ctrl.pitch.core.kp=%f\r\n", ctrl.pitch.core.kp);
                    printf("ctrl.pitch.core.ki=%f\r\n", ctrl.pitch.core.ki);
                    printf("ctrl.pitch.core.kd=%f\r\n", ctrl.pitch.core.kd);
                    
                    printf("\r\n ROL ����\r\n");//�����ǰROL �����в���
                    printf("ctrl.roll.shell.kp=%f\r\n", ctrl.roll.shell.kp);
                    printf("ctrl.roll.shell.ki=%f\r\n", ctrl.roll.shell.ki);
                    printf("ctrl.roll.core.kp=%f\r\n", ctrl.roll.core.kp);
                    printf("ctrl.roll.core.ki=%f\r\n", ctrl.roll.core.ki);
                    printf("ctrl.roll.creo.kd=%f\r\n", ctrl.roll.core.kd);
                    
                    printf("\r\n YAW ����\r\n");//�����ǰYAW �����в���
                    printf("ctrl.yaw.shell.kp=%f\r\n", ctrl.yaw.shell.kp);
                    printf("ctrl.yaw.shell.ki=%f\r\n", ctrl.yaw.shell.ki);
                    printf("ctrl.yaw.core.kp=%f\r\n", ctrl.yaw.core.kp);
                    printf("ctrl.yaw.core.ki=%f\r\n", ctrl.yaw.core.ki);
                    printf("ctrl.yaw.core.kd=%f\r\n", ctrl.yaw.core.kd);
                    printf("�յ�Sָ��\r\n");
                    float_count_i = 0;
                    return ;
                }
                float_count_i ++;
                if( 5 == float_count_i)
                {                  
                   Compare_All();
                }
            }
            else//����5�������� num_buffer �洢
            {
                num_buffer[float_count_j++] =  UCA0RXBUF;
            }
            if(5 == float_count_j)
            {
                float_count_j = 0;
                //Flag.command = 1;//��ֵ�����־λ
                for(float_count_i = 0;float_count_i < 5; float_count_i++)//���յ�����ȥ 48 ��ASC���ɶ�Ӧ����ֵ
                {
                    num_buffer[float_count_i] = num_buffer[float_count_i] - 48;
                }
                //��С��
                float_num1 = num_buffer[0] * 10000 + num_buffer[1] * 1000 + num_buffer[2] * 100 + num_buffer[3] * 10 + num_buffer[4];
                float_num2 = float_num1 / 1000.0;           
                float_count_i = 0;
                Change();
            }          
            break;
            
        case 4:
            break;
            
        default:
            break;
    }
}

/* 
    �ض���C����
*/
int putchar(int ch)
{
    while(!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = (char) ch;
    return ch;
}


/* 
    ���ܣ��ַ��Ƚ�
    
*/
uchar Compare(uchar *buffer1, uchar *buffer2)
{
    for(uchar n =0; n < Compare_Long; n ++)
    {
        if(buffer1[n] != buffer2[n])
        {
            return 0;
        }
    }         
    return 1;
}

/* 
    ���ڱȽ����е��ַ���ȷ��Ҫ�޸ĵĲ���

*/

void Compare_All(void)
{                                 
    Flag.ROL_SP = Compare(buffer, ROL_SP);//�Ƚ�roll
    Flag.ROL_SI = Compare(buffer, ROL_SI);
    Flag.ROL_CP = Compare(buffer, ROL_CP);
    Flag.ROL_CI = Compare(buffer, ROL_CI);
    Flag.ROL_CD = Compare(buffer, ROL_CD);
    
    Flag.PIT_SP = Compare(buffer, PIT_SP);//�Ƚ�pitch
    Flag.PIT_SI = Compare(buffer, PIT_SI);
    Flag.PIT_CP = Compare(buffer, PIT_CP);
    Flag.PIT_CI = Compare(buffer, PIT_CI);
    Flag.PIT_CD = Compare(buffer, PIT_CD);
    
    Flag.YAW_SP = Compare(buffer, YAW_SP);//�Ƚ�yaw
    Flag.YAW_SI = Compare(buffer, YAW_SI);
    Flag.YAW_CP = Compare(buffer, YAW_CP);
    Flag.YAW_CI = Compare(buffer, YAW_CI);
    Flag.YAW_CD = Compare(buffer, YAW_CD);
    
    if(Flag.ROL_SP | Flag.ROL_SI | Flag.ROL_CP | Flag.ROL_CI | Flag.ROL_CD)
    {
        Flag.command = 1;
    }
    
    if(Flag.PIT_SP | Flag.PIT_SI | Flag.PIT_CP | Flag.PIT_CI | Flag.PIT_CD)
    {
        Flag.command  = 1;
    }
    
    if(Flag.YAW_SP | Flag.YAW_SI | Flag.YAW_CP | Flag.YAW_CI | Flag.YAW_CD)
    {
        Flag.command = 1;
    }
    
    if(Flag.command != 1)
    {
        printf("������Ч\r\n");
        float_count_i = 0;
        return ;
    }
    
    if(Flag.ROL_SP)//�ж�ROL ��־λ
    {
        printf("׼���޸�ctrl.roll.shell.kp\r\n");
    }
    
    if(Flag.ROL_SI)
    {
        printf("׼���޸�ctrl.roll.shell.ki\r\n");
    }
    
    if(Flag.ROL_CP)
    {
        printf("׼���޸�ctrl.roll.core.kp\r\n");
    }
    
    if(Flag.ROL_CI)
    {
        printf("׼���޸�ctrl.roll.core.ki\r\n");    
    }
    
    if(Flag.ROL_CD)
    {
        printf("׼���޸�ctrl.roll.core.kd\r\n");
    }
    
    if(Flag.PIT_SP)//�ж�PIT��־λ
    {
        printf("׼���޸�ctrl.pitch.shell.kp\r\n");
    }
    
    if(Flag.PIT_SI)
    {
        printf("׼���޸�ctrl.pitch.shell.ki\r\n");
    }
    
    if(Flag.PIT_CP)
    {
        printf("׼���޸�ctrl.pitch.core.kp\r\n");
    }
    
    if(Flag.PIT_CI)
    {
        printf("׼���޸�ctrl.pitch.core.ki\r\n");
    }
    
    if(Flag.PIT_CD)
    {
        printf("׼���޸�ctrl.pitch.core.id\r\n");
    }
    
    if(Flag.YAW_SP)//�ж�YAW��־λ
    {
        printf("׼���޸�ctrl.yaw.shell.kp\r\n");
    }
    
    if(Flag.YAW_SI)
    {
        printf("׼���޸�ctrl.yaw.shell.ki\r\n");
    }
    
    if(Flag.YAW_CP)
    {
        printf("׼���޸�ctrl.yaw.core.kp\r\n");
    }
    
    if(Flag.YAW_CI)
    {
        printf("׼���޸�ctrl.yaw.core.ki\r\n");
    }
    
    if(Flag.YAW_CD)
    {
        printf("׼���޸�ctrl.yaw.core.kd\r\n");
    }
}

/* 
    �޸Ĳ�����ֵ
*/

void Change(void)
{
    if(Flag.command)
    {
        Flag.command = 0;
        
        if(Flag.ROL_SP)//�޸�roll��PID
        {
            Flag.ROL_SP = 0;
            write_flash_buffer[ROL_SP_num] = float_num2;
            float_write_flash();
            ctrl.roll.shell.kp = float_read_flash(ROL_SP_addr);
            printf("\r\n ��ǰctrl.roll.shell.kp=%f\r\n", ctrl.roll.shell.kp);
        }
        
        if(Flag.ROL_SI)
        {
            Flag.ROL_SI = 0;
            write_flash_buffer[ROL_SI_num] = float_num2;
            float_write_flash();
            ctrl.roll.shell.ki = float_read_flash(ROL_SI_addr);
            printf("\r\n ��ǰctrl.roll.shll.ki=%f\r\n", ctrl.roll.shell.ki);
        }
        
        if(Flag.ROL_CP)
        {
            Flag.ROL_CP = 0;
            write_flash_buffer[ROL_CP_num] = float_num2;
            float_write_flash();
            ctrl.roll.core.kp = float_read_flash(ROL_CP_addr);
            printf("\r\n ��ǰctrl.roll.core.kp=%f\r\n", ctrl.roll.core.kp);
        }
        
        if(Flag.ROL_CI)
        {
            Flag.ROL_CI = 0;
            write_flash_buffer[ROL_CI_num] = float_num2;
            float_write_flash();
            ctrl.roll.core.ki = float_read_flash(ROL_CI_addr);
            printf("\r\n ��ǰctrl.roll.core.ki=%f\r\n", ctrl.roll.core.ki);
        }
        
        if(Flag.ROL_CD)
        {
            Flag.ROL_CD = 0;
            write_flash_buffer[ROL_CD_num] = float_num2;
            float_write_flash();
            ctrl.roll.core.kd =float_read_flash(ROL_CD_addr);
            printf("\r\n ��ǰctrl.roll.core.kd=%f\r\n", ctrl.roll.core.kd);
        }   
        
        if(Flag.PIT_SP)//�޸�ptich��PID
        {
            Flag.PIT_SP = 0;
            write_flash_buffer[PIT_SP_num] = float_num2;
            float_write_flash();
            ctrl.pitch.shell.kp = float_read_flash(PIT_SP_addr);
            printf("\r\n ��ǰctrl.pitch.shell.kp=%f\r\n", ctrl.pitch.shell.kp);
        }
        
        if(Flag.PIT_SI)
        {
            Flag.PIT_SI = 0;
            write_flash_buffer[PIT_SI_num] = float_num2;
            float_write_flash();
            ctrl.pitch.shell.ki = float_read_flash(PIT_SI_addr);
            printf("\r\n ��ǰctrl.pitch.shell.ki=%f\r\n", ctrl.pitch.shell.ki);
        }
        
        if(Flag.PIT_CP)
        {
            Flag.PIT_CP = 0;
            write_flash_buffer[PIT_CP_num] = float_num2;
            float_write_flash();
            ctrl.pitch.core.kp = float_read_flash(PIT_CP_addr);
            printf("\r\n ��ǰctrl.pitch.core.kp=%f\r\n", ctrl.pitch.core.kp);            
        }
        
        if(Flag.PIT_CI)
        {
            Flag.PIT_CI = 0;
            write_flash_buffer[PIT_CI_num] = float_num2;
            float_write_flash();
            ctrl.pitch.core.ki = float_read_flash(PIT_CI_addr);
            printf("\r\n ��ǰctrl.pitch.core.ki=%f\r\n", ctrl.pitch.core.ki);        
        }
        
        if(Flag.PIT_CD)
        {
            Flag.PIT_CD = 0;
            write_flash_buffer[PIT_CD_num] = float_num2;
            float_write_flash();
            ctrl.pitch.core.kd = float_read_flash(PIT_CD_addr);
            printf("\r\n ��ǰctrl.pitch.core.kd=%f\r\n", ctrl.pitch.core.kd);
        }
        
        if(Flag.YAW_SP)
        {
            Flag.YAW_SP = 0;
            write_flash_buffer[YAW_SP_num] = float_num2;
            float_write_flash();
            ctrl.yaw.shell.kp = float_read_flash(YAW_SP_addr);
            printf("\r\n ��ǰctrl.yaw.shell.kp=%f\r\n", ctrl.yaw.shell.kp);
        }
        
        if(Flag.YAW_SI)
        {
            Flag.YAW_SI = 0;
            write_flash_buffer[YAW_SI_num] = float_num2;
            float_write_flash();
            ctrl.yaw.shell.ki = float_read_flash(YAW_SI_addr);
            printf("\r\n ��ǰctrl.yaw.shell.ki=%f\r\n", ctrl.yaw.shell.ki);
        }
        
        if(Flag.YAW_CP)
        {
            Flag.YAW_CP = 0;
            write_flash_buffer[YAW_CP_num] = float_num2;
            float_write_flash();
            ctrl.yaw.core.kp = float_read_flash(YAW_CP_addr);
            printf("\r\n ��ǰctrl.yaw.core.kp=%f\r\n", ctrl.yaw.core.kp);
        }
        
        if(Flag.YAW_CI)
        {
            Flag.YAW_CI = 0;
            write_flash_buffer[YAW_CI_num] = float_num2;
            float_write_flash();
            ctrl.yaw.core.ki = float_read_flash(YAW_CI_addr);
            printf("\r\n ��ǰctrl.yaw.core.ki=%f\r\n", ctrl.yaw.core.ki);
        }
        
        if(Flag.YAW_CD)
        {
            Flag.YAW_CD = 0;
            write_flash_buffer[YAW_CD_num] = float_num2;
            float_write_flash();
            ctrl.yaw.core.kd = float_read_flash(YAW_CD_addr);
            printf("\r\n ��ǰctrl.yaw.core.kd=%f\r\n", ctrl.yaw.core.kd);
        }
    }  
     
}








