#include "include.h"
#include <stdio.h>

#include "delay.h"
#include "srom.h"
//#define H  400  //�߶�400����
int SumX, SumY, dx, dy;


void SPI_init(void)//�ı��ٶ�
{   
    P6DIR |= BIT1+BIT2+BIT3;//P6.1 Cs  P6.2 Rest  P6.3  NPD
    P3SEL |= BIT1+BIT2+BIT3;//P3.1 SIMO  P3.2 SOMI P3.3 CLK(����ΪSPIͨ��
    P3DIR |= BIT1+BIT3;
    P3DIR &= ~BIT2;
    UCB0CTL1 |= UCSWRST;                      // �����λSPIģ��
    UCB0CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;    // ����ģʽ ������SPI 8λ����SPI���� ���״̬Ϊ�ߵ�ƽ��CKPL=1)����λ��ǰ
//    UCB0CTL0 &= ~UCCKPH;
    UCB0CTL1 |= UCSSEL_2;//ʱ��ѡ��SMCLK
    UCB0BR0 = 0x2c; UCB0BR1 = 0;//��Ƶϵ��Ϊ44
//    UCB0BR0 = 0x20; UCB0BR1 = 0;//��Ƶϵ��Ϊ32
 //   UCB0MCTL=0;//�������          
    UCB0CTL1 &= ~UCSWRST;                     // ��ɼĴ������� 
   // UCA1IE |= UCRXIE;                        // Enable USCI_A0 RX TX interrupt
}


uint SPI_SendReceive(uint data)  //SPI���շ�
{ 
//    uint temp;
    uint retry=0;
    while(!(UCB0IFG&UCTXIFG))//�ȴ����ͼĴ���Ϊ��
    {
        retry++;
        if(retry>200)
            return 0;
    }			        
    UCB0TXBUF=data;//��������
    retry=0;
    while(!(UCB0IFG&UCRXIFG))//�ȴ����ܼĴ������յ�һ���������ź�
    {
        retry++;
	if(retry>200)
	return 0;
    }
//    temp=UCB0RXBUF;//�ѽ��յ����ݴ�����
    return UCB0RXBUF; 
}
uint read_register(uint adress)//���Ĵ���
{
    uint value;
    Cs_0;
    value=SPI_SendReceive(adress+0x00);//��һλΪ0Ϊ��
    Delay_us(75);
    value=SPI_SendReceive(0xff);
    Cs_1;
    return value; 
}
void write_register(uint adress,uint vlue)//д�Ĵ���
{
    Cs_0;
    SPI_SendReceive(adress+0x80);//��λΪ1Ϊд
    SPI_SendReceive(vlue);
    Cs_1;
    Delay_us(51);  
}
void ADNS3080_reset(void)  //ADNS3080��λ(��)
{
    Reset_0;
    Delay_ms(5);
    Reset_1;
    Delay_ms(5);
    Reset_0;//�����ź�
}
void Write_srom(void)//дSROM�������ֲ�19ҳ
{
    int i;
    Cs_0;
    write_register(0x20,0x44);
    Delay_us(51);
    write_register(0x23,0x07);
    Delay_us(51);
    write_register(0x24,0x88);
    Delay_us(51);
    Cs_1;//  ͻ����дģʽ
    Delay_us(340);//�ȴ�����һ֡ʱ��
    Cs_0;
    write_register(SROM_Enable,0x18);
    Cs_1;//ͻ����дģʽ
    Delay_us(41);//>40us
    Cs_0;
    for(i=0;i<=1985;i++)
    {
        write_register(0x60,SROM[i]);
        Delay_us(11);// >10us
    }
    Cs_1;
    Delay_us(105);	//>104us 
}
uint read_busy(void)//д֡�ʵ��б�==1æ
{
    int temp;
    Cs_0;
    temp=SPI_SendReceive(Extended_Config+0x00);
    Delay_us(75);
    temp=SPI_SendReceive(0xff);
    temp&=0x80;
    Cs_1;
    return temp;
}
void clear_motion(void)
{
    Cs_0;
    SPI_SendReceive(Motion_Clear+0x80);
    SPI_SendReceive(0xff);	//���X,Y����
    Cs_1;
}

void ADNS_Configuration(void)//ADNS����
{
    Cs_0;
    write_register(Configuration_bits,0x10);		//�Ĵ���Ϊ0X0A�����÷ֱ���Ϊ1600
    Delay_ms(3);
    write_register(Extended_Config,0x01);//�Ĵ���Ϊ0X0B,֡��Ϊȷ���ģ���rame_Period_Max_Bound registers����
    Delay_ms(3);
    if(read_busy()!=1)
    {  							      //��Ϊ3000֡ÿ��
        Cs_1;//ͻ����дģʽ
        Delay_ms(2);
        Cs_0;
        SPI_SendReceive(Frame_Period_Max_Bound_Lower+0x80);	//����֡�ʣ���д��λ����д��λ
        SPI_SendReceive(0x40); //   C0 5000֡��	   
        SPI_SendReceive(Frame_Period_Max_Bound_Upper+0x80);
        SPI_SendReceive(0x1f);	 // 12
    } 
    clear_motion();
    Cs_1;
}


void ADNS3080_Init(void)//ADNS30803��ʼ��
{
    //  SPI_Speed(256);//�ı��ٶȣ�2��256��Ƶ
    ADNS3080_reset(); //��λ
    NPD_1;//����NPD����˯��
    Delay_ms(10);
    Write_srom();
    ADNS_Configuration();
    //  printf("%d\n",read_register(0x1f));	//�鿴�Ƿ����سɹ�
}
void Read_Data_burst(void)
{
    static int SumX;
    static int SumY;
    //  float sum_x,sum_y;
    unsigned char move=0;
    int  x=0;
    int  y=0;
    Cs_0;
    SPI_SendReceive(0x50);   
    Delay_us(75);
    move=SPI_SendReceive(0xFF);             
    x=SPI_SendReceive(0xFF);
    y=SPI_SendReceive(0xFF);
	if(x&0x80)
    {
        //x��2����ת��	
        x -= 1;
        x = ~x;	
        x=(-1)*x;
        x-=256;
    }
	if(y&0x80)
    {
        //y��2����ת��	
        y -= 1;
        y = ~y;	
        y=(-1)*y;
        y-=256;
    } 
	SumX=SumX+x;             //�ۼ�x�Ķ�������
	SumY=SumY+y;		//�ۼ�y�Ķ�������
	Cs_1;
	Delay_us(4);
	Cs_1;
    //        sum_x=(25.4*(float)SumX *H)/(12*1600);//����=d_x*(25.4/1600)*n
    //        sum_y=(25.4*(float)SumY *H)/(12*1600);
    if((move&0x10)!=1)
	{
		
        if(move&0x80)
        {
            printf("%d,%d\n",SumX,SumY);
        }
		else
		{
			x=0;
			y=0;
		}
		
	}
	else
	{
        x=0;
        y=0;
	}
}

void ADNS3080_Read_Position(void)
{
    unsigned char move = 0;
    int x = 0;
    int y = 0;
    Cs_0;
    SPI_SendReceive(0x50);   
    Delay_us(75);
    move=SPI_SendReceive(0xFF);             
    x=SPI_SendReceive(0xFF);
    y=SPI_SendReceive(0xFF);
    if(x & 0x80)
    {
        //x��2����ת��	
        x -= 1;
        x = ~x;	
        x = (-1)*x;
        x -= 256;
    }
	if(y & 0x80)
    {
        //y��2����ת��	
        y -= 1;
        y = ~y;	
        y = (-1)*y;
        y -=256;
    } 
    
    Cs_1;
	Delay_us(4);
	Cs_1;
    
    if(move & 0x80)
	{
		dx=x;
		dy=y;
        
        SumX+=dx;             //�ۼ�X������ƶ�����
		SumY+=dy;			 //�ۼ�Y������ƶ�����
//        printf("SumX=%d, SumY=%d\r\n", SumX, SumY);
	}
    else
    {
        x=0;
        y=0;
    }
}

struct ADNS_PID PID_ADNS3080;
float ADNS_temp1, ADNS_thr_last, ADNS_thr_temp1, ADNS_thr_temp, ADNS_THR_Lock;

void ADNS3080_CONTROL(void)
{
    PID_ADNS3080.pxout = -(PID_ADNS3080.P) * (SumX / 100);
    PID_ADNS3080.pyout = -(PID_ADNS3080.P) * (SumY / 100);
    PID_ADNS3080.dxout = -(PID_ADNS3080.D) * (dx / 100);
    PID_ADNS3080.dyout = -(PID_ADNS3080.D) * (dy / 100);
    
    PID_ADNS3080.Xout = PID_ADNS3080.pxout + PID_ADNS3080.dxout;
    PID_ADNS3080.Yout = PID_ADNS3080.pyout + PID_ADNS3080.dyout;
//    printf("Xout=%f, Yout=%f\r\n", PID_ADNS3080.Xout, PID_ADNS3080.Yout);
    if(PID_ADNS3080.Xout > 60)
    {
        PID_ADNS3080.Xout = 60;
    }
    
    if(PID_ADNS3080.Xout < - 60)
    {
        PID_ADNS3080.Xout = -60;
    }
    
    if(PID_ADNS3080.Yout > 60)
    {
        PID_ADNS3080.Yout = 60;
    }
    
    if(PID_ADNS3080.Yout < - 60)
    {
        PID_ADNS3080.Yout = -60;
    }
}