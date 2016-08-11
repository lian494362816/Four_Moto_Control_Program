#include "include.h"
#include <stdio.h>

#include "delay.h"
#include "srom.h"
//#define H  400  //高度400毫米
int SumX, SumY, dx, dy;


void SPI_init(void)//改变速度
{   
    P6DIR |= BIT1+BIT2+BIT3;//P6.1 Cs  P6.2 Rest  P6.3  NPD
    P3SEL |= BIT1+BIT2+BIT3;//P3.1 SIMO  P3.2 SOMI P3.3 CLK(设置为SPI通信
    P3DIR |= BIT1+BIT3;
    P3DIR &= ~BIT2;
    UCB0CTL1 |= UCSWRST;                      // 软件复位SPI模块
    UCB0CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;    // 工作模式 ：三线SPI 8位数据SPI主机 不活动状态为高电平（CKPL=1)，高位在前
//    UCB0CTL0 &= ~UCCKPH;
    UCB0CTL1 |= UCSSEL_2;//时钟选择SMCLK
    UCB0BR0 = 0x2c; UCB0BR1 = 0;//分频系数为44
//    UCB0BR0 = 0x20; UCB0BR1 = 0;//分频系数为32
 //   UCB0MCTL=0;//无需调制          
    UCB0CTL1 &= ~UCSWRST;                     // 完成寄存器配置 
   // UCA1IE |= UCRXIE;                        // Enable USCI_A0 RX TX interrupt
}


uint SPI_SendReceive(uint data)  //SPI的收发
{ 
//    uint temp;
    uint retry=0;
    while(!(UCB0IFG&UCTXIFG))//等待发送寄存器为空
    {
        retry++;
        if(retry>200)
            return 0;
    }			        
    UCB0TXBUF=data;//发送数据
    retry=0;
    while(!(UCB0IFG&UCRXIFG))//等待接受寄存器接收到一个完整的信号
    {
        retry++;
	if(retry>200)
	return 0;
    }
//    temp=UCB0RXBUF;//把接收到数据存起来
    return UCB0RXBUF; 
}
uint read_register(uint adress)//读寄存器
{
    uint value;
    Cs_0;
    value=SPI_SendReceive(adress+0x00);//第一位为0为读
    Delay_us(75);
    value=SPI_SendReceive(0xff);
    Cs_1;
    return value; 
}
void write_register(uint adress,uint vlue)//写寄存器
{
    Cs_0;
    SPI_SendReceive(adress+0x80);//首位为1为写
    SPI_SendReceive(vlue);
    Cs_1;
    Delay_us(51);  
}
void ADNS3080_reset(void)  //ADNS3080复位(高)
{
    Reset_0;
    Delay_ms(5);
    Reset_1;
    Delay_ms(5);
    Reset_0;//脉冲信号
}
void Write_srom(void)//写SROM，数据手册19页
{
    int i;
    Cs_0;
    write_register(0x20,0x44);
    Delay_us(51);
    write_register(0x23,0x07);
    Delay_us(51);
    write_register(0x24,0x88);
    Delay_us(51);
    Cs_1;//  突发，写模式
    Delay_us(340);//等待大于一帧时间
    Cs_0;
    write_register(SROM_Enable,0x18);
    Cs_1;//突发，写模式
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
uint read_busy(void)//写帧率的判别，==1忙
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
    SPI_SendReceive(0xff);	//清除X,Y数据
    Cs_1;
}

void ADNS_Configuration(void)//ADNS配置
{
    Cs_0;
    write_register(Configuration_bits,0x10);		//寄存器为0X0A，设置分辨率为1600
    Delay_ms(3);
    write_register(Extended_Config,0x01);//寄存器为0X0B,帧率为确定的，由rame_Period_Max_Bound registers决定
    Delay_ms(3);
    if(read_busy()!=1)
    {  							      //设为3000帧每秒
        Cs_1;//突发，写模式
        Delay_ms(2);
        Cs_0;
        SPI_SendReceive(Frame_Period_Max_Bound_Lower+0x80);	//设置帧率，先写低位，再写高位
        SPI_SendReceive(0x40); //   C0 5000帧率	   
        SPI_SendReceive(Frame_Period_Max_Bound_Upper+0x80);
        SPI_SendReceive(0x1f);	 // 12
    } 
    clear_motion();
    Cs_1;
}


void ADNS3080_Init(void)//ADNS30803初始化
{
    //  SPI_Speed(256);//改变速度（2到256分频
    ADNS3080_reset(); //复位
    NPD_1;//拉高NPD，免睡眠
    Delay_ms(10);
    Write_srom();
    ADNS_Configuration();
    //  printf("%d\n",read_register(0x1f));	//查看是否下载成功
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
        //x的2补码转换	
        x -= 1;
        x = ~x;	
        x=(-1)*x;
        x-=256;
    }
	if(y&0x80)
    {
        //y的2补码转换	
        y -= 1;
        y = ~y;	
        y=(-1)*y;
        y-=256;
    } 
	SumX=SumX+x;             //累加x的读入数据
	SumY=SumY+y;		//累加y的读入数据
	Cs_1;
	Delay_us(4);
	Cs_1;
    //        sum_x=(25.4*(float)SumX *H)/(12*1600);//距离=d_x*(25.4/1600)*n
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
        //x的2补码转换	
        x -= 1;
        x = ~x;	
        x = (-1)*x;
        x -= 256;
    }
	if(y & 0x80)
    {
        //y的2补码转换	
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
        
        SumX+=dx;             //累加X读入的移动数据
		SumY+=dy;			 //累加Y读入的移动数据
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