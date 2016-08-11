#include "include.h"


/*====================================================================================================*/
/*====================================================================================================*
** 函数名称: IIR_I_Filter
** 功能描述: IIR直接I型滤波器
** 输    入: InData 为当前数据
**           *x     储存未滤波的数据
**           *y     储存滤波后的数据
**           *b     储存系数b
**           *a     储存系数a
**           nb     数组*b的长度
**           na     数组*a的长度
**           LpfFactor
** 输    出: OutData         
** 说    明: 无
** 函数原型: y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
                    a1*y(n-1) - a2*y(n-2)
**====================================================================================================*/
/*====================================================================================================*/
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
    double z1,z2;
    short i;
    double OutData;
    
    for(i=nb-1; i>0; i--)
    {
        x[i]=x[i-1];
    }
    
    x[0] = InData;
    
    for(z1=0,i=0; i<nb; i++)
    {
        z1 += x[i]*b[i];
    }
    
    for(i=na-1; i>0; i--)
    {
        y[i]=y[i-1];
    }
    
    for(z2=0,i=1; i<na; i++)
    {
        z2 += y[i]*a[i];
    }
    
    y[0] = z1 - z2; 
    OutData = y[0];
    
    return OutData;
}


/*====================================================================================================*/
/*====================================================================================================*
**函数 : LPF_1st
**功能 : 一阶低通滤波
**输入 :  
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}
// #define WIDTH_NUM 101
// #define FIL_ITEM  10

void Moving_Average(float in,float moavarray[],uint16 len, uint16 fil_cnt[2],float *out)
{
	uint16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}

#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  4

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM ];
float med_filter_out[MED_FIL_ITEM];

unsigned char med_fil_cnt[MED_FIL_ITEM];


float Moving_Median(unsigned char item,unsigned char width_num,float in)
{
	unsigned char i,j;
	float t;
	float tmp[MED_WIDTH_NUM];
	
	if(item >= MED_FIL_ITEM || width_num >= MED_WIDTH_NUM )
	{
		return 0;
	}
	else
	{
		if( ++med_fil_cnt[item] >= width_num )	
		{
			med_fil_cnt[item] = 0;
		}
		
		med_filter_tmp[item][ med_fil_cnt[item] ] = in;
		
		for(i=0;i<width_num;i++)
		{
			tmp[i] = med_filter_tmp[item][i];
		}
		
		for(i=0;i<width_num-1;i++)
		{
			for(j=0;j<(width_num-1-i);j++)
			{
				if(tmp[j] > tmp[j+1])
				{
					t = tmp[j];
					tmp[j] = tmp[j+1];
					tmp[j+1] = t;
				}
			}
		}		
		return ( tmp[(uint16)width_num/2] );
	}
}

