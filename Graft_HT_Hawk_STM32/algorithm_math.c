#include "include.h"

/*====================================================================================================*/
/*====================================================================================================*
**���� : Q_rsqrt
**���� : ���ټ��� 1/Sqrt(x) 
**���� : number  
**��� : ���
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration ����һ��ţ�ٵ�����
	return y;
} 


/**************************ʵ�ֺ���********************************************
*����ԭ��:    rad(double angle)
*��������:    �Ƕ�ת��Ϊ����
���������    �Ƕ�
���������    ����
*******************************************************************************/
double Rad(double angle)
{
    return (angle * M_PI / 180.0);
}


/***********************************************
  * @brief  �ɱ���������Ӧ����
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
        error = -error;
	}
    if(error >0.6f)
	{
        error = 0.6f;
	}
	result = 1 - 1.667 * error;
	if(result < 0)
	{
        result = 0;
	}
	return result;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:    degree(double rad)
*��������:    ����ת��Ϊ�Ƕ�	
���������    ����
���������    �Ƕ�
*******************************************************************************/
double Degree(double rad)
{
    return (rad / M_PI * 180.0);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    data_limit(float data,flaot toplimit,float lowerlimit)
*��������:    �����޷�
���������    data       Ҫ���������� 
*             toplimit   ����
*             lowerlimit ����
���������    ��
*******************************************************************************/
float data_limit(float data,float toplimit,float lowerlimit)
{
  if(data > toplimit)  data = toplimit;
  else if(data < lowerlimit) data = lowerlimit;
	return data;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    array_assign(int16_t *array,int16_t value)
*��������:    �����鸳ֵ
���������    *array   Ŀ������ָ�� 
*             value      
���������    ��
*******************************************************************************/
void array_assign(int16 *array,uint16_t value,uint16 length)
{
   for(uint16_t i=0;i<length;i++)
   {
     *(array+i) = value;
   } 
}

/* �������Ǻ�����̩��չ��ʽ �����ֵ*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}

float my_deathzoom(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}

float my_deathzoom_2(float x,float zoom)
{
	float t;
	
	if( x> -zoom && x < zoom )
	{
		t = 0;
	}
	else
	{
		t = x;
	}

  return (t);
}


//����ƽ�����㷨
float my_sqrt(float number)
{
	long i;
	float x, y;
	const float f = 1.5F;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}