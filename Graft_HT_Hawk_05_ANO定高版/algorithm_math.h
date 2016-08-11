#ifndef __Algorithm_math_H
#define	__Algorithm_math_H


#define M_PI  (float)3.1415926535
#define squa( Sq )        (((float)Sq)*((float)Sq))
#define toRad( Math_D )	  ((float)(Math_D)*0.0174532925f)
#define toDeg( Math_R )	  ((float)(Math_R)*57.2957795f)
#define absu16( Math_X )  (Math_X<0? -(Math_X):Math_X)
#define absFloat( Math_X )(Math_X<0? -(Math_X):Math_X)
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

#define ABS(x) ( (x)>0?(x):-(x) )

float Q_rsqrt(float number);
double Rad(double angle);
float VariableParameter(float error);
double Degree(double rad);
float data_limit(float data,float toplimit,float lowerlimit);
void array_assign(int16 *array,uint16_t value,uint16 length);
float COS(float x);
float SIN(float y);
float my_deathzoom(float x,float zoom);
float my_deathzoom_2(float x,float zoom);
float my_sqrt(float number);
#endif