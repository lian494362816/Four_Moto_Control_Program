#ifndef __Algorithm_filter_H
#define  __Algorithm_filter_H

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);
float Moving_Median(unsigned char item,unsigned char width_num,float in);
void Moving_Average(float in,float moavarray[],uint16 len ,uint16 fil_cnt[2],float *out);
#endif