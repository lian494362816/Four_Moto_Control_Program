#ifndef __Algorithm_filter_H
#define  __Algorithm_filter_H

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
fp32 LPF_1st(fp32 oldData, fp32 newData, fp32 lpf_factor);

#endif