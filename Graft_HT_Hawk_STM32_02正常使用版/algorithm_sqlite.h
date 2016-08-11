#ifndef __Algorithm_Sqlite_H
#define __Algorithm_Sqlite_H


void paramLoad(void);
struct _PID{
    float P;
    float I;
    float D;
    float pout;
    float dout;
    float out;
};
#endif