#ifndef _ATTITUDE_CONTROL_
#define _ATTITUDE_CONTROL_

#include "r_cg_userdefine.h"

//extern motor mot_ang,motout_ang;
//extern angle angle0;
//extern float opr,opz,ipr,ipz,iir,iiz,idr,idz,inaim0x,inaim0y,inaim0z,integralx,integraly,integralz;//o:Õ‚ª∑ p:P r£∫X.Y
//extern int motrlim,limr,limz;

int duallooppid(ppid par,float *delta0,float *data0,float data,float aim,float *integral);
void cal_mot_all(void);
void limitang(motor *mot_in,int motrlim);
motor *getmot(void);

#endif