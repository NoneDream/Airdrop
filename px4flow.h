#ifndef _HIGHT_
#define _HIGHT_

#include "r_cg_userdefine.h"

//extern float hight0,oph,iph,iih,idh,inaim0h,integralh;
//extern int limh;
//extern int motout_hit;

//��������
void send_com(void);

//��ȡ����������
void get_data(void);

//�߶����ݽ������˲�����λ�����ף�
void hight_filter(void);

//����������ظ߶�
float return_hight(void);

#endif