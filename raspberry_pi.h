#ifndef _RASPBERRY_PI_
#define _RASPBERRY_PI_

#include "r_cg_userdefine.h"

//׼��������λ������
void receive_start(void);

//�����λ�����ݣ��ɹ�����0��ʧ�ܷ���-1����������θ�λ����
int receive_check(void);

//��λ�����ݽ���
void translate(void);

//�����������λ�������˲����ŷ��������
void report_angle(void)��

//�����������λ�����ͷ�����״̬
void report(void);

//����ָ������
remotedata returncommand(void);

#endif