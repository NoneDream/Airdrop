#include "r_cg_macrodriver.h"
#include "raspberry_pi.h"
#include "r_cg_serial.h"
#include "system.h"
#include "delay.h"

//�궨��ʵ�ָߵ�λ����
#define SWAP8(x) (((x&0xff00)>> 8)|((x&0xff)<<8))

pi_frame urxdata;
const int len=13;
const uint8_t flag[2]={0x00,0x22};
uint8_t sign_buf=0xff;
remotedata command_buf;
float hight_buf;
extern bool sendwait1,receivewait1;
extern ppid angle_parameter_r,angle_parameter_z,hight_parameter;
extern int mot_min_h;

//׼��������λ������
void receive_start(void)
{
	R_UART1_Start();
	receivewait1=true;
	R_UART1_Receive((uint8_t *)&urxdata.flag,len-2);
}

//�����λ�����ݣ��ɹ�����0��ʧ�ܷ���-1����������θ�λ����
int receive_check(void)
{
	unsigned char sum=0,*check;
	
	R_UART1_Create();//��λUART1
	
	if(receivewait1)return -1;//�����ճ���
	
	for(check=(unsigned char *)&urxdata.flag;check<(unsigned char *)&urxdata.end;check++)sum=(unsigned char)(sum+(*check));//����У���
	
	if(sum==urxdata.end)return 0;
	else return -1;
}

//��λ�����ݽ���
void translate(void)
{
	/*typedef struct
	{
		char head[2];//0xff,0x00
		char flag;//0x01:command; 0x02:parameter_r; 0x03:parameter_z; 0x04:parameter_h; 0x11:start; 0x22:stop;
		int16_t data1;//x; op;
		int16_t data2;//y; ip;
		int16_t data3;//vz; ii;
		int16_t data4;//h; id
		char end;
	}pi_frame;*/
	if(urxdata.flag==1)
	{
		command_buf.x=((float)urxdata.data1)/10.0;
		command_buf.y=((float)urxdata.data2)/10.0;
		command_buf.h=((float)urxdata.data4)/10.0;
		command_buf.vz=((float)urxdata.data3)/10.0;
	}
	else
	{
		switch(urxdata.flag)
		{
			case 34:
			{
				stop();
				break;
			}
			case 17:
			{
				start();
				break;
			}
			case 2:
			{
				angle_parameter_r.waihuan_p=((float)urxdata.data1)/100.0;
				angle_parameter_r.neihuan_p=((float)urxdata.data2)/100.0;
				angle_parameter_r.neihuan_i=((float)urxdata.data3)/100.0;
				angle_parameter_r.neihuan_d=((float)urxdata.data4)/100.0;
				break;
			}
			case 3:
			{
				angle_parameter_z.waihuan_p=((float)urxdata.data1)/100.0;
				angle_parameter_z.neihuan_p=((float)urxdata.data2)/100.0;
				angle_parameter_z.neihuan_i=((float)urxdata.data3)/100.0;
				angle_parameter_z.neihuan_d=((float)urxdata.data4)/100.0;
				break;
			}
			case 4:
			{
				hight_parameter.waihuan_p=((float)urxdata.data1)/100.0;
				hight_parameter.neihuan_p=((float)urxdata.data2)/100.0;
				hight_parameter.neihuan_i=((float)urxdata.data3)/100.0;
				hight_parameter.neihuan_d=((float)urxdata.data4)/100.0;
				break;
			}
			case 5:
			{
				if(mot_min_h<(MOT_MAX-2000))mot_min_h=mot_min_h+50;
				break;
			}
			case 6:
			{
				if(mot_min_h>(MOT_MIN-2000))mot_min_h=mot_min_h-50;
				break;
			}
		}
	}
}

//�����������λ�������˲����ŷ��������
void report_angle(void)
{
//����������
/*typedef struct
{
	char head[2];//�̶�0x5A,0x5A
	char type;//0x45��ʾŷ��������
	unsigned char len;//���ݳ��ȣ�ӦΪ6
	int16_t x;
	int16_t y;
	int16_t z;
	char checksum;//[����]��У���
}gy95_frame;*/
	gy95_frame report;
	unsigned char *check;
	extern angle angnow��//�˲�֮������
	
	report.head[0]=0x5a;
	report.head[1]=0x5a;
	report.type=0x45;
	report.len=6;
	report.x=(int16_t)angnow.x*100;
	report.y=(int16_t)angnow.y*100;
	report.z=(int16_t)angnow.z*100;
	report.x=SWAP8(report.x);
	report.y=SWAP8(report.y);
	report.z=SWAP8(report.z);
	
	report.checksum=0;
	for(check=(unsigned char *)report.head;check<(unsigned char *)&report.checksum;check++)
		report.checksum=(unsigned char)(report.checksum+(*check));//����У���
		
	sendwait1=true;
	R_UART1_Send((uint8_t *)&report,11);
	while(sendwait1){NOP();}
}

void report_pwm(void)
{
//��λ������
/*typedef struct
{
	char head[2];//0xff,0x00
	char flag;//0x01:command; 0x02:parameter_r; 0x03:parameter_z; 0x04:parameter_h; 0x05:pwm; 0x11:start; 0x22:stop;
	int16_t data1;//x; op;
	int16_t data2;//y; ip;
	int16_t data3;//vz; ii;
	int16_t data4;//h; id
	unsigned char end;
}pi_frame;*/

	pi_frame report;
	unsigned char *check;
	extern motor motout;
	
	report.head[0]=0xff;
	report.head[1]=0x00;
	report.flag=0x05;
	report.data1=motout.TDR1;
	report.data2=motout.TDR2;
	report.data3=motout.TDR3;
	report.data4=motout.TDR4;
	
	report.end=0;
	for(check=(unsigned char *)report.head;check<(unsigned char *)&report.end;check++)
		report.end=(unsigned char)(report.end+(*check));//����У���
		
	sendwait1=true;
	R_UART1_Send((uint8_t *)&report,12);
	while(sendwait1){NOP();}
	
}

//�����������λ�����ͷ�����״̬
void report(void)
{
	report_angle();//�����˲����ŷ��������
	report_pwm();//���͵������
}

//����ָ������
remotedata returncommand(void)
{
	return command_buf;
}