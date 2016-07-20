#include "r_cg_macrodriver.h"
#include "raspberry_pi.h"
#include "r_cg_serial.h"
#include "system.h"
#include "delay.h"

pi_frame urxdata;
const int len=13;
const uint8_t flag[2]={0x00,0x22};
uint8_t sign_buf=0xff;
remotedata command_buf;
float hight_buf;
extern bool sendwait1,receivewait1;
extern ppid angle_parameter_r,angle_parameter_z,hight_parameter;
extern int mot_min_h;

void function(void)
{
	if(0==receive())
	{
		translate();
	}
	else
	{
		delay_ms(15);
	}
}
int receive(void)
{
	char sum=0,*check;
	
	R_UART1_Start();
	while(true)
	{
		urxdata.head[0]=urxdata.head[1];
		receivewait1=true;
		R_UART1_Receive((uint8_t *)&urxdata.head[1],1);
		while(receivewait1){NOP();}
		if(urxdata.head[0]==-1&&urxdata.head[1]==0)break;
	}
	receivewait1=true;
	R_UART1_Receive((uint8_t *)&urxdata.flag,len-2);
	while(receivewait1){NOP();}
	for(check=&urxdata.flag;check<&urxdata.end;check++)sum=(char)(sum+(*check));
	if(sum==urxdata.end)
	{
		sendwait1=true;
		R_UART1_Send((uint8_t *)&urxdata,len);
		while(sendwait1){NOP();}
		R_UART1_Stop();
		return 0;
	}
	else
	{
		return -1;
	}
}

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

remotedata returncommand(void)
{
	return command_buf;
}