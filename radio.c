#include "r_cg_macrodriver.h"
#include "system.h"
#include "radio.h"
#include "r_cg_serial.h"
#include "GY95.h"
#include "remote_control.h"
#include "delay.h"

//UART1:
extern bool sendwait1,receivewait1;

//getcom:
bool head_or_data=true;//true:head  false:data
unsigned char data_buf[20];
int num;

/*void getcom(void)
{
	int i;
	unsigned char data_buf[],sum=0;
	R_UART1_Receive(radiobuf,num);
	for(i=0;i<(num-1);i++)sum+=data_buf[i];
	if(!(sum==data_buf[num-1]))return;		//判断sum
	if(!(data_buf[0]==0xAA&&data_buf[1]==0xAF))return;		//判断帧头
/////////////////////////////////////////////////////////////////////////////////////
	if(data_buf[2]==0X01)
	{
		if(data_buf[4]==0X01)
			MPU6050_CalOff_Acc();
		if(*(data_buf+4)==0X02)
			MPU6050_CalOff_Gyr();
		if(*(data_buf+4)==0X03)
		{MPU6050_CalOff_Acc();MPU6050_CalOff_Gyr();}
//		if(*(data_buf+4)==0X04)
//			Cal_Compass();
//		if(*(data_buf+4)==0X05)
//			MS5611_CalOffset();
	}
	if(data_buf[2]==0X02)
	{
		if(data_buf[4]==0X01)
		{
			Send_PID1 = 1;
			Send_PID2 = 1;
			Send_PID3 = 1;
			Send_PID4 = 1;
			Send_PID5 = 1;
			Send_PID6 = 1;
		}
		if(data_buf[4]==0X02)Send_Offset = 1;
	}
	if(data_buf[2]==0X10)								//PID1
	{
			PID_ROL.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_ROL.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			PID_ROL.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PIT.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PIT.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			PID_PIT.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_YAW.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_YAW.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_YAW.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
			PID_ALT.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_ALT.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_ALT.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_POS.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_POS.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_POS.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_PID_1.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_PID_1.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_PID_1.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
			PID_PID_2.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_PID_2.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_PID_2.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PID_3.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PID_3.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_PID_3.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_PID_4.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_PID_4.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_PID_4.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
			PID_PID_5.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_PID_5.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_PID_5.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PID_6.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PID_6.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_PID_6.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_PID_7.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_PID_7.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_PID_7.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
			PID_PID_8.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_PID_8.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_PID_8.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PID_9.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PID_9.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_PID_9.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			PID_PID_10.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			PID_PID_10.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
			PID_PID_10.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
			PID_PID_11.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			PID_PID_11.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			PID_PID_11.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			PID_PID_12.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			PID_PID_12.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
			PID_PID_12.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			Data_Send_Check(sum);
			EE_SAVE_PID();
	}
	if(*(data_buf+2)==0X16)								//OFFSET
	{
			AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
			AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
	}

}*/
void getcom(void)//上位机 -> 飞控
{
	int i;
	unsigned char sum=0;
	if(head_or_data)
	{
		if(!(data_buf[0]==0xAA&&data_buf[1]==0xAF))
		{
			R_UART1_Receive(data_buf,4);
			return;// 检查帧头
		}
		else if(data_buf[2]==0X01)
		{
			sensorinit();//陀螺仪校准
			R_UART1_Receive(data_buf,4);
		}
		else if(data_buf[2]==0X02)//要求返回PID值
		{
			send_PID1();
			send_PID3();
			R_UART1_Receive(data_buf,4);
		}
		else
		{
			head_or_data=false;
			data_buf[0]=data_buf[2];
			num=data_buf[3]-0x04;
			R_UART1_Receive(data_buf+1,num);
		}
		return;
	}
	
	sum=(unsigned char)(0xaa+0xaf+(unsigned char)(num+4));
	for(i=0;i<(num-1);i++)sum+=data_buf[i];
	if(!(sum==data_buf[num-1]))return;//校验和
	
	if(data_buf[0]==0X10)//PID1
	{
			iph = (float)((int)(*(data_buf+1)<<8)|*(data_buf+2))/100;
			iih = (float)((int)(*(data_buf+3)<<8)|*(data_buf+4))/1000;
			idh = (float)((int)(*(data_buf+5)<<8)|*(data_buf+6))/100;//h
			ipr = (float)((int)(*(data_buf+7)<<8)|*(data_buf+8))/100;
			iir = (float)((int)(*(data_buf+9)<<8)|*(data_buf+10))/1000;
			idr = (float)((int)(*(data_buf+11)<<8)|*(data_buf+12))/100;//x or y
			ipz = (float)((int)(*(data_buf+13)<<8)|*(data_buf+14))/100;
			iiz = (float)((int)(*(data_buf+15)<<8)|*(data_buf+16))/100;
			idz = (float)((int)(*(data_buf+17)<<8)|*(data_buf+18))/100;//z
	}
	if(data_buf[0]==0X12)//PID3
	{
			oph = (float)((int)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			opr = (float)((int)(*(data_buf+6)<<8)|*(data_buf+7))/100;
			opz = (float)((int)(*(data_buf+8)<<8)|*(data_buf+9))/100;
	}
	head_or_data=true;
	R_UART1_Receive(data_buf,4);
}
void sendattitude(void)
{
	//sendstatus();
	//sendremote();
	//sendpwm();
}
void sendstatus(void)//发送姿态数据
{
	unsigned char data_to_send[18],sum=0;
	int i;
	union{int j;unsigned char a[2];}tmp;
	angle angledata;
	angledata=getangle();
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0x01;
	data_to_send[3]=18;
	
	tmp.j=((int)(angledata.x*100));
	data_to_send[4]=tmp.a[1];
	data_to_send[5]=tmp.a[0];
	
	tmp.j=((int)(angledata.y*100));
	data_to_send[6]=tmp.a[1];
	data_to_send[7]=tmp.a[0];
	
	tmp.j=((int)(angledata.z*100));
	data_to_send[8]=tmp.a[1];
	data_to_send[9]=tmp.a[0];
	
	//temp=Alt_CSB;
	data_to_send[10]=0;//BYTE1(_temp);
	data_to_send[11]=0;//BYTE0(_temp);
	
	//vs32 _temp2 = Alt;
	data_to_send[12]=0;//BYTE3(_temp2);
	data_to_send[13]=0;//BYTE2(_temp2);
	data_to_send[14]=0;//BYTE1(_temp2);
	data_to_send[15]=0;//BYTE0(_temp2);*/
		
	if(iflocked())data_to_send[16]=0xA0;	
	else if(!iflocked())data_to_send[16]=0xA1;
	
	for(i=0;i<17;++i)sum+=data_to_send[i];
	data_to_send[17]=sum;

	sendwait1=true;
	R_UART1_Send(data_to_send,18);
	while(sendwait1){NOP();}
}
void sendremote(void)//发送遥控器数据
{
	unsigned char data_to_send[25],sum=0;
	int i;
	union{int j;unsigned char a[2];}tmp;
	remotedata command;
	command=getcommand();
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0x03;
	data_to_send[3]=20;
	
	tmp.j=((int)(command.h*100));
	data_to_send[4]=tmp.a[1];
	data_to_send[5]=tmp.a[0];
	
	tmp.j=((int)(command.x*100));
	data_to_send[6]=tmp.a[1];
	data_to_send[7]=tmp.a[0];
	
	tmp.j=((int)(command.y*100));
	data_to_send[8]=tmp.a[1];
	data_to_send[9]=tmp.a[0];
	
	tmp.j=((int)(command.vz*100));
	data_to_send[10]=tmp.a[1];
	data_to_send[11]=tmp.a[0];
	
	data_to_send[12]=0;
	data_to_send[13]=0;
	data_to_send[14]=0;
	data_to_send[15]=0;
	data_to_send[16]=0;
	data_to_send[17]=0;
	data_to_send[18]=0;
	data_to_send[19]=0;
	data_to_send[20]=0;
	data_to_send[21]=0;
	data_to_send[22]=0;
	data_to_send[23]=0;
	
	for(i=0;i<24;++i)sum+=data_to_send[i];
	
	data_to_send[24]=sum;

	R_UART1_Send(data_to_send,25);
}
void sendpwm(void)//发送四个电机的PWM值
{
	unsigned char data_to_send[13],sum=0;
	int i;
	union{int j;unsigned char a[2];}tmp;
	remotedata command;
	command=getcommand();
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0x06;
	data_to_send[3]=8;
	
	tmp.j=((int)TDR01);
	data_to_send[4]=tmp.a[1];
	data_to_send[5]=tmp.a[0];
	
	tmp.j=((int)TDR02);
	data_to_send[6]=tmp.a[1];
	data_to_send[7]=tmp.a[0];
	
	tmp.j=((int)TDR03);
	data_to_send[8]=tmp.a[1];
	data_to_send[9]=tmp.a[0];
	
	tmp.j=((int)TDR04);
	data_to_send[10]=tmp.a[1];
	data_to_send[11]=tmp.a[0];
	
	for(i=0;i<12;++i)sum+=data_to_send[i];
	
	data_to_send[12]=sum;

	R_UART1_Send(data_to_send,13);
}
void send_PID1(void)//外环P
{
	
}
void send_PID3(void)//内环P、I、D
{
	
}