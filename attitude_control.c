#include "r_cg_macrodriver.h"
#include "attitude_control.h"
#include "GY95.h"
#include "px4flow.h"
#include "raspberry_pi.h"
#include "system.h"

motor motout;
angle angle0,integral,delta0,ang;
extern angle angle_before;
float integral_h,hight0£¬delta0_h;
ppid 	angle_parameter_r={0.1,7,1.7,0,300},\
	angle_parameter_z={0,16,3.5,0,300},\
	hight_parameter={0.02,300,12,0,600};
int motrlimit=1000,outh,mot_min_h=100;
int outx,outy,outz;
remotedata command;

int duallooppid(ppid par,float *delta0,float *data0,float data,float aim,float *integral)
{
	int output;
	float neihuan_aim,delta,lim;

	neihuan_aim=(aim-data)*par.waihuan_p;
	delta=neihuan_aim-(data-(*data0));
	*integral=(*integral)+delta;
	
	lim=par.output_limit/par.neihuan_i;
	if(*integral<-lim)*integral=-lim;
	else if(*integral>lim)*integral=lim;
	
	output=((int)((delta*par.neihuan_p)+((delta-(*delta0))*par.neihuan_d)+((*integral)*par.neihuan_i)));
	*delta0=delta;
	*data0=data;
	if(output<-par.output_limit)
	{
		*integral=(*integral)-delta;
		return -par.output_limit;
	}
	else if(output>par.output_limit)
	{
		*integral=(*integral)-delta;
		return par.output_limit;
	}
	else return output;
}

void cal_mot_all(void)
{
	//int outx,outy,outz;
	motor mot_ang;
	angle *angnow;
	//remotedata command;
	
	command=returncommand();//getcommand();
	angnow=angle_filter();
	
	outx=duallooppid(angle_parameter_r,&(delta0.x),&(angle0.x),angle_before.x,command.x,&(integral.x));
	outy=duallooppid(angle_parameter_r,&(delta0.y),&(angle0.y),angle_before.y,command.y,&(integral.y));
	outz=duallooppid(angle_parameter_z,&(delta0.z),&(angle0.z),angle_before.z,command.vz,&(integral.z));
	//(command.vz+angle_before.z)
#ifdef DEBUG_MOT_X
	outx=DEBUG_MOT_X;
#endif
#ifdef DEBUG_MOT_Y
	outy=DEBUG_MOT_Y;
#endif
#ifdef DEBUG_MOT_Z
	outz=DEBUG_MOT_Z;
#endif
	
	mot_ang.TDR1=+outx+outy-outz;
	mot_ang.TDR2=-outx+outy+outz;
	mot_ang.TDR3=-outx-outy-outz;
	mot_ang.TDR4=+outx-outy+outz;
	
	limitang(&mot_ang,motrlimit);
	
	motout.TDR1=mot_ang.TDR1+outh+MOT_MIN;
	motout.TDR2=mot_ang.TDR2+outh+MOT_MIN;
	motout.TDR3=mot_ang.TDR3+outh+MOT_MIN;
	motout.TDR4=mot_ang.TDR4+outh+MOT_MIN;
}

void cal_mot_h(void)
{
	float hight;
	
	hight=hight_filter();
	if(hight<4)
	{
		integral.x=0;
		integral.y=0;
		integral.z=0;
	}
	if(integral_h<0)integral_h=0;
	outh=duallooppid(hight_parameter,&delta0_h,&hight0,hight,command.h,&integral_h);
#ifdef DEBUG_MOT_H
	outh=DEBUG_MOT_H;
#endif
	
	outh=outh+mot_min_h;
	
	//if(outh<mot_min_h)outh=mot_min_h;

}

void limitang(motor *mot_in,int motrlim)
{
	if(mot_in->TDR1<-motrlim)mot_in->TDR1=-motrlim;else if(mot_in->TDR1>motrlim)mot_in->TDR1=motrlim;
	if(mot_in->TDR2<-motrlim)mot_in->TDR2=-motrlim;else if(mot_in->TDR2>motrlim)mot_in->TDR2=motrlim;
	if(mot_in->TDR3<-motrlim)mot_in->TDR3=-motrlim;else if(mot_in->TDR3>motrlim)mot_in->TDR3=motrlim;
	if(mot_in->TDR4<-motrlim)mot_in->TDR4=-motrlim;else if(mot_in->TDR4>motrlim)mot_in->TDR4=motrlim;
}

motor *getmot(void)
{
	return &motout;
}