#ifndef _SYSTEM_
#define _SYSTEM_

#include "r_cg_userdefine.h"

#define DEBUG

	#ifdef DEBUG

	//#define DEBUG_MOT_H 0
	//#define DEBUG_MOT_X 0
	//#define DEBUG_MOT_Y 0
	#define DEBUG_MOT_Z 0

	#endif

#define SIGN_STOP_EM P5.2

#define MOT_MAX 3800
#define MOT_MIN 2020
#define PROTECT 60

void sysinit(void);
void start(void);
void stop(void);
void mot_preoutput(void);
void mot_output(void);
void report(void);

#endif