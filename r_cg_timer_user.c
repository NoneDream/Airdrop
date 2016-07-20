/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2015 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_timer_user.c
* Version      : CodeGenerator for RL78/G13 V2.03.02.01 [15 May 2015]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for TAU module.
* Creation Date: 2016/5/31
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTTM00 r_tau0_channel0_interrupt
#pragma interrupt INTTM05 r_tau0_channel5_interrupt
#pragma interrupt INTTM06 r_tau0_channel6_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_timer.h"
/* Start user code for include. Do not edit comment generated here */
#include "GY95.h"
#include "attitude_control.h"
#include "system.h"
#include "px4flow.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
extern bool sign_stop;
extern angle angin;
extern remotedata command_buf;
extern float integral_h;
extern int mot_min_h;
extern angle integral;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_tau0_channel0_interrupt
* Description  : This function is INTTM00 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_tau0_channel0_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //EI();
    //mot_output();
    //getangledata();
    //cal_mot_ang();
    //mot_preoutput();
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_tau0_channel5_interrupt
* Description  : This function is INTTM05 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_tau0_channel5_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    LED=1U;
    
    if(sign_stop||!SIGN_STOP_EM)
    {
	    TDR01=2000;TDR02=2000;TDR03=2000;TDR04=2000;
	    sign_stop=true;
	    command_buf.h=0;
	    integral_h=0;
	    integral.x=0;
	    integral.y=0;
	    integral.z=0;
	    mot_min_h=100;
    }
    else
    {
	    mot_min_h=1000;
	    command_buf.h=10;
    }
    if(angin.x>PROTECT)
    {
    	sign_stop=true;
    }
    if(angin.x<-PROTECT)
    {
    	sign_stop=true;
    }
    if(angin.y>PROTECT)
    {
    	sign_stop=true;
    }
    if(angin.y<-PROTECT)
    {
    	sign_stop=true;
    }
    EI();
    if(!sign_stop)mot_output();
    get_data();//hight
    getangledata();//angle
    if(sign_stop)return;
    cal_mot_h();
    cal_mot_all();
    mot_preoutput();//limit
    
    LED=0U;
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_tau0_channel6_interrupt
* Description  : This function is INTTM06 interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_tau0_channel6_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    //EI();
    //get_data();
    //cal_mot_h();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
