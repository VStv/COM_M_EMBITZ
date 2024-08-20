//*********************************************************************************************
//                                      Common_macro.h
//*********************************************************************************************
#ifndef __COMMON_MACRO__
#define __COMMON_MACRO__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "stm32f10x.h"



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
#define SET_FLAG(reg, flag)	    	(reg |= (1UL<<flag))
#define CLEAR_FLAG(reg, flag)    	(reg &= ~(1UL<<flag))
#define TEST_FLAG(reg, flag)		(reg & (1UL<<flag))

#define TOGLE_FLAG(reg, flag)	    (reg ^= (1UL<<flag))


#define RUN_TIMER(timer_x, time_x)	SET_FLAG(soft_timer_en, timer_x);	soft_timer[timer_x] = time_x

#define STOP_TIMER(timer_x)			CLEAR_FLAG(soft_timer_en, timer_x)

#define	ERROR_ACTION(CODE,POS)		do{}while(0)


//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------




#endif