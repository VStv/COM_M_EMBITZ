//*********************************************************************************************
//                                         Systick.h
//*********************************************************************************************

#ifndef __SYSTICK__
#define __SYSTICK__

//---------------------------------------------------------------------------------------------
//                                      Include section 
//---------------------------------------------------------------------------------------------
#include "config.h"

#include "rtc.h"


//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
//Systick control
#define STOP_SYSTICK()	    	SysTick->CTRL &= (uint32_t)0xFFFFFFFE
#define START_SYSTICK()   		SysTick->CTRL |= (uint32_t)0x00000001






//---------------------------------------------------------------------------------------------
//                                      Typedef section
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                   Function's prototypes
//----------------------------------------------------------------------------------------------
void usDelay(uint32_t);

#endif
