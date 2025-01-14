//*********************************************************************************************
//                                        Dwt.h
//*********************************************************************************************

#ifndef __DWT__
#define __DWT__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"

//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define DBG_ARR_SIZE                10


//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
#define DWT_TICKCNT_INIT()          CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 			\
                                    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk

#define DWT_TICKCNT_RD(arr, i, max) arr[i] = DWT->CYCCNT;                                       \
                                    if(arr[i] > max) {max = arr[i];}                            \
                                    if(++i >= DBG_ARR_SIZE) {i = 0;}                            \

#define DWT_TICKCNT_RST()  			DWT->CYCCNT = 0


//---------------------------------------------------------------------------------------------
//                                       Typedef section
//---------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void DelayDWT_us(uint32_t);


#endif

