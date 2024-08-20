//*********************************************************************************************
//                                         Systick.c
//*********************************************************************************************

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "systick.h"


//---------------------------------------------------------------------------------------------
//                                    Tables of Constants
//---------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern uint8_t 	                    del_sygnal;
//uint32_t 	                        system_tick[TICK_AMOUNT];

extern rtc_strct_t			        Rtc_fact, Rtc_sync;     //, Rtc_set


//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//--------------------------------------- us Delay --------------------------------------------
void usDelay(
			uint32_t us_del
			)
{
	uint32_t tick_cnt, del_cnt, temp32;
	int32_t end_cnt;
	
	del_sygnal = 0;
    tick_cnt = SysTick->VAL;
    temp32 = SystemCoreClock/1000000;
	del_cnt = us_del*temp32;
    if(del_cnt >= tick_cnt)
    {
        while(!del_sygnal);
        del_sygnal = 0;
        del_cnt -= tick_cnt;
        while(del_cnt >= SysTick->LOAD)
        {
            while(!del_sygnal);
            del_sygnal = 0;
            del_cnt -= SysTick->LOAD;
        }
        end_cnt = SysTick->LOAD - del_cnt;
    }
    else
    {
        end_cnt = tick_cnt - del_cnt;
    }
    while(tick_cnt > end_cnt)
		tick_cnt = SysTick->VAL;
    return;
}


//---------------------------------------------------------------------------------------------
//                                     Interrupt handlers
//---------------------------------------------------------------------------------------------
//void SysTick_Handler(void)
//{ 
//    if(Rtc_fact.rtc_flag == RTC_SYNC_START)
//    {
//        // Sync process
//        
//        
//        
//        Rtc_fact.s_counter = Rtc_sync.s_counter;
//        TIM1->CNT = (uint16_t)(Rtc_sync.ms_counter >> 2);
//        
//        // Time is valid
//        Rtc_fact.time.inval = 0;
//        Rtc_fact.time_without_sync = 0;
//        
//        // Sync finished
//        Rtc_fact.rtc_flag = NO_CMD;
//    }	
//    
//    // get actual ms_time 
//	Rtc_fact.ms_counter = ((uint32_t)TIM1->CNT)>>1;

//	// main programm tick
//	system_tick[MAIN_TICK] = CYCLE_START;
//	
//	// usDelay service
//	del_sygnal = 1;
//}

