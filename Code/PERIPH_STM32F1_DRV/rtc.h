//*********************************************************************************************
//                                          Rtc.h
//*********************************************************************************************

#ifndef __RTC__
#define __RTC__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "stm32F10x.h"
#include "config.h"


//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define FIRSTDAY 			5
#define PWR_OFFSET  		(PWR_BASE - PERIPH_BASE)
#define DBP_BitNumber  		0x00000008
#define CR_DBP_BB  			(PERIPH_BB_BASE + (PWR_OFFSET * 32) + (DBP_BitNumber * 4))
#define RCC_OFFSET  		(RCC_BASE - PERIPH_BASE)
#define BDCR_OFFSET  		(RCC_OFFSET + 0x20)
#define BDRST_BitNumber		0x10
#define BDCR_BDRST_BB  		(PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (BDRST_BitNumber * 4))

#define SUMMER_TIME 		1
#define NO_SUMMER_TIME 		0

#define SYNC_OVERTIME       (1*60*60)

//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef struct {
	uint8_t 	year;	   		/* 0..99 */
	uint8_t   	month;	   		/* 1..12 */
	uint8_t   	mday;	       	/* 1.. 31 */
	uint8_t   	wday;	       	/* 0..6, Sunday = 0*/
	uint8_t   	hour;	       	/* 0..23 */
	uint8_t   	min;	       	/* 0..59 */
	uint8_t   	sec;	       	/* 0..59 */
	uint8_t   	dst;	       	/* 0 - Winter, !=0 - Summer */
	uint8_t   	inval;	       	/* 0 - valid, !=0 - invalid (use only in fact time)*/
} RTC_t;


typedef enum {
	NO_CMD = 0,
	RTC_SYNC_START,
	RTC_SYNC_CMPLT,
//	RTC_PPS_WAIT,
} rtc_ctrl_t;


typedef enum {
	RTC_SETUP_RDY = 0,
	RTC_SETUP_DONE,
} rtc_flags_t;



typedef struct {
	uint32_t	        s_counter;
	uint32_t	        ms_counter;
	RTC_t 		        time;
	uint32_t   	        summer_time_need;       // use only in fact time

	rtc_ctrl_t          rtc_state;              // use only in fact time
    rtc_flags_t         rtc_flag;
	uint32_t	        time_without_sync;      // use only in fact time (time from last sync)
	uint32_t	        rtc_err;                // use only in fact time

    uint32_t   	        rtc_Time_Stamp;         // use only in sync time
    uint32_t   	        rtc_Time_Delay;         // use only in sync time
} rtc_strct_t;


//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void SoftRtcLoad(rtc_strct_t *, uint32_t);
void SoftRtcUpdateHandler(rtc_strct_t *, uint32_t);
void SoftRtcTimeCorrect(rtc_strct_t *);
void SoftRtcSyncProcess(rtc_strct_t *, rtc_strct_t *);
void SoftRtcGetTime(rtc_strct_t *);

void GetRtcTimeFromCounter(uint32_t, RTC_t *, uint32_t);
uint32_t GetRtcCounterFromTime(RTC_t *, uint32_t);




#endif
