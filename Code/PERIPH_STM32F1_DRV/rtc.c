//*********************************************************************************************
//                                          Rtc.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "rtc.h"



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern uint32_t 	                systick_ms;

//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//-------------------------------------
static uint32_t DateToElapsedDays	(
									RTC_t *date_time
									)
{
	uint32_t temp32, whole_month_days[12] = {0,31,59,90,120,151,181,212,243,273,304,334};

	temp32 = date_time->year * 365;
	if(date_time->year)
		temp32 += ((date_time->year - 1)>>2) + 1;
	temp32 += whole_month_days[date_time->month - 1];
	if((date_time->year & 0x0003) == 0 && date_time->month > 2)
		temp32++;
	temp32 += (date_time->mday - 1);
	return temp32;
}


//-------------------------------------
static uint8_t DateToWday	(
							RTC_t *date_time
							)
{
	uint32_t temp32;

	temp32 = DateToElapsedDays(date_time);
	temp32 += 6;
	return temp32%7;
}


//-------------------------------------
static uint32_t DateToSeconds	(
                                RTC_t *date_time
                                )
{
	uint32_t temp32;

	temp32 = DateToElapsedDays(date_time);// +1
	temp32 *= 24;
	temp32 += date_time->hour;
	temp32 *= 60;
	temp32 += date_time->min;
	temp32 *= 60;
	temp32 += date_time->sec;
	return temp32;
}


//-------------------------------------
static void FindJumpSundays	(
							RTC_t *rtc,
							RTC_t *march_jump,
							RTC_t *october_jump
							)
{
	uint32_t wday;
	uint8_t temp8;

	march_jump->year = rtc->year;
	march_jump->month = 3;
	march_jump->mday = 25;
	wday = DateToWday(march_jump);
	while(wday != 0)
	{
		march_jump->mday++;
		wday++;
		if(wday == 7)
			wday = 0;
	}

	october_jump->year = rtc->year;
	october_jump->month = 10;
	temp8 = march_jump->mday + 3;
	if(temp8 > 31)
		temp8 -= 7;
	october_jump->mday = temp8;

	march_jump->hour = 3;
	october_jump->hour = 4;
}


//-------------------------- Soft RTC load from hardware RTC ----------------------------------
void SoftRtcLoad(
				rtc_strct_t *rtc_fact,
                uint32_t period
				)
{
	rtc_fact->s_counter = 0;            //((uint32_t)RTC->CNTH << 16) | RTC->CNTL;
    rtc_fact->summer_time_need = 0;
    rtc_fact->time.inval = 1;
    rtc_fact->time_without_sync = period;
}


//---------------------------- Get current time from soft RTC  --------------------------------
void SoftRtcGetTime (
                    rtc_strct_t *rtc_fact
                    )
{
    // actual time
    GetRtcTimeFromCounter(rtc_fact->s_counter, &rtc_fact->time, rtc_fact->summer_time_need);
}


// -------------------- Get data for time structure from seconds counter ----------------------
/**********************************************************************************************
* Function Name  : counter_to_struct                                                          *
* Description    : fills time-struct based on counter-value                                   *
* Input          : - counter-value (unit seconds, 0 -> 1.1.2000 00:00:00),                    *
*                  - Pointer to time-struct                  								  *
*                  - time jump flag                                     					  *
* Output         : time-struct, 								                  *
* Return         : none                                                                       *
**********************************************************************************************/
void GetRtcTimeFromCounter	(
                            uint32_t rtc_cnt,
                            RTC_t *rtc,
                            uint32_t time_jump_need
                            )
{
	uint32_t temp32, full_days, full_4years, full_days_of_curr_4years, num_of_year_of_curr_4years;
	uint32_t full_days_of_curr_year;
	uint32_t full_days_of_full_years;
	uint32_t whole_month_days[13] = {0,31,59,90,120,151,181,212,243,273,304,334,365};
	uint32_t whole_month_days_of_leap_year[13] = {0,31,60,91,121,152,182,213,244,274,305,335,366};
	uint32_t *pntr;
	RTC_t march_jump, october_jump;

	temp32 = rtc_cnt;																				// ((uint32_t)RTC->CNTH << 16) | RTC->CNTL;
	rtc->sec = temp32 % 60;
	temp32 /= 60;
	rtc->min = temp32 % 60;
	temp32 /= 60;
	rtc->hour = temp32 % 24;
	full_days = temp32 / 24;
	rtc->wday = (full_days + 6) % 7;

	full_4years = full_days / 1461;
	full_days_of_curr_4years = full_days % 1461;
	num_of_year_of_curr_4years = 0;
	if(full_days_of_curr_4years >= 366)				// !!!!!!!!!!!!!!!!!!!!!!!!!
		num_of_year_of_curr_4years = 1 + (full_days_of_curr_4years - 366)/365;
	rtc->year = full_4years * 4 + num_of_year_of_curr_4years;

	full_days_of_full_years = full_4years * 1461 + num_of_year_of_curr_4years * 365;
	if(num_of_year_of_curr_4years > 0)
		full_days_of_full_years++;
	full_days_of_curr_year = full_days - full_days_of_full_years;

	if(num_of_year_of_curr_4years == 0)
		pntr = whole_month_days_of_leap_year;
	else
		pntr = whole_month_days;
	for(int i=1; i<=12; i++)
	{
		if(full_days_of_curr_year < *(pntr + i))
		{
			rtc->month = i;
			rtc->mday = full_days_of_curr_year - *(pntr + i - 1) + 1;
			break;
		}
	}
	rtc->dst = 0;

	if(time_jump_need)
	{
		// find dates of time jumping
		FindJumpSundays(rtc, &march_jump, &october_jump);
		if(	(rtc->month > 3 && rtc->month < 10)
			||
			(rtc->month == 3 && rtc->mday > march_jump.mday)
			||
			(rtc->month == 10 && rtc->mday < october_jump.mday)
			||
			(rtc->month == 3 && rtc->mday == march_jump.mday && rtc->hour >= 3)
			||
			(rtc->month == 10 && rtc->mday == october_jump.mday && rtc->hour < 3)
			||
			(rtc->month == 10 && rtc->mday == october_jump.mday && rtc->hour == 2 && rtc->min < 59)
			||
			(rtc->month == 10 && rtc->mday == october_jump.mday && rtc->hour == 2 && rtc->min == 59 && rtc->sec <= 59)	)
		{
			rtc->hour++;
			rtc->dst = 1;
			if(rtc->hour > 23)
			{
				rtc->hour = 0;
				rtc->mday++;
				if(	((rtc->mday > 30) && ((rtc->month == 4) || (rtc->month == 6) || (rtc->month == 9)))
					||
					((rtc->mday > 31) && ((rtc->month == 3) || (rtc->month == 5) || (rtc->month == 7) || (rtc->month == 8)))	)
				{
					rtc->mday = 1;
					rtc->month++;
				}
			}
		}
	}
}


// --------------------- Get seconds counter from time structure ------------------------------
/**********************************************************************************************
* Function Name  : struct_to_counter                                                          *
* Description    : calculates the counter value based on the structure RTC_t fields values    *
* Input          : - Pointer to time-struct time-struct gets filled        					  *
*                  - time jump flag                                     					  *
* Output         : - counter-value (unit seconds, 0 -> 1.1.2000 00:00:00),                    *
* Return         : - counter-value (unit seconds, 0 -> 1.1.2000 00:00:00)                     *
**********************************************************************************************/
uint32_t GetRtcCounterFromTime	(
                                RTC_t *rtc,
                                uint32_t time_jump_need
                                )
{
	RTC_t march_jump, october_jump;

	if(time_jump_need)
	{
		// find dates of time jumping
		FindJumpSundays(rtc, &march_jump, &october_jump);
		rtc->dst = 0;
		if(	(rtc->month > 3 && rtc->month < 10)
			||
			(rtc->month == 3 && rtc->mday > march_jump.mday)
			||
			(rtc->month == 10 && rtc->mday < october_jump.mday)
			||
			(rtc->month == 3 && rtc->mday == march_jump.mday && rtc->hour >= 3)
			||
			(rtc->month == 10 && rtc->mday == october_jump.mday && rtc->hour < 4)
			||
			(rtc->month == 10 && rtc->mday == october_jump.mday && rtc->hour == 3 && rtc->min < 59)
			||
			(rtc->month == 10 && rtc->mday == october_jump.mday && rtc->hour == 3 && rtc->min == 59 && rtc->sec <= 59))
		{
			rtc->dst = 1;
			rtc->hour--;
			if(rtc->hour == 0xff)
			{
				rtc->hour = 23;
				rtc->mday--;
				if(rtc->mday == 0)
				{
					rtc->mday = 30;
					if((rtc->month == 4) || (rtc->month == 6) || (rtc->month == 8) || (rtc->month == 9))
					{
						rtc->mday = 31;
					}
					rtc->month--;
				}
			}
		}
	}
	return DateToSeconds(rtc);
}


//-------------------------- Soft RTC restore in hardware RTC ---------------------------------
void RestoreSoftRtc	(
					uint32_t cnt
					)
{
	PWR->CR |= PWR_CR_DBP;                			//разрешить доступ к области резарвных данных Backup
	while(!(RTC->CRL & RTC_CRL_RTOFF));  			//проверить закончены ли изменения регистров RTC
	RTC->CRL |= RTC_CRL_CNF;						//Разрешить Запись в регистры RTC_CNT(счётный),RTC_PRL(предделитель),RTC_ALR(аварий)
	RTC->CNTH = cnt>>16;
	RTC->CNTL = cnt;
	RTC->CRL &= ~RTC_CRL_CNF;            			//Запретить запись в регистры  RTC_CNT, RTC_PRL, RTC_ALR
	while(!(RTC->CRL & RTC_CRL_RTOFF)); 			//Дождаться окончания записи регистров RTC
	PWR->CR &= ~PWR_CR_DBP;                			//Запретить доступ к области резарвных данных Backup.
}


/*
//---------------------------------- RTC initialization ---------------------------------------
void RtcInit(void)															// with external ZQ
{
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;					// разрешить тактирование модулей управления питанием и управления резервной областью
    if(BKP->DR1 != (uint16_t)0x5AA5)
    {
		PWR->CR |= PWR_CR_DBP;												// разрешить доступ к области резервных данных
        RCC->BDCR |= RCC_BDCR_BDRST;										// reset Backup Domain
        RCC->BDCR &= ~RCC_BDCR_BDRST;
        RCC->BDCR |= RCC_BDCR_RTCSEL_LSE | RCC_BDCR_RTCEN;					// select LSE as RTC clock source, enable RTC clock    			!!!! RCC_BDCR_RTCSEL_LSI | RCC_BDCR_RTCEN; 	!!!!!

        RCC->BDCR |= RCC_BDCR_LSEON; 										// enable LSE         											!!!! RCC->CSR |= RCC_CSR_LSION;				!!!!!!!!
        while(!(RCC->BDCR & RCC_BDCR_LSEON)){};								// RCC_BDCR_LSERDY wait till LSE is ready (RCC_BDCR_LSERDY) 	!!!! (RCC->CSR & RCC_CSR_LSION)				!!!!!

        while(!(RTC->CRL & RTC_CRL_RTOFF)){};								// Дождаться окончания записи регистров RTC
		RTC->CRL |= (uint16_t)RTC_CRL_CNF;									// enter config mode
        RTC->CNTH = (uint16_t)0x0028;
        RTC->CNTL = (uint16_t)0xDE80;
        RTC->PRLH = 0;
        RTC->PRLL = 0x7FFF; 												// set RTC prescaler: set RTC period to 1sec 0x8000(0x7FFF);
        RTC->CRL &= (uint16_t)~RTC_CRL_CNF;									// exit config mode Запретить запись в регистры RTC_CNT, RTC_PRL, RTC_ALR

        while(!(RTC->CRL & RTC_CRL_RTOFF)){};								// Дождаться окончания записи регистров RTC
        RTC->CRL &= (uint16_t)~RTC_CRL_SECF;

        while (!(RTC->CRL & RTC_CRL_RTOFF)){};								// Дождаться окончания записи регистров RTC
		BKP->DR1 = (uint16_t)0x5AA5;

        RTC->CRL &= (uint16_t)~RTC_CRL_RSF; 								// Синхронизировать RTC
        while (!(RTC->CRL & RTC_CRL_RSF));									// wait for RTC registers synchronization
        PWR->CR &= ~PWR_CR_DBP;												// disable access to backup domain write
    }
    AFIO->EVCR = 0xA0; 														// PC13-calibration clock output enable
	PWR->CR = PWR_CR_PLS_2V5|PWR_CR_PVDE;									// Power voltage detector enable, level - 2.5V
}


//---------------------------------- RTC initialization ---------------------------------------
uint32_t RtcInit3(
				uint32_t *err
				)															// with external ZQ
{
	int cnt;

	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;					// разрешить тактирование модулей управления питанием и управления резервной областью
    if(BKP->DR1 != (uint16_t)0x5AA5)
    {
//		if(*err)
//		{
//			GPIOC->CRH &= 0x00ffffff;
//			GPIOC->CRH |= 0x22000000;
//			__NOP(); __NOP(); __NOP();
//			GPIOC->BSRR = GPIO_BSRR_BS15;
//			cnt = 400;
//			while(cnt != 0)
//				cnt--;
//			GPIOC->BSRR = GPIO_BSRR_BR15;
//			GPIOC->CRH &= 0x00ffffff;
//		}
		switch(*err)
		{
			case 0:
			case 1:
				PWR->CR |= PWR_CR_DBP;												// разрешить доступ к области резервных данных
				RCC->BDCR |= RCC_BDCR_BDRST;										// reset Backup Domain
				RCC->BDCR &= ~RCC_BDCR_BDRST;
				RCC->BDCR |= RCC_BDCR_RTCSEL_LSE | RCC_BDCR_RTCEN;					// select LSE as RTC clock source, enable RTC clock    			!!!! RCC_BDCR_RTCSEL_LSI | RCC_BDCR_RTCEN; 	!!!!!

				RCC->BDCR |= RCC_BDCR_LSEON; 										// enable LSE         											!!!! RCC->CSR |= RCC_CSR_LSION;				!!!!!!!!
				cnt = 0;
				while(!(RCC->BDCR & RCC_BDCR_LSEON))								// RCC_BDCR_LSERDY wait till LSE is ready (RCC_BDCR_LSERDY) 	!!!! (RCC->CSR & RCC_CSR_LSION)				!!!!!
				{
					if(++cnt > 10)
						return 1;
				}

			case 2:
				cnt = 0;
				while(!(RTC->CRL & RTC_CRL_RTOFF))									// Дождаться окончания записи регистров RTC
				{
					if(++cnt > 10)
						return 2;
				}
			case 3:
				RTC->CRL |= (uint16_t)RTC_CRL_CNF;									// enter config mode
				RTC->CNTH = (uint16_t)0x0028;
				RTC->CNTL = (uint16_t)0xDE80;
				RTC->PRLH = 0;
				RTC->PRLL = 0x7FFF; 												// set RTC prescaler: set RTC period to 1sec 0x8000(0x7FFF);
				RTC->CRL &= (uint16_t)~RTC_CRL_CNF;									// exit config mode Запретить запись в регистры RTC_CNT, RTC_PRL, RTC_ALR
				cnt = 0;
				while(!(RTC->CRL & RTC_CRL_RTOFF))									// Дождаться окончания записи регистров RTC
				{
					if(++cnt > 100)
						return 3;
				}
				RTC->CRL &= (uint16_t)~RTC_CRL_SECF;

			case 4:
				cnt = 0;
				while (!(RTC->CRL & RTC_CRL_RTOFF))									// Дождаться окончания записи регистров RTC
				{
					if(++cnt > 10)
						return 4;
				}
				BKP->DR1 = (uint16_t)0x5AA5;
			case 5:
				RTC->CRL &= (uint16_t)~RTC_CRL_RSF; 								// Синхронизировать RTC
				cnt = 0;
				while (!(RTC->CRL & RTC_CRL_RSF))									// wait for RTC registers synchronization
				{
					if(++cnt > 10)
						return 5;
				}
				PWR->CR &= ~PWR_CR_DBP;												// disable access to backup domain write
		}
    }
    AFIO->EVCR = 0xA0; 														// PC13-calibration clock output enable
	PWR->CR = PWR_CR_PLS_2V5|PWR_CR_PVDE;									// Power voltage detector enable, level - 2.5V
	return (uint32_t)(-1);
}
*/

////---------------------------------- RTC initialization ---------------------------------------
//void RtcInit1(void)															// with external ZQ & full reload
//{
//	register uint16_t res, res1, wrBkp1;
//	//разрешить тактирование модулей управления питанием и управлением
//	//резервной областью
//	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;

//	PWR->CR |= PWR_CR_DBP;												// разрешить доступ к области резервных данных
//	BKP->DR1 = 0;															// !!!!!!!!!!!!


//	wrBkp1 = BKP->DR1;
//	if(wrBkp1 != (uint16_t)0x5AA5)
//	{
//		PWR->CR |= PWR_CR_DBP;//разрешить доступ к области резарвных данных
//		// reset Backup Domain
//		res = RTC->CNTH;
//		res1 = RTC->CNTL;
//		RCC->BDCR |= RCC_BDCR_BDRST;
//		RCC->BDCR &= ~RCC_BDCR_BDRST;
//		RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;//select LSE as RTC clock source	Выбрать LSE   как источник  тактирования RTC
//		RCC->BDCR |= RCC_BDCR_RTCEN; // enable RTC clock	Разрешить работу RTC
//
//		RCC->BDCR &= ~(RCC_BDCR_LSEON | RCC_BDCR_LSEBYP);
//		RCC->BDCR |= RCC_BDCR_LSEON; // enable LSE	установить бит разрешения работы LSE
//		while (!(RCC->BDCR & RCC_BDCR_LSERDY)){};//wait till LSE is ready	дождаться установки бита готовности	LSERDY
//		//LSERDY - Устанавливается и сбрасывается аппаратно, чтобы указать, когда внешний  генератор 32кГц является стабильным.
//		//После сброса LSEON бита , низкий уровень LSERDY после 6 циклов наружной низкой тактовой частоты генератора .
//		//0: внешний генератор 32 кГц не готов;  1: Внешний генератор 32 кГц готов;
//
//		while (!(RTC->CRL & RTC_CRL_RTOFF)){};//Дождаться окончания записи регистров RTC
//		RTC->CRL |= (uint16_t)RTC_CRL_CNF;//enter config mode //Разрешить Запись в регистры RTC_CNT(счётный),RTC_PRL(предделитель),
//												//RTC_ALR(аварий)
//		RTC->CNTH = res;
//		RTC->CNTL = res1;
//		RTC->PRLH = 0;
//		RTC->PRLL = 0x7FFF; // set RTC prescaler: set RTC period to 1sec 0x8000(0x7FFF);
//		RTC->CRL &= (uint16_t)~RTC_CRL_CNF;	// exit config mode Запретить запись в регистры RTC_CNT, RTC_PRL, RTC_ALR
//		while (!(RTC->CRL & RTC_CRL_RTOFF)){};//Дождаться окончания записи регистров RTC
//		RTC->CRL &= (uint16_t)~RTC_CRL_SECF;
//		while (!(RTC->CRL & RTC_CRL_RTOFF)){};//Дождаться окончания записи регистров RTC
//		BKP->DR1 = (uint16_t)0x5AA5;
//
//		RTC->CRL &= (uint16_t)~RTC_CRL_RSF; //Синхронизировать RTC
//		while (!(RTC->CRL & RTC_CRL_RSF));// wait for RTC registers synchronization	 Дождаться синхронизации RTC
//		PWR->CR &= ~PWR_CR_DBP;// enable backup domain write protection Запретить доступ к области резарвных данных.
//	}
//	PWR->CR = PWR_CR_PLS_2V5|PWR_CR_PVDE;
//}


////---------------------------------- RTC initialization ---------------------------------------
//void RtcInit2(void)															// with internal ZQ (40 kHz) & full reload
//{
//    register uint16_t res, res1, wrBkp1;
//
//	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;					// разрешить тактирование модулей управления питанием и управления резервной областью

//	PWR->CR |= PWR_CR_DBP;												// разрешить доступ к области резервных данных
//	BKP->DR1 = 0;															// !!!!!!!!!!!!
//
//    wrBkp1 = BKP->DR1;
//    if(wrBkp1 != (uint16_t)0x5AA5)
//    {
//		PWR->CR |= PWR_CR_DBP;												// разрешить доступ к области резервных данных
//        res = RTC->CNTH;
//        res1 = RTC->CNTL;
//        RCC->BDCR |= RCC_BDCR_BDRST;										// reset Backup Domain
//        RCC->BDCR &= ~RCC_BDCR_BDRST;
//        RCC->BDCR |= RCC_BDCR_RTCSEL_LSI | RCC_BDCR_RTCEN;					// select LSE as RTC clock source, enable RTC clock    			RCC_BDCR_RTCSEL_LSE | RCC_BDCR_RTCEN;!!!! !!!!!
//
//        RCC->CSR |= RCC_CSR_LSION; 										// enable LSE         											RCC->BDCR |= RCC_BDCR_LSEON;!!!! !!!!!!!!
//        while(!(RCC->CSR & RCC_CSR_LSION)){};								// RCC_BDCR_LSERDY wait till LSE is ready (RCC_BDCR_LSERDY) 	!!!!(RCC->BDCR & RCC_BDCR_LSEON) !!!!!
//
//        while(!(RTC->CRL & RTC_CRL_RTOFF)){};								// Дождаться окончания записи регистров RTC
//		RTC->CRL |= (uint16_t)RTC_CRL_CNF;									// enter config mode
//        RTC->CNTH = res;
//        RTC->CNTL = res1;
//        RTC->PRLH = 0;
//        RTC->PRLL = (40000-1); 												// set RTC prescaler: set RTC period to 1sec 0x8000(0x7FFF);
//        RTC->CRL &= (uint16_t)~RTC_CRL_CNF;									// exit config mode Запретить запись в регистры RTC_CNT, RTC_PRL, RTC_ALR

//        while(!(RTC->CRL & RTC_CRL_RTOFF)){};								// Дождаться окончания записи регистров RTC
//        RTC->CRL &= (uint16_t)~RTC_CRL_SECF;

//        while (!(RTC->CRL & RTC_CRL_RTOFF)){};								// Дождаться окончания записи регистров RTC
//		BKP->DR1 = (uint16_t)0x5AA5;
//
//        RTC->CRL &= (uint16_t)~RTC_CRL_RSF; 								// Синхронизировать RTC
//        while (!(RTC->CRL & RTC_CRL_RSF));									// wait for RTC registers synchronization
//        PWR->CR &= ~PWR_CR_DBP;												// disable access to backup domain write
//    }
//    AFIO->EVCR = 0xA0; 														// PC0-calibration clock output enable
//	PWR->CR = PWR_CR_PLS_2V5|PWR_CR_PVDE;									// Power voltage detector enable, level - 2.5V

//}





//**********************************************************************************************
//                                   Interrupt procedures
//**********************************************************************************************
//--------------- Soft RTC seconds counter update (Executes in TIM1 IRQ Handler) ---------------
void SoftRtcUpdateHandler   (
                            rtc_strct_t *rtc_fact,
                            uint32_t period
                            )
{
    // increase seconds counter
    rtc_fact->s_counter++;

	SoftRtcGetTime(rtc_fact);

    // Check sync timeout
    if(rtc_fact->time_without_sync < period)
    {
        rtc_fact->time_without_sync++;
        if(rtc_fact->time_without_sync == period)
            rtc_fact->time.inval = 1;
    }
}


//---------------- Soft RTC time correction (Executes in SysTick IRQ Handler) ------------------
void SoftRtcTimeCorrect (
                        rtc_strct_t *rtc_sync
                        )
{
    uint32_t delta, temp, temp2 = 0;

    temp = (SystemCoreClock / configTICK_RATE_HZ) * systick_ms + SysTick->VAL;
    if(temp < rtc_sync->rtc_Time_Stamp)
        temp += SystemCoreClock;
    delta = temp - rtc_sync->rtc_Time_Stamp + rtc_sync->rtc_Time_Delay + temp2;

    // we have time correction value that equal to
    // time interval between start trnsmission moment of 1st bit of 1st byte and current time (ms)
    temp = rtc_sync->ms_counter + delta / (SystemCoreClock / configTICK_RATE_HZ);
    rtc_sync->ms_counter = temp % 1000;
    rtc_sync->s_counter += temp / 1000;
}


// ---------------- Soft RTC synchronization (Executes in SysTick IRQ Handler) -----------------
void SoftRtcSyncProcess (
                        rtc_strct_t *rtc_sync,
                        rtc_strct_t *rtc_fact
                        )
{
    if(++systick_ms >= 1000)
        systick_ms = 0;

    if(rtc_fact->rtc_state == RTC_SYNC_START)
    {
        // Time correction
        SoftRtcTimeCorrect(rtc_sync);

        // Sync process:    stop TIM1,
        //                  reset TIM1 IRQ flag,
        //                  save difference of rtc_fact & rtc_sync counters,
        //                  update s_counter & TIM1->CNT,
        //                  start TIM1
        TIM1->CR1 &= ~TIM_CR1_CEN;
//      NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		TIM1->SR &= ~TIM_SR_UIF;

        rtc_fact->s_counter = rtc_sync->s_counter;
        TIM1->CNT = (uint16_t)(rtc_sync->ms_counter << 1);

//      NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        TIM1->CR1 |= TIM_CR1_CEN;

        // Time is valid
        rtc_fact->time.inval = 0;
        rtc_fact->time_without_sync = 0;

        // Sync finished
        rtc_fact->rtc_state = RTC_SYNC_CMPLT;
    }
}




