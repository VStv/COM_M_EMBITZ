//*********************************************************************************************
//                                          Timer.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "timer.h"

//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//--------------------------------- Timer 1 initialization ------------------------------------
void TimerInit(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;  													// TIM1 clock switch on
//	if(BKP->DR2 != PSC_SAVED)
//	{
//		PWR->CR |= PWR_CR_DBP;																// enable access to DBP
//		BKP->DR3 = (uint16_t)(SystemCoreClock/2000 - 1); 									// 2kHz (SystemCoreClock/2000) - 1
//		BKP->DR2 = PSC_SAVED;
//		PWR->CR &= ~PWR_CR_DBP;																// disable access to DBP
//	}
	TIM1->PSC = (uint16_t)(SystemCoreClock/2000 - 1);       // BKP->DR3;																	//
	TIM1->ARR = 2000 - 1;
	TIM1->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 15);
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	TIM1->CR1 |= TIM_CR1_CEN;																// Запускаем счет таймера
//	__disable_irq();
}


//--------------------------------- Timer 1 initialization ------------------------------------
void TimerDeinit(
                TIM_TypeDef *Tim_Inst
                )
{
    if(Tim_Inst == MYTIMER1)
    {
        NVIC_DisableIRQ(MYTIMER1_IRQn);
        RCC->MYTIMER1_RSTR |= RCC_MYTIMER1_RST;
        RCC->MYTIMER1_ENR &= ~RCC_MYTIMER1_EN;
    }
    else if(Tim_Inst == MYTIMER2)
    {
        NVIC_DisableIRQ(MYTIMER2_IRQn);
        RCC->MYTIMER2_RSTR |= RCC_MYTIMER2_RST;
        RCC->MYTIMER2_ENR &= ~RCC_MYTIMER2_EN;
    }
    else
    {
        NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
        RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
        RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
    }
}


//----------------------------------- MyTimer configuring --------------------------------------
uint32_t ConfigMyTimer  (
                        TIM_TypeDef *Tim_Inst,
                        uint32_t t_sym
                        )
{
    // TIMx clock switch on
    if(Tim_Inst == MYTIMER1)
        RCC->MYTIMER1_ENR |= RCC_MYTIMER1_EN;
    else if(Tim_Inst == MYTIMER2)
        RCC->MYTIMER2_ENR |= RCC_MYTIMER2_EN;
    else
        return INSTANCE_ERR;

    // Prescaler = 72 (1tick = 1us)
    Tim_Inst->PSC = (uint16_t)(72 - 1);

    // Fclk = (SystemCoreClock)/72
    // Mode: upcounting with autoreload 65536us
    Tim_Inst->CR1 |= TIM_CR1_ARPE;
    Tim_Inst->ARR = 0xffff;

	// Compare t1_5sym, Compare t3_5sym, Compare t10sym
	Tim_Inst->CCR1 = (uint16_t)(t_sym * 15 / 10);
	Tim_Inst->CCR2 = (uint16_t)(t_sym * 35 / 10);
//	Tim_Inst->CCR3 = (uint16_t)(60000);

	// Preload enable
	Tim_Inst->CCMR1 |= (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
//	Tim_Inst->CCMR2 |= TIM_CCMR2_OC3PE;

	// Interrupts enable: Compare t1_5sym, Compare t3_5sym, Compare t10sym
    Tim_Inst->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE);// | TIM_DIER_CC3IE);

    // Interrupt priority
    if(Tim_Inst == MYTIMER1)
    {
        NVIC_SetPriority(MYTIMER1_IRQn, 8);
//        NVIC_EnableIRQ(MYTIMER1_IRQn);
    }
    else if(Tim_Inst == MYTIMER2)
    {
        NVIC_SetPriority(MYTIMER2_IRQn, 8);
//        NVIC_EnableIRQ(MYTIMER2_IRQn);
    }
    else
        return INSTANCE_ERR;
    return RESULT_OK;
}


//----------------------------------- MyTimer configuring --------------------------------------
void UpdateMyTimerSettings  (
                            TIM_TypeDef *Tim_Inst,
                            uint32_t t_sym
                            )
{
	// Compare t1_5sym, Compare t3_5sym, Compare t10sym
	Tim_Inst->CCR1 = (uint16_t)(t_sym * 15 / 10);
	Tim_Inst->CCR2 = (uint16_t)(t_sym * 35 / 10);
}


//----------------------------------- MyTimer start --------------------------------------
void StartMyTimer   (
                    TIM_TypeDef *Tim_Inst
                    )
{
    // Interrupt priority
    if(Tim_Inst == MYTIMER1)
    {
        NVIC_EnableIRQ(MYTIMER1_IRQn);
    }
    else
    {
        NVIC_EnableIRQ(MYTIMER2_IRQn);
    }
    Tim_Inst->CR1 |= TIM_CR1_CEN;
}


//----------------------------------- MyTimer start --------------------------------------
void StopMyTimer   (
                    TIM_TypeDef *Tim_Inst
                    )
{
    // Interrupt priority
    if(Tim_Inst == MYTIMER1)
    {
        NVIC_DisableIRQ(MYTIMER1_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(MYTIMER2_IRQn);
    }
    Tim_Inst->CR1 &= ~TIM_CR1_CEN;
}


