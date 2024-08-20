//*********************************************************************************************
//                                          Timer.h
//*********************************************************************************************

#ifndef __TIMER__
#define __TIMER__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "config.h"

//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define	PSC_SAVED					(uint16_t)0xabcd

//----------------------------------- TIM3 (for USART1) ---------------------------------------
#define MYTIMER1					TIM2
#define MYTIMER1_ENR				APB1ENR
#define MYTIMER1_RSTR				APB1RSTR
#define RCC_MYTIMER1_EN				RCC_APB1ENR_TIM2EN
#define RCC_MYTIMER1_RST			RCC_APB1RSTR_TIM2RST
#define MYTIMER1_IRQn				TIM2_IRQn
#define MYTIMER1_IRQHandler			TIM2_IRQHandler


//----------------------------------- TIM2 (for USART2) ---------------------------------------
#define MYTIMER2					TIM3
#define MYTIMER2_ENR				APB1ENR
#define MYTIMER2_RSTR				APB1RSTR
#define RCC_MYTIMER2_EN				RCC_APB1ENR_TIM3EN
#define RCC_MYTIMER2_RST			RCC_APB1RSTR_TIM3RST
#define MYTIMER2_IRQn				TIM3_IRQn
#define MYTIMER2_IRQHandler			TIM3_IRQHandler

//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
// Timer start
#define START_MYTIMER(tim)          tim->CR1 |= TIM_CR1_CEN    

// Timer stop
#define STOP_MYTIMER(tim)           tim->CR1 &= ~TIM_CR1_CEN

// Update timer
#define UPDATE_MYTIMER(tim)   		tim->EGR |= TIM_EGR_UG;		\
                                    tim->SR = 0

//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void TimerInit(void);
void TimerDeinit(TIM_TypeDef *);
uint32_t ConfigMyTimer(TIM_TypeDef *, uint32_t);
void UpdateMyTimerSettings(TIM_TypeDef *, uint32_t);

void StartMyTimer(TIM_TypeDef *);
void StopMyTimer(TIM_TypeDef *);


#endif

