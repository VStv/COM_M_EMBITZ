//*********************************************************************************************
//                                          Exti.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "exti.h"



//---------------------------------------------------------------------------------------------
//                                    Tables of Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//---------------------------------- EXTI initialization --------------------------------------
void ExtiInit(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  														// Clock for alt. func.
/*
	// Setup EXTI0 (PPS <-> MYEXTI1)
	RCC->MYEXTI1_ENR |= RCC_MYEXTI1_EN;  														// Clock for Port switch on
	SetAsInput(MYEXTI1_Port, MYEXTI1_Pin, 1);
	AFIO->EXTICR[MYEXTI1_Pin/4] &= ~(uint16_t)(0x000f << (4*(MYEXTI1_Pin & 0x03)));					// ~AFIO_EXTICR1_EXTI0 choose line PYx as EXTIx source
	AFIO->EXTICR[MYEXTI1_Pin/4] |= (uint16_t)(EXTICR_MYEXTI1_PORT_MASK << (4*(MYEXTI1_Pin & 0x03)));	// AFIO_EXTICR1_EXTI0_PB
	EXTI->RTSR |= (uint32_t)(1<<MYEXTI1_Pin);													// choose rising trigger
	EXTI->FTSR |= (uint32_t)(1<<MYEXTI1_Pin);													// choose falling trigger
	NVIC_SetPriority(MYEXTI1_IRQn, 0);
//	NVIC_EnableIRQ(MYEXTI1_IRQn);
	EXTI->EMR &= ~((uint32_t)(1<<MYEXTI1_Pin));
	EXTI->IMR |= (uint32_t)(1<<MYEXTI1_Pin);													// activate EXTIx interrupt
	EXTI->PR |= (uint32_t)(1<<MYEXTI1_Pin);
*/
	// Setup EXTI7 (CSIN2 <-> MYEXTI2)
	RCC->MYEXTI2_ENR |= RCC_MYEXTI2_EN;  														// Clock for Port switch on
	SetAsInput(MYEXTI2_Port, MYEXTI2_Pin, 1);
	//__disable_irq();
	AFIO->EXTICR[MYEXTI2_Pin/4] &= ~(uint16_t)(0x000f << (4*(MYEXTI2_Pin & 0x03)));				// ~AFIO_EXTICR2_EXTI7 choose line PYx as EXTIx source
	AFIO->EXTICR[MYEXTI2_Pin/4] |= (uint16_t)(EXTICR_MYEXTI2_PORT_MASK << (4*(MYEXTI2_Pin & 0x03)));	// AFIO_EXTICR2_EXTI7_PC
	EXTI->RTSR |= (uint32_t)(1<<MYEXTI2_Pin);													// choose rising trigger
	EXTI->FTSR |= (uint32_t)(1<<MYEXTI2_Pin);													// choose falling trigger
	NVIC_SetPriority(MYEXTI2_IRQn, 6);
	NVIC_EnableIRQ(MYEXTI2_IRQn);
	EXTI->EMR &= ~((uint32_t)(1<<MYEXTI2_Pin));
	EXTI->IMR &= ~((uint32_t)(1<<MYEXTI2_Pin));													// activate EXTIx interrupt
	EXTI->PR |= (uint32_t)(1<<MYEXTI2_Pin);														// activate EXTIx interrupt

	// Setup EXTI8 (IN_CPU2 <-> MYEXTI3)
	RCC->MYEXTI3_ENR |= RCC_MYEXTI3_EN;  														// Clock for Port switch on
	SetAsInput(MYEXTI3_Port, MYEXTI3_Pin, 1);
	//__disable_irq();
	AFIO->EXTICR[MYEXTI3_Pin/4] &= ~(uint16_t)(0x000f << (4*(MYEXTI3_Pin & 0x03)));				// ~AFIO_EXTICR3_EXTI8 choose line PYx as EXTIx source
	AFIO->EXTICR[MYEXTI3_Pin/4] |= (uint16_t)(EXTICR_MYEXTI3_PORT_MASK << (4*(MYEXTI3_Pin & 0x03)));	// AFIO_EXTICR3_EXTI8_PC
	EXTI->RTSR |= (uint32_t)(1<<MYEXTI3_Pin);													// choose rising trigger
	EXTI->FTSR |= (uint32_t)(1<<MYEXTI3_Pin);													// choose falling trigger
	NVIC_SetPriority(MYEXTI3_IRQn, 6);
	NVIC_EnableIRQ(MYEXTI3_IRQn);
	EXTI->EMR &= ~((uint32_t)(1<<MYEXTI3_Pin));
	EXTI->IMR &= ~((uint32_t)(1<<MYEXTI3_Pin));													// activate EXTIx interrupt
	EXTI->PR |= (uint32_t)(1<<MYEXTI3_Pin);

	// Setup EXTI2 (ENCOM1 <-> MYEXTI4)
	RCC->MYEXTI4_ENR |= RCC_MYEXTI4_EN;  														// Clock for Port switch on
	SetAsInput(MYEXTI4_Port, MYEXTI4_Pin, 1);
	//__disable_irq();
	AFIO->EXTICR[MYEXTI4_Pin/4] &= ~(uint16_t)(0x000f << (4*(MYEXTI4_Pin & 0x03)));				// ~AFIO_EXTICR3_EXTI2 choose line PYx as EXTIx source
	AFIO->EXTICR[MYEXTI4_Pin/4] |= (uint16_t)(EXTICR_MYEXTI4_PORT_MASK << (4*(MYEXTI4_Pin & 0x03)));	// AFIO_EXTICR3_EXTI2_PC
	EXTI->FTSR |= (uint32_t)(1<<MYEXTI4_Pin);													// choose rising trigger
	NVIC_SetPriority(MYEXTI4_IRQn, 2);
	NVIC_EnableIRQ(MYEXTI4_IRQn);
	EXTI->EMR &= ~((uint32_t)(1<<MYEXTI4_Pin));
	EXTI->IMR |= (uint32_t)(1<<MYEXTI4_Pin);													// activate EXTIx interrupt
	EXTI->PR |= (uint32_t)(1<<MYEXTI4_Pin);
}


