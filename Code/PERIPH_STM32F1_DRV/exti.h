//*********************************************************************************************
//                                          Exti.h
//*********************************************************************************************

#ifndef __EXTI__
#define __EXTI__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "gpio.h"

//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define CSIN2_ENR					APB2ENR
#define RCC_CSIN2_EN				RCC_APB2ENR_IOPCEN
#define CSIN2_IRQn					EXTI9_5_IRQn
//-----------
#define INCPU2_ENR					APB2ENR
#define RCC_INCPU2_EN				RCC_APB2ENR_IOPCEN
#define INCPU2_IRQn					EXTI9_5_IRQn
//-----------
#define ENCOM1_ENR					APB2ENR
#define RCC_ENCOM1_EN				RCC_APB2ENR_IOPDEN
#define ENCOM1_IRQn					EXTI2_IRQn
//-----------
#define EXTICR_CSIN2_PORT_MASK		0x0002
#define EXTICR_INCPU2_PORT_MASK		0x0002
#define EXTICR_ENCOM1_PORT_MASK		0x0003




#define MYEXTI2_Port				CSIN2_Port
#define MYEXTI2_Pin					CSIN2_Pin
#define MYEXTI2_ENR					CSIN2_ENR
#define RCC_MYEXTI2_EN				RCC_CSIN2_EN
#define MYEXTI2_IRQn				CSIN2_IRQn
//-----------
#define MYEXTI3_Port				INCPU2_Port
#define MYEXTI3_Pin					INCPU2_Pin
#define MYEXTI3_ENR					INCPU2_ENR
#define RCC_MYEXTI3_EN				RCC_INCPU2_EN
#define MYEXTI3_IRQn				INCPU2_IRQn
//-----------
#define MYEXTI4_Port				ENCOM1_Port
#define MYEXTI4_Pin					ENCOM1_Pin
#define MYEXTI4_ENR					ENCOM1_ENR
#define RCC_MYEXTI4_EN				RCC_ENCOM1_EN
#define MYEXTI4_IRQn				ENCOM1_IRQn
//-----------
#define EXTICR_MYEXTI2_PORT_MASK	EXTICR_CSIN2_PORT_MASK
#define EXTICR_MYEXTI3_PORT_MASK	EXTICR_INCPU2_PORT_MASK
#define EXTICR_MYEXTI4_PORT_MASK	EXTICR_ENCOM1_PORT_MASK
//-----------
#define MYEXTI2_IRQHandler			EXTI9_5_IRQHandler
#define MYEXTI3_IRQHandler			EXTI9_5_IRQHandler
#define MYEXTI4_IRQHandler			EXTI2_IRQHandler


//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void ExtiInit(void);


#endif

