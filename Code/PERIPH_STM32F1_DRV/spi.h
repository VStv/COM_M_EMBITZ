//*********************************************************************************************
//                                          Spi.h
//*********************************************************************************************

#ifndef __SPI__
#define __SPI__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "gpio.h"



//---------------------------------------------------------------------------------------------
//                                     Compile control
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
//------------------------------------------ SPI1 ---------------------------------------------
#define MYSPI1						SPI1
#define MYSPI1_ENR					APB2ENR
#define MYSPI1_RSTR					APB2RSTR
#define RCC_MYSPI1_EN				RCC_APB2ENR_SPI1EN
#define RCC_MYSPI1_RST				RCC_APB2RSTR_SPI1RST
#define MYSPI1_IRQn					SPI1_IRQn
#define MYSPI1_IRQHandler			SPI1_IRQHandler
//-----------
#define GPIO_MYSPI1_ENR				APB2ENR
#define RCC_GPIO_MYSPI1_EN			RCC_APB2ENR_IOPAEN
//-----------
#define DMA_MYSPI1_TX				DMA1
#define DMA_MYSPI1_TX_ENR			AHBENR
#define RCC_DMA_MYSPI1_TX_EN		RCC_AHBENR_DMA1EN
#define CH_DMA_MYSPI1_TX			DMA1_Channel3
#define CH_DMA_MYSPI1_TX_IRQn		DMA1_Channel3_IRQn
#define DMA_MYSPI1_TX_IRQHandler	DMA1_Channel3_IRQHandler
#define DMA_MYSPI1_TX_CCR_MASK		(DMA_CCR3_TCIE | DMA_CCR3_HTIE | DMA_CCR3_TEIE)
#define DMA_MYSPI1_TX_ISR_MASK		(DMA_ISR_TCIF3 | DMA_ISR_HTIF3 | DMA_ISR_TEIF3 | DMA_ISR_GIF3)
#define DMA_MYSPI1_TX_IFCR_MASK		(DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTEIF3 | DMA_IFCR_CGIF3)
#define DMA_MYSPI1_TX_IFCR_CGIF		DMA_IFCR_CGIF3
#define DMA_MYSPI1_TX_ISR_TCIF		DMA_ISR_TCIF3
#define DMA_MYSPI1_TX_CCR_EN		DMA_CCR3_EN
#define DMA_MYSPI1_TX_CCR_TCIE		DMA_CCR3_TCIE
#define DMA_MYSPI1_TX_CCR_HTIE		DMA_CCR3_HTIE
#define DMA_MYSPI1_TX_CCR_DIR		DMA_CCR3_DIR
#define DMA_MYSPI1_TX_CCR_MINC		DMA_CCR3_MINC
#define DMA_MYSPI1_TX_PSIZE_0		DMA_CCR3_PSIZE_0
//-----------
#define DMA_MYSPI1_RX				DMA1
#define DMA_MYSPI1_RX_ENR			AHBENR
#define RCC_DMA_MYSPI1_RX_EN		RCC_AHBENR_DMA1EN
#define CH_DMA_MYSPI1_RX			DMA1_Channel2
#define CH_DMA_MYSPI1_RX_IRQn		DMA1_Channel2_IRQn
#define DMA_MYSPI1_RX_IRQHandler	DMA1_Channel2_IRQHandler
#define DMA_MYSPI1_RX_CCR_MASK		(DMA_CCR2_TCIE | DMA_CCR2_HTIE | DMA_CCR2_TEIE)
#define DMA_MYSPI1_RX_ISR_MASK		(DMA_ISR_TCIF2 | DMA_ISR_HTIF2 | DMA_ISR_TEIF2 | DMA_ISR_GIF2)
#define DMA_MYSPI1_RX_IFCR_MASK		(DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTEIF2 | DMA_IFCR_CGIF2)
#define DMA_MYSPI1_RX_IFCR_CGIF		DMA_IFCR_CGIF2
#define DMA_MYSPI1_RX_ISR_TCIF		DMA_ISR_TCIF2
#define DMA_MYSPI1_RX_CCR_EN		DMA_CCR2_EN
#define DMA_MYSPI1_RX_CCR_TCIE		DMA_CCR2_TCIE
#define DMA_MYSPI1_RX_CCR_HTIE		DMA_CCR2_HTIE
#define DMA_MYSPI1_RX_CCR_DIR		DMA_CCR2_DIR
#define DMA_MYSPI1_RX_CCR_MINC		DMA_CCR2_MINC
#define DMA_MYSPI1_RX_PSIZE_0		DMA_CCR2_PSIZE_0
//-----------
#define DMA_MYSPI1					DMA_MYSPI1_TX
#define DMA_MYSPI1_ENR				DMA_MYSPI1_TX_ENR
#define RCC_DMA_MYSPI1_EN			RCC_DMA_MYSPI1_TX_EN






//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
#define SPI_ENABLE()                MYSPI1->CR1 |= SPI_CR1_SPE
#define SpiRxTxData(data)           *(uint8_t *)&(MYSPI1->DR) = data
#define SPI_RX()                    SpiRxTxData(0xff)
#define SPI_TX(data)                SpiRxTxData(data)
#define NODE_SELECT()               RESET_PIN(CSS1_Port, CSS1_Pin)
#define NODE_RELEASE()              SET_PIN(CSS1_Port, CSS1_Pin)



//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void MySpiInit(void);
void MySpiDeinit(void);



#endif
