//*********************************************************************************************
//                                         Usart.h
//*********************************************************************************************

#ifndef __USART__
#define __USART__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "gpio.h"
#include "timer.h"

//---------------------------------------------------------------------------------------------
//                                     Compiling control
//---------------------------------------------------------------------------------------------
//#define USE_DMA						0
//#define USE_INT						1



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
//----------------------------------------- USART1 --------------------------------------------
#define MYUART1						USART1
#define MYUART1_ENR					APB2ENR
#define MYUART1_RSTR			    APB2RSTR
#define RCC_MYUART1_EN				RCC_APB2ENR_USART1EN
#define RCC_MYUART1_RST				RCC_APB2RSTR_USART1RST
#define MYUART1_IRQn				USART1_IRQn
#define MYUART1_IRQHandler			USART1_IRQHandler
//-----------
#define GPIO_MYUART1_ENR			APB2ENR
#define RCC_GPIO_MYUART1_EN			RCC_APB2ENR_IOPAEN
//-----------
#define DMA_MYUART1_TX				DMA1
#define DMA_MYUART1_TX_ENR			AHBENR
#define RCC_DMA_MYUART1_TX_EN		RCC_AHBENR_DMA1EN
#define CH_DMA_MYUART1_TX			DMA1_Channel4
#define CH_DMA_MYUART1_TX_IRQn		DMA1_Channel4_IRQn
#define DMA_MYUART1_TX_IRQHandler	DMA1_Channel4_IRQHandler
#define DMA_MYUART1_TX_CCR_MASK		(DMA_CCR4_TCIE | DMA_CCR4_HTIE | DMA_CCR4_TEIE)
//#define DMA_MYUART1_TX_ISR_MASK		(DMA_ISR_TCIF4 | DMA_ISR_HTIF4 | DMA_ISR_TEIF4 | DMA_ISR_GIF4)
//#define DMA_MYUART1_TX_IFCR_MASK	(DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTEIF4 | DMA_IFCR_CGIF4)
#define DMA_MYUART1_TX_IFCR_CGIF	DMA_IFCR_CGIF4
#define DMA_MYUART1_TX_ISR_TCIF		DMA_ISR_TCIF4
#define DMA_MYUART1_TX_CCR_EN		DMA_CCR4_EN
#define DMA_MYUART1_TX_CCR_TCIE		DMA_CCR4_TCIE
#define DMA_MYUART1_TX_CCR_HTIE		DMA_CCR4_HTIE
#define DMA_MYUART1_TX_CCR_DIR		DMA_CCR4_DIR
#define DMA_MYUART1_TX_CCR_MINC		DMA_CCR4_MINC
//-----------
#define DMA_MYUART1_RX				DMA1
#define DMA_MYUART1_RX_ENR			AHBENR
#define RCC_DMA_MYUART1_RX_EN		RCC_AHBENR_DMA1EN
#define CH_DMA_MYUART1_RX			DMA1_Channel5
#define CH_DMA_MYUART1_RX_IRQn		DMA1_Channel5_IRQn
#define DMA_MYUART1_RX_IRQHandler	DMA1_Channel5_IRQHandler
#define DMA_MYUART1_RX_CCR_MASK		(DMA_CCR5_TCIE | DMA_CCR5_HTIE | DMA_CCR5_TEIE)
//#define DMA_MYUART1_RX_ISR_MASK		(DMA_ISR_TCIF5 | DMA_ISR_HTIF5 | DMA_ISR_TEIF5 | DMA_ISR_GIF5)
//#define DMA_MYUART1_RX_IFCR_MASK	(DMA_IFCR_CTCIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTEIF5 | DMA_IFCR_CGIF5)
#define DMA_MYUART1_RX_IFCR_CGIF	DMA_IFCR_CGIF5
#define DMA_MYUART1_RX_ISR_TCIF		DMA_ISR_TCIF5
#define DMA_MYUART1_RX_CCR_EN		DMA_CCR5_EN
#define DMA_MYUART1_RX_CCR_TCIE		DMA_CCR5_TCIE
#define DMA_MYUART1_RX_CCR_HTIE		DMA_CCR5_HTIE
#define DMA_MYUART1_RX_CCR_DIR		DMA_CCR5_DIR
#define DMA_MYUART1_RX_CCR_MINC		DMA_CCR5_MINC
//-----------
#define DMA_MYUART1					DMA_MYUART1_TX
#define DMA_MYUART1_ENR				DMA_MYUART1_TX_ENR
#define RCC_DMA_MYUART1_EN			RCC_DMA_MYUART1_TX_EN
//-----------


//------------------------------------------ USART2 --------------------------------------------
#define MYUART2						USART2
#define MYUART2_ENR					APB1ENR
#define MYUART2_RSTR			    APB1RSTR
#define RCC_MYUART2_EN				RCC_APB1ENR_USART2EN
#define RCC_MYUART2_RST				RCC_APB1RSTR_USART2RST
#define MYUART2_IRQn				USART2_IRQn
#define MYUART2_IRQHandler			USART2_IRQHandler
//-----------
#define GPIO_MYUART2_ENR			APB2ENR
#define RCC_GPIO_MYUART2_EN			RCC_APB2ENR_IOPAEN
//-----------
#define DMA_MYUART2_TX				DMA1
#define DMA_MYUART2_TX_ENR			AHBENR
#define RCC_DMA_MYUART2_TX_EN		RCC_AHBENR_DMA1EN
#define CH_DMA_MYUART2_TX			DMA1_Channel7
#define CH_DMA_MYUART2_TX_IRQn		DMA1_Channel7_IRQn
#define DMA_MYUART2_TX_IRQHandler	DMA1_Channel7_IRQHandler
#define DMA_MYUART2_TX_CCR_MASK		(DMA_CCR7_TCIE | DMA_CCR7_HTIE | DMA_CCR7_TEIE)
//#define DMA_MYUART2_TX_ISR_MASK		(DMA_ISR_TCIF7 | DMA_ISR_HTIF7 | DMA_ISR_TEIF7 | DMA_ISR_GIF7)
//#define DMA_MYUART2_TX_IFCR_MASK	(DMA_IFCR_CTCIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CTEIF7 | DMA_IFCR_CGIF7)
#define DMA_MYUART2_TX_IFCR_CGIF	DMA_IFCR_CGIF7
#define DMA_MYUART2_TX_ISR_TCIF		DMA_ISR_TCIF7
#define DMA_MYUART2_TX_CCR_EN		DMA_CCR7_EN
#define DMA_MYUART2_TX_CCR_TCIE		DMA_CCR7_TCIE
#define DMA_MYUART2_TX_CCR_HTIE		DMA_CCR7_HTIE
#define DMA_MYUART2_TX_CCR_DIR		DMA_CCR7_DIR
#define DMA_MYUART2_TX_CCR_MINC		DMA_CCR7_MINC
//-----------
#define DMA_MYUART2_RX				DMA1
#define DMA_MYUART2_RX_ENR			AHBENR
#define RCC_DMA_MYUART2_RX_EN		RCC_AHBENR_DMA1EN
#define CH_DMA_MYUART2_RX			DMA1_Channel6
#define CH_DMA_MYUART2_RX_IRQn		DMA1_Channel6_IRQn
#define DMA_MYUART2_RX_IRQHandler	DMA1_Channel6_IRQHandler
#define DMA_MYUART2_RX_CCR_MASK		(DMA_CCR6_TCIE | DMA_CCR6_HTIE | DMA_CCR6_TEIE)
//#define DMA_MYUART2_RX_ISR_MASK		(DMA_ISR_TCIF6 | DMA_ISR_HTIF6 | DMA_ISR_TEIF6 | DMA_ISR_GIF6)
//#define DMA_MYUART2_RX_IFCR_MASK	(DMA_IFCR_CTCIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTEIF6 | DMA_IFCR_CGIF6)
#define DMA_MYUART2_RX_IFCR_CGIF	DMA_IFCR_CGIF6
#define DMA_MYUART2_RX_ISR_TCIF		DMA_ISR_TCIF6
#define DMA_MYUART2_RX_CCR_EN		DMA_CCR6_EN
#define DMA_MYUART2_RX_CCR_TCIE		DMA_CCR6_TCIE
#define DMA_MYUART2_RX_CCR_HTIE		DMA_CCR6_HTIE
#define DMA_MYUART2_RX_CCR_DIR		DMA_CCR6_DIR
#define DMA_MYUART2_RX_CCR_MINC		DMA_CCR6_MINC
//-----------
#define DMA_MYUART2					DMA_MYUART2_TX
#define DMA_MYUART2_ENR				DMA_MYUART2_TX_ENR
#define RCC_DMA_MYUART2_EN			RCC_DMA_MYUART2_TX_EN

//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
// Clear all request flags
#define CLEAR_IRQ_MYUART(uart, reg)	uart->SR &= ~(USART_SR_RXNE | USART_SR_TC);			\
									reg = uart->DR;										\
									reg = uart->SR




//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------





//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
uint32_t ConfigMyUart(USART_TypeDef *, uint32_t);
uint32_t UartInit(USART_TypeDef *, TIM_TypeDef *);
void MyUartDeinit(USART_TypeDef *);


uint32_t UpdateMyUartSettings(USART_TypeDef *, uint32_t, uint32_t, uint32_t);
uint32_t ConfigDmaMyUartTx(USART_TypeDef *);
uint32_t StartDmaMyUartTx(USART_TypeDef *, uint8_t *, uint16_t *);
uint32_t StopDmaMyUartTx(USART_TypeDef *);
uint32_t StopMyUart(USART_TypeDef *);
uint32_t StartMyUartRx(USART_TypeDef *, uint16_t *);						




#endif
