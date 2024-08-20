//*********************************************************************************************
//                                          Spi.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "spi.h"




//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//----------------------------------- Config GPIO for SPI -------------------------------------
static void ConfigMySpiGpio(void)
{
    /* Enable the peripheral clock of GPIOA & alt. func. */
    RCC->GPIO_MYSPI1_ENR |= RCC_GPIO_MYSPI1_EN | RCC_APB2ENR_AFIOEN;
	SetAsInput(MISO1_Port, MISO1_Pin, 1);
//	__disable_irq();
	SetAsAltFuncOutput(MOSI1_Port, MOSI1_Pin, PSH_PULL, FAST);
	SetAsAltFuncOutput(SCK1_Port, SCK1_Pin, PSH_PULL, FAST);
	SetAsPpOutput(CSS1_Port, CSS1_Pin, 1);
}


//---------------------------------- Config SPI as master -------------------------------------
static void ConfigMySpiMaster(void)
{
	/* Enable the peripheral clock SPI */
    RCC->MYSPI1_ENR |= RCC_MYSPI1_EN;

	/* Configure SPI in master */
	/* (1) Master selection, BR: Fpclk/8 (SPI_CLK is set to 72/8 MHz, CPOL and CPHA at zero (rising first edge) */
	/* (2) TX and RX with DMA, slave select output enabled, RXNE IT, 8-bit Rx fifo */
	/* (3) Enable SPI */
	MYSPI1->CR1 = 	SPI_CR1_MSTR    |
					SPI_CR1_SSI		|
					SPI_CR1_SSM		|
//					SPI_CR1_BR_1    |
//					SPI_CR1_BR_2    |
                    SPI_CR1_BR_0; 																/* (1) */
	MYSPI1->CR1 |= 	SPI_CR1_SPE; 																/* (3) */
	MYSPI1->CR2 |= 	SPI_CR2_RXNEIE;
    /* Configure IT 					*/
    /* (4) Set priority for SPI1_IRQn 	*/
    /* (5) Enable SPI1_IRQn 			*/
    NVIC_SetPriority(MYSPI1_IRQn, 7);                                                           /* (4) */
    NVIC_EnableIRQ(MYSPI1_IRQn);                                                                /* (5) */
}


//----------------------------------- SPI initialization --------------------------------------
void MySpiInit(void)
{
    ConfigMySpiGpio();
	ConfigMySpiMaster();
//	__disable_irq();
}


//--------------------------------------- SPI deinit -------------------------------------------
void MySpiDeinit(void)
{
	NVIC_DisableIRQ(MYSPI1_IRQn);
    RCC->MYSPI1_RSTR |= RCC_MYSPI1_RST;
    RCC->MYSPI1_ENR &= ~RCC_MYSPI1_EN;
}



//---------------------------------------------------------------------------------------------
//                                     Interrupt handlers
//---------------------------------------------------------------------------------------------


