//*********************************************************************************************
//                                         Usart.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "usart.h"


//---------------------------------------------------------------------------------------------
//                                     Global variable
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//------------------------------- Uart instance initialization ---------------------------------
uint32_t UartInit   (
                    USART_TypeDef *Uart_Inst,
                    TIM_TypeDef *Tim_Inst
                    )
{
	uint32_t err;

    // Init timer for timeouts (T3/T2) (1tick = 1us)
    err = ConfigMyTimer(Tim_Inst, (10 * 1000000UL / 19200));
    if(err == RESULT_OK)
    {
        // USART_RX: on
        err = ConfigMyUart( Uart_Inst, 19200);
    }
    return err;
}


//------------------------------ Configure GPIO for USART1 ------------------------------------
uint32_t ConfigMyUart	(
						USART_TypeDef *Uart_Inst,
						uint32_t baudrate
						)
{
	uint32_t Fpclk, err = RESULT_OK;

	if(Uart_Inst == MYUART1)
	{
		/* Enable the peripheral clock of GPIOA & alt. func. */
		RCC->GPIO_MYUART1_ENR |= RCC_GPIO_MYUART1_EN | RCC_APB2ENR_AFIOEN;
		SetAsZInput(RX1_Port, RX1_Pin);
		SetAsAltFuncOutput(TX1_Port, TX1_Pin, PSH_PULL, FAST);
		SetAsPpOutput(ENTX1_Port, ENTX1_Pin, 0);

		/* Enable the peripheral clock MYUART1, Fpclk2 = 72MHz*/
		RCC->MYUART1_ENR |= RCC_MYUART1_EN;

		/* Configure MYUART1 bitrate */
		/* oversampling by 16: 	Fpclk2/UART_BAUD_RATE = 0x04E2 */
		Fpclk = SystemCoreClock;

		/* */
		NVIC_SetPriority(MYUART1_IRQn, 7);
	}
	else if(Uart_Inst == MYUART2)
	{
		/* Enable the peripheral clock of GPIOA & alt. func. */
		RCC->GPIO_MYUART2_ENR |= RCC_GPIO_MYUART2_EN | RCC_APB2ENR_AFIOEN;
		SetAsZInput(RX2_Port, RX2_Pin);
		SetAsAltFuncOutput(TX2_Port, TX2_Pin, PSH_PULL, FAST);
		SetAsPpOutput(ENTX2_Port, ENTX2_Pin, 0);

		/* Enable the peripheral clock MYUART2, Fpclk1 = 36MHz*/
		RCC->MYUART2_ENR |= RCC_MYUART2_EN;

		/* Configure MYUART2 bitrate */
		/* oversampling by 16: 	Fpclk1/UART_BAUD_RATE = 0x0271 */
		Fpclk = SystemCoreClock / 2;

		/*  */
		NVIC_SetPriority(MYUART2_IRQn, 7);
	}
	else
		return INSTANCE_ERR;

	/* Configure MYUART1 bitrate */
	Uart_Inst->BRR = Fpclk / baudrate;										// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	/* UART enabled */
	Uart_Inst->CR1 |= USART_CR1_UE;

    // enable DMA for transmitter
    Uart_Inst->CR3 |= USART_CR3_DMAT;

	/* Config DMA for UART TX */
	err = ConfigDmaMyUartTx(Uart_Inst);

	return err;
}


//---------------------------------- Update Uart settings -------------------------------------
uint32_t UpdateMyUartSettings	(
								USART_TypeDef *Uart_Inst,
								uint32_t buadrate,
								uint32_t stop_bits,
								uint32_t parity_bits
								)
{
	uint32_t temp;

	if(Uart_Inst == MYUART1)
	{
//		/* Enable the peripheral clock MYUART1, Fpclk2 = 72MHz*/
//		RCC->MYUART1_ENR |= RCC_MYUART1_EN;

		/* Configure MYUART1 bitrate */
		/* oversampling by 16: 	Fpclk2/UART_BAUD_RATE = 0x04E2 */
		temp = SystemCoreClock;
	}
	else if(Uart_Inst == MYUART2)
	{
//		/* Enable the peripheral clock MYUART2, Fpclk1 = 36MHz*/
//		RCC->MYUART2_ENR |= RCC_MYUART2_EN;

		/* Configure MYUART2 bitrate */
		/* oversampling by 16: 	Fpclk1/UART_BAUD_RATE = 0x0271 */
		temp = SystemCoreClock/2;
	}
	else
		return INSTANCE_ERR;

	Uart_Inst->BRR = temp/buadrate;

	// stop-bits
	temp = Uart_Inst->CR2 & ~USART_CR2_STOP;
	Uart_Inst->CR2 = temp | ((stop_bits == STOP_BIT_1) ? 0x0000 : 0x2000);

	// parity-bits
	temp = Uart_Inst->CR1 & ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
	if(parity_bits == PARITY_BIT_EVEN)
		temp |= (USART_CR1_PCE | USART_CR1_M);
	else if(parity_bits == PARITY_BIT_ODD)
		temp |= (USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
	Uart_Inst->CR1 = temp;

	return RESULT_OK;
}


//------------------------------- Configure DMA1 for USART Tx ---------------------------------!!!
uint32_t ConfigDmaMyUartTx	(
							USART_TypeDef *Uart_Inst
							)
{
	if(Uart_Inst == MYUART1)
	{
		RCC->DMA_MYUART1_TX_ENR |= RCC_DMA_MYUART1_TX_EN;
		CH_DMA_MYUART1_TX->CCR &= ~(DMA_MYUART1_TX_CCR_EN | DMA_MYUART1_TX_CCR_MASK);
		DMA_MYUART1_TX->IFCR |= DMA_MYUART1_TX_IFCR_CGIF;
		CH_DMA_MYUART1_TX->CPAR = (uint32_t)&(MYUART1->DR);
	//  CH_DMA_MYUART1_TX->CMAR = (uint32_t)buf;
	//  CH_DMA_MYUART1_TX->CNDTR = *size;
		CH_DMA_MYUART1_TX->CCR |= (uint32_t)DMA_MYUART1_TX_CCR_DIR;
		CH_DMA_MYUART1_TX->CCR |= (uint32_t)DMA_MYUART1_TX_CCR_MINC;
		NVIC_SetPriority(CH_DMA_MYUART1_TX_IRQn, 7);
		return RESULT_OK;
	}
	else if(Uart_Inst == MYUART2)
	{
		RCC->DMA_MYUART2_TX_ENR |= RCC_DMA_MYUART2_TX_EN;
		CH_DMA_MYUART2_TX->CCR &= ~(DMA_MYUART2_TX_CCR_EN | DMA_MYUART1_TX_CCR_MASK);
		DMA_MYUART2_TX->IFCR |= DMA_MYUART2_TX_IFCR_CGIF;
		CH_DMA_MYUART2_TX->CPAR = (uint32_t)&(MYUART2->DR);
	//    CH_DMA_MYUART2_TX->CMAR = (uint32_t)buf;
	//    CH_DMA_MYUART2_TX->CNDTR = *size;
		CH_DMA_MYUART2_TX->CCR |= (uint32_t)DMA_MYUART2_TX_CCR_DIR;
		CH_DMA_MYUART2_TX->CCR |= (uint32_t)DMA_MYUART2_TX_CCR_MINC;
		NVIC_SetPriority(CH_DMA_MYUART2_TX_IRQn, 7);
		return RESULT_OK;
	}
	else
		return INSTANCE_ERR;
}


//------------------------------- DMA USART1 transmit switch-on --------------------------------
uint32_t StartDmaMyUartTx	(
							USART_TypeDef *Uart_Inst,
							uint8_t *buf,
							uint16_t *size
							)
{
	volatile uint32_t temp;

	if(Uart_Inst == MYUART1 || Uart_Inst == MYUART2)
	{
		// disable UART interrupts, reset UART interrupt flags
		Uart_Inst->CR1 &= ~(USART_CR1_TCIE | USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_IDLEIE | USART_CR1_PEIE);
		CLEAR_IRQ_MYUART(Uart_Inst, temp);

		// enable transmitter
		Uart_Inst->CR1 |= USART_CR1_TE;

		if(Uart_Inst == MYUART1)
		{
			// enable TX line
			SET_PIN(ENTX1_Port, ENTX1_Pin);

            CH_DMA_MYUART1_TX->CCR &= ~DMA_MYUART1_TX_CCR_EN;

			// point to data buffer & size
			CH_DMA_MYUART1_TX->CMAR = (uint32_t)buf;
			CH_DMA_MYUART1_TX->CNDTR = *size;

			// reset DMA interrupt flags & switch-on DMA
			NVIC_DisableIRQ(MYUART1_IRQn);
			DMA_MYUART1_TX->IFCR |= DMA_MYUART1_TX_IFCR_CGIF;
			CH_DMA_MYUART1_TX->CCR |= DMA_MYUART1_TX_CCR_TCIE;
			NVIC_EnableIRQ(CH_DMA_MYUART1_TX_IRQn);
			CH_DMA_MYUART1_TX->CCR |= DMA_MYUART1_TX_CCR_EN;
		}
		else
		{
			// enable TX line
			SET_PIN(ENTX2_Port, ENTX2_Pin);

			CH_DMA_MYUART2_TX->CCR &= ~DMA_MYUART2_TX_CCR_EN;
			// point to data buffer & size
			CH_DMA_MYUART2_TX->CMAR = (uint32_t)buf;
			CH_DMA_MYUART2_TX->CNDTR = *size;

			// reset DMA interrupt flags & switch-on DMA
			DMA_MYUART2_TX->IFCR |= DMA_MYUART2_TX_IFCR_CGIF;
			CH_DMA_MYUART2_TX->CCR |= DMA_MYUART2_TX_CCR_TCIE;
			CH_DMA_MYUART2_TX->CCR |= DMA_MYUART2_TX_CCR_EN;
			NVIC_DisableIRQ(MYUART2_IRQn);
			NVIC_EnableIRQ(CH_DMA_MYUART2_TX_IRQn);
		}
		return RESULT_OK;
	}
	else
		return INSTANCE_ERR;
}


//--------------------------------- DMA USART transmit stop ------------------------------------
uint32_t StopDmaMyUartTx(
						USART_TypeDef *Uart_Inst
						)
{
	if(Uart_Inst == MYUART1)
	{
		NVIC_DisableIRQ(CH_DMA_MYUART1_TX_IRQn);
		CH_DMA_MYUART1_TX->CCR &= ~(DMA_MYUART1_TX_CCR_EN | DMA_MYUART1_TX_CCR_MASK);
		return RESULT_OK;
	}
	else if(Uart_Inst == MYUART2)
	{
		NVIC_DisableIRQ(CH_DMA_MYUART2_TX_IRQn);
		CH_DMA_MYUART2_TX->CCR &= ~(DMA_MYUART2_TX_CCR_EN | DMA_MYUART2_TX_CCR_MASK);
		return RESULT_OK;
	}
	else
		return INSTANCE_ERR;
}



//-------------------------------------- USART start -------------------------------------------
uint32_t StartMyUartRx	(
						USART_TypeDef *Uart_Inst,
						uint16_t *rx_cnt
						)
{
	volatile uint32_t temp;

	// clear all request flags
	CLEAR_IRQ_MYUART(Uart_Inst, temp);

	if(Uart_Inst == MYUART1)
	{
		// enable RX line
		RESET_PIN(ENTX1_Port, ENTX1_Pin);
		NVIC_EnableIRQ(MYUART1_IRQn);
	}
	else if(Uart_Inst == MYUART2)
	{
		// enable RX line
		RESET_PIN(ENTX2_Port, ENTX2_Pin);
		NVIC_EnableIRQ(MYUART2_IRQn);
	}
	else
		return INSTANCE_ERR;

	// reset rx_counter
	*rx_cnt = 0;

	// enable receiving & interrupt
	Uart_Inst->CR1 |= (USART_CR1_RXNEIE | USART_CR1_RE);
	return RESULT_OK;
}


//--------------------------------------- USART stop -------------------------------------------
uint32_t StopMyUart	(
					USART_TypeDef *Uart_Inst
					)
{
	if(StopDmaMyUartTx(Uart_Inst) != RESULT_OK)
		return INSTANCE_ERR;

	if(Uart_Inst == MYUART1)
		NVIC_DisableIRQ(MYUART1_IRQn);
	else
		NVIC_DisableIRQ(MYUART2_IRQn);

	Uart_Inst->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);
	return RESULT_OK;
}


//---------------------------------- DMA deinitialization ------------------------------------
static void DmaMyUartDeinit (
                            USART_TypeDef *Uart_Inst
                            )
{
	if(Uart_Inst == MYUART1)
	{
        NVIC_DisableIRQ(CH_DMA_MYUART1_TX_IRQn);
        RCC->DMA_MYUART1_TX_ENR &= ~RCC_DMA_MYUART1_TX_EN;
    }
	else
    {
        NVIC_DisableIRQ(CH_DMA_MYUART2_TX_IRQn);
        RCC->DMA_MYUART2_TX_ENR &= ~RCC_DMA_MYUART2_TX_EN;
    }
}


//---------------------------------- USART deinitialization ------------------------------------
void MyUartDeinit   (
					USART_TypeDef *Uart_Inst
					)
{
	DmaMyUartDeinit(Uart_Inst);
    if(Uart_Inst == MYUART1)
    {
        NVIC_DisableIRQ(MYUART1_IRQn);
        RCC->MYUART1_RSTR |= RCC_MYUART1_RST;
        RCC->MYUART1_ENR &= ~RCC_MYUART1_EN;
    }
	else
    {
		NVIC_DisableIRQ(MYUART2_IRQn);
        RCC->MYUART2_RSTR |= RCC_MYUART2_RST;
        RCC->MYUART2_ENR &= ~RCC_MYUART2_EN;
    }
}





