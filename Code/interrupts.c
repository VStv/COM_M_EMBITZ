//*********************************************************************************************
//                                       Interrupts.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "interrupts.h"



//---------------------------------------------------------------------------------------------
//                                        Constants
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
#ifdef _TICK_COUNT_ON_
extern uint32_t 		            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
#endif



extern Serbus_inst_t			    Serbus0;
extern Serbus_inst_t			    Serbus1;

extern rtc_strct_t			        Rtc_fact;
extern Sync_Inst_t                  Sync_Data;

#ifdef _FLASH_IRQ_ON_
extern Flash_struc_t                Flash_struc;
#endif




//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------
//                                     Interrupt handlers
//---------------------------------------------------------------------------------------------

void HardFault_Handler(void)
{
#ifdef _DEBUG_ON_
    asm(
	      "TST lr, #4 \n"           //Тестируем 3ий бит указателя стека
	      "ITE EQ \n"               //Значение указателя стека имеет бит 3?
	      "MRSEQ %[ptr], MSP  \n"   //Да, сохраняем основной указатель стека
	      "MRSNE %[ptr], PSP  \n"   //Нет, сохраняем указатель стека процесса
	      : [ptr] "=r" (stack_ptr)
        );
#endif
    while(1)
    {
#ifdef _DEBUG_ON_

#else
        NVIC_SystemReset();
#endif
    }
}


//---------------------------------------------------------------------------------------------
void UsageFault_Handler(void)
{
    while(1)
    {
#ifdef _DEBUG_ON_

#else
        NVIC_SystemReset();
#endif
    }
}


////---------------------------------------------------------------------------------------------
//void Default_Handler(void)
//{
//	NVIC_SystemReset();
//}


//---------------------------------------------------------------------------------------------
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM1->SR & TIM_SR_UIF)
	{
		// clear interrupt flag
		TIM1->SR &= ~TIM_SR_UIF;
        SoftRtcUpdateHandler(&Rtc_fact, Sync_Data.Sync_Period);
	}
}


//------------------------------------ EXTI9..5 handler ---------------------------------------
void MYEXTI2_IRQHandler(void)
{
	if(EXTI->PR & ((uint32_t)(1<<MYEXTI2_Pin)))
	{
		EXTI->PR = (uint32_t)(1<<MYEXTI2_Pin);
        ParbusLinkTxHandler();
	}

	if(EXTI->PR & ((uint32_t)(1<<MYEXTI3_Pin)))
	{
		EXTI->PR = (uint32_t)(1<<MYEXTI3_Pin);
        ParbusLinkRxHandler();
	}
}


//------------------------------------ EXTI2 handler ---------------------------------------
void MYEXTI4_IRQHandler(void)
{
	if(EXTI->PR & ((uint32_t)(1<<MYEXTI4_Pin)))
	{
		EXTI->PR = (uint32_t)(1<<MYEXTI4_Pin);

        uint8_t resetEn = 1;
		for(uint8_t i = 0; i < 25; i++)
        {
            if(!TEST_PIN(ENCOM1_Port, ENCOM1_Pin))
            {
                DelayDWT_us(10);
            }
            else
            {
                resetEn = 0;
                break;
            }
        }
        if(resetEn == 1)
        {
            NVIC_SystemReset();
        }
	}
}


//---------------------------------- SPI handler (RXNE) ---------------------------------------
void MYSPI1_IRQHandler(void)
{
	if(MYSPI1->SR & SPI_SR_RXNE)
	{
		ByteMySpiRxHandler();
    }
}


//---------------------------- USART interrupt handler (Rx, Idle) -----------------------------
void MYUART1_IRQHandler(void)
{
    volatile uint32_t temp;

	if(Serbus0.Serbus_Periph.Periph_Usart->SR & USART_SR_RXNE)
	{
		// clear interrupt flag
		Serbus0.Serbus_Periph.Periph_Usart->SR &= ~USART_SR_RXNE;
		ReceivedSymbolHandler(&Serbus0);
	}

	if(Serbus0.Serbus_Periph.Periph_Usart->SR & USART_SR_TC)
	{
		// clear interrupt flag
		Serbus0.Serbus_Periph.Periph_Usart->SR &= ~USART_SR_TC;
        TransmissionFrameEndHandler(&Serbus0);
	}

	if(Serbus0.Serbus_Periph.Periph_Usart->SR & (USART_SR_NE | USART_SR_FE | USART_SR_PE | USART_SR_ORE))
	{
		MyUartErrHandler(&Serbus0);
		// clear all request flags
		CLEAR_IRQ_MYUART(Serbus0.Serbus_Periph.Periph_Usart, temp);
	}
}


//------------------------------ DMA1 Channel4 handler (Tx) -----------------------------------
void DMA_MYUART1_TX_IRQHandler(void)
{
	if(DMA_MYUART1_TX->ISR & DMA_MYUART1_TX_ISR_TCIF)
	{
        // clear all request flags
        DMA_MYUART1_TX->IFCR |= DMA_MYUART1_TX_IFCR_CGIF;

		DmaTransmissionFrameHandler(&Serbus0);
    }
}


//------------------------------ MYTIMER1 interrupt handler () ---------------------------------
void MYTIMER1_IRQHandler(void)
{
	if(Serbus0.Serbus_Periph.Periph_Timer->SR & TIM_SR_CC1IF)
	{
        // T = 1.5symbols matched
		// clear interrupt flag
		Serbus0.Serbus_Periph.Periph_Timer->SR &= ~TIM_SR_CC1IF;
        SET_FLAG(Serbus0.State_Flags, f_New_Frame_Received);
	}

	if(Serbus0.Serbus_Periph.Periph_Timer->SR & TIM_SR_CC2IF)
	{
		// T = 3.5symbols matched
		// clear interrupt flag
		Serbus0.Serbus_Periph.Periph_Timer->SR &= ~TIM_SR_CC2IF;
        // transfer data on application layer
        ReceivedFrameHandler(&Serbus0);
	}
}


//---------------------------- USART interrupt handler (Rx, Idle) -----------------------------
void MYUART2_IRQHandler(void)
{
    volatile uint32_t temp;

	if(Serbus1.Serbus_Periph.Periph_Usart->SR & USART_SR_RXNE)
	{
		// clear interrupt flag
		Serbus1.Serbus_Periph.Periph_Usart->SR &= ~USART_SR_RXNE;
		ReceivedSymbolHandler(&Serbus1);
	}

	if(Serbus1.Serbus_Periph.Periph_Usart->SR & USART_SR_TC)
	{
		// clear interrupt flag
		Serbus1.Serbus_Periph.Periph_Usart->SR &= ~USART_SR_TC;
        TransmissionFrameEndHandler(&Serbus1);
	}

	if(Serbus1.Serbus_Periph.Periph_Usart->SR & (USART_SR_NE | USART_SR_FE | USART_SR_PE | USART_SR_ORE))
	{
		MyUartErrHandler(&Serbus1);
		// clear all request flags
		CLEAR_IRQ_MYUART(Serbus1.Serbus_Periph.Periph_Usart, temp);
	}
}


//------------------------------ DMA1 Channel7 handler (Tx) -----------------------------------
void DMA_MYUART2_TX_IRQHandler(void)
{
	if(DMA_MYUART2_TX->ISR & DMA_MYUART2_TX_ISR_TCIF)
	{
        // clear all request flags
        DMA_MYUART2_TX->IFCR |= DMA_MYUART2_TX_IFCR_CGIF;
        DmaTransmissionFrameHandler(&Serbus1);
    }
}


//----------------------------- MYTIMER1 interrupt handler () ---------------------------------
void MYTIMER2_IRQHandler(void)
{
	if(Serbus1.Serbus_Periph.Periph_Timer->SR & TIM_SR_CC1IF)
	{
        // T = 1.5 symbols matched
		// clear interrupt flag
		Serbus1.Serbus_Periph.Periph_Timer->SR &= ~TIM_SR_CC1IF;
        SET_FLAG(Serbus1.State_Flags, f_New_Frame_Received);
	}

	if(Serbus1.Serbus_Periph.Periph_Timer->SR & TIM_SR_CC2IF)
	{
		// T = 3.5 symbols matched
		// clear interrupt flag
		Serbus1.Serbus_Periph.Periph_Timer->SR &= ~TIM_SR_CC2IF;
        // transfer data on application layer
        ReceivedFrameHandler(&Serbus1);
	}
}


#ifdef _FLASH_IRQ_ON_
//------------------------------- Flash Interrupt handler -------------------------------------
void FLASH_IRQHandler(void)
{
	if(FLASH->SR & FLASH_SR_EOP)
	{
		FLASH->SR |= FLASH_SR_EOP;
        FlashProgHandler(&Flash_struc);
    }
	if(FLASH->SR2 & FLASH_SR_EOP)
	{
		FLASH->SR2 |= FLASH_SR_EOP;
		FlashProgHandler(&Flash_struc);
    }
}
#endif



