//*********************************************************************************************
//                                      Serial_bus.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "serial_bus.h"


//---------------------------------------------------------------------------------------------
//                                    Tables of Constants
//---------------------------------------------------------------------------------------------
const uint8_t Default_Serbus0_Settings[] = {MODULE_ON, PROTOCOL_MODBUS, BAUDRATE_57600, PARITY_BIT_NO, STOP_BIT_2, 1, SINGLE_CHAR_MSG_DSBL};
const uint8_t Default_Serbus1_Settings[] = {MODULE_ON, PROTOCOL_MODBUS, BAUDRATE_57600, PARITY_BIT_NO, STOP_BIT_2, 2, SINGLE_CHAR_MSG_DSBL};



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern uint8_t						Data_Buf_Serbus0[RAM_SERBUS_BUF_SIZE];
extern uint32_t 			        error_log_Serbus0[ERR_LOG_SIZE];
extern Serbus_inst_t			    Serbus0;
extern SemaphoreHandle_t            Serbus0_DataRcvd_Semph;
extern SemaphoreHandle_t            Serbus0_DataSent_Semph;
extern SemaphoreHandle_t            Serbus0_Rqst_Semph;
extern SemaphoreHandle_t            Serbus0_Resp_Semph;

extern uint8_t						Data_Buf_Serbus1[RAM_SERBUS_BUF_SIZE];
extern uint32_t 			        error_log_Serbus1[ERR_LOG_SIZE];
extern Serbus_inst_t			    Serbus1;
extern SemaphoreHandle_t            Serbus1_DataRcvd_Semph;
extern SemaphoreHandle_t            Serbus1_DataSent_Semph;
extern SemaphoreHandle_t            Serbus1_Rqst_Semph;
extern SemaphoreHandle_t            Serbus1_Resp_Semph;

extern TaskHandle_t                 xSerbus_0_Handle;
extern TaskHandle_t                 xSerbus_1_Handle;

extern Serbus_Param_t               Serbus0_Settings;
extern Serbus_Param_t               Serbus1_Settings;


extern Eeprom_inst_t       			Eeprom_M95xxx;
extern SemaphoreHandle_t            Eep_Mutex;
extern SemaphoreHandle_t            Eep_Cmplt_Semph;

#ifdef _TICK_COUNT_ON_
extern uint32_t 		            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
#endif

extern uint32_t 	                systick_ms;

#ifdef _DEBUG_ON_
extern uint32_t                     task_enter_cnt[TASK_NUM_SIZE];
extern uint32_t                     task_stage[TASK_NUM_SIZE];
#endif


//---------------------------------------------------------------------------------------------
//                                       Functions
//---------------------------------------------------------------------------------------------
//*********************************************************************************************
//                                       Link layer
//*********************************************************************************************
//----------------------------------------------------------------------------------------------
static void SerbusStructInit(
                            Serbus_inst_t *pSerbus
                            )
{
    // Setup: instance, buffer, error log, UART default settings
	if(pSerbus == &Serbus0)
    {
        pSerbus->Serbus_Periph.Periph_Usart = MYUART1;
        pSerbus->Serbus_Periph.Periph_Timer = MYTIMER1;
        pSerbus->Serbus_Periph.Periph_Settings = &Serbus0_Settings;
        pSerbus->errors = error_log_Serbus0;
        pSerbus->Rqst_LPDU = Data_Buf_Serbus0 + RAM_SERBUS_RXBUF_OFFSET;
        pSerbus->Resp_LPDU = Data_Buf_Serbus0 + RAM_SERBUS_TXBUF_OFFSET;
        pSerbus->DataRcvd = &Serbus0_DataRcvd_Semph;
        pSerbus->DataSent = &Serbus0_DataSent_Semph;
        pSerbus->RqstToApp = &Serbus0_Rqst_Semph;
        pSerbus->RespFromApp = &Serbus0_Resp_Semph;
    }
	else
    {
        pSerbus->Serbus_Periph.Periph_Usart = MYUART2;
        pSerbus->Serbus_Periph.Periph_Timer = MYTIMER2;
        pSerbus->Serbus_Periph.Periph_Settings = &Serbus1_Settings;
        pSerbus->errors = error_log_Serbus1;
        pSerbus->Rqst_LPDU = Data_Buf_Serbus1 + RAM_SERBUS_RXBUF_OFFSET;
        pSerbus->Resp_LPDU = Data_Buf_Serbus1 + RAM_SERBUS_TXBUF_OFFSET;
        pSerbus->DataRcvd = &Serbus1_DataRcvd_Semph;
        pSerbus->DataSent = &Serbus1_DataSent_Semph;
        pSerbus->RqstToApp = &Serbus1_Rqst_Semph;
        pSerbus->RespFromApp = &Serbus1_Resp_Semph;
    }
//    pSerbus->Serbus_Periph.Periph_Settings->On_Off = MODULE_OFF;

    pSerbus->Rqst_LPCI.Frame_Size = 0;
    pSerbus->Rqst_LPCI.Byte_Counter = 0;

    pSerbus->Resp_LPCI.Frame_Size = 0;
    pSerbus->Resp_LPCI.Byte_Counter = 0;

    pSerbus->State_Flags = 0;
    pSerbus->Error_Flags = 0;
}
/**/


//----------------------- Serial bus settings reading from EEPROM ------------------------------
static void ReadingSettingsSerbus	(
                                    Serbus_Param_t *pSerbus_Settings,
                                    uint8_t *buf
                                    )
{
    // Save settings
    pSerbus_Settings->On_Off = (buf[EEPROM_SERBUS_CONTROL_OFFSET] == MODULE_OFF) ? MODULE_OFF : MODULE_ON;
    pSerbus_Settings->Protocol = (buf[EEPROM_SERBUS_PROTOCOL_OFFSET] > PROTOCOL_103) ? PROTOCOL_MODBUS : buf[EEPROM_SERBUS_PROTOCOL_OFFSET];
    pSerbus_Settings->Switch_Parity = (buf[EEPROM_SERBUS_PARITY_OFFSET] > PARITY_BIT_ODD) ? PARITY_BIT_NO : buf[EEPROM_SERBUS_PARITY_OFFSET];
    pSerbus_Settings->Address = buf[EEPROM_SERBUS_SATIONADDR_OFFSET];
    pSerbus_Settings->Additional_Param = buf[EEPROM_SERBUS_ADDPARAM_OFFSET];
    pSerbus_Settings->Switch_Stop_Bit = (buf[EEPROM_SERBUS_STOPBIT_OFFSET] > STOP_BIT_2) ? STOP_BIT_2 : buf[EEPROM_SERBUS_STOPBIT_OFFSET];

    uint32_t baudrate_list[6] = {4800UL, 9600UL, 19200UL, 38400UL, 57600UL, 115200UL};
    pSerbus_Settings->Switch_Baud_Rate = baudrate_list[buf[EEPROM_SERBUS_BAUDRATE_OFFSET]];
    pSerbus_Settings->Pause_1symbol_us = (10 + pSerbus_Settings->Switch_Stop_Bit) * 1000000UL / pSerbus_Settings->Switch_Baud_Rate;
}


//---------------------------- Check correctness of settings ------------------------------
static uint32_t CheckSettings	(
								uint8_t *buf
								)
{
	uint8_t min_val[EEPROM_SERBUS_SETTINGS_SIZE] = {0,   0,   0,   0,   0,   1,   0};
	uint8_t max_val[EEPROM_SERBUS_SETTINGS_SIZE] = {1,   1,   5,   2,   1,  247,   1};

	// check range of data
	for(int k = 0; k < EEPROM_SERBUS_SETTINGS_SIZE; k++)
	{
		if(buf[k] < min_val[k] || buf[k] > max_val[k])
			return OUT_OF_RANGE;
	}
	return RESULT_OK;
}


//------------------------------- Uart instance initialization ---------------------------------
static void SetupSerbusPeriph   (
                                Serbus_inst_t *pSerbus
                                )
{
	uint32_t err;

    // USART: stop (DMA - Off, Rx - Off, Tx - Off, Ints - Off)
    StopMyTimer(pSerbus->Serbus_Periph.Periph_Timer);
    err = StopMyUart(pSerbus->Serbus_Periph.Periph_Usart);
    if(err == RESULT_OK)
    {
        // Load new UART settings (amount of stop-bits, parity bit, baudrate)
        err = UpdateMyUartSettings(	pSerbus->Serbus_Periph.Periph_Usart,
                                    pSerbus->Serbus_Periph.Periph_Settings->Switch_Baud_Rate,
                                    pSerbus->Serbus_Periph.Periph_Settings->Switch_Stop_Bit,
                                    pSerbus->Serbus_Periph.Periph_Settings->Switch_Parity);
        if(err == RESULT_OK)
        {
            // Init timer for timeouts (T3/T2) (1tick = 1us)
            UpdateMyTimerSettings(  pSerbus->Serbus_Periph.Periph_Timer,
                                    pSerbus->Serbus_Periph.Periph_Settings->Pause_1symbol_us);
            // USART_RX: on
            err = FrameRxStartHandler(pSerbus);
            if(err == RESULT_OK)
            {
                // Flags: reset
                pSerbus->State_Flags = 0;
                pSerbus->Error_Flags = 0;
                return;
            }
        }
    }
    pSerbus->errors[err]++;
    CLEAR_FLAG(pSerbus->State_Flags, f_Serbus_Init);
    SET_FLAG(pSerbus->Error_Flags, f_Err_Init);
}
/**/

//---------------------------- Serial bus instance initialization ------------------------------
void GetSerbusSettings  (
                        Serbus_Param_t *pSerbus_Setting
                        )
{
    uint32_t addr; //static
    uint8_t temp_arr[EEPROM_SERBUS_SETTINGS_SIZE]; //static

    // read serial bus on-off state from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);

    addr = (pSerbus_Setting == &Serbus0_Settings) ? EEPROM_SERBUS0_ADDR : EEPROM_SERBUS1_ADDR;

    // start reading from EEPROM
    ReadEeprom(addr, EEPROM_SERBUS_SETTINGS_SIZE, temp_arr);
    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        CLEAR_FLAG(pSerbus_Setting->Flags, f_Err_Settings_Read);

        // Check read settings
        if(CheckSettings(temp_arr) != RESULT_OK)
        {
            // Load default settings
            if(pSerbus_Setting == &Serbus0_Settings)
                memcpy(temp_arr, Default_Serbus0_Settings, EEPROM_SERBUS_SETTINGS_SIZE);
            else
                memcpy(temp_arr, Default_Serbus1_Settings, EEPROM_SERBUS_SETTINGS_SIZE);
        }
        ReadingSettingsSerbus(pSerbus_Setting, temp_arr);
//        SetupSerbusPeriph(pSerbus);
//        SET_FLAG(pSerbus->State_Flags, f_Serbus_Init);
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        SET_FLAG(pSerbus_Setting->Flags, f_Err_Settings_Read);
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);
    // data already has been read from EEPROM
}

/*
//---------------------------- Serial bus instance turn-on ------------------------------
void TurnOnSerbus	(
					Serbus_inst_t *pSerbus
					)
{
    static uint32_t addr;
    static uint8_t temp;

    // read serial bus on-off state from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);

    // stop usart & timer
    StopMyTimer(pSerbus->Serbus_Periph.Periph_Timer);
    StopMyUart(pSerbus->Serbus_Periph.Periph_Usart);
    // start reading from EEPROM
    addr = (pSerbus == &Serbus0) ? EEPROM_SERBUS0_ADDR : EEPROM_SERBUS1_ADDR;
    ReadEeprom(addr, 1, &temp);
    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        CLEAR_FLAG(pSerbus->Error_Flags, f_Err_Settings_Read);

        // Check read settings
        if(temp == 1)
            pSerbus->Serbus_Periph.Periph_Settings.On_Off = MODULE_ON;
        else
            pSerbus->Serbus_Periph.Periph_Settings.On_Off = MODULE_OFF;
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        SET_FLAG(pSerbus->Error_Flags, f_Err_Settings_Read);
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);
    // data already has been read from EEPROM
}
*/

//---------------------------- Serial bus instance task ------------------------------
void vSerbusTask(
                void *pvParameters
                )
{
    uint32_t res;
    Serbus_inst_t *pSerbus;

    pSerbus = (Serbus_inst_t *)pvParameters;

#ifdef _DEBUG_ON_
    uint32_t task_num;
    if(pSerbus == &Serbus0)
        task_num = SERBUS0_TASK_NUM;
    else
        task_num = SERBUS1_TASK_NUM;
#endif

    for(;;)
	{
#ifdef _DEBUG_ON_
			task_stage[task_num] = 0;
			++task_enter_cnt[task_num];
#endif
			// serial bus processing
//			if(
            xSemaphoreTake(*(pSerbus->DataRcvd), portMAX_DELAY);
//             == pdPASS)
//			{
#ifdef _DEBUG_ON_
				task_stage[task_num] = 1;
#endif
                // transfer frame to application (Modbus or IEC103) and wait for response
                if( pSerbus->Serbus_Periph.Periph_Settings->Protocol == PROTOCOL_MODBUS
                    &&
                    (pSerbus->Rqst_LPDU[0] == pSerbus->Serbus_Periph.Periph_Settings->Address || pSerbus->Rqst_LPDU[0] == 0) )
                {
                    SET_FLAG(pSerbus->State_Flags, f_Modbus_Frame_Received);
                }
                else if(pSerbus->Serbus_Periph.Periph_Settings->Protocol == PROTOCOL_103
                        &&
                        (pSerbus->Rqst_LPDU[0] == 0x10 || pSerbus->Rqst_LPDU[0] == 0x68) )
                {
                    SET_FLAG(pSerbus->State_Flags, f_Iec103_Frame_Received);
                }
                else
                {
                    CLEAR_FLAG(pSerbus->State_Flags, f_Modbus_Frame_Received);
                    CLEAR_FLAG(pSerbus->State_Flags, f_Iec103_Frame_Received);
                }

				if(TEST_FLAG(pSerbus->State_Flags, f_Modbus_Frame_Received) || TEST_FLAG(pSerbus->State_Flags, f_Iec103_Frame_Received))
				{
					xSemaphoreGive(*(pSerbus->RqstToApp));
					if(xSemaphoreTake(*(pSerbus->RespFromApp), MODBUS_RESP_TIMEOUT) == pdPASS)
					{
#ifdef _DEBUG_ON_
						task_stage[task_num] = 2;
#endif
						if(pSerbus->Resp_LPCI.Frame_Size > 0 && pSerbus->Resp_LPCI.Frame_Size <= 256)
							res = RESULT_OK;
						else
							res = NOT_READY;
					}
					else
						res = NOT_READY+1;
				}
				else
					res = NOT_READY+2;

				// check response from application ()
				if(res == RESULT_OK)
				{
					// it means we have data for transmission in Serbus0.Resp_LPDU and tx_size in Serbus0.Resp_LPCI.Frame_Size
					// start transmission
					FrameTxStartHandler(pSerbus);
				}
				else
				{
					// if no response more than __ms - restart receiving
					res = FrameRxStartHandler(pSerbus);
				}
//			}
    }
}


//---------------------------------------------------------------------------------------------
BaseType_t CreateSerbusTask (
                            Serbus_inst_t *pSerbus
                            )
{
    const char *task_name;
    TaskHandle_t *task_handle;
    const char task0[] = "Serbus0";
    const char task1[] = "Serbus1";

    if(pSerbus == &Serbus0)
    {
        UartInit(MYUART1, MYTIMER1);

        Serbus0_DataRcvd_Semph = xSemaphoreCreateBinary();
        Serbus0_DataSent_Semph = xSemaphoreCreateBinary();
        Serbus0_Rqst_Semph = xSemaphoreCreateBinary();
        Serbus0_Resp_Semph = xSemaphoreCreateBinary();
        task_handle = &xSerbus_0_Handle;
        task_name = task0;
    }
    else if(pSerbus == &Serbus1)
    {
        UartInit(MYUART2, MYTIMER2);

        Serbus1_DataRcvd_Semph = xSemaphoreCreateBinary();
        Serbus1_DataSent_Semph = xSemaphoreCreateBinary();
        Serbus1_Rqst_Semph = xSemaphoreCreateBinary();
        Serbus1_Resp_Semph = xSemaphoreCreateBinary();
        task_handle = &xSerbus_1_Handle;
        task_name = task1;
    }
    else
        return pdFALSE;

    SerbusStructInit(pSerbus);

    SetupSerbusPeriph(pSerbus);

    SET_FLAG(pSerbus->State_Flags, f_Serbus_Init);

    return xTaskCreate( vSerbusTask,
                        task_name,
                        configMINIMAL_STACK_SIZE,
                        (void *)pSerbus,
                        (tskIDLE_PRIORITY + 2),
                        task_handle		);
}



//**********************************************************************************************
//                                   Interrupt procedures
//**********************************************************************************************
//----------------------- New Frame Receive Start Handler ----------------------
uint32_t FrameRxStartHandler(
                            Serbus_inst_t *pSerbus
                            )
{
    uint32_t err;

    StopMyTimer(pSerbus->Serbus_Periph.Periph_Timer);
    err = StartMyUartRx(pSerbus->Serbus_Periph.Periph_Usart,
                        &pSerbus->Rqst_LPCI.Byte_Counter    );
    if(err != RESULT_OK)
        pSerbus->errors[err]++;
    return err;
}


//---------------------- Received Symbol Handler (executes in UART interrupt) -----------------------
void ReceivedSymbolHandler  (
                            Serbus_inst_t *pSerbus
                            )
{
    uint32_t err;

    // symbol received
    // reset counter
    pSerbus->Serbus_Periph.Periph_Timer->CNT = 0;

    if(pSerbus->Rqst_LPCI.Byte_Counter == 0)
    {
        pSerbus->Time_Stamp = (SystemCoreClock / configTICK_RATE_HZ) * systick_ms + SysTick->VAL;

        pSerbus->Rqst_LPCI.Frame_Size = 0;

        // start timer
        StartMyTimer(pSerbus->Serbus_Periph.Periph_Timer);
        UPDATE_MYTIMER(pSerbus->Serbus_Periph.Periph_Timer);
    }
    // save data
    pSerbus->Rqst_LPDU[pSerbus->Rqst_LPCI.Byte_Counter++] = (uint8_t)pSerbus->Serbus_Periph.Periph_Usart->DR;

    // Check buffer overflow
    if(pSerbus->Rqst_LPCI.Byte_Counter > (RAM_SERBUS_RXBUF_SIZE - 1))
    {
        SET_FLAG(pSerbus->Error_Flags, f_Err_Buf_Overflow_Rx);
        pSerbus->errors[BUFOVER_ERR]++;
        // restart RX
        err = FrameRxStartHandler(pSerbus);
        if(err != RESULT_OK)
        {
            SET_FLAG(pSerbus->Error_Flags, f_Err_Frame_Rx_Start);
            pSerbus->errors[err]++;
        }
    }

    // if frame alredy has received, but one more symbol received after this - frame is wrong, need for start again
    if(TEST_FLAG(pSerbus->State_Flags, f_New_Frame_Received))
    {
        CLEAR_FLAG(pSerbus->State_Flags, f_New_Frame_Received);
        SET_FLAG(pSerbus->Error_Flags, f_Err_Wrong_Frame);
        pSerbus->errors[WRONG_FRAME_ERR]++;
        // restart RX
        err = FrameRxStartHandler(pSerbus);
        if(err != RESULT_OK)
        {
            SET_FLAG(pSerbus->Error_Flags, f_Err_Frame_Rx_Start);
            pSerbus->errors[err]++;
        }
    }
}


//----------------------- USART interrupt handler (executes in UART interrupt) ----------------------
void MyUartErrHandler	(
                        Serbus_inst_t *pSerbus
						)
{
	uint32_t temp;

    temp = pSerbus->Serbus_Periph.Periph_Usart->SR & (USART_SR_NE | USART_SR_FE | USART_SR_PE | USART_SR_ORE);
	if(temp & USART_SR_ORE)
		pSerbus->errors[OVERRUN_ERR]++;
	if(temp & USART_SR_NE)
		pSerbus->errors[NOISE_ERR]++;
	if(temp & USART_SR_FE)
		pSerbus->errors[FRAME_ERR]++;
	if(temp & USART_SR_PE)
		pSerbus->errors[PARITY_ERR]++;
}


//--------------------- Received Frame Handler (executes in TIM interrupt) -------------------------
void ReceivedFrameHandler   (
                            Serbus_inst_t *pSerbus
                            )
{
//    static portBASE_TYPE xTaskWokenSerRx0 = pdFALSE;
//    static portBASE_TYPE xTaskWokenSerRx1 = pdFALSE;
//
//    portBASE_TYPE *pxTaskWokenSerRx;
//    if(pSerbus == &Serbus0)
//        pxTaskWokenSerRx = &xTaskWokenSerRx0;
//    else
//        pxTaskWokenSerRx = &xTaskWokenSerRx1;

    // Process RS485 (0..1)
    // Disable receiving & RX interrupt
    pSerbus->Serbus_Periph.Periph_Usart->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_RE);

    // stop timer
    StopMyTimer(pSerbus->Serbus_Periph.Periph_Timer);

    // Clear flag NEW_FRAME_RECEIVED
    CLEAR_FLAG(pSerbus->State_Flags, f_New_Frame_Received);

    pSerbus->Rqst_LPCI.Frame_Size = pSerbus->Rqst_LPCI.Byte_Counter;
    xSemaphoreGiveFromISR(*(pSerbus->DataRcvd), NULL);  //pxTaskWokenSerRx);
//    portYIELD_FROM_ISR(*pxTaskWokenSerRx);
}


//----------------------- New Frame Transmit Start Handler ----------------------
void FrameTxStartHandler(
                        Serbus_inst_t *pSerbus
                        )
{
	uint32_t err;

    // stop timer
    StopMyTimer(pSerbus->Serbus_Periph.Periph_Timer);

    // Start TX
    err = StartDmaMyUartTx(	pSerbus->Serbus_Periph.Periph_Usart,
                            pSerbus->Resp_LPDU,
                            &pSerbus->Resp_LPCI.Frame_Size );
    if(err != RESULT_OK)
    {
        pSerbus->errors[INSTANCE_ERR]++;
        // set error flag (if need)
        SET_FLAG(pSerbus->Error_Flags, f_Err_Init);
    }
}


//------------------- DMA Transmission Frame Handler (executes in DMA interrupt) -------------------
void DmaTransmissionFrameHandler(
                                Serbus_inst_t *pSerbus
                                )
{
    if(StopDmaMyUartTx(pSerbus->Serbus_Periph.Periph_Usart) != RESULT_OK)
    {
        pSerbus->errors[INSTANCE_ERR]++;
        // set error flag (if need)
    }
    // Enable TC IRQ for transmission end
    pSerbus->Serbus_Periph.Periph_Usart->CR1 |= USART_CR1_TCIE;
	if(pSerbus->Serbus_Periph.Periph_Usart == MYUART1)
		NVIC_EnableIRQ(MYUART1_IRQn);
	else if(pSerbus->Serbus_Periph.Periph_Usart == MYUART2)
		NVIC_EnableIRQ(MYUART2_IRQn);
	else
        pSerbus->errors[INSTANCE_ERR]++;
        // set error flag (if need)
}


//------------------- Transmission Frame End Handler (executes in UART interrupt) -------------------
void TransmissionFrameEndHandler(
                                Serbus_inst_t *pSerbus
                                )
{
    uint32_t err;

    // response transmission is finished -> start RX
    // stop transmit, reset TC IRQ enable
    pSerbus->Serbus_Periph.Periph_Usart->CR1 &= ~(USART_CR1_TE | USART_CR1_TCIE);

    // start RX
    err = FrameRxStartHandler(pSerbus);
    if(err != RESULT_OK)
        pSerbus->errors[err]++;
}


