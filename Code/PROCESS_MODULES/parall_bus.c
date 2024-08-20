//*********************************************************************************************
//                                      Parall_bus.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "parall_bus.h"


//---------------------------------------------------------------------------------------------
//                                    Tables of Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
#ifdef _TICK_COUNT_ON_
extern uint32_t 		            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
#endif

extern SemaphoreHandle_t            Parbus_Semph;

extern Parbus_inst_t	            Parbus;
extern uint8_t                      Parbus_Data_Buf[RAM_PARBUS_BUF_SIZE];
extern uint8_t			    	    Ied_Data_Buf[RAM_IED_BUF_SIZE];
extern SemaphoreHandle_t            Parbus_Mutex;
extern SemaphoreHandle_t            Parbus_DataRcvd_Semph;
extern SemaphoreHandle_t            Parbus_To_Modbus_Rqst_Semph;
extern SemaphoreHandle_t            Modbus_To_Parbus_Resp_Semph;


extern SemaphoreHandle_t            Modbus_To_Parbus_RetransRqst_Semph;
extern SemaphoreHandle_t            Parbus_To_Modbus_RetransResp_Semph;
extern SemaphoreHandle_t            VDI_To_Parbus_Semph;

extern TaskHandle_t                 xParbusHandle;

extern rtc_strct_t			        Rtc_fact;


#ifdef _IEC103_ON_
extern Serbus_inst_t	            Serbus0;
extern Serbus_inst_t	            Serbus1;
extern Iec103_inst_t		        Iec103_0;
extern Iec103_inst_t		        Iec103_1;
#endif

#ifdef _DEBUG_ON_
extern uint32_t                     task_enter_cnt[TASK_NUM_SIZE];
extern uint32_t                     task_stage[TASK_NUM_SIZE];
extern uint32_t                     pb_reset_cnt;
#endif

//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
//                                       Link layer
//---------------------------------------------------------------------------------------------
//--------------------------- Parallel bus transfer initialization -----------------------------
void ParbusPeriphInit(void)
{
	ExtiInit();
//	SetAsInput(ENCOM1_Port, ENCOM1_Pin, 1);
	SetAsPpOutput(OUTCPU2_Port, OUTCPU2_Pin, 1);
	SetAsPpOutput(CSOUT2_Port, CSOUT2_Pin, 1);

	SetAsPpOutput(OECOM2_Port, OECOM2_Pin, 1);
	SetAsPpOutput(DIRCOM2_Port, DIRCOM2_Pin, 0);
	PARBUS_INPUT();
}


//----------------------------------- Parallel bus reset ---------------------------------------
static void ParbusReset(void)
{
	// Reset Buffer Direction, Unselect Buffer (Reset->DIR, Set->EN)
	RESET_PIN(DIRCOM2_Port, DIRCOM2_Pin);
    SET_PIN(OECOM2_Port, OECOM2_Pin);

	// Set lines CS_OUT, OUT_CPU
    SET_PIN(CSOUT2_Port, CSOUT2_Pin);
    SET_PIN(OUTCPU2_Port, OUTCPU2_Pin);

    // Disable interruption CS_IN2, INCPU2
	EXTI->IMR &= ~(uint32_t)(1<<CSIN2_Pin | 1<<INCPU2_Pin);
	EXTI->PR = (uint32_t)(1<<CSIN2_Pin | 1<<INCPU2_Pin);
	NVIC_EnableIRQ(CSIN2_IRQn);
	NVIC_EnableIRQ(INCPU2_IRQn);

	// Set Input Direction (bit 0...7)
	PARBUS_INPUT();

    Parbus.State_Flags = 0;
}


//---------------------------------- Parallel bus transfer -------------------------------------
static void StartTxData (
                        uint8_t byte
                        )
{
    Parbus.Rqst_LPCI.Byte_Counter = 0;

	// Setting the data lines as output (bit 0...7)
    PARBUS_OUTPUT();
    PARALL_BUS_TX_BYTE(byte);

    // Enable interruption CS_IN2
	EXTI->PR = (uint32_t)(1<<CSIN2_Pin);														// activate EXTIx interrupt
	EXTI->IMR |= (uint32_t)(1<<CSIN2_Pin);

	// Set the direction of the buffer and enable the buffer
    RESET_PIN(OECOM2_Port, OECOM2_Pin);
    SET_PIN(DIRCOM2_Port, DIRCOM2_Pin);
	RESET_PIN(CSOUT2_Port, CSOUT2_Pin);
}


//---------------------------------------------------------------------------------------------
static uint32_t CheckCrcParbus	(
                                uint8_t *frame
                                )
{
    Parbus_APDU_t *pAPDU;

    pAPDU = (Parbus_APDU_t *)frame;
    if(pAPDU->ASDU[pAPDU->ASDU_Size] != CalcOrdCRC((uint8_t *)pAPDU, pAPDU->ASDU_Size + 2, 0))
		return CRC_ERR;
	return RESULT_OK;
}


//---------------------------------------------------------------------------------------------
void vParbusTask(
                void *pvParameters
                )
{
    uint32_t flag = 0;

    vTaskDelay(10);

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[PARBUS_TASK_NUM] = 0;
        ++task_enter_cnt[PARBUS_TASK_NUM];
#endif

            xSemaphoreTake(*(Parbus.ProcessEnbld), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[PARBUS_TASK_NUM] = 1;
#endif
            // we works with buf1 (instead Parbus.Rqst_LPDU)
            // and buf1_data_size (instead Parbus.Rqst_LPCI.Frame_Size)

            // check data to CPU
            // We got data & now we have to transfer it to CPU via parallel bus request
            if(xSemaphoreTake(*(Parbus.RespFromApp), 0) == pdPASS)
            {
#ifdef _DEBUG_ON_
                task_stage[PARBUS_TASK_NUM] = 12;
#endif
                CLEAR_FLAG(Parbus.State_Flags, f_Retrans_Rqst_From_Modbus);
                Parbus.Rqst_LPDU[0] = 0x44;
            }
            else if(xSemaphoreTake(Modbus_To_Parbus_RetransRqst_Semph, 0) == pdPASS)
            {
#ifdef _DEBUG_ON_
                task_stage[PARBUS_TASK_NUM] = 22;
#endif
                SET_FLAG(Parbus.State_Flags, f_Retrans_Rqst_From_Modbus);
                Parbus.Rqst_LPDU[0] = 0x44;
            }
            else if(xSemaphoreTake(VDI_To_Parbus_Semph, 0) == pdPASS)
            {
#ifdef _DEBUG_ON_
                task_stage[PARBUS_TASK_NUM] = 32;
#endif
                Parbus.Rqst_LPDU[0] = 0x22;
#ifdef _IEC103_ON_
                xSemaphoreGive(*(Iec103_0.VDI_From_Parbus));
                xSemaphoreGive(*(Iec103_1.VDI_From_Parbus));
#endif
            }

#ifdef _TICK_COUNT_ON_
            DWT_TICKCNT_RST();
#endif
            // Finalize frame
            portENTER_CRITICAL(); //vTaskSuspendAll();
            switch(Parbus.Rqst_LPDU[0])
            {
                case 0x22:
                    Parbus.Rqst_LPDU[1] = RAM_VIRT_DAT_SIZE;
                    memcpy( Parbus.Rqst_LPDU + 2,
                            Ied_Data_Buf + RAM_VIRT_DAT_OFFSET,
                            Parbus.Rqst_LPDU[1]    );
					((Parbus_APDU_t *)Parbus.Rqst_LPDU)->ASDU[Parbus.Rqst_LPDU[1]] = CalcOrdCRC(Parbus.Rqst_LPDU, Parbus.Rqst_LPDU[1] + 2, 0);
                    Parbus.Rqst_LPCI.Frame_Size = Parbus.Rqst_LPDU[1] + 3;
                    break;

                case 0x44:
                    Parbus.Rqst_LPDU[1] = Ied_Data_Buf[RAM_DATA_FOR_CPU_SIZE_OFFSET];
                    memcpy( Parbus.Rqst_LPDU + 2,
                            Ied_Data_Buf + RAM_DATA_FOR_CPU_OFFSET,
                            Parbus.Rqst_LPDU[1]    );
					((Parbus_APDU_t *)Parbus.Rqst_LPDU)->ASDU[Parbus.Rqst_LPDU[1]] = CalcOrdCRC(Parbus.Rqst_LPDU, Parbus.Rqst_LPDU[1] + 2, 0);
                    Parbus.Rqst_LPCI.Frame_Size = Parbus.Rqst_LPDU[1] + 3;
                    break;

                case 0x33:
                case 0x55:
                case 0x77:
                case 0x88:
                    Parbus.Rqst_LPCI.Frame_Size = 1;
                    break;

                default:
                    Parbus.Rqst_LPDU[0] = 0x11;
                    Parbus.Rqst_LPCI.Frame_Size = 1;
                    break;
            }
            portEXIT_CRITICAL(); //xTaskResumeAll();

            StartTxData(Parbus.Rqst_LPDU[0]);

			if(Parbus.Rqst_LPDU[0] != 0x22 && Parbus.Rqst_LPDU[0] != 0x44)
			{
#ifdef _DEBUG_ON_
                task_stage[PARBUS_TASK_NUM] = 2;
#endif
				if(xSemaphoreTake(*(Parbus.DataRcvd), PARBUS_RESP_TIMEOUT) == pdPASS)
				{
#ifdef _TICK_COUNT_ON_
                    DWT_TICKCNT_RD(tmp_cycles_count_arr, cycles_count_arr_indx, cycles_count_max);
#endif

#ifdef _DEBUG_ON_
                    task_stage[PARBUS_TASK_NUM] = 3;
#endif
                    // Parbus.Resp_LPDU -> buf2
                    // Parbus.Resp_LPCI.Frame_Size -> buf2_data_size
                    // next we works with buf2
                    // result -> put in buf1 (for Parbus.Rqst_LPDU)

					if(CheckCrcParbus((uint8_t *)Parbus.Resp_LPDU) == RESULT_OK)
					{
						CLEAR_FLAG(Parbus.State_Flags, f_Err_Crc_Parbus);
                        switch(Parbus.Resp_LPDU[0])
                        {
                            case 0x11:
                                // 0x11: Read status DI, RL
                                portENTER_CRITICAL();
                                memcpy( (Ied_Data_Buf + RAM_DIKL_DAT_OFFSET),
                                        (Parbus.Resp_LPDU + 2),
                                        Parbus.Resp_LPDU[1]    );
                                portEXIT_CRITICAL();
                                // Test signal flags
                                // Check data ready flag
                                if(TEST_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 0))
                                {
                                    CLEAR_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 0);
                                    Parbus.Rqst_LPDU[0] = 0x55;
                                }
                                // Check analog data ready flag
                                else if(TEST_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 1) && (flag == 0))
                                {
                                    CLEAR_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 1);
                                    Parbus.Rqst_LPDU[0] = 0x33;
                                    flag = 1;
                                }
                                // Check discrete data ready flag
                                else if(TEST_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 2) && (flag == 1))
                                {
                                    CLEAR_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 2);
                                    Parbus.Rqst_LPDU[0] = 0x77;
                                    flag = 2;
                                }
                                // Check discrete data ready flag
                                else if(TEST_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 3) && (flag == 2))
                                {
                                    CLEAR_FLAG(Ied_Data_Buf[RAM_DIKL_DAT_OFFSET + 13], 3);
                                    Parbus.Rqst_LPDU[0] = 0x88;
                                    flag = 0;
                                }
                                else
                                    Parbus.Rqst_LPDU[0] = 0x11;
#ifdef _IEC103_ON_
                                xSemaphoreGive(*(Iec103_0.DIKL_From_Parbus));
                                xSemaphoreGive(*(Iec103_1.DIKL_From_Parbus));
#endif
                                break;

                            case 0x33:
                                // 0x33: Read Analog Data (Full)
                                portENTER_CRITICAL();
                                memcpy( Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET,
                                        Parbus.Resp_LPDU + 2,
                                        Parbus.Resp_LPDU[1]    );
                                if(!TEST_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_RDY))
                                    SET_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_RDY);

                                portEXIT_CRITICAL();
                                Parbus.Rqst_LPDU[0] = 0x11;
                                break;

                            case 0x55:
                                // 0x55: Read CPU
                                portENTER_CRITICAL();
                                // Transfer data to other application (Modbus etc.) -
                                memcpy( Ied_Data_Buf + RAM_DATA_FROM_CPU_OFFSET,
                                        Parbus.Resp_LPDU + 2,
                                        Parbus.Resp_LPDU[1]    );
                                Ied_Data_Buf[RAM_DATA_FROM_CPU_SIZE_OFFSET] = Parbus.Resp_LPDU[1];
                                portEXIT_CRITICAL();
                                Parbus.Rqst_LPDU[0] = 0x11;

                                // send protocol pointer in protocol forward task
                                if(!TEST_FLAG(Parbus.State_Flags, f_Retrans_Rqst_From_Modbus))
								{
                                    xSemaphoreGive(*(Parbus.RqstToApp));
								}
                                else
								{
									CLEAR_FLAG(Parbus.State_Flags, f_Retrans_Rqst_From_Modbus);
                                    xSemaphoreGive(Parbus_To_Modbus_RetransResp_Semph);
								}
                                break;

                            case 0x77:
                                // 0x77: Read discrete data (Full)
                                portENTER_CRITICAL();
                                memcpy( Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET,
                                        Parbus.Resp_LPDU + 2,
                                        Parbus.Resp_LPDU[1]    );
                                portEXIT_CRITICAL();
                                Parbus.Rqst_LPDU[0] = 0x11;
                                break;

                            case 0x88:
                                // 0x88: Read discrete data (VD)
                                portENTER_CRITICAL();
                                memcpy( Ied_Data_Buf + RAM_VD_DAT_OFFSET,
                                        Parbus.Resp_LPDU + 2,
                                        Parbus.Resp_LPDU[1]    );
                                portEXIT_CRITICAL();
                                Parbus.Rqst_LPDU[0] = 0x11;
                                break;

                            default:
                                Parbus.Rqst_LPDU[0] = 0x11;
                                break;
                        }
					}
					else
					{
						SET_FLAG(Parbus.State_Flags, f_Err_Crc_Parbus);
                        Parbus.Rqst_LPDU[0] = 0x11;
					}
				}
				else
                {
					ParbusReset();
                    Parbus.Rqst_LPDU[0] = 0x11;
#ifdef _DEBUG_ON_
                    task_stage[PARBUS_TASK_NUM] = 20;
                    pb_reset_cnt++;
#endif
                }
			}
            else
                Parbus.Rqst_LPDU[0] = 0x11;

#ifdef _DEBUG_ON_
            task_stage[PARBUS_TASK_NUM] = 4;
#endif
            xSemaphoreTake(*(Parbus.ProcessEnbld), 0);
	}
}


//---------------------------------------------------------------------------------------------
uint16_t ParbusLinkProcess	(
							uint8_t *rqst_buf,
							uint16_t rqst_size,
							uint8_t *resp_buf
							)
{
    vTaskSuspendAll();
    memcpy( Ied_Data_Buf + RAM_DATA_FOR_CPU_OFFSET,
			rqst_buf,
			rqst_size );
    Ied_Data_Buf[RAM_DATA_FOR_CPU_SIZE_OFFSET] = rqst_size;
    xTaskResumeAll();

    xSemaphoreGive(Modbus_To_Parbus_RetransRqst_Semph);
    if(xSemaphoreTake(Parbus_To_Modbus_RetransResp_Semph, 10*PARBUS_RESP_TIMEOUT) == pdPASS)
    {
        vTaskSuspendAll();
        memcpy( resp_buf,
                Ied_Data_Buf + RAM_DATA_FROM_CPU_OFFSET,
                Ied_Data_Buf[RAM_DATA_FROM_CPU_SIZE_OFFSET] );
        xTaskResumeAll();
        return (uint16_t)Ied_Data_Buf[RAM_DATA_FROM_CPU_SIZE_OFFSET];
    }
    else
        return 0;
}


//---------------------------------------------------------------------------------------------
BaseType_t CreateParbusTask (
                            Parbus_inst_t *pParbus
                            )
{
    if(pParbus != NULL)
    {
        ParbusPeriphInit();

        Parbus_Semph = xSemaphoreCreateBinary();
        Parbus_Mutex = xSemaphoreCreateMutex();
        Parbus_DataRcvd_Semph = xSemaphoreCreateBinary();
        VDI_To_Parbus_Semph = xSemaphoreCreateBinary();
        Parbus_To_Modbus_Rqst_Semph = xSemaphoreCreateBinary();
        Modbus_To_Parbus_Resp_Semph = xSemaphoreCreateBinary();

        Modbus_To_Parbus_RetransRqst_Semph = xSemaphoreCreateBinary();
        Parbus_To_Modbus_RetransResp_Semph = xSemaphoreCreateBinary();

        Parbus.ProcessEnbld = &Parbus_Semph;
        Parbus.Mtx = &Parbus_Mutex;
        Parbus.DataRcvd = &Parbus_DataRcvd_Semph;
        Parbus.RqstToApp = &Parbus_To_Modbus_Rqst_Semph;
        Parbus.RespFromApp = &Modbus_To_Parbus_Resp_Semph;

        Parbus.Resp_LPDU = Parbus_Data_Buf + RAM_PARBUS_RESP_DATA_BUF_OFFSET;
        Parbus.Rqst_LPDU = Parbus_Data_Buf + RAM_PARBUS_RQST_DATA_BUF_OFFSET;

        Parbus.State_Flags = 0;

        return xTaskCreate( vParbusTask,
                            (char *)"ParBus",
                            (configMINIMAL_STACK_SIZE),
                            NULL,
                            (tskIDLE_PRIORITY + 3),
                            &xParbusHandle			);
    }
    return pdFALSE;
}


//**********************************************************************************************
//                                   Interrupt procedures
//**********************************************************************************************
//---------------------------------------------------------------------------------------------
void ParbusLinkTxHandler(void)
{
    if(xParbusHandle != NULL)
    {
        // Determine  CS_IN2 edge
        if(!TEST_PIN(CSIN2_Port, CSIN2_Pin))
        // Falling edge (active)
        {
            if(++Parbus.Rqst_LPCI.Byte_Counter >= Parbus.Rqst_LPCI.Frame_Size)
            {
                // Finalize transmition
                // Disable external int. MYEXTI2_Pin
                EXTI->IMR &= ~(uint32_t)(1<<CSIN2_Pin);

                // Reset buffer direction, unselect buffer, set data bus as input
                SET_PIN(OECOM2_Port, OECOM2_Pin);
                RESET_PIN(DIRCOM2_Port, DIRCOM2_Pin);
                PARBUS_INPUT();
                if(Parbus.Rqst_LPDU[0] != 0x22 && Parbus.Rqst_LPDU[0] != 0x44)
                {
                    // Reset flags
                    Parbus.State_Flags &= ~(1<<f_Err_Crc_Parbus | 1<<f_End_Receive_Parbus );
                    PARBUS_RX_START();
                }
            }
            SET_PIN(CSOUT2_Port, CSOUT2_Pin);
        }
        else
        // Rising edge
        {
            // Send next byte to Parallel Bus
            PARALL_BUS_TX_BYTE(Parbus.Rqst_LPDU[Parbus.Rqst_LPCI.Byte_Counter]);
            // Set reqest on CS_OUT2
            RESET_PIN(CSOUT2_Port, CSOUT2_Pin);
        }
    }
}


//---------------------------------------------------------------------------------------------
void ParbusLinkRxHandler(void)
{
    if(xParbusHandle != NULL)
    {
        if(!TEST_PIN(INCPU2_Port, INCPU2_Pin))
        // Falling edge (active)
        {
            // Read data from bus
            Parbus.Resp_LPDU[Parbus.Resp_LPCI.Byte_Counter] = PARALL_BUS_RX_BYTE();
            // Find the amount of bytes in the packet
            if(++Parbus.Resp_LPCI.Byte_Counter == 3)
            {
                Parbus.Resp_LPCI.Frame_Size = Parbus.Resp_LPDU[1] + 3;
            }
            // Check the number of received bytes
            if( (Parbus.Resp_LPCI.Byte_Counter > 3)
                &&
                (Parbus.Resp_LPCI.Byte_Counter == Parbus.Resp_LPCI.Frame_Size) )
            {
                // End of receive Parall Bus
                SET_FLAG(Parbus.State_Flags, f_End_Receive_Parbus);
            }
            // Reset line OUT_CPU2
            RESET_PIN(OUTCPU2_Port, OUTCPU2_Pin);
        }
        else
        // Rising edge ()
        {
            if(TEST_FLAG(Parbus.State_Flags, f_End_Receive_Parbus))
            // End of receiving
            {
                CLEAR_FLAG(Parbus.State_Flags, f_End_Receive_Parbus);
                // Finalize receive
                // Disable external int. MYEXTI3_Pin
                EXTI->IMR &= ~((uint32_t)(1<<INCPU2_Pin));
                // Unselect Buffer
                SET_PIN(OECOM2_Port, OECOM2_Pin);
                xSemaphoreGiveFromISR(*(Parbus.DataRcvd), NULL);        //&xTaskWokenRx
            }
            // Set line OUT_CPU2
            SET_PIN(OUTCPU2_Port, OUTCPU2_Pin);
        }
    }
}






