//*********************************************************************************************
//                                          Main.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "main.h"
#pragma pack (4)
//---------------------------------------------------------------------------------------------
//                                        Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variable
//---------------------------------------------------------------------------------------------
uint32_t 	            systick_ms;

#ifdef _TICK_COUNT_ON_
uint32_t 	            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
#endif

SemaphoreHandle_t       Rst_Semph = NULL;


Parbus_inst_t	        Parbus;
uint8_t                 Parbus_Data_Buf[RAM_PARBUS_BUF_SIZE];
uint8_t				    Ied_Data_Buf[RAM_IED_BUF_SIZE];

SemaphoreHandle_t       Parbus_Semph = NULL;
SemaphoreHandle_t       Parbus_Mutex;
SemaphoreHandle_t       Parbus_DataRcvd_Semph = NULL;
SemaphoreHandle_t       Parbus_To_Modbus_Rqst_Semph = NULL;
SemaphoreHandle_t       Modbus_To_Parbus_Resp_Semph = NULL;
// ----------------------------------------------
    SemaphoreHandle_t       Modbus_To_Parbus_RetransRqst_Semph = NULL;
    SemaphoreHandle_t       Parbus_To_Modbus_RetransResp_Semph = NULL;
    SemaphoreHandle_t       VDI_To_Parbus_Semph = NULL;
// ----------------------------------------------


Modbus_inst_t	        Modbus;
uint8_t                 Modbus_Data_Buf[RAM_MODBUS_BUF_SIZE];
SemaphoreHandle_t       Modbus_Mutex;
SemaphoreHandle_t       Modbus_DataRcvd_Semph = NULL;
SemaphoreHandle_t       Modbus_DataSent_Semph = NULL;
SemaphoreHandle_t       Modbus_To_Parbus_Rqst_Semph = NULL;
SemaphoreHandle_t       Parbus_To_Modbus_Resp_Semph = NULL;

uint8_t                 mbdf_rqst_buf[256], mbdf_resp_buf[256];
uint16_t                mbdf_rqst_size, mbdf_resp_size;
uint8_t                 pbdf_rqst_buf[256], pbdf_resp_buf[256];
uint16_t                pbdf_rqst_size, pbdf_resp_size;
uint8_t                 sb_0_df_rqst_buf[256], sb_0_df_resp_buf[256];
uint16_t                sb_0_df_rqst_size, sb_0_df_resp_size;
uint8_t                 sb_1_df_rqst_buf[256], sb_1_df_resp_buf[256];
uint16_t                sb_1_df_rqst_size, sb_1_df_resp_size;
uint8_t                 i103_0_df_rqst_buf[256], i103_0_df_resp_buf[256];
uint16_t                i103_0_df_rqst_size, i103_0_df_resp_size;
uint8_t                 i103_1_df_rqst_buf[256], i103_1_df_resp_buf[256];
uint16_t                i103_1_df_rqst_size, i103_1_df_resp_size;
uint8_t                 mb_rqst_buf[256], mb_resp_buf[256];
uint8_t                 mb_rqst_size, mb_resp_size;


Serbus_inst_t	        Serbus0;
Serbus_Param_t          Serbus0_Settings;
uint8_t				    Data_Buf_Serbus0[RAM_SERBUS_BUF_SIZE];
uint32_t 			    error_log_Serbus0[ERR_LOG_SIZE];
SemaphoreHandle_t       Serbus0_DataRcvd_Semph = NULL;
SemaphoreHandle_t       Serbus0_DataSent_Semph = NULL;
SemaphoreHandle_t       Serbus0_Rqst_Semph = NULL;
SemaphoreHandle_t       Serbus0_Resp_Semph = NULL;


Serbus_inst_t	        Serbus1;
Serbus_Param_t          Serbus1_Settings;
uint8_t				    Data_Buf_Serbus1[RAM_SERBUS_BUF_SIZE];
uint32_t 			    error_log_Serbus1[ERR_LOG_SIZE];
SemaphoreHandle_t       Serbus1_DataRcvd_Semph = NULL;
SemaphoreHandle_t       Serbus1_DataSent_Semph = NULL;
SemaphoreHandle_t       Serbus1_Rqst_Semph = NULL;
SemaphoreHandle_t       Serbus1_Resp_Semph = NULL;


Eeprom_inst_t           Eeprom_M95xxx;
SemaphoreHandle_t       Eep_Cmplt_Semph = NULL;
SemaphoreHandle_t       Eep_Mutex;

Id_Inst_t               Id_Data;
uint8_t				    Device_Name[8];

Vers_Inst_t		        Vers_Data;
uint8_t                 Frmwr_Vers_Actual_str[EEPROM_VERSION_SIZE];
uint8_t                 Frmwr_Vers_Eeprom_str[EEPROM_VERSION_SIZE];
uint8_t                 Hrdwr_Vers_Actual_str[EEPROM_VERSION_SIZE];
uint8_t                 Hrdwr_Vers_Eeprom_str[EEPROM_VERSION_SIZE];
uint8_t                 Btldr_Vers_Actual_str[EEPROM_VERSION_SIZE];
uint8_t                 Btldr_Vers_Eeprom_str[EEPROM_VERSION_SIZE];


uint8_t				    Modbus_Relationship_Tab[EEPROM_MODBUS_REL_TAB_SIZE];
Rel_Tab_struct_t 		Modbus_Rel_Tab_Structure;
Table_Region_t			Modbus_Region_Tab[MODBUS_REG_TAB_SIZE];

rtc_strct_t			    Rtc_fact, Rtc_sync;
Sync_Inst_t             Sync_Data;
uint8_t                 Sync_Src_Prior_Set[SYNC_SRC_AMOUNT];

#ifdef _DEBUG_ON_
uint32_t                tick_cnt, idle_task_enter_cnt;
uint32_t                task_enter_cnt[TASK_NUM_SIZE];
uint32_t                task_stage[TASK_NUM_SIZE];
uint32_t                pb_reset_cnt;
#endif

#ifdef _MONITOR_ON_
uint32_t                heap_free, min_heap_free;
TaskHandle_t            xMonitorHandle = NULL;
TaskStatus_t            Task_Inf_Buf[TASK_NUM_SIZE + 1];
//unsigned long           ulTotalRunTime;
#endif

BaseType_t              task_ready[TASK_NUM_SIZE];

TaskHandle_t            xStartAllHandle = NULL;
TaskHandle_t            xParbusHandle = NULL;
TaskHandle_t            xSerbus_0_Handle = NULL;
TaskHandle_t            xSerbus_1_Handle = NULL;
TaskHandle_t            xModbusHandle = NULL;
TaskHandle_t            xParbusFwdHandle = NULL;
TaskHandle_t            xSerbus_0_FwdHandle = NULL;
TaskHandle_t            xSerbus_1_FwdHandle = NULL;
TaskHandle_t            xModbusFwdHandle = NULL;

#ifdef _FLASH_IRQ_ON_
SemaphoreHandle_t       FlashEraseFinSemph = NULL;
SemaphoreHandle_t       FlashWriteDoneSemph = NULL;
#endif

#ifdef _BOOTLOADER_ON_
uint8_t                 Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_BUF_SIZE];
uint8_t                 Fw_Upd_Key[5];
SemaphoreHandle_t       Fw_Upd_Rqst_Semph = NULL;
SemaphoreHandle_t       Fw_Upd_Confrm_Semph = NULL;
TaskHandle_t            xFwUpdCtrlHandle = NULL;
Flash_struc_t           Flash_struc;
uint16_t                Flash_Buf[FLASH_SAVED_WORDS];
#endif



#ifdef _IEC103_ON_
Iec103_inst_t		    Iec103_0;
uint8_t                 Iec103_0_Data_Buf[RAM_IEC103_BUF_SIZE];
uint8_t				    Queue_0_Data_Buf[RAM_MSG_QUEUE_BUF_SIZE];
uint8_t                 Iec103_0_Server_Buf[RAM_MODBUS_BUF_SIZE];
SemaphoreHandle_t       Iec103_0_DataRcvd_Semph = NULL;
SemaphoreHandle_t       Iec103_0_DataSent_Semph = NULL;
SemaphoreHandle_t       Iec103_0_To_Modbus_Rqst_Semph = NULL;
SemaphoreHandle_t       Modbus_To_Iec103_0_Resp_Semph = NULL;
SemaphoreHandle_t       VDI_To_Iec103_0_Semph = NULL;
SemaphoreHandle_t       DIKL_To_Iec103_0_Semph = NULL;

Iec103_inst_t		    Iec103_1;
uint8_t                 Iec103_1_Data_Buf[RAM_IEC103_BUF_SIZE];
uint8_t				    Queue_1_Data_Buf[RAM_MSG_QUEUE_BUF_SIZE];
uint8_t                 Iec103_1_Server_Buf[RAM_MODBUS_BUF_SIZE];
SemaphoreHandle_t       Iec103_1_DataRcvd_Semph = NULL;
SemaphoreHandle_t       Iec103_1_DataSent_Semph = NULL;
SemaphoreHandle_t       Iec103_1_To_Modbus_Rqst_Semph = NULL;
SemaphoreHandle_t       Modbus_To_Iec103_1_Resp_Semph = NULL;
SemaphoreHandle_t       VDI_To_Iec103_1_Semph = NULL;
SemaphoreHandle_t       DIKL_To_Iec103_1_Semph = NULL;

uint8_t                 GI_Tab[EEPROM_INTERVIEW_SET_SIZE];
uint8_t                 SE_Tab[EEPROM_INTERVIEW_SET_SIZE];
uint8_t                 SE_DIKLVDI_Buf[RAM_VIRT_DAT_SIZE + RAM_DISCRET_DAT_SIZE];

uint8_t				    Iec103_Relationship_Tab[EEPROM_IEC103_REL_TAB_SIZE];
Rel_Tab_struct_t 		Iec103_Rel_Tab_Structure;
Table_Region_t			Iec103_Region_Tab[IEC103_REG_TAB_SIZE];

TaskHandle_t            xIec103_0_Handle = NULL;
TaskHandle_t            xIec103_1_Handle = NULL;
TaskHandle_t            xIec103_0_ProcHandle = NULL;
TaskHandle_t            xIec103_1_ProcHandle = NULL;
TaskHandle_t            xIec103_0_FwdHandle = NULL;
TaskHandle_t            xIec103_1_FwdHandle = NULL;
#endif




//---------------------------------------------------------------------------------------------
//                                        Main function
//---------------------------------------------------------------------------------------------
int main(void)
{
#ifdef _BOOTLOADER_ON_
	SCB->VTOR = FLASH_APP_ADDR;																// vector interrupts table offset (if bootloader uses)
#endif

    __disable_irq();
    DWT_TICKCNT_INIT();

    InitGpio();

    SetAsInput(ENCOM1_Port, ENCOM1_Pin, 1);

	while(!TEST_PIN(ENCOM1_Port, ENCOM1_Pin))
	{
		DelayDWT_us(500);
	}

//    SetAsPpOutput(LED_Port, LED_Pin, 0);
//    SET_PIN(LED_Port, LED_Pin);


#ifdef _DEBUG_ON_
    printf("Hello!!!\r\n");

#endif

    // Create Task for start
	xTaskCreate(vStartAllTask,
                (char *)"StartAll",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY,
                &xStartAllHandle	    );

    __enable_irq();

//    SET_PIN(LED_Port, LED_Pin);
//    while(1);

    vTaskStartScheduler();
    for(;;);
//    return 0;
}


//---------------------------------------------------------------------------------------------
void vApplicationIdleHook( void )
{
#ifdef _DEBUG_ON_
    ++idle_task_enter_cnt;
#endif
}


//---------------------------------------------------------------------------------------------
void vApplicationMallocFailedHook( void )
{
    while(1)
    {
#ifdef _DEBUG_ON_

#else
        NVIC_SystemReset();
#endif
    }
}


//---------------------------------------------------------------------------------------------
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
    while(1)
    {
#ifdef _DEBUG_ON_

#else
        NVIC_SystemReset();
#endif
    }
}


//---------------------------------------------------------------------------------------------
void vApplicationTickHook( void )
{
#ifdef _DEBUG_ON_
    ++tick_cnt;
#endif
    if(task_ready[START_TASK_NUM] == pdTRUE)
    {
        if(xSemaphoreTakeFromISR(Rst_Semph, NULL))
        {
            NVIC_SystemReset();
        }

        // Check sync complition
        if(Rtc_fact.rtc_state == RTC_SYNC_CMPLT && !TEST_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_DONE))
        {
            SET_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_DONE);
        }

        // Get time from Parbus & set Rtc_fact
        if(!TEST_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_DONE) && TEST_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_RDY))
        {
            SET_FLAG(Rtc_fact.rtc_flag, RTC_SETUP_DONE);

            // Transfer time data from F_33 Parbus buffer to Rtc_fact
            Rtc_fact.time.year = *(Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET + 4);
            Rtc_fact.time.month = *(Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET + 5);
            Rtc_fact.time.mday = *(Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET + 6);
            Rtc_fact.time.hour = *(Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET + 7);
            Rtc_fact.time.min = *(Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET + 8);
            Rtc_fact.time.sec = *(Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET + 9);

            // Time conversion (Date-Time -> sec)
            Rtc_fact.s_counter = GetRtcCounterFromTime(&Rtc_fact.time, Rtc_fact.summer_time_need);
        }

        // Sync process
        SoftRtcSyncProcess(&Rtc_sync, &Rtc_fact);

        // get actual ms_time
        Rtc_fact.ms_counter = ((uint32_t)TIM1->CNT) >> 1;

#ifdef _BOOTLOADER_ON_
        BldrEnterKey(&Rtc_fact);
#endif
        if(xParbusHandle != NULL)
        {
            xSemaphoreGiveFromISR(*(Parbus.ProcessEnbld), NULL);    // &xHigherPriorityTaskWoken0
        }
    }
}



//---------------------------------------------------------------------------------------------
void vStartAllTask	(
					void *pvParameters
					)
{
#ifdef _DEBUG_ON_
        ++task_enter_cnt[START_TASK_NUM];
#endif

        Rst_Semph = xSemaphoreCreateBinary();

        StartEeprom();

        // Initialization of IED ID
        IdInit(&Id_Data);

		// Check version inited?
        VersInit(&Vers_Data);

        // Sync initialization
        SyncInit(&Sync_Data);

        TimerInit();
        SoftRtcLoad(&Rtc_fact, Sync_Data.Sync_Period);

        // Initialization of Modbus remapping table
        RemapTabInit(&Modbus_Rel_Tab_Structure, EEPROM_MODBUS_REL_TAB_ADDR, EEPROM_MODBUS_REL_TAB_SIZE);

#ifdef _IEC103_ON_
        // Initialization of Iec103 remapping table
        RemapTabInit(&Iec103_Rel_Tab_Structure, EEPROM_IEC103_REL_TAB_ADDR, EEPROM_IEC103_REL_TAB_SIZE);
        // Initialization of Iec103 interview settings tables
        GetInterviewSetTab(GI_Tab, EEPROM_GI_SET_ADDR);
        GetInterviewSetTab(SE_Tab, EEPROM_SE_SET_ADDR);
#endif
        GetSerbusSettings(&Serbus0_Settings);
        GetSerbusSettings(&Serbus1_Settings);

        task_ready[START_TASK_NUM] = CreateMainTasks();

        vTaskDelete(xStartAllHandle);
}


//---------------------------------------------------------------------------------------------
BaseType_t CreateMainTasks(void)
{

#ifdef _BOOTLOADER_ON_
    task_ready[FW_UPD_TASK_NUM] = CreateFwUpdateTask();
#endif
    task_ready[PARBUS_TASK_NUM] = CreateParbusTask(&Parbus);
    task_ready[MODBUS_TASK_NUM] = CreateModbusTask(&Modbus);
    task_ready[SERBUS0_TASK_NUM] = CreateSerbusTask(&Serbus0);
    task_ready[SERBUS1_TASK_NUM] = CreateSerbusTask(&Serbus1);

#ifdef _IEC103_ON_
    task_ready[IEC103_0_TASK_NUM] = CreateIec103Task(&Iec103_0);
    task_ready[IEC103_1_TASK_NUM] = CreateIec103Task(&Iec103_1);
#endif

	if(task_ready[PARBUS_TASK_NUM] && task_ready[MODBUS_TASK_NUM])
    {
        task_ready[PB_FWD_TASK_NUM] = xTaskCreate(  vParbusDataFwdTask,
                                                    (char *)"ParbusDataFwd",
                                                    configMINIMAL_STACK_SIZE,
                                                    NULL,
                                                    (tskIDLE_PRIORITY + 1),
                                                    &xParbusFwdHandle	    );

        task_ready[MB_FWD_TASK_NUM] = xTaskCreate(  vModbusDataFwdTask,
                                                    (char *)"ModbusDataFwd",
                                                    configMINIMAL_STACK_SIZE,
                                                    NULL,
                                                    (tskIDLE_PRIORITY + 1),
                                                    &xModbusFwdHandle	    );

        if(task_ready[SERBUS0_TASK_NUM]
#ifdef _IEC103_ON_
            && task_ready[IEC103_0_TASK_NUM]
#endif
            )
        {
            task_ready[SB0_FWD_TASK_NUM] =      xTaskCreate(vSerbusDataFwdTask,
                                                            (char *)"Serbus0DataFwd",
                                                            configMINIMAL_STACK_SIZE,
                                                            (void *)&Serbus0,
                                                            (tskIDLE_PRIORITY + 1),
                                                            &xSerbus_0_FwdHandle		);

#ifdef _IEC103_ON_
            task_ready[IEC103_0_FWD_TASK_NUM] = xTaskCreate(vIec103DataFwdTask,
                                                            (char *)"Iec103_0_DataFwd",
                                                            configMINIMAL_STACK_SIZE,
                                                            (void *)&Iec103_0,
                                                            (tskIDLE_PRIORITY + 1),
                                                            &xIec103_0_FwdHandle	);
#endif
        }
        else
            return pdFALSE;

        if(task_ready[SERBUS1_TASK_NUM]
    #ifdef _IEC103_ON_
            && task_ready[IEC103_1_TASK_NUM]
    #endif
            )
        {
            task_ready[SB1_FWD_TASK_NUM] =      xTaskCreate(vSerbusDataFwdTask,
                                                            (char *)"Serbus1DataFwd",
                                                            configMINIMAL_STACK_SIZE,
                                                            (void *)&Serbus1,
                                                            (tskIDLE_PRIORITY + 1),
                                                            &xSerbus_1_FwdHandle		);
#ifdef _IEC103_ON_
            task_ready[IEC103_1_FWD_TASK_NUM] = xTaskCreate(vIec103DataFwdTask,
                                                            (char *)"Iec103_1_DataFwd",
                                                            configMINIMAL_STACK_SIZE,
                                                            (void *)&Iec103_1,
                                                            (tskIDLE_PRIORITY + 1),
                                                            &xIec103_1_FwdHandle	);
#endif
        }
        else
            return pdFALSE;

    }
    else
        return pdFALSE;

#ifdef _MONITOR_ON_
	task_ready[MONITOR_TASK_NUM] = xTaskCreate( vMonitorTask,
                                                (char *)"Monitor",
                                                configMINIMAL_STACK_SIZE,
                                                NULL,
                                                (tskIDLE_PRIORITY),
                                                &xMonitorHandle	);
#endif
    return pdTRUE;
}




#ifdef _MONITOR_ON_
//---------------------------------------------------------------------------------------------
void vMonitorTask   (
                    void *pvParameters
                    )
{
    uint32_t uxArraySize;

    for(;;)
	{
#ifdef _DEBUG_ON_
        ++task_enter_cnt[MONITOR_TASK_NUM];
#endif

        uxArraySize = uxTaskGetNumberOfTasks();
        uxTaskGetSystemState(   Task_Inf_Buf,
                                uxArraySize,
                                NULL );                 // &ulTotalRunTime

/*        EBmonitor_flush(stdout);
        for(uint32_t x = 0; x < uxArraySize; x++)
        {
            printf("%16s: %u, %u, %3u\r\n",
                    Task_Inf_Buf[x].pcTaskName,
                    Task_Inf_Buf[x].eCurrentState,
                    Task_Inf_Buf[x].uxCurrentPriority,
                    Task_Inf_Buf[x].usStackHighWaterMark );
        }
*/
        printf("Heap Free: %u\r\n", xPortGetFreeHeapSize());

        vTaskDelay(1000);
    }
}
#endif


