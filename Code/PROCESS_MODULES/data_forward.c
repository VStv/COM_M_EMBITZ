//*********************************************************************************************
//                                      Data_forward.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "data_forward.h"


//---------------------------------------------------------------------------------------------
//                                    Tables of Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
#ifdef _TICK_COUNT_ON_
extern uint32_t 		            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
#endif

extern uint8_t				        Ied_Data_Buf[RAM_IED_BUF_SIZE];

extern Serbus_inst_t			    Serbus0;
extern Serbus_inst_t			    Serbus1;
extern Parbus_inst_t	            Parbus;
extern Modbus_inst_t	            Modbus;

extern uint8_t                      mbdf_rqst_buf[256], mbdf_resp_buf[256];
extern uint16_t                     mbdf_rqst_size, mbdf_resp_size;
extern uint8_t                      pbdf_rqst_buf[256], pbdf_resp_buf[256];
extern uint16_t                     pbdf_rqst_size, pbdf_resp_size;

extern uint8_t                      sb_0_df_rqst_buf[256], sb_0_df_resp_buf[256];
extern uint16_t                     sb_0_df_rqst_size, sb_0_df_resp_size;
extern uint8_t                      sb_1_df_rqst_buf[256], sb_1_df_resp_buf[256];
extern uint16_t                     sb_1_df_rqst_size, sb_1_df_resp_size;

extern uint8_t                      i103_0_df_rqst_buf[256], i103_0_df_resp_buf[256];
extern uint16_t                     i103_0_df_rqst_size, i103_0_df_resp_size;
extern uint8_t                      i103_1_df_rqst_buf[256], i103_1_df_resp_buf[256];
extern uint16_t                     i103_1_df_rqst_size, i103_1_df_resp_size;



#ifdef _IEC103_ON_
extern Iec103_inst_t		    	Iec103_0;
extern Iec103_inst_t		    	Iec103_1;
#endif


#ifdef _DEBUG_ON_
extern uint32_t                     task_enter_cnt[TASK_NUM_SIZE];
extern uint32_t                     task_stage[TASK_NUM_SIZE];
#endif




//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//---------------- Data (request) from client (CPU) forwards to server (Modbus) ---------------
void vParbusDataFwdTask (
                        void *pvParameters
                        )
{
    uint8_t *buf1, *buf2;
    uint16_t *p_buf1_data_size, *p_buf2_data_size;

    buf1 = pbdf_rqst_buf;
    buf2 = pbdf_resp_buf;
    p_buf1_data_size = &pbdf_rqst_size;
    p_buf2_data_size = &pbdf_resp_size;

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[PB_FWD_TASK_NUM] = 0;
        ++task_enter_cnt[PB_FWD_TASK_NUM];
#endif
        xSemaphoreTake(*(Parbus.RqstToApp), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[PB_FWD_TASK_NUM] = 1;
#endif
            vTaskSuspendAll();
            *p_buf1_data_size = (uint16_t)Ied_Data_Buf[RAM_DATA_FROM_CPU_SIZE_OFFSET];
            memcpy(buf1, Ied_Data_Buf + RAM_DATA_FROM_CPU_OFFSET, *p_buf1_data_size);
            xTaskResumeAll();

            xSemaphoreTake(*(Modbus.Mtx), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[PB_FWD_TASK_NUM] = 2;
//            printf("%i\r\n", task_enter_cnt[PB_FWD_TASK_NUM]);
#endif
            *p_buf2_data_size = ModbusLinkProcess(buf1, *p_buf1_data_size, buf2, 0);

            xSemaphoreGive(*(Modbus.Mtx));

            vTaskSuspendAll();
            Ied_Data_Buf[RAM_DATA_FOR_CPU_SIZE_OFFSET] = *p_buf2_data_size;
            memcpy(Ied_Data_Buf + RAM_DATA_FOR_CPU_OFFSET, buf2, *p_buf2_data_size);
            xTaskResumeAll();

            xSemaphoreGive(*(Parbus.RespFromApp));
    }
}


//---------------- Data (request) from client (Modbus) forwards to server (CPU) ---------------
void vModbusDataFwdTask (
                        void *pvParameters
                        )
{
    uint8_t *buf1, *buf2;
    uint16_t *p_buf1_data_size, *p_buf2_data_size;

    buf1 = mbdf_rqst_buf;
    buf2 = mbdf_resp_buf;
    p_buf1_data_size = &mbdf_rqst_size;
    p_buf2_data_size = &mbdf_resp_size;

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[MB_FWD_TASK_NUM] = 0;
        ++task_enter_cnt[MB_FWD_TASK_NUM];
#endif
        xSemaphoreTake(*(Modbus.RqstToApp), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[MB_FWD_TASK_NUM] = 1;
#endif
            vTaskSuspendAll();
            *p_buf1_data_size = Modbus.Rqst_PDU_Size;
            memcpy(buf1, Modbus.Rqst_PDU, *p_buf1_data_size);
            xTaskResumeAll();

            xSemaphoreTake(*(Parbus.Mtx), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[MB_FWD_TASK_NUM] = 2;
#endif
            *p_buf2_data_size = ParbusLinkProcess(buf1, *p_buf1_data_size, buf2);
            xSemaphoreGive(*(Parbus.Mtx));

            vTaskSuspendAll();
            Modbus.Resp_PDU_Size = *p_buf2_data_size;
            memcpy(Modbus.Resp_PDU, buf2, *p_buf2_data_size);
            xTaskResumeAll();

            xSemaphoreGive(*(Modbus.RespFromApp));
    }
}


//--------- Data (request) from client (Serbus0) forwards to server (Modbus, IEC103) --------
void vSerbusDataFwdTask(
                        void *pvParameters
                        )
{
    Serbus_inst_t *pSerbus;
    uint8_t *buf1, *buf2;
    uint16_t *p_buf1_data_size, *p_buf2_data_size;

    pSerbus = (Serbus_inst_t *)pvParameters;

    if(pSerbus == &Serbus0)
    {
        buf1 = sb_0_df_rqst_buf;
        buf2 = sb_0_df_resp_buf;
        p_buf1_data_size = &sb_0_df_rqst_size;
        p_buf2_data_size = &sb_0_df_resp_size;
    }
    else
    {
        buf1 = sb_1_df_rqst_buf;
        buf2 = sb_1_df_resp_buf;
        p_buf1_data_size = &sb_1_df_rqst_size;
        p_buf2_data_size = &sb_1_df_resp_size;
    }

#ifdef _DEBUG_ON_
    uint32_t task_num;
    if(pSerbus == &Serbus0)
        task_num = SB0_FWD_TASK_NUM;
    else
        task_num = SB1_FWD_TASK_NUM;
#endif

#ifdef _IEC103_ON_
    Iec103_inst_t *pIec103_inst;
    if(pSerbus == &Serbus0)
        pIec103_inst = &Iec103_0;
    else
        pIec103_inst = &Iec103_1;
#endif

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[task_num] = 0;
        ++task_enter_cnt[task_num];
#endif
        xSemaphoreTake(*(pSerbus->RqstToApp), portMAX_DELAY);
            vTaskSuspendAll();
            *p_buf1_data_size = pSerbus->Rqst_LPCI.Frame_Size;
            memcpy(buf1, pSerbus->Rqst_LPDU, *p_buf1_data_size);
            xTaskResumeAll();

            if(TEST_FLAG(pSerbus->State_Flags, f_Modbus_Frame_Received))
            {
#ifdef _DEBUG_ON_
                task_stage[task_num] = 11;
#endif
                xSemaphoreTake(*(Modbus.Mtx), portMAX_DELAY);
#ifdef _DEBUG_ON_
                task_stage[task_num] = 12;
#endif
                *p_buf2_data_size = ModbusLinkProcess(  buf1,
                                                        *p_buf1_data_size,
                                                        buf2,
                                                        (uint8_t)pSerbus->Serbus_Periph.Periph_Settings->Address );
                xSemaphoreGive(*(Modbus.Mtx));
                CLEAR_FLAG(pSerbus->State_Flags, f_Modbus_Frame_Received);
            }
#ifdef _IEC103_ON_
            else if(TEST_FLAG(pSerbus->State_Flags, f_Iec103_Frame_Received))
            {
#ifdef _DEBUG_ON_
                task_stage[task_num] = 21;
#endif
                *p_buf2_data_size = Iec103LinkProcess(  pIec103_inst,
                                                        buf1,
                                                        *p_buf1_data_size,
                                                        buf2,
                                                        (uint8_t)pSerbus->Serbus_Periph.Periph_Settings->Address,
                                                        pSerbus->Time_Stamp  );
                CLEAR_FLAG(pSerbus->State_Flags, f_Iec103_Frame_Received);
            }
#endif
            vTaskSuspendAll();
            pSerbus->Resp_LPCI.Frame_Size = *p_buf2_data_size;
            memcpy(pSerbus->Resp_LPDU, buf2, *p_buf2_data_size);
            xTaskResumeAll();

            xSemaphoreGive(*(pSerbus->RespFromApp));
    }
}


#ifdef _IEC103_ON_
//---------------- Data (request) from client (Iec103) forwards to server (Modbus) ---------------
void vIec103DataFwdTask (
                        void *pvParameters
                        )
{
    uint8_t *buf1, *buf2;
    uint16_t *p_buf1_data_size, *p_buf2_data_size;
    Iec103_inst_t *pIec103_inst;

    pIec103_inst = (Iec103_inst_t *)pvParameters;

    if(pIec103_inst == &Iec103_0)
    {
        buf1 = i103_0_df_rqst_buf;
        buf2 = i103_0_df_resp_buf;
        p_buf1_data_size = &i103_0_df_rqst_size;
        p_buf2_data_size = &i103_0_df_resp_size;
    }
    else
    {
        buf1 = i103_1_df_rqst_buf;
        buf2 = i103_1_df_resp_buf;
        p_buf1_data_size = &i103_1_df_rqst_size;
        p_buf2_data_size = &i103_1_df_resp_size;
    }

#ifdef _DEBUG_ON_
    uint32_t task_num;
    if(pIec103_inst == &Iec103_0)
        task_num = IEC103_0_FWD_TASK_NUM;
    else
        task_num = IEC103_1_FWD_TASK_NUM;
#endif

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[task_num] = 0;
        ++task_enter_cnt[task_num];
#endif
        xSemaphoreTake(*(pIec103_inst->RqstToApp), portMAX_DELAY);

#ifdef _DEBUG_ON_
            task_stage[task_num] = 1;
#endif
            vTaskSuspendAll();
            *p_buf1_data_size = (uint16_t)pIec103_inst->Gen_Cmd.RqstToAppSize;
            memcpy(buf1, pIec103_inst->Gen_Cmd.RqstToAppBuf, *p_buf1_data_size);
            xTaskResumeAll();

            xSemaphoreTake(*(Modbus.Mtx), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[task_num] = 2;
#endif
			*p_buf2_data_size = ModbusLinkProcess(buf1, *p_buf1_data_size, buf2, 0);

            xSemaphoreGive(*(Modbus.Mtx));

            vTaskSuspendAll();
            pIec103_inst->Gen_Cmd.RespFromAppSize = *p_buf2_data_size;
            memcpy(pIec103_inst->Gen_Cmd.RespFromAppBuf, buf2, *p_buf2_data_size);
            xTaskResumeAll();

        xSemaphoreGive(*(pIec103_inst->RespFromApp));
    }
}
#endif


