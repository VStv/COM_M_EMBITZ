//*********************************************************************************************
//                                      iec60870_103.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "iec60870_103.h"

#ifdef _IEC103_ON_

//---------------------------------------------------------------------------------------------
//                                   Tables of Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern Serbus_Param_t           Serbus0_Settings;
extern Serbus_Param_t           Serbus1_Settings;

extern Id_Inst_t                Id_Data;

extern Iec103_inst_t		    Iec103_0;
extern uint8_t                  Iec103_0_Data_Buf[RAM_IEC103_BUF_SIZE];
extern uint8_t				    Queue_0_Data_Buf[RAM_MSG_QUEUE_BUF_SIZE];
extern uint8_t                 	Iec103_0_Server_Buf[RAM_MODBUS_BUF_SIZE];
extern SemaphoreHandle_t        Iec103_0_DataRcvd_Semph;
extern SemaphoreHandle_t        Iec103_0_DataSent_Semph;
extern SemaphoreHandle_t        Iec103_0_To_Modbus_Rqst_Semph;
extern SemaphoreHandle_t        Modbus_To_Iec103_0_Resp_Semph;
extern SemaphoreHandle_t        VDI_To_Iec103_0_Semph;
extern SemaphoreHandle_t        DIKL_To_Iec103_0_Semph;

extern Iec103_inst_t		    Iec103_1;
extern uint8_t                  Iec103_1_Data_Buf[RAM_IEC103_BUF_SIZE];
extern uint8_t				    Queue_1_Data_Buf[RAM_MSG_QUEUE_BUF_SIZE];
extern uint8_t                 	Iec103_1_Server_Buf[RAM_MODBUS_BUF_SIZE];
extern SemaphoreHandle_t        Iec103_1_DataRcvd_Semph;
extern SemaphoreHandle_t        Iec103_1_DataSent_Semph;
extern SemaphoreHandle_t        Iec103_1_To_Modbus_Rqst_Semph;
extern SemaphoreHandle_t        Modbus_To_Iec103_1_Resp_Semph;
extern SemaphoreHandle_t        VDI_To_Iec103_1_Semph;
extern SemaphoreHandle_t        DIKL_To_Iec103_1_Semph;



extern Rel_Tab_struct_t 	    Iec103_Rel_Tab_Structure;

extern uint8_t                  SE_Tab[EEPROM_INTERVIEW_SET_SIZE];
extern uint8_t                  SE_DIKLVDI_Buf[RAM_VIRT_DAT_SIZE + RAM_DISCRET_DAT_SIZE];


extern uint8_t                  GI_Tab[EEPROM_INTERVIEW_SET_SIZE];
extern uint8_t                  Ied_Data_Buf[RAM_IED_BUF_SIZE];



extern rtc_strct_t			    Rtc_fact, Rtc_sync;

extern TaskHandle_t             xIec103_0_Handle;
extern TaskHandle_t             xIec103_1_Handle;
extern TaskHandle_t             xIec103_0_ProcHandle;
extern TaskHandle_t             xIec103_1_ProcHandle;


extern BaseType_t               task_ready[TASK_NUM_SIZE];


#ifdef _DEBUG_ON_
extern uint32_t                 task_enter_cnt[TASK_NUM_SIZE];
extern uint32_t                 task_stage[TASK_NUM_SIZE];
#endif


//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//								********* Link layer *********
//---------------------------------------------------------------------------------------------
//------------------------------ IEC103 link layer processing ---------------------------------
static uint32_t Iec103LinkCheckRqst (
                                    Iec103_inst_t *pIec103_inst,
                                    uint8_t *rqst_buf,
                                    uint16_t rqst_size,
                                    uint8_t addr,
                                    uint32_t time_stamp
                                    )
{
	uint8_t user_data_start, link_addr;

    // Check start symbol
    switch(rqst_buf[0])
    {
        case VAR_LEN_FRAME_START_SYM:
        // variable length frame
            // check frame length & header (start symbols (0,3), length (1, 2))
            if(	(rqst_buf[3] != rqst_buf[0])||
                (rqst_buf[1] != rqst_buf[2])||
                (rqst_buf[1] != rqst_size - 6) )
                return WRONG_FRAME_ERR;

            pIec103_inst->Rqst_APCI.Ctrl_Byte = rqst_buf[4];
            link_addr =  rqst_buf[5];
            user_data_start = 4;
            pIec103_inst->Rqst_APCI.Data_Size = rqst_buf[1] - 2;
            break;

        case FIX_LEN_FRAME_START_SYM:
        // constant length frame
            pIec103_inst->Rqst_APCI.Ctrl_Byte = rqst_buf[1];
            link_addr = rqst_buf[2];

            // check frame length
            if(rqst_size != FIX_LEN_FRAME_LENGTH)
                return WRONG_FUNC_SIZE_ERR;

            user_data_start = 1;
            pIec103_inst->Rqst_APCI.Data_Size = 0;
            break;

        default:
        // error
            return WRONG_FRAME_ERR;
    }

    // check transfer direction
    if(!TEST_FLAG(pIec103_inst->Rqst_APCI.Ctrl_Byte, PRM_BIT))
        return WRONG_FRAME_ERR;

    // check address
    if(link_addr == 0xff)
    {
        if((pIec103_inst->Rqst_APCI.Ctrl_Byte & FCODE_MASK) == FCODE_CONTR_DATA_NO_ACK)
            return INVALID_ADDR_ERR;
    }
    else if(link_addr != addr)
        // request for unknown server
        // wait for new request
        return INVALID_ADDR_ERR;

    // Check final symbol
    if(rqst_buf[rqst_size - 1] != FIN_SYM)
        return WRONG_FRAME_ERR;

    // Check CRC
    if(rqst_buf[rqst_size - 2] != CalcOrdCRC(rqst_buf + user_data_start, pIec103_inst->Rqst_APCI.Data_Size + 2, 0x00))
        return CRC_ERR;

    // so IEC103 frame - ok
    // Transfer data to IEC103 ASDU
    if(pIec103_inst->Rqst_APCI.Data_Size)
    {
        memcpy( pIec103_inst->Rqst_ASDU,
                rqst_buf + user_data_start + 2,
                pIec103_inst->Rqst_APCI.Data_Size);
    }
    pIec103_inst->Time_Stamp = time_stamp;

    // Come to application layer
    return RESULT_OK;
}


//------------------------------ IEC103 link layer processing ---------------------------------
static uint16_t Iec103LinkSnglCharFrame (
										uint8_t *buf
										)
{
    buf[0] = SINGLE_CHAR_FRAME_START_SYM;
    return 1;
}


//------------------------------ IEC103 link layer processing ---------------------------------
static uint16_t Iec103LinkFixLenFrame	(
                                        Iec103_inst_t *pIec103_inst,
										uint8_t *buf,
										uint8_t addr
										)
{
    buf[0] = FIX_LEN_FRAME_START_SYM;
    buf[1] = pIec103_inst->Resp_APCI.Ctrl_Byte;
    buf[2] = addr;
	buf[3] = CalcOrdCRC(buf + 1, 2, 0x00);
	buf[4] = FIN_SYM;
	pIec103_inst->Resp_APCI.Last_Frame_Size = FIX_LEN_FRAME_LENGTH;
    return FIX_LEN_FRAME_LENGTH;
}


//------------------------------ IEC103 link layer processing ---------------------------------
static uint16_t Iec103LinkVarLenFrame   (
                                        Iec103_inst_t *pIec103_inst,
										uint8_t *buf,
										uint8_t addr
										)
{
	uint16_t frame_len;

	frame_len = pIec103_inst->Resp_APCI.Data_Size + 8;
    buf[0] = VAR_LEN_FRAME_START_SYM;
    buf[1] = frame_len - 6;
    buf[2] = frame_len - 6;
    buf[3] = VAR_LEN_FRAME_START_SYM;
    buf[4] = pIec103_inst->Resp_APCI.Ctrl_Byte;
    buf[5] = addr;
    memcpy( buf + 6,
            pIec103_inst->Resp_ASDU,
            pIec103_inst->Resp_APCI.Data_Size);

	buf[frame_len - 2] = CalcOrdCRC(buf + 4, frame_len - 6, 0x00);
	buf[frame_len - 1] = FIN_SYM;
	pIec103_inst->Resp_APCI.Last_Frame_Size = frame_len;
    return frame_len;
}


//------------------------------ IEC103 link layer processing ---------------------------------
static uint16_t Iec103LinkGetResp   (
                                    Iec103_inst_t *pIec103_inst,
									uint8_t *buf,
									uint8_t addr
									)
{
    if(!TEST_FLAG(pIec103_inst->Flags, f_Repeat_Last_Resp))
    {
        switch(pIec103_inst->Resp_APCI.Frame_Type)
        {
            case SINGLE_CHAR_FRAME:
                return Iec103LinkSnglCharFrame(buf);

            case FIX_LEN_FRAME:
                return Iec103LinkFixLenFrame(pIec103_inst, buf, addr);

            case VAR_LEN_FRAME:
                return Iec103LinkVarLenFrame(pIec103_inst, buf, addr);

            default:
                // no response
                return 0;
        }
    }
    else
    {
        // use last data for transfer (*buf, *size)
        return pIec103_inst->Resp_APCI.Last_Frame_Size;
    }
}


//------------------------------ IEC103 link layer processing ---------------------------------
uint16_t Iec103LinkProcess  (
                            Iec103_inst_t *pIec103_inst,
							uint8_t *rqst_buf,
							uint16_t rqst_size,
							uint8_t *resp_buf,
							uint8_t addr,
                            uint32_t time_stamp
							)
{
    uint16_t data_size;

	if(Iec103LinkCheckRqst(pIec103_inst, rqst_buf, rqst_size, addr, time_stamp) == RESULT_OK)
    {
        xSemaphoreGive(*(pIec103_inst->DataRcvd));
        if(xSemaphoreTake(*(pIec103_inst->DataSent), IEC103_RESP_TIMEOUT) == pdPASS)
		{
            data_size = Iec103LinkGetResp(pIec103_inst, resp_buf, addr);
			CLEAR_FLAG(pIec103_inst->Flags, f_Repeat_Last_Resp);
			return data_size;
		}
        else
            return 0;
    }
    else
        return 0;
}


// ------------------------------------- Check FCB --------------------------------------------
static uint32_t Iec103CheckFcb  (
                                Iec103_inst_t *pIec103_inst
                                )
{
	if(TEST_FLAG((pIec103_inst->Flags >> f_Last_FCB), 0) != TEST_FLAG((pIec103_inst->Rqst_APCI.Ctrl_Byte >> FCB_BIT), 0))
	{
		// Ok
		TOGLE_FLAG(pIec103_inst->Flags, f_Last_FCB);
		return RESULT_OK;
	}
	return RESULT_ALT;
}


// --------------------------------------------------------------------------------------------
static void Iec103FixLenMsg	(
                            Iec103_inst_t *pIec103_inst,
							uint8_t fcode
							)
{
    pIec103_inst->Resp_APCI.Ctrl_Byte &= ~FCODE_MASK;
    pIec103_inst->Resp_APCI.Ctrl_Byte |= (fcode << FCODE_BIT0);
	if(pIec103_inst->Queue.Msg_Queue_Len > 0)
        pIec103_inst->Resp_APCI.Ctrl_Byte |= (1<<ACD_BIT);
    else
        pIec103_inst->Resp_APCI.Ctrl_Byte &= ~(1<<ACD_BIT);
    pIec103_inst->Resp_APCI.Frame_Type = FIX_LEN_FRAME;
}


// --------------------------------------------------------------------------------------------
static void Iec103Confirm   (
                            Iec103_inst_t *pIec103_inst,
                            uint8_t fcode
                            )
{
    if(fcode == FCODE_MONIT_POS_ACK)
    {
        if(pIec103_inst->Settings.Single_Char_Msg_Enbl && (pIec103_inst->Queue.Msg_Queue_Len == 0))
            pIec103_inst->Resp_APCI.Frame_Type = SINGLE_CHAR_FRAME;
        else
            Iec103FixLenMsg(pIec103_inst, FCODE_MONIT_POS_ACK);
    }
    else
        Iec103FixLenMsg(pIec103_inst, FCODE_MONIT_NEG_ACK);
}




// --------------------------------------------------------------------------------------------
//						   ********* Application layer *********
// --------------------------------------------------------------------------------------------
//------------------------------- IEC103 data initialization ----------------------------------
static void Iec103StructInit(
                            Iec103_inst_t *pIec103_inst
                            )
{
	if(pIec103_inst == &Iec103_0)
    {
        pIec103_inst->Rqst_ASDU = Iec103_0_Data_Buf + RAM_IEC103_RQST_DATA_BUF_OFFSET;
        pIec103_inst->Resp_ASDU = Iec103_0_Data_Buf + RAM_IEC103_RESP_DATA_BUF_OFFSET;
        pIec103_inst->Queue.Msg_Queue_Buf = Queue_0_Data_Buf;
        pIec103_inst->DataRcvd = &Iec103_0_DataRcvd_Semph;
        pIec103_inst->DataSent = &Iec103_0_DataSent_Semph;
        pIec103_inst->RqstToApp = &Iec103_0_To_Modbus_Rqst_Semph;
        pIec103_inst->RespFromApp = &Modbus_To_Iec103_0_Resp_Semph;
        pIec103_inst->DIKL_From_Parbus = &DIKL_To_Iec103_0_Semph;
        pIec103_inst->VDI_From_Parbus = &VDI_To_Iec103_0_Semph;
        pIec103_inst->Gen_Cmd.RqstToAppBuf = Iec103_0_Server_Buf + RAM_MODBUS_RQST_DATA_BUF_OFFSET;
        pIec103_inst->Gen_Cmd.RespFromAppBuf = Iec103_0_Server_Buf + RAM_MODBUS_RESP_DATA_BUF_OFFSET;
        pIec103_inst->Settings.Station_Addr = Serbus0_Settings.Address;
        pIec103_inst->Settings.Single_Char_Msg_Enbl = Serbus0_Settings.Additional_Param;
        pIec103_inst->Settings.Pause_1symbol_us = Serbus0_Settings.Pause_1symbol_us;
    }
    else
    {
        pIec103_inst->Rqst_ASDU = Iec103_1_Data_Buf + RAM_IEC103_RQST_DATA_BUF_OFFSET;
        pIec103_inst->Resp_ASDU = Iec103_1_Data_Buf + RAM_IEC103_RESP_DATA_BUF_OFFSET;
        pIec103_inst->Queue.Msg_Queue_Buf = Queue_1_Data_Buf;
        pIec103_inst->DataRcvd = &Iec103_1_DataRcvd_Semph;
        pIec103_inst->DataSent = &Iec103_1_DataSent_Semph;
        pIec103_inst->RqstToApp = &Iec103_1_To_Modbus_Rqst_Semph;
        pIec103_inst->RespFromApp = &Modbus_To_Iec103_1_Resp_Semph;
        pIec103_inst->DIKL_From_Parbus = &DIKL_To_Iec103_1_Semph;
        pIec103_inst->VDI_From_Parbus = &VDI_To_Iec103_1_Semph;
        pIec103_inst->Gen_Cmd.RqstToAppBuf = Iec103_1_Server_Buf + RAM_MODBUS_RQST_DATA_BUF_OFFSET;
        pIec103_inst->Gen_Cmd.RespFromAppBuf = Iec103_1_Server_Buf + RAM_MODBUS_RESP_DATA_BUF_OFFSET;
        pIec103_inst->Settings.Station_Addr = Serbus1_Settings.Address;
        pIec103_inst->Settings.Single_Char_Msg_Enbl = Serbus1_Settings.Additional_Param;
        pIec103_inst->Settings.Pause_1symbol_us = Serbus1_Settings.Pause_1symbol_us;
    }
    pIec103_inst->Rqst_APCI.Data_Size = 0;
    pIec103_inst->Rqst_APCI.Ctrl_Byte = 0;
    pIec103_inst->Rqst_APCI.Frame_Type = 0;
    pIec103_inst->Rqst_APCI.Last_Frame_Size = 0;

    pIec103_inst->Resp_APCI.Data_Size = 0;
    pIec103_inst->Resp_APCI.Ctrl_Byte = 0;
    pIec103_inst->Resp_APCI.Frame_Type = 0;
    pIec103_inst->Resp_APCI.Last_Frame_Size = 0;

    MsgQueueClear(&(pIec103_inst->Queue));

	pIec103_inst->GC_Status = GC_NO;
	pIec103_inst->Gen_Cmd.RqstToAppSize = 0;
	pIec103_inst->Gen_Cmd.RespFromAppSize = 0;

    pIec103_inst->Flags = 0;

    // Initialization of Iec103 protocol data
    pIec103_inst->CI_Entry_Offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_ANLG_IN_MEAS].Start_Offset;
    InterviewStructInit(&pIec103_inst->Spor_Evnt, SE_Tab, SE_DIKLVDI_Buf);
    InterviewStructInit(&pIec103_inst->Gen_Intrv, GI_Tab, 0);

    pIec103_inst->Settings.Station_Name = Id_Data.Name;
    pIec103_inst->Settings.Station_Func = Id_Data.Func;
    SET_FLAG(pIec103_inst->Flags, f_Iec103_Init);
}


// -------------------- Write time data to information of ASDU Inf buffer ---------------------
static void Iec103TimeToInfBuf	(
								Iec103_ASDU_t *Iec103_msg,
								rtc_strct_t *rtc_data
								)
{
	uint8_t temp_sec;
	uint8_t *pntr;
	uint16_t temp_msec, temp;

	if(Iec103_msg->ASDU_TYP == TYP_MONIT_SYNC)
		pntr = Iec103_msg->ASDU_Inf_el + INF_EL_POS_T7;
	else
		pntr = Iec103_msg->ASDU_Inf_el + INF_EL_POS_T4;

    do
    {
        temp_sec = rtc_data->time.sec;
        temp_msec = rtc_data->ms_counter;
        *(pntr + 2) = (rtc_data->time.min  & 0x3f) | (rtc_data->time.inval ? 0x80 : 0x00);
        *(pntr + 3) = (rtc_data->time.hour & 0x1f) | (rtc_data->time.dst ? 0x80 : 0x00);
		if(Iec103_msg->ASDU_TYP == TYP_MONIT_SYNC)
		{
			*(pntr + 4) = rtc_data->time.mday & 0x1f;
			*(pntr + 5) = rtc_data->time.month & 0x0f;
			*(pntr + 6) = rtc_data->time.year & 0x7f;
		}
    }
    while(temp_sec != rtc_data->time.sec);
	temp = (uint16_t)temp_sec * 1000UL + temp_msec;
    *(pntr + 0) = (uint8_t)temp;
    *(pntr + 1) = (uint8_t)(temp >> 8);
}


// -------------------- Read time data from information of ASDU Rx buffer ---------------------
static void Iec103TimeFromInfBuf(
                                uint8_t *buf,
                                rtc_strct_t *rtc_data
                                )
{
	uint16_t temp_msec;

    temp_msec = *(uint16_t *)(buf + 6);
    rtc_data->ms_counter = temp_msec % 1000;
    rtc_data->time.sec = (uint8_t)(temp_msec / 1000);
    rtc_data->time.min = *(buf + 8) & 0x3f;
    rtc_data->time.hour = *(buf + 9) & 0x1f;
    rtc_data->time.mday = *(buf + 10) & 0x1f;
    rtc_data->time.month = *(buf + 11) & 0x0f;
    rtc_data->time.year = *(buf + 12) & 0x7f;
}





// --------------------------------------------------------------------------------------------
//					       		  Communication unit reset
// --------------------------------------------------------------------------------------------
// ------------------------------ Reset CU or FCB handler -------------------------------------
static uint32_t Iec103ResetStartHandler (
                                        Iec103_inst_t *pIec103_inst,
                                        uint8_t *fcode
                                        )
{
    if(*fcode == FCODE_CONTR_RST_CU)
        pIec103_inst->Rst_Status = RST_CU_STATE;
    else
        pIec103_inst->Rst_Status = RST_FCB_STATE;
    return RESULT_OK;
}


// ---------------------------------- Reset CU or FCB -----------------------------------------
static void Iec103ResetProcess  (
                                Iec103_inst_t *pIec103_inst
                                )
{
    Iec103_ASDU_t iec103_asdu;
	uint8_t asdu_cot, asdu_inf;

    // Clear space for ASDU
    uint8_t *pntr;
    pntr = (uint8_t *)&iec103_asdu;
    for(int k = 0; k < sizeof(Iec103_ASDU_t); k++)
        *pntr++ = 0;

    if(pIec103_inst->Rst_Status == RST_CU_STATE)
    {
        // Clear message queue
        MsgQueueClear(&(pIec103_inst->Queue));
    }
    // Abort GI, FD, GS
    pIec103_inst->GI_Status = GI_INIT_STATE;
    // Execute: Reset FCB & put ASDU5 (COT = RST_CU/RST_FCB) to event buffer

	// Reset FCB
	CLEAR_FLAG(pIec103_inst->Flags, f_Last_FCB);

    // Time invalid
//    Rtc_fact.time.inval = 1;

	// Fill ASDU fields
	iec103_asdu.ASDU_TYP                = TYP_MONIT_ID_MSG;
    iec103_asdu.ASDU_Var_Struc_Class    = 0x81;
    iec103_asdu.ASDU_Addr               = pIec103_inst->Settings.Station_Addr;
	iec103_asdu.ASDU_FUN                = pIec103_inst->Settings.Station_Func;

	if(pIec103_inst->Rst_Status == RST_CU_STATE)
	{
		asdu_cot                        = COT_MONIT_RESET_CU;
		asdu_inf                        = INF_RESET_CU;
	}
	else
	{
		asdu_cot                        = COT_MONIT_RESET_FCB;
		asdu_inf                        = INF_RESET_FCB;
	}

    // exit if communication unit isn't initialized
    if(pIec103_inst->Rst_Status != RST_CMPLT_STATE)
	{
        pIec103_inst->Rst_Status = RST_CMPLT_STATE;

		// Put ASDU5 with COT=START to queue head
		iec103_asdu.ASDU_COT            = COT_MONIT_START;
		iec103_asdu.ASDU_INF            = INF_START;
		MsgAddToQueueHead(  &(pIec103_inst->Queue),
                            (uint8_t *)&iec103_asdu,
                            sizeof(Iec103_ASDU_t)   );
	}
	// Put ASDU5 with COT=RESET CU/FCB to queue head
	iec103_asdu.ASDU_COT                = asdu_cot;
	iec103_asdu.ASDU_INF                = asdu_inf;
    MsgAddToQueueHead(  &(pIec103_inst->Queue),
                        (uint8_t *)&iec103_asdu,
                        sizeof(Iec103_ASDU_t)   );
}


// --------------------------------------------------------------------------------------------
//						   					Synchronization
//---------------------------------------------------------------------------------------------
static uint32_t Iec103SyncStartHandler  (
                                        Iec103_inst_t *pIec103_inst
                                        )
{
    pIec103_inst->Sync_Status = SYNC_START_STATE;
    return RESULT_OK;
}


// --------------------------------------------------------------------------------------------
void Iec103SyncProcess  (
                        Iec103_inst_t *pIec103_inst
                        )
{
    Iec103_ASDU_t iec103_asdu;

    // Clear space for ASDU
    uint8_t *pntr;
    pntr = (uint8_t *)&iec103_asdu;
    for(int k = 0; k < sizeof(Iec103_ASDU_t); k++)
        *pntr++ = 0;

    if(Rtc_fact.rtc_state == NO_CMD)
    {
        // Get sync time from ASDU Rx Buffer
        Iec103TimeFromInfBuf(pIec103_inst->Rqst_ASDU, &Rtc_sync);

        // Time conversion (Date-Time -> sec)
        Rtc_sync.s_counter = GetRtcCounterFromTime(&Rtc_sync.time, Rtc_fact.summer_time_need);

        portENTER_CRITICAL();
        // time stamp & sync delay transfering to Rtc_sync
        Rtc_sync.rtc_Time_Stamp = pIec103_inst->Time_Stamp;
        Rtc_sync.rtc_Time_Delay = pIec103_inst->Settings.Pause_1symbol_us * 72;

        // Start synchronization
        Rtc_fact.rtc_state = RTC_SYNC_START;
        portEXIT_CRITICAL();
    }

    else if(Rtc_fact.rtc_state == RTC_SYNC_CMPLT)
    {
        Rtc_fact.rtc_state = NO_CMD;

        // Fill ASDU fields
        iec103_asdu.ASDU_TYP                = TYP_MONIT_SYNC;
        iec103_asdu.ASDU_Var_Struc_Class    = 0x81;
        iec103_asdu.ASDU_COT                = COT_MONIT_SYNC;
        iec103_asdu.ASDU_Addr               = pIec103_inst->Settings.Station_Addr;
        iec103_asdu.ASDU_FUN                = FUN_GLB;
        iec103_asdu.ASDU_INF                = INF_TIME_SYNC;

        // Add ASDU message to queue tail
        MsgAddToQueueTail(  &(pIec103_inst->Queue),
                            (uint8_t *)&iec103_asdu,
                            sizeof(Iec103_ASDU_t)    );
        // ASDU located to Queue

        pIec103_inst->Sync_Status = SYNC_WAIT_STATE;
    }
}


// --------------------------------------------------------------------------------------------
//						   				General Interview
// --------------------------------------------------------------------------------------------
static uint32_t Iec103GenIntervStartHandler (
                                            Iec103_inst_t *pIec103_inst
                                            )
{
    Iec103_ASDU_t *iec103_asdu;
    iec103_asdu = (Iec103_ASDU_t *)pIec103_inst->Rqst_ASDU;

	if(!TEST_FLAG(Iec103_Rel_Tab_Structure.Flags, f_Table_Ok))
		return REL_TAB_ERR;

	pIec103_inst->GI_Status = GI_ACTIVE_STATE;
	pIec103_inst->Gen_Intrv.Signal_Num = 0;
	pIec103_inst->Gen_Intrv.Scn = iec103_asdu->ASDU_Inf_el[INF_EL_POS_SCN];
	return RESULT_OK;
}


//---------------------------------------------------------------------------------------------
static void Iec103GenIntervFinalAsdu(
                                    Iec103_inst_t *pIec103_inst
                                    )
{
    Iec103_ASDU_t iec103_asdu;

    // Clear space for ASDU
    uint8_t *pntr;
    pntr = (uint8_t *)&iec103_asdu;
    for(int k = 0; k < sizeof(Iec103_ASDU_t); k++)
        *pntr++ = 0;

    // Fill ASDU fields
    iec103_asdu.ASDU_TYP                    = TYP_MONIT_GI_FIN;
    iec103_asdu.ASDU_Var_Struc_Class        = 0x81;
    iec103_asdu.ASDU_COT                    = COT_MONIT_GI_FIN;
    iec103_asdu.ASDU_Addr                   = pIec103_inst->Settings.Station_Addr;
    iec103_asdu.ASDU_FUN                    = FUN_GLB;
    iec103_asdu.ASDU_INF                    = INF_GI_FIN;
    iec103_asdu.ASDU_Inf_el[INF_EL_POS_SCN]	= 0;

    // Add ASDU message to queue head
    MsgAddToQueueTail(  &(pIec103_inst->Queue),
                        (uint8_t *)&iec103_asdu,
                        sizeof(Iec103_ASDU_t)    );
    // ASDU located to Queue

    pIec103_inst->GI_Status = GI_INIT_STATE;
}


// --------------------------- Get ASDU with general interview data ---------------------------
static uint32_t Iec103GenIntervDataAsdu (
                                        Iec103_inst_t *pIec103_inst
                                        )
{
    Iec103_ASDU_t iec103_asdu;
    uint32_t sgnl_entry_offset, data_set_offset;
	uint8_t *data_buf, byte_num, bit_num;

    // Clear space for ASDU
    uint8_t *pntr;
    pntr = (uint8_t *)&iec103_asdu;
    for(int k = 0; k < sizeof(Iec103_ASDU_t); k++)
        *pntr++ = 0;

    // Fill ASDU fields
    iec103_asdu.ASDU_TYP                		= TYP_MONIT_TIME_TAG_MSG;
    iec103_asdu.ASDU_Var_Struc_Class    		= 0x81;
    iec103_asdu.ASDU_COT                		= COT_MONIT_GI;
    iec103_asdu.ASDU_Addr               		= pIec103_inst->Settings.Station_Addr;
    iec103_asdu.ASDU_FUN                		= pIec103_inst->Settings.Station_Func;

    // Search signals & set INF, DPI
    uint32_t sgnl_amount = Iec103_Rel_Tab_Structure.Rel_Tab[(IEC103_DISCR_INOUT_STATE - 1) * REL_ADDR_TABLE_LINE_LEN + 4];
    for(uint32_t i = pIec103_inst->Gen_Intrv.Signal_Num; i < sgnl_amount; i++)
    {
		// Analyze enabled signals (VDI, DI, KL)
		if(i < VDI_AMOUNT)
        {
            data_set_offset = EEPROM_INTERVIEW_VDI_OFFSET;
            byte_num = i / 8;
            bit_num = i % 8;
        }
        else if(i < (VDI_AMOUNT + DI_AMOUNT))
        {
            data_set_offset = EEPROM_INTERVIEW_DI_OFFSET;
            byte_num = (i - VDI_AMOUNT) / 8;
            bit_num = (i - VDI_AMOUNT) % 8;
        }
        else
        {
            data_set_offset = EEPROM_INTERVIEW_KL_OFFSET;
            byte_num = (i - VDI_AMOUNT - DI_AMOUNT) / 8;
            bit_num = (i - VDI_AMOUNT - DI_AMOUNT) % 8;
        }

		if(!(pIec103_inst->Gen_Intrv.Data_Set[data_set_offset + byte_num] & 1<<bit_num))
			continue;

		sgnl_entry_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_INOUT_STATE].Start_Offset + i * REL_ADDR_TABLE_LINE_LEN;
		// entry is found (signal_entry_offset)

		// ------------- Extract data from buffer --------------
		// we have: signal_entry_offset
		// source buffer or error
		switch(Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 1])
		{
			case 0x81:
				data_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
				break;
			case 0x83:
				data_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
				break;
			default:
				// rel_tab error
				data_buf = 0;
				break;
		}

		if(data_buf)
		{
			iec103_asdu.ASDU_INF = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset];
			byte_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 2];
			bit_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 3];
			iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = (*(data_buf + byte_num) & 1<<bit_num) ? 2 : 1;
		}
		else
		{
			// tab error
			iec103_asdu.ASDU_INF = 0;
			iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = 3;
		}
        // Add time lable
        Iec103TimeToInfBuf(&iec103_asdu, &Rtc_fact);

        // INF & DPI have gotten -> save signal_num & return
        pIec103_inst->Gen_Intrv.Signal_Num = ++i;

        // Add ASDU message to queue head
        MsgAddToQueueTail(  &(pIec103_inst->Queue),
                            (uint8_t *)&iec103_asdu,
                            sizeof(Iec103_ASDU_t)    );
        // ASDU located to Queue
        return RESULT_OK;
    }
    return RESULT_ALT;
}


// --------------------------------------------------------------------------------------------
static void Iec103GenIntervProcess  (
                                    Iec103_inst_t *pIec103_inst
                                    )
{
    // if GI is not active - error
    if(pIec103_inst->GI_Status != GI_ACTIVE_STATE)
        return;

    // Check GI signals
    if(Iec103GenIntervDataAsdu(pIec103_inst) == RESULT_OK)
        return;
    else
    {
        // Finish general interview
        Iec103GenIntervFinalAsdu(pIec103_inst);
        return;
    }
}


// --------------------------------------------------------------------------------------------
//						   			 General Commands transfer
//---------------------------------------------------------------------------------------------
static uint32_t Iec103GenCmdStartHandler(
                                        Iec103_inst_t *pIec103_inst
                                        )
{
    Iec103_ASDU_t *iec103_asdu;
    iec103_asdu = (Iec103_ASDU_t *)(pIec103_inst->Rqst_ASDU);

    if( //Iec103GenCmdCheckAsdu() == RESULT_OK &&
        iec103_asdu->ASDU_FUN == pIec103_inst->Settings.Station_Func )
    {
        if( (iec103_asdu->ASDU_INF == INF_CMD_KVIT)  ||
            (iec103_asdu->ASDU_INF == INF_CMD_SWITCH)||
            (iec103_asdu->ASDU_INF == INF_CMD_OSC)   ||
            (iec103_asdu->ASDU_INF >= INF_CMD_KL1 && iec103_asdu->ASDU_INF <= INF_CMD_KL40))
        {
            if(pIec103_inst->GC_Status == GC_NO)
            {
                pIec103_inst->GC_Status = GC_RCVD;

                // Put data to Gen_Cmd structure
                pIec103_inst->Gen_Cmd.DCO = iec103_asdu->ASDU_Inf_el[INF_EL_POS_DCO];
                pIec103_inst->Gen_Cmd.RII = iec103_asdu->ASDU_Inf_el[INF_EL_POS_RII];
                pIec103_inst->Gen_Cmd.Cmd_INF = iec103_asdu->ASDU_INF;
                return RESULT_OK;
            }
            else
                return RESULT_ALT;
        }
        else
            return RESULT_ALT;
    }
    return RESULT_ALT;
}


// --------------------------------------------------------------------------------------------
static uint8_t Iec103RqstToModbus  	(
                                    Iec103_inst_t *pIec103_inst
									)
{
    uint32_t start_offset, final_offset, entry_offset;
	uint8_t *new_line_pntr;

	if(!TEST_FLAG(Iec103_Rel_Tab_Structure.Flags, f_Table_Ok))
		return 0;

	switch(pIec103_inst->Gen_Cmd.DCO)
	{
		case 1:
			// Off
			start_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_OUT_OFF].Start_Offset;
			final_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_OUT_OFF].Final_Offset;
			pIec103_inst->Gen_Cmd.RqstToAppBuf[4] = 0;
			break;

		case 2:
			// On
			start_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_OUT_ON].Start_Offset;
			final_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_OUT_ON].Final_Offset;
			pIec103_inst->Gen_Cmd.RqstToAppBuf[4] = 0x0f;
			break;

		default:
			return 0;
	}

	if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
		return 0;

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(pIec103_inst->Gen_Cmd.Cmd_INF == Iec103_Rel_Tab_Structure.Rel_Tab[offset])
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
		return 0;
    // entry is found (entry_offset)

	// Prepare Modbus request 0x42
	new_line_pntr = Iec103_Rel_Tab_Structure.Rel_Tab + entry_offset;

	// copy request from table to Iec103 request
	memcpy(pIec103_inst->Gen_Cmd.RqstToAppBuf, new_line_pntr + 1, 3);
	pIec103_inst->Gen_Cmd.RqstToAppBuf[3] = 1;
	return 5;
}


// --------------------------------------------------------------------------------------------
static void Iec103GenCmdProcess (
                                Iec103_inst_t *pIec103_inst
                                )
{
    static Iec103_ASDU_t iec103_asdu;

    // if general command is not active - exit
    if(pIec103_inst->GC_Status == GC_NO)
        return;

    // Clear space for ASDU
    uint8_t *pntr;
    pntr = (uint8_t *)&iec103_asdu;
    for(int k = 0; k < sizeof(Iec103_ASDU_t); k++)
        *pntr++ = 0;

    // if general command received
    if(pIec103_inst->GC_Status == GC_RCVD)
    {
		// prepare Cmd data for request to ModBus (0x42 ah al 0x01 0x0f)
		pIec103_inst->Gen_Cmd.RqstToAppSize = Iec103RqstToModbus(pIec103_inst);
		xSemaphoreGive(*(pIec103_inst->RqstToApp));
        if(xSemaphoreTake(*(pIec103_inst->RespFromApp), IEC103_RESP_TIMEOUT) == pdPASS)
        {
            if(pIec103_inst->Gen_Cmd.RespFromAppBuf[4] == pIec103_inst->Gen_Cmd.RqstToAppBuf[4])
			{
				// put positive ack & add to queue head
				// Fill ASDU fields
				iec103_asdu.ASDU_TYP                	= TYP_MONIT_TIME_TAG_MSG;
				iec103_asdu.ASDU_Var_Struc_Class    	= 0x81;
				iec103_asdu.ASDU_COT                	= COT_MONIT_CMD_ACK;
				iec103_asdu.ASDU_Addr               	= pIec103_inst->Settings.Station_Addr;
				iec103_asdu.ASDU_FUN                	= pIec103_inst->Settings.Station_Func;
				iec103_asdu.ASDU_INF                	= pIec103_inst->Gen_Cmd.Cmd_INF;
				iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI]	= 0;
				iec103_asdu.ASDU_Inf_el[INF_EL_POS_SIN] = pIec103_inst->Gen_Cmd.RII;
				Iec103TimeToInfBuf(&iec103_asdu, &Rtc_fact);
				MsgAddToQueueHead(  &(pIec103_inst->Queue),
									(uint8_t *)&iec103_asdu,
									sizeof(Iec103_ASDU_t)    );

				// put new state message & add to general command messages
				iec103_asdu.ASDU_COT                	= COT_MONIT_REMOTE_OP;
				iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI]	= pIec103_inst->Gen_Cmd.DCO;
				iec103_asdu.ASDU_Inf_el[INF_EL_POS_SIN] = pIec103_inst->Gen_Cmd.RII;
				MsgAddToQueueHead(  &(pIec103_inst->Queue),
									(uint8_t *)&iec103_asdu,
									sizeof(Iec103_ASDU_t)    );

				pIec103_inst->GC_Status = GC_NO;
				return;
			}
        }
		// Timeout is over or command is not available
		// put negative ack & add to queue head
		iec103_asdu.ASDU_TYP                	= TYP_MONIT_TIME_TAG_MSG;
		iec103_asdu.ASDU_Var_Struc_Class    	= 0x81;
		iec103_asdu.ASDU_COT                	= COT_MONIT_CMD_NACK;
		iec103_asdu.ASDU_Addr               	= pIec103_inst->Settings.Station_Addr;
		iec103_asdu.ASDU_FUN                	= pIec103_inst->Settings.Station_Func;
		iec103_asdu.ASDU_INF                	= pIec103_inst->Gen_Cmd.Cmd_INF;
		iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI]	= 0;
		iec103_asdu.ASDU_Inf_el[INF_EL_POS_SIN] = iec103_asdu.ASDU_Inf_el[INF_EL_POS_RII];
		Iec103TimeToInfBuf(&iec103_asdu, &Rtc_fact);
        MsgAddToQueueHead(  &(pIec103_inst->Queue),
                            (uint8_t *)&iec103_asdu,
                            sizeof(Iec103_ASDU_t)    );
    }
    pIec103_inst->GC_Status = GC_NO;
    return;
}




// --------------------------------------------------------------------------------------------
//						   				   Sporadic events
//---------------------------------------------------------------------------------------------
static void Iec103SporadEventsMonitor   (
                                        Iec103_inst_t *pIec103_inst
                                        )
{
    Iec103_ASDU_t iec103_asdu;
    uint8_t *old_data_buf, *data_buf, byte_num, bit_num;
    uint32_t sgnl_entry_offset;

    // Clear space for ASDU
    uint8_t *pntr;
    pntr = (uint8_t *)&iec103_asdu;
    for(int k = 0; k < sizeof(Iec103_ASDU_t); k++)
        *pntr++ = 0;

    // Fill ASDU fields
    iec103_asdu.ASDU_TYP                		= TYP_MONIT_TIME_TAG_MSG;
    iec103_asdu.ASDU_Var_Struc_Class    		= 0x81;
    iec103_asdu.ASDU_COT                		= COT_MONIT_SPORAD;
    iec103_asdu.ASDU_Addr               		= pIec103_inst->Settings.Station_Addr;
    iec103_asdu.ASDU_FUN                		= pIec103_inst->Settings.Station_Func;
    Iec103TimeToInfBuf(&iec103_asdu, &Rtc_fact);

    // Check Virtual DI
    if(xSemaphoreTake(*(pIec103_inst->VDI_From_Parbus), 0) == pdPASS)
    {
        for(uint32_t i = 0; i < VDI_AMOUNT; i++)
        {
            if(!(pIec103_inst->Spor_Evnt.Data_Set[EEPROM_INTERVIEW_VDI_OFFSET + i / 8] & 1<<(i % 8)))
                continue;

            sgnl_entry_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_INOUT_STATE].Start_Offset + i * REL_ADDR_TABLE_LINE_LEN;
            // entry is found (signal_entry_offset)

            // ------------- Extract data from buffer --------------
            // we have: signal_entry_offset
            // result: data buffer
            if(Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 1] == 0x81)
            {
				data_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
				old_data_buf = pIec103_inst->Spor_Evnt.Old_Data;
            }
            else
                // rel_tab error
                data_buf = 0;

            if(data_buf)
            {
                byte_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 2];
                bit_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 3];
				if((*(data_buf + byte_num) & 1<<bit_num) != (*(old_data_buf + byte_num) & 1<<bit_num))
				{
					iec103_asdu.ASDU_INF = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset];
					iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = (*(data_buf + byte_num) & 1<<bit_num) ? 2 : 1;
				}
				else
					continue;
            }
            else
            {
                // tab error
                iec103_asdu.ASDU_INF = 0;
                iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = 3;
            }

			// Add ASDU to queue tail
			MsgAddToQueueTail(	&(pIec103_inst->Queue),
								(uint8_t *)&iec103_asdu,
								sizeof(Iec103_ASDU_t)	);
            // ASDU located to Queue
        }
        // Save new VDI state
        vTaskSuspendAll();
        memcpy( pIec103_inst->Spor_Evnt.Old_Data,
                Ied_Data_Buf + RAM_VIRT_DAT_OFFSET,
                RAM_VIRT_DAT_SIZE					);
        xTaskResumeAll();
    }

    // Check DI & KL
    if(xSemaphoreTake(*(pIec103_inst->DIKL_From_Parbus), 0) == pdPASS)
    {
        for(uint32_t i = 0; i < DI_AMOUNT; i++)
        {
            if(!(pIec103_inst->Spor_Evnt.Data_Set[EEPROM_INTERVIEW_DI_OFFSET + i / 8] & 1<<(i % 8)))
                continue;

            sgnl_entry_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_INOUT_STATE].Start_Offset + (i + VDI_AMOUNT) * REL_ADDR_TABLE_LINE_LEN;
            // entry is found (signal_entry_offset)

            // ------------- Extract data from buffer --------------
            // we have: signal_entry_offset
            // result: data buffer
            if(Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 1] == 0x83)
            {
				data_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
				old_data_buf = pIec103_inst->Spor_Evnt.Old_Data + RAM_VIRT_DAT_SIZE;
            }
            else
                // rel_tab error
                data_buf = 0;

            if(data_buf)
            {
                byte_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 2];
                bit_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 3];
				if((*(data_buf + byte_num) & 1<<bit_num) != (*(old_data_buf + byte_num) & 1<<bit_num))
				{
					iec103_asdu.ASDU_INF = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset];
					iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = (*(data_buf + byte_num) & 1<<bit_num) ? 2 : 1;
				}
				else
					continue;
            }
            else
            {
                // tab error
                iec103_asdu.ASDU_INF = 0;
                iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = 3;
            }

			// Add ASDU to queue tail
			MsgAddToQueueTail(	&(pIec103_inst->Queue),
								(uint8_t *)&iec103_asdu,
								sizeof(Iec103_ASDU_t)	);
            // ASDU located to Queue
        }

        for(uint32_t i = 0; i < KL_AMOUNT; i++)
        {
            if(!(pIec103_inst->Spor_Evnt.Data_Set[EEPROM_INTERVIEW_KL_OFFSET + i / 8] & 1<<(i % 8)))
                continue;

            sgnl_entry_offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_DISCR_INOUT_STATE].Start_Offset + (i + VDI_AMOUNT + DI_AMOUNT) * REL_ADDR_TABLE_LINE_LEN;
            // entry is found (signal_entry_offset)

            // ------------- Extract data from buffer --------------
            // we have: signal_entry_offset
            // result: data buffer
            if(Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 1] == 0x83)
            {
				data_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
				old_data_buf = pIec103_inst->Spor_Evnt.Old_Data + RAM_VIRT_DAT_SIZE;
            }
            else
                // rel_tab error
                data_buf = 0;

            if(data_buf)
            {
                byte_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 2];
                bit_num = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset + 3];
				if((*(data_buf + byte_num) & 1<<bit_num) != (*(old_data_buf + byte_num) & 1<<bit_num))
				{
					iec103_asdu.ASDU_INF = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset];
					iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = (*(data_buf + byte_num) & 1<<bit_num) ? 2 : 1;
				}
				else
					continue;
            }
            else
            {
                // tab error
                iec103_asdu.ASDU_INF = 0;
                iec103_asdu.ASDU_Inf_el[INF_EL_POS_DPI] = 3;
            }

			// Add ASDU to queue tail
			MsgAddToQueueTail(	&(pIec103_inst->Queue),
								(uint8_t *)&iec103_asdu,
								sizeof(Iec103_ASDU_t)	);
            // ASDU located to Queue
        }

        // Save new DIKL state
        vTaskSuspendAll();
        memcpy( pIec103_inst->Spor_Evnt.Old_Data + RAM_VIRT_DAT_SIZE,
                Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET,
                RAM_DISCRET_DAT_SIZE 					);
        xTaskResumeAll();
    }
}





// --------------------------------------------------------------------------------------------
//						                Data class 1 request
// --------------------------------------------------------------------------------------------
static void Iec103DataClass1RqstHandler (
                                        Iec103_inst_t *pIec103_inst
                                        )
{
    Iec103_ASDU_t *iec103_asdu;
    iec103_asdu = (Iec103_ASDU_t *)pIec103_inst->Resp_ASDU;

    if(Iec103CheckFcb(pIec103_inst) == RESULT_OK)
    {
        // Get data from queue
		if(MsgGetFromQueue( &(pIec103_inst->Queue),
                            pIec103_inst->Resp_ASDU,
                            sizeof(Iec103_ASDU_t)    ) == RESULT_OK)
        {
            switch(iec103_asdu->ASDU_TYP)
            {
                case TYP_MONIT_TIME_TAG_MSG:
                    pIec103_inst->Resp_APCI.Data_Size = TYP_MONIT_TIME_TAG_MSG_LEN;
                    break;

                case TYP_MONIT_ID_MSG:
					// load IED ID to ASDU_Inf_Buf
					iec103_asdu->ASDU_Inf_el[INF_EL_POS_COL] = COL_NO_GS;
					memcpy( iec103_asdu->ASDU_Inf_el + INF_EL_POS_IED_ID,
							pIec103_inst->Settings.Station_Name,
							8                                    );
					memcpy( iec103_asdu->ASDU_Inf_el + INF_EL_POS_SW_ID,
							FW_VERS + 4,
							4                                    );
                    pIec103_inst->Resp_APCI.Data_Size = TYP_MONIT_ID_MSG_LEN;
                    break;

                case TYP_MONIT_SYNC:
                    Iec103TimeToInfBuf(iec103_asdu, &Rtc_fact);
                    pIec103_inst->Resp_APCI.Data_Size = TYP_MONIT_SYNC_LEN;
                    break;

                case TYP_MONIT_GI_FIN:
                    pIec103_inst->Resp_APCI.Data_Size = TYP_MONIT_GI_FIN_LEN;
                    break;

                default:
                    break;
            }
            pIec103_inst->Resp_APCI.Ctrl_Byte &= ~FCODE_MASK;
            pIec103_inst->Resp_APCI.Ctrl_Byte |= (FCODE_MONIT_DATA_RESP << FCODE_BIT0);
            if(pIec103_inst->Queue.Msg_Queue_Len > 0)
                pIec103_inst->Resp_APCI.Ctrl_Byte |= (1<<ACD_BIT);
            else
                pIec103_inst->Resp_APCI.Ctrl_Byte &= ~(1<<ACD_BIT);
            pIec103_inst->Resp_APCI.Frame_Type = VAR_LEN_FRAME;
		}
        else
		{
			// no any asdu
			Iec103FixLenMsg(pIec103_inst, FCODE_MONIT_NO_DATA_RESP);
		}
    }
    else
    {
        SET_FLAG(pIec103_inst->Flags, f_Repeat_Last_Resp);
    }
}






// --------------------------------------------------------------------------------------------
//						   			    Data class 2 request
//---------------------------------------------------------------------------------------------
static uint32_t Iec103MsgGetFromCyclInterv  (
                                            Iec103_inst_t *pIec103_inst
                                            )
{
    uint32_t sgnl_entry_offset, i;
    uint8_t inf;
    Iec103_ASDU_t *iec103_asdu;
    iec103_asdu = (Iec103_ASDU_t *)pIec103_inst->Resp_ASDU;

    // Fill ASDU fields
    iec103_asdu->ASDU_TYP                = TYP_MONIT_MEASURE_VAL;
    iec103_asdu->ASDU_COT                = COT_MONIT_CYCLIC;
    iec103_asdu->ASDU_Addr               = pIec103_inst->Settings.Station_Addr;
    iec103_asdu->ASDU_FUN                = pIec103_inst->Settings.Station_Func;

    // Get INF
	sgnl_entry_offset = pIec103_inst->CI_Entry_Offset;
    inf = Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset];
    if(inf == 0xff)
        return RESULT_ALT;
    iec103_asdu->ASDU_INF                = inf;

    // Get ASDU_Var_Struc_Class
    i = 0;
    do
    {
        sgnl_entry_offset += REL_ADDR_TABLE_LINE_LEN;
        i++;
    }
    while(Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset] == inf);

    iec103_asdu->ASDU_Var_Struc_Class    = i;
    return RESULT_OK;
}


//---------------------------------------------------------------------------------------------
static uint16_t Iec103CyclIntervConvToMea  	(
											uint8_t *data_tab,
											uint32_t entry_offset,
											uint32_t elem_num
											)
{
    uint16_t temp16;

    uint8_t *entry_pntr, *source_buf, source_num, data_offset, data_size, data_signed, sign = 0;
	uint32_t meas_val, norm_val;

	entry_pntr = data_tab + entry_offset + (elem_num * REL_ADDR_TABLE_LINE_LEN);
    source_num = entry_pntr[1];
	norm_val = *(uint32_t *)(entry_pntr + 4);
    if(norm_val == 0 || source_num != 0x82)
		return (uint16_t)(1 << MEA_ER);

	source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
    data_offset = entry_pntr[2];
	data_signed = (entry_pntr[3] & 0x80) ? 1 : 0;
	data_size = entry_pntr[3] & ~0x80;
	if(data_size > 0 && data_size < 5)
	{
        vTaskSuspendAll();
		meas_val = 0;
		for(uint8_t i = 0; i < data_size; i++)
		{
			meas_val |= (*(source_buf+data_offset+(data_size-1)-i))<< 8*i;
		}
        xTaskResumeAll();
	}
	else
		meas_val = 0xffffffff;

	if(data_signed && (meas_val & (1 << (8*data_size - 1))))
	{
		sign = 1;
		meas_val &= ~(1 << (8*data_size - 1));
	}

    if(meas_val > norm_val)
		return (uint16_t)((MAX_MEA << MEA_LSB) + (1 << MEA_ER) + (1 << MEA_OV));

	temp16 = (uint16_t)(((uint64_t)meas_val << 12) / norm_val);
    if(temp16 == 4096)
		--temp16;
	if(sign)
        temp16 = -((int16_t)temp16);

    temp16 = ((int16_t)temp16 << 3) & 0xfff8;
    return temp16;
}


//---------------------------------------------------------------------------------------------
static void Iec103CyclIntervAsduInfToRespBuf(
                                            Iec103_inst_t *pIec103_inst
                                            )
{
    uint32_t sgnl_entry_offset, N_elem;
    uint16_t mea_val;
    uint8_t *tab;
    Iec103_ASDU_t *iec103_asdu;
    iec103_asdu = (Iec103_ASDU_t *)pIec103_inst->Resp_ASDU;

    sgnl_entry_offset = pIec103_inst->CI_Entry_Offset;
    N_elem = iec103_asdu->ASDU_Var_Struc_Class & ~0x80;
    tab = Iec103_Rel_Tab_Structure.Rel_Tab;

    for(uint32_t i = 0; i < N_elem; i++)
    {
        // Get MEA
        mea_val = Iec103CyclIntervConvToMea(tab, sgnl_entry_offset, i);

        // Put MEA to ASDU
        iec103_asdu->ASDU_Inf_el[2*i] = (uint8_t)mea_val;
        iec103_asdu->ASDU_Inf_el[2*i + 1] = (uint8_t)(mea_val >> 8);
    }
    // Now ASDU_Inf in [Resp_ASDU]

    pIec103_inst->Resp_APCI.Data_Size = (6 + N_elem * 2);

    // New value of CI_Entry_Offset
    sgnl_entry_offset += (N_elem * REL_ADDR_TABLE_LINE_LEN);
    if(Iec103_Rel_Tab_Structure.Rel_Tab[sgnl_entry_offset] == 0xff)
        pIec103_inst->CI_Entry_Offset = Iec103_Rel_Tab_Structure.Region_Tab[IEC103_ANLG_IN_MEAS].Start_Offset;
    else
        pIec103_inst->CI_Entry_Offset = sgnl_entry_offset;
}


// --------------------------------------------------------------------------------------------
static void Iec103DataClass2RqstHandler (
                                        Iec103_inst_t *pIec103_inst
                                        )
{
    if(Iec103CheckFcb(pIec103_inst) == RESULT_OK)
    {
		// Check Iec103_Tab
		if(!TEST_FLAG(Iec103_Rel_Tab_Structure.Flags, f_Table_Ok))
		{
			Iec103FixLenMsg(pIec103_inst, FCODE_MONIT_NEG_ACK);
			return;
		}

		// Go to analize ASDU
        if(Iec103MsgGetFromCyclInterv(pIec103_inst) == RESULT_OK)
        {
            Iec103CyclIntervAsduInfToRespBuf(pIec103_inst);

            pIec103_inst->Resp_APCI.Ctrl_Byte &= ~FCODE_MASK;
            pIec103_inst->Resp_APCI.Ctrl_Byte |= (FCODE_MONIT_DATA_RESP << FCODE_BIT0);
            if(pIec103_inst->Queue.Msg_Queue_Len > 0)
                pIec103_inst->Resp_APCI.Ctrl_Byte |= (1<<ACD_BIT);
            else
                pIec103_inst->Resp_APCI.Ctrl_Byte &= ~(1<<ACD_BIT);
            pIec103_inst->Resp_APCI.Frame_Type = VAR_LEN_FRAME;
        }
        else
		{
			// no any asdu
			Iec103FixLenMsg(pIec103_inst, FCODE_MONIT_NO_DATA_RESP);
		}
    }
    else
    {
        // Repeat last response
        SET_FLAG(pIec103_inst->Flags, f_Repeat_Last_Resp);
    }
}



// --------------------------------------------------------------------------------------------
//							             Link state request
// --------------------------------------------------------------------------------------------
static void Iec103LinkStateRqstHandler  (
                                        Iec103_inst_t *pIec103_inst
                                        )
{
	Iec103FixLenMsg(pIec103_inst, FCODE_MONIT_LINK_STATE_RESP);
}




// --------------------------------------------------------------------------------------------
//								            Testing mode
// --------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------
//							          Monitor direction block
// --------------------------------------------------------------------------------------------




// --------------------------------------------------------------------------------------------
//								        Fault data transfer
// --------------------------------------------------------------------------------------------




// ------------------------------------ Received user data ------------------------------------
static uint32_t Iec103RcvdUserDataHandler   (
                                            Iec103_inst_t *pIec103_inst,
                                            uint8_t *fcode
                                            )
{
    Iec103_ASDU_t *iec103_asdu;
    iec103_asdu = (Iec103_ASDU_t *)(pIec103_inst->Rqst_ASDU);

    if(*fcode == FCODE_CONTR_DATA_NEED_ACK && Iec103CheckFcb(pIec103_inst) != RESULT_OK)
    {
        // Repeat last response
        SET_FLAG(pIec103_inst->Flags, f_Repeat_Last_Resp);
        return RESULT_OK;
    }
    else
    {
        switch(iec103_asdu->ASDU_TYP)
        {
            case TYP_CONTR_SYNC:
                return Iec103SyncStartHandler(pIec103_inst);

            case TYP_CONTR_GI_START:
                return Iec103GenIntervStartHandler(pIec103_inst);

            case TYP_CONTR_GEN_CMD:
                return Iec103GenCmdStartHandler(pIec103_inst);

            default:
                break;
        }
    }
    return WRONG_ASDU_ERR;
}


//----------------------------------- Iec103 ADU handler task ---------------------------------
void vIec103Task(
                void *pvParameters
                )
{
    uint8_t func_code;
    Iec103_inst_t *pIec103_inst;
    uint32_t task_num;

    pIec103_inst = (Iec103_inst_t *)pvParameters;

#ifdef _DEBUG_ON_
    if(pIec103_inst == &Iec103_0)
        task_num = IEC103_0_TASK_NUM;
    else
        task_num = IEC103_1_TASK_NUM;
#endif

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[task_num] = 0;
        ++task_enter_cnt[task_num];
#endif
        // if Iec103 frame has received
        xSemaphoreTake(*(pIec103_inst->DataRcvd), portMAX_DELAY);
#ifdef _DEBUG_ON_
        task_stage[task_num] = 1;
#endif
        // Received pack processing
        // --------------------- Functional codes processing ------------------------
        func_code = pIec103_inst->Rqst_APCI.Ctrl_Byte & FCODE_MASK;
        if(pIec103_inst->Rst_Status != RST_CMPLT_STATE)
        {
            // only Reset allowed
            if( func_code == FCODE_CONTR_RST_CU ||     // REQ -> 	<- CON 		FCV = 0		(S2)    Ok
                func_code == FCODE_CONTR_RST_FCB    )  // REQ -> 	<- CON 		FCV = 0		(S2)    Ok
            {
                    // Reset CU/FCB
                if(Iec103ResetStartHandler(pIec103_inst, &func_code) == RESULT_OK)
                {
                    // Confirmation
                    Iec103Confirm(pIec103_inst, FCODE_MONIT_POS_ACK);
                }
            }
        }
        else
        {
            // everything allowed
            switch(func_code)
            {
                case FCODE_CONTR_RST_CU:                // REQ -> 	<- CON 		FCV = 0		(S2)    Ok
                case FCODE_CONTR_RST_FCB:               // REQ -> 	<- CON		FCV = 0		(S2)    Ok
                    // Reset CU/FCB
                    if(Iec103ResetStartHandler(pIec103_inst, &func_code) == RESULT_OK)
                    {
                        // Confirmation
                        Iec103Confirm(pIec103_inst, FCODE_MONIT_POS_ACK);
                    }
                    break;

                case FCODE_CONTR_DATA_NEED_ACK:	        // REQ -> 	<- CON 		FCV = 1		(S2)
                    // User data with ack
                    if(Iec103RcvdUserDataHandler(pIec103_inst, &func_code) == RESULT_OK)
                    {
                        // Confirmation
                        Iec103Confirm(pIec103_inst, FCODE_MONIT_POS_ACK);
                    }
                    else
                        Iec103Confirm(pIec103_inst, FCODE_MONIT_NEG_ACK);
                    break;

                case FCODE_CONTR_DATA_NO_ACK:	        // REQ -> 				FCV = 0		(S1)
                    // User data without ack
                    Iec103RcvdUserDataHandler(pIec103_inst, &func_code);
                    break;

                case FCODE_CONTR_LINK_STATE_RQST:	    // REQ -> 	<- RESP 	FCV = 0		(S3)	Ok
                    // Request for link state
                    Iec103LinkStateRqstHandler(pIec103_inst);
                    break;

                case FCODE_CONTR_DATA_1_RQST:       	// REQ -> 	<- RESP 	FCV = 1		(S3)	Ok
                    // Request for data class 1
                    Iec103DataClass1RqstHandler(pIec103_inst);
                    break;

                case FCODE_CONTR_DATA_2_RQST:       	// REQ -> 	<- RESP 	FCV = 1		(S3)	Ok
                    // Request for data class 2
                    Iec103DataClass2RqstHandler(pIec103_inst);
                    break;

                default:
                    break;
            }
        }
        xSemaphoreGive(*(pIec103_inst->DataSent));

        // Iec103 services processing
        if(pIec103_inst->Rst_Status == RST_CU_STATE || pIec103_inst->Rst_Status == RST_FCB_STATE)
        {
            // Reset complete processing
            Iec103ResetProcess(pIec103_inst);
        }
        else if(pIec103_inst->Sync_Status == SYNC_START_STATE)
        {
            // Time syncronization processing
            Iec103SyncProcess(pIec103_inst);
        }
        else if(pIec103_inst->GI_Status == GI_ACTIVE_STATE)
        {
            // General interview  processing
            Iec103GenIntervProcess(pIec103_inst);
        }
        else if(pIec103_inst->GC_Status != GC_NO)
        {
            // General commands complete processing
            Iec103GenCmdProcess(pIec103_inst);
        }
        // Sporadic events monitoring
        Iec103SporadEventsMonitor(pIec103_inst);
    }
}


//---------------------------------------------------------------------------------------------
BaseType_t CreateIec103Task (
                            Iec103_inst_t *pIec103_inst
                            )
{
    const char *task_name;
    TaskHandle_t *task_handle;
    const char task0[] = "Iec103_0";
    const char task1[] = "Iec103_1";

    if(pIec103_inst == &Iec103_0)
    {
        Iec103_0_DataRcvd_Semph = xSemaphoreCreateBinary();
        Iec103_0_DataSent_Semph = xSemaphoreCreateBinary();
        VDI_To_Iec103_0_Semph = xSemaphoreCreateBinary();
        DIKL_To_Iec103_0_Semph = xSemaphoreCreateBinary();
        Iec103_0_To_Modbus_Rqst_Semph = xSemaphoreCreateBinary();
        Modbus_To_Iec103_0_Resp_Semph = xSemaphoreCreateBinary();
        task_handle = &xIec103_0_Handle;
        task_name = task0;
    }
    else if(pIec103_inst == &Iec103_1)
    {
        Iec103_1_DataRcvd_Semph = xSemaphoreCreateBinary();
        Iec103_1_DataSent_Semph = xSemaphoreCreateBinary();
        VDI_To_Iec103_1_Semph = xSemaphoreCreateBinary();
        DIKL_To_Iec103_1_Semph = xSemaphoreCreateBinary();
        Iec103_1_To_Modbus_Rqst_Semph = xSemaphoreCreateBinary();
        Modbus_To_Iec103_1_Resp_Semph = xSemaphoreCreateBinary();
        task_handle = &xIec103_1_Handle;
        task_name = task1;
    }
    else
        return pdFALSE;

    Iec103StructInit(pIec103_inst);
    return xTaskCreate( vIec103Task,
                        task_name,
                        configMINIMAL_STACK_SIZE,
                        (void *)pIec103_inst,
                        (tskIDLE_PRIORITY + 1),
                        task_handle		);
}


#endif


