//*********************************************************************************************
//                                         Modbus.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "modbus.h"


//---------------------------------------------------------------------------------------------
//                                   Tables of Constants
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------

extern SemaphoreHandle_t            Rst_Semph;


extern Modbus_inst_t	            Modbus;
extern uint8_t                      Modbus_Data_Buf[RAM_MODBUS_BUF_SIZE];

extern SemaphoreHandle_t            Modbus_Mutex;
extern SemaphoreHandle_t            Modbus_DataRcvd_Semph;
extern SemaphoreHandle_t            Modbus_DataSent_Semph;
extern SemaphoreHandle_t            Modbus_To_Parbus_Rqst_Semph;
extern SemaphoreHandle_t            Parbus_To_Modbus_Resp_Semph;


extern Eeprom_inst_t                Eeprom_M95xxx;
extern SemaphoreHandle_t            Eep_Mutex;
extern SemaphoreHandle_t            Eep_Cmplt_Semph;

extern Rel_Tab_struct_t 	    	Modbus_Rel_Tab_Structure;
extern uint8_t				        Ied_Data_Buf[RAM_IED_BUF_SIZE];

extern uint8_t                      mb_rqst_buf[256], mb_resp_buf[256];
extern uint8_t                      mb_rqst_size, mb_resp_size;


#ifdef _BOOTLOADER_ON_
extern uint8_t                      Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_BUF_SIZE];
extern uint8_t                      Fw_Upd_Key[5];
extern SemaphoreHandle_t            Fw_Upd_Rqst_Semph;
extern SemaphoreHandle_t            Fw_Upd_Confrm_Semph;
#endif


extern TaskHandle_t                 xModbusHandle;
extern TaskHandle_t                 xDataResetHandle;

#ifdef _DEBUG_ON_
extern uint32_t                     task_enter_cnt[TASK_NUM_SIZE];
extern uint32_t                     task_stage[TASK_NUM_SIZE];
#endif


//----------------------------------------------------------------------------------------------
//                                        Functions
//----------------------------------------------------------------------------------------------
//------------------------------ Modbus link layer processing ---------------------------------
static uint32_t ModbusLinkCheckRqst (
                                    uint8_t *buf,
                                    uint16_t rqst_size,
                                    uint8_t addr
                                    )
{
    uint16_t crc;

    // check address
    if( buf[0] == addr || buf[0] == 0 )
    {
        // check CRC
        crc = CalcCrc16(buf, rqst_size - 2, 0xffff);
        if( buf[rqst_size - 2] == (uint8_t)(crc >> 8) &&
            buf[rqst_size - 1] == (uint8_t)(crc) )
        {
            // Transfer data to Modbus PDU
            vTaskSuspendAll();
            Modbus.Rqst_PDU_Size = rqst_size - 3;
            memcpy( Modbus.Rqst_PDU, buf + 1, Modbus.Rqst_PDU_Size);
            xTaskResumeAll();
            return RESULT_OK;
        }
        else
            return CRC_ERR;
    }
    else
        return INVALID_ADDR_ERR;
}


//------------------------------ Modbus link layer processing ---------------------------------
static uint16_t ModbusLinkGetResp   (
									uint8_t *buf,
									uint8_t addr
									)
{
    uint16_t crc, res;

    vTaskSuspendAll();
    memcpy( buf + 1, Modbus.Resp_PDU, Modbus.Resp_PDU_Size);
    buf[0] = addr;
    crc = CalcCrc16(buf, Modbus.Resp_PDU_Size + 1, 0xffff);
    buf[Modbus.Resp_PDU_Size + 1] = (uint8_t)(crc >> 8);
    buf[Modbus.Resp_PDU_Size + 2] = (uint8_t)(crc);
    res = Modbus.Resp_PDU_Size + 3;
    xTaskResumeAll();
	return res;
}


//------------------------------ Modbus link layer processing ---------------------------------
uint16_t ModbusLinkProcess  (
							uint8_t *rqst_buf,
							uint16_t rqst_size,
							uint8_t *resp_buf,
							uint8_t addr
							)
{
    uint16_t res;

    if(addr)
    {
        if(ModbusLinkCheckRqst(rqst_buf, rqst_size, addr) == RESULT_OK)
        {
            xSemaphoreGive(*(Modbus.DataRcvd));
            if(xSemaphoreTake(*(Modbus.DataSent), MODBUS_RESP_TIMEOUT) == pdPASS)
            {
                if(rqst_buf[0] != 0)
                    return ModbusLinkGetResp(resp_buf, addr);
                else
                    return 0;
            }
            else
                return 0;
        }
        else
            return 0;
    }
    else
    {
        vTaskSuspendAll();
        memcpy(Modbus.Rqst_PDU, rqst_buf, rqst_size);
        Modbus.Rqst_PDU_Size = (uint8_t)rqst_size;
        xTaskResumeAll();

        xSemaphoreGive(*(Modbus.DataRcvd));
        if(xSemaphoreTake(*(Modbus.DataSent), MODBUS_RESP_TIMEOUT) == pdPASS)        // portMAX_DELAY
        {
            vTaskSuspendAll();
            memcpy(resp_buf, Modbus.Resp_PDU, Modbus.Resp_PDU_Size);
            res = (uint16_t)Modbus.Resp_PDU_Size;
            xTaskResumeAll();
            return res;
        }
        else
            return 0;
    }
}


//--------------------------- Error handler for wrong modbus function -------------------------
static uint8_t ModbusErrHandler (
                                uint8_t func,
                                uint8_t except,
                                uint8_t *resp_buf
                                )
{
    resp_buf[0] = 0x80 | func;
    resp_buf[1] = except;
    return 2;
}


//----------------------------- Modbus funcs for accsessing to CPU ----------------------------
static uint8_t ModbusToCpuHandler   (
                                    uint8_t *rqst_buf,
                                    uint8_t rqst_size,
                                    uint8_t *resp_buf
                                    )
{
    uint8_t resp_size;

    vTaskSuspendAll();
    Modbus.Rqst_PDU_Size = rqst_size;
    memcpy(Modbus.Rqst_PDU, rqst_buf, Modbus.Rqst_PDU_Size);
    xTaskResumeAll();

	// transfer Modbus request in Parbus request to CPU
    xSemaphoreGive(*(Modbus.RqstToApp));
	// wait for response from CPU
    if(xSemaphoreTake(*(Modbus.RespFromApp), 20*PARBUS_RESP_TIMEOUT) == pdPASS)
    // CPU Data (Resp) -> Modbus_PDU (Resp)
    {
        vTaskSuspendAll();
        resp_size = Modbus.Resp_PDU_Size;
        memcpy(resp_buf, Modbus.Resp_PDU, resp_size);
        xTaskResumeAll();
        return resp_size;
    }
    else
        return ModbusErrHandler(rqst_buf[0], MODBUS_SERVER_DEVICE_BUSY, resp_buf);
}


//----------------------------------- Modbus func 0x01 ----------------------------------------
static uint8_t Modbus0x01Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, data_cnt, bit_cnt, byte_cnt, addr;
    uint8_t *new_line_pntr, *source_buf, temp_byte;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x01, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x01].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x01].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x01, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];
    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

	// If table hasn't entry with requested address
	if(!(addr >= start_addr && (addr + size_val - 1) <= final_addr))
        return ModbusErrHandler(0x01, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val >= 1 && size_val <= 2000))
        return ModbusErrHandler(0x01, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x01, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled Resp_PDU from entry_offset
    data_cnt = 0;
    while(data_cnt < size_val)
    {
        // update pointer on table's new line
        new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

        // Check address continuity (bit registers address)
        if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
            return ModbusErrHandler(0x01, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

        byte_cnt = data_cnt / 8;
        bit_cnt = data_cnt % 8;

        switch(new_line_pntr[3])
        {
            case 0x81:
                source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                break;
            case 0x82:
                source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                break;
            case 0x83:
                source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                break;
            default:
                return ModbusErrHandler(0x01, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
        }
        // extract data from parallel buffer, fill transmit buffer with data
        if(bit_cnt == 0)
            temp_byte = 0;
        else
            temp_byte = resp_buf[2 + byte_cnt];

        if(TEST_FLAG(source_buf[new_line_pntr[4]], new_line_pntr[5]))
            temp_byte |= (1 << bit_cnt);
        else
            temp_byte &= ~(1 << bit_cnt);

        resp_buf[2 + byte_cnt] = temp_byte;
        ++addr;
        ++data_cnt;
    }

    resp_buf[0] = 0x01;
    resp_buf[1] = byte_cnt + 1;
    return resp_buf[1] + 2;
}


//----------------------------------- Modbus func 0x02 ----------------------------------------
static uint8_t Modbus0x02Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, data_cnt, bit_cnt, byte_cnt, addr;
    uint8_t *new_line_pntr, *source_buf, temp_byte;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x02, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x02].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x02].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x02, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];
    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

	// If table hasn't entry with requested address
	if(!(addr >= start_addr && (addr + size_val - 1) <= final_addr))
        return ModbusErrHandler(0x02, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val >= 1 && size_val <= 2000))
        return ModbusErrHandler(0x02, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x02, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled Resp_PDU from entry_offset
    data_cnt = 0;
    while(data_cnt < size_val)
    {
        // update pointer on table's new line
        new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

        // Check address continuity (bit registers address)
        if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
            return ModbusErrHandler(0x02, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

        byte_cnt = data_cnt / 8;
        bit_cnt = data_cnt % 8;

        switch(new_line_pntr[3])
        {
            case 0x81:
                source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                break;
            case 0x82:
                source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                break;
            case 0x83:
                source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                break;
            default:
                return ModbusErrHandler(0x02, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
        }

        // extract data from parallel buffer, fill transmit buffer with data
        if(bit_cnt == 0)
            temp_byte = 0;
        else
            temp_byte = resp_buf[2 + byte_cnt];

        if(TEST_FLAG(source_buf[new_line_pntr[4]], new_line_pntr[5]))
            temp_byte |= (1 << bit_cnt);
        else
            temp_byte &= ~(1 << bit_cnt);

        resp_buf[2 + byte_cnt] = temp_byte;
        ++addr;
        ++data_cnt;
    }
    resp_buf[0] = 0x02;
    resp_buf[1] = byte_cnt + 1;
    return resp_buf[1] + 2;
}


//----------------------------------- Modbus func 0x03 ----------------------------------------
static uint8_t Modbus0x03Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, data_cnt, byte_cnt, addr;
    uint8_t *new_line_pntr, *source_buf, bytes_amnt, byte_num, source_num;
    uint8_t buf1[256], buf2[256], buf1_data_size;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x03, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x03].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x03].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x03, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];
    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

    // If table hasn't entry with requested address
	if(!(addr >= start_addr && (addr + size_val - 1) <= final_addr))
        return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val >= 1 && size_val <= 125))
        return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// Modbus requuest verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled Resp_PDU from entry_offset
    source_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 3);
    if(source_num & 0x80)
    {
        // source - buffer
        switch(source_num)
        {
            case 0x81:
                source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                break;
            case 0x82:
                source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                break;
            case 0x83:
                source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                break;
            default:
                return ModbusErrHandler(0x03, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
        }

        for(data_cnt = 0; data_cnt < size_val; data_cnt++)
        {
            byte_cnt = data_cnt * 2;

            // update pointer on table's new line
            new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

            // Check address continuity (2-byte registers address)
            if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

            // Check data source
            if(!(new_line_pntr[3] & 0x80))
                return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

            byte_num = new_line_pntr[4];
            bytes_amnt = new_line_pntr[5];

            if(bytes_amnt == 1)
            {
                resp_buf[2 + byte_cnt] = 0;
                resp_buf[2 + byte_cnt + 1] = source_buf[byte_num];
            }
            else
            {
                resp_buf[2 + byte_cnt] = source_buf[byte_num];
                resp_buf[2 + byte_cnt + 1] = source_buf[byte_num + 1];
            }
            ++addr;
        }
    }
    else
    {
        // source - Parbus response
        // Check address continuity (3-byte addresses)
        for(data_cnt = 0; data_cnt < size_val; data_cnt++)
        {
            new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

            // Check address continuity (2-byte registers address)
            if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

            // Check data source
            if(new_line_pntr[3] != 0x44)
                return ModbusErrHandler(0x03, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
            ++addr;
        }
        // Varification is OK

        // Modbus request 0x44 (prepare request to parallel bus)
        new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset;

        // copy second request from table to Modbus request
        memcpy(buf1, new_line_pntr + 3, 4);
        buf1[4] = size_val * 2;
        buf1_data_size = 5;

        // Forward request to Parbus
        ModbusToCpuHandler(buf1, buf1_data_size, buf2);

        // transfer response data from Resp_0x44 to Resp_0x03
        memcpy(resp_buf + 2, buf2 + 5, size_val * 2);
    }
    resp_buf[0] = 0x03;
    resp_buf[1] = size_val * 2;
    return resp_buf[1] + 2;
}


//----------------------------------- Modbus func 0x04 ----------------------------------------
static uint8_t Modbus0x04Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, data_cnt, byte_cnt, addr;
    uint8_t *new_line_pntr, *source_buf, bytes_amnt, byte_num, source_num;
    uint8_t buf1[256], buf2[256], buf1_data_size;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x04, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x04].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x04].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x04, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];
    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

    // If table hasn't entry with requested address
	if(!(addr >= start_addr && (addr + size_val - 1) <= final_addr))
        return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val >= 1 && size_val <= 125))
        return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// Modbus requuest verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled Resp_PDU from entry_offset
    source_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 3);
    if(source_num & 0x80)
    {
        // source - buffer
        switch(source_num)
        {
            case 0x81:
                source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                break;
            case 0x82:
                source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                break;
            case 0x83:
                source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                break;
            default:
                return ModbusErrHandler(0x04, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
        }

        for(data_cnt = 0; data_cnt < size_val; data_cnt++)
        {
            byte_cnt = data_cnt * 2;

            // update pointer on table's new line
            new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

            // Check address continuity (2-byte registers address)
            if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

            // Check data source
            if(!(new_line_pntr[3] & 0x80))
                return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

            byte_num = new_line_pntr[4];
            bytes_amnt = new_line_pntr[5];
            if(bytes_amnt == 1)
            {
                resp_buf[2 + byte_cnt]     = 0;
                resp_buf[2 + byte_cnt + 1] = source_buf[byte_num];
            }
            else
            {
                resp_buf[2 + byte_cnt]     = source_buf[byte_num];
                resp_buf[2 + byte_cnt + 1] = source_buf[byte_num + 1];
            }
            ++addr;
        }
    }
    else
    {
        // source - Parbus response
        // Check address continuity (3-byte addresses)
        for(data_cnt = 0; data_cnt < size_val; data_cnt++)
        {
            new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

            // Check address continuity (2-byte registers address)
            if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

            // Check data source
            if(new_line_pntr[3] != 0x44)
                return ModbusErrHandler(0x04, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
            ++addr;
        }
        // Varification is OK

        // Modbus request 0x44 (prepare request to parallel bus)
        new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset;

        // copy second request from table to Modbus request
        memcpy(buf1, new_line_pntr + 3, 4);
        buf1[4] = size_val * 2;
        buf1_data_size = 5;

        // Forward request to Parbus
        ModbusToCpuHandler(buf1, buf1_data_size, buf2);

        // transfer response data from Resp_0x44 to Resp_0x03
        memcpy(resp_buf + 2, buf2 + 5, size_val * 2);
    }
    resp_buf[0] = 0x04;
    resp_buf[1] = size_val * 2;
    return resp_buf[1] + 2;
}


//------------------------------ Modbus func 0x05 ---------------------------------------------
static uint8_t Modbus0x05Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, addr;
    uint8_t *new_line_pntr, *source_buf, source_num, byte_num, bit_num;
    uint8_t buf1[256], buf2[256], buf1_data_size;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x05, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    // ------------------------------------- Check data ----------------------------------------
    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x05].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x05].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x05, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];
    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

	// If table hasn't entry with requested address
	if(!(addr >= start_addr && addr <= final_addr))
        return ModbusErrHandler(0x05, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val == 0 || size_val == 0xff00))
        return ModbusErrHandler(0x05, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x05, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled transmit buffer from entry_offset
    source_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 3);
    if(source_num & 0x80)
    {
        // source - buffer
        switch(source_num)
        {
            case 0x81:
                source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                break;
            case 0x82:
                source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                break;
            case 0x83:
                source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                break;
            default:
                return ModbusErrHandler(0x05, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
        }
        byte_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 4);
        bit_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 5);

        // modify data in parallel buffer
        vTaskSuspendAll();
        if(size_val == 0xff00)
            source_buf[byte_num] |= (1 << bit_num);
        else
            source_buf[byte_num] &= ~(1 << bit_num);
        xTaskResumeAll();
    }
    else
    {
        // source - response
        // Check function
        new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset;
        if(new_line_pntr[3] != 0x42)
            return ModbusErrHandler(0x05, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
        // Varification is OK

        // Modbus request 0x42 (prepare request to parallel bus)
        // copy second request from table to Modbus request
        memcpy(buf1, new_line_pntr + 3, 3);
        buf1[3] = 1;
        buf1[4] = (size_val == 0xff00) ? 0x0f : 0x00;
        buf1_data_size = 5;

        // Forward request to Parbus
        ModbusToCpuHandler(buf1, buf1_data_size, buf2);

        // transfer response data from Resp_0x44 to Resp_0x03
        memcpy(resp_buf + 2, buf2 + 5, size_val * 2);

        // check response data from Resp_0x42
        if(buf2[4] & 0x80)
            return ModbusErrHandler(0x05, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
    }
    memcpy(resp_buf, rqst_buf, 5);
    return 5;
}


//----------------------------------- Modbus func 0x06 -----------------------------------------
static uint8_t Modbus0x06Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, addr;
    uint8_t *new_line_pntr, *source_buf, source_num, byte_num;
    uint8_t buf1[256], buf2[256], buf1_data_size;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x06, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    // ------------------------------------- Check data ----------------------------------------
    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x06].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x06].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x06, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

	// If table hasn't entry with requested address
	if(!(addr >= start_addr && addr <= final_addr))
        return ModbusErrHandler(0x06, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
	// verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x06, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled transmit buffer from entry_offset
	source_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 3);
	if(source_num & 0x80)
	{
		// source - buffer
		switch(source_num)
        {
            case 0x81:
                source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                break;
            case 0x82:
                source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                break;
            case 0x83:
                source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                break;
            default:
                return ModbusErrHandler(0x06, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
        }

		byte_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 4);

		// modify data in parallel buffer
        vTaskSuspendAll();
		source_buf[byte_num]     = rqst_buf[3];
		source_buf[byte_num + 1] = rqst_buf[4];
        xTaskResumeAll();
	}
	else
	{
		// source - response
		// Check function
		new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset;
		if(new_line_pntr[3] != 0x66)
            return ModbusErrHandler(0x06, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
		// Varification is OK

        // Modbus request 0x66 (prepare request to parallel bus)
        // copy second request from table to Modbus request
        memcpy(buf1, new_line_pntr + 3, 5);
        buf1[5] = resp_buf[3];
        buf1[6] = resp_buf[4];
        buf1_data_size = 7;

        // Forward request to Parbus
        ModbusToCpuHandler(buf1, buf1_data_size, buf2);

        // transfer response data from Resp_0x44 to Resp_0x03
        if(memcmp((void *)buf1, (void *)buf2, 5))
            return ModbusErrHandler(0x06, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
    }
    memcpy(resp_buf, rqst_buf, 5);
    return 5;
}


//----------------------------------- Modbus func 0x0f -----------------------------------------
static uint8_t Modbus0x0fHandler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, addr, data_cnt, bit_cnt, byte_cnt;
    uint8_t *new_line_pntr, *source_buf, source_num, byte_num, bit_num;
    uint8_t buf1[256], buf2[256], buf1_data_size;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x0f, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    // ------------------------------------- Check data ----------------------------------------
    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x0f].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x0f].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];

    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

	// If table hasn't entry with requested address
	if(!(addr >= start_addr && addr <= final_addr))
        return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val >= 1 && size_val <= 0x7b0))
        return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled transmit buffer from entry_offset
    source_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 3);
    if(source_num & 0x80)
    {
        data_cnt = 0;
        while(data_cnt < size_val)
        {
            // update pointer on table's new line
            new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

            // Check address continuity (2-byte registers address)
            if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

            // check data source (buffer or response)
            if(!(new_line_pntr[3] & 0x80))
                return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
            // source of data - buffer

            byte_cnt = data_cnt / 8;
            bit_cnt = data_cnt % 8;

            switch(source_num)
            {
                case 0x81:
                    source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                    break;
                case 0x82:
                    source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                    break;
                case 0x83:
                    source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                    break;
                default:
                    return ModbusErrHandler(0x0f, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
            }

            byte_num = new_line_pntr[4];
            bit_num = new_line_pntr[5];

            // modify data in parallel buffer
            vTaskSuspendAll();
            if(TEST_FLAG(rqst_buf[6 + byte_cnt], bit_cnt))
                source_buf[byte_num] |= (1 << bit_num);
            else
                source_buf[byte_num] &= ~(1 << bit_num);
            xTaskResumeAll();

            ++addr;
            ++data_cnt;
        }
    }
    else
    {
		if(size_val > 249)
            return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

        data_cnt = 0;
        while(data_cnt < size_val)
        {
            // update pointer on table's new line
            new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

            // Check address continuity (bit registers address)
            if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

            // Check function
            if(new_line_pntr[3] != 0x42)
                return ModbusErrHandler(0x0f, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
            // Varification is OK

            byte_cnt = data_cnt / 8;
            bit_cnt = data_cnt % 8;

            // Modbus request 0x42 (prepare request to parallel bus)
            // copy request from table to Modbus request
            if(data_cnt == 0)
            {
                memcpy(buf1, new_line_pntr + 3, 3);
                buf1[3] = (uint8_t)size_val;
            }
            buf1[4 + data_cnt] = (TEST_FLAG(rqst_buf[6 + byte_cnt], bit_cnt)) ? 0x0f : 0x00;
            ++addr;
            ++data_cnt;
        }
        buf1_data_size = size_val + 4;

        // Transfer request 0x42 to Parbus
        ModbusToCpuHandler(buf1, buf1_data_size, buf2);

        // check response data
        data_cnt = 0;
        while(data_cnt < size_val)
        {
            if(buf2[4 + data_cnt] & 0x80)
                return ModbusErrHandler(0x0f, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
            ++data_cnt;
        }
    }
    memcpy(resp_buf, rqst_buf, 5);
    return 5;
}


//----------------------------------- Modbus func 0x10 -----------------------------------------
static uint8_t Modbus0x10Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
    uint32_t start_offset, final_offset, entry_offset;
	uint32_t start_addr, final_addr, size_val, addr, data_cnt, addr_phy;
    uint8_t *new_line_pntr, *source_buf, source_num, byte_num;
    uint8_t buf1[256], buf2[256], buf1_data_size;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x10, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

    // ------------------------------------- Check data ----------------------------------------
    start_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x10].Start_Offset;
    final_offset = Modbus_Rel_Tab_Structure.Region_Tab[MODBUS_0x10].Final_Offset;
    if(start_offset == (uint32_t)(-1) || final_offset == (uint32_t)(-1))
    {
        return ModbusErrHandler(0x10, MODBUS_ILLEGAL_FUNCTION, resp_buf);
    }

	addr = ((uint32_t)rqst_buf[1])<<8 | (uint32_t)rqst_buf[2];
	size_val = ((uint32_t)rqst_buf[3] << 8) | (uint32_t)rqst_buf[4];

    start_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[start_offset + 2];
    final_addr = ((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[final_offset + 2];

	// If table hasn't entry with requested address
	if(!(addr >= start_addr && addr <= final_addr))
        return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

	// If arguments illegal
	if(!(size_val >= 1 && size_val <= 0x7b))
        return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
	// verification is OK

    // --------------------------- Find entry in Relationship Table ----------------------------
    // we have: start_offset->start_addr, final_offset->final_addr, addr-> entry_offset(must find)
    // result: entry_offset
    entry_offset = (uint32_t)(-1);
    for(uint32_t offset = start_offset; offset <= final_offset; offset += REL_ADDR_TABLE_LINE_LEN)
    {
        if(addr == (((uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 1]<<8) | (uint32_t)Modbus_Rel_Tab_Structure.Rel_Tab[offset + 2]))
        {
            entry_offset = offset;
            break;
        }
    }
    if(entry_offset == (uint32_t)(-1))
        return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
    // entry is found (entry_offset)

    // ------------- Extract data from buffer (or from responses of parallel bus) --------------
    // we have: entry_offset
    // result: filled transmit buffer from entry_offset
	source_num = *(Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + 3);
	if(source_num & 0x80)
	{
		data_cnt = 0;
		while(data_cnt < size_val)
		{
			// update pointer on table's new line
			new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

			// Check address continuity (2-byte registers address)
			if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

			// check data source (buffer or response)
			if(!(new_line_pntr[3] & 0x80))
                return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
			// source of data - buffer
            switch(source_num)
            {
                case 0x81:
                    source_buf = Ied_Data_Buf + RAM_VIRT_DAT_OFFSET;
                    break;
                case 0x82:
                    source_buf = Ied_Data_Buf + RAM_ANALOG_DAT_OFFSET;
                    break;
                case 0x83:
                    source_buf = Ied_Data_Buf + RAM_DISCRET_DAT_OFFSET;
                    break;
                default:
                    return ModbusErrHandler(0x10, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
            }
			byte_num = new_line_pntr[4];

            vTaskSuspendAll();
			if(new_line_pntr[5] == 1)
			{
				source_buf[byte_num]     = 0;
				source_buf[byte_num + 1] = rqst_buf[6 + 2*data_cnt + 1];
			}
			else
			{
				source_buf[byte_num]     = rqst_buf[6 + 2*data_cnt];
				source_buf[byte_num + 1] = rqst_buf[6 + 2*data_cnt + 1];
			}
            xTaskResumeAll();
			++addr;
			++data_cnt;
		}
	}
	else
	{
		if(!(source_num == 0x66 || source_num == 0x69))
            return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

        new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset;
		addr_phy = ((uint32_t)new_line_pntr[4]<<16) | ((uint32_t)new_line_pntr[5]<<8) | (uint32_t)new_line_pntr[6];
		data_cnt = 0;
		while(data_cnt < size_val)
		{
			// update pointer on table's new line
			new_line_pntr = Modbus_Rel_Tab_Structure.Rel_Tab + entry_offset + data_cnt * REL_ADDR_TABLE_LINE_LEN;

			// Check source
			if(new_line_pntr[3] != source_num)
                return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

			// Check address continuity (virtual addresses - 2-byte)
			if((((uint32_t)new_line_pntr[1]<<8) | (uint32_t)new_line_pntr[2]) != addr)
                return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

			// Check address continuity (physical addresses - 3-byte)
			if((source_num == 0x66)&&((((uint32_t)new_line_pntr[4]<<16) | ((uint32_t)new_line_pntr[5]<<8) | (uint32_t)new_line_pntr[6]) != addr_phy))
                return ModbusErrHandler(0x10, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
			// Varification is OK

			if(source_num == 0x66)
			{
                // Modbus request 0x66 (prepare request to parallel bus)
                // copy request from table to Modbus request
                if(data_cnt == 0)
                {
                    memcpy(buf1, new_line_pntr + 3, 4);
                    buf1[4] = (uint8_t)(size_val * 2);
                }
				if(new_line_pntr[7] == 1)
				{
					buf1[5 + 2 * data_cnt]     = 0;
					buf1[5 + 2 * data_cnt + 1] = rqst_buf[5 + 2 * data_cnt + 1];
				}
				else
				{
					buf1[5 + 2 * data_cnt]     = rqst_buf[5 + 2 * data_cnt];
					buf1[5 + 2 * data_cnt + 1] = rqst_buf[5 + 2 * data_cnt + 1];
				}
            }
			else
			{
                // Modbus request 0x69 (prepare request to parallel bus)
                // copy request from table to Modbus request
				if(data_cnt == 0)
				{
					memcpy( buf1, new_line_pntr + 3, 5);
				}
				buf1[5 + 2 * data_cnt]     = rqst_buf[5 + 2 * data_cnt];
				buf1[5 + 2 * data_cnt + 1] = rqst_buf[5 + 2 * data_cnt + 1];
			}
			++addr;
			++data_cnt;
			addr_phy += 2;
		}
        buf1_data_size = size_val * 2 + 5;

        // Transfer request 0x42 to Parbus
        ModbusToCpuHandler(buf1, buf1_data_size, buf2);

        // check response data from Resp_0x66
        if(buf2[0] != buf1[0])
            return ModbusErrHandler(0x10, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
	}
    memcpy(resp_buf, rqst_buf, 5);
    return 5;
}


//----------------------------------- Modbus func 0x43 -----------------------------------------
static uint8_t Modbus0x43Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
	uint8_t size_val;
	uint32_t start_addr, fin_addr;
    uint8_t temp;

	// If table is empty
	if(!TEST_FLAG(Modbus_Rel_Tab_Structure.Flags, f_Table_Ok))
        return ModbusErrHandler(0x43, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);

	// check amount of data values & data address range
	size_val = rqst_buf[4];
	start_addr = ((uint32_t)rqst_buf[1])<<16 | ((uint32_t)rqst_buf[2])<<8 | ((uint32_t)rqst_buf[3]);
	fin_addr = start_addr + size_val - 1;

	if(fin_addr > EEPROM_COM_SETTINGS_END_ADDR)
	{
#ifdef _BOOTLOADER_ON_
        if(start_addr < BOOTROM_SPACE_START_ADDR || fin_addr > (BOOTROM_SPACE_START_ADDR + 4 - 1))
            return ModbusErrHandler(0x43, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
		if(size_val == 0 || size_val > 4)
            return ModbusErrHandler(0x43, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

		// read from virtual ROM
        memcpy( resp_buf + 5,
                Boot_ROM_Space_Buf + RAM_BOOTROM_SPACE_RDBUF_OFFSET + start_addr - BOOTROM_SPACE_START_ADDR,
                size_val            );

        // action - initialize firmware update request
        if(size_val == 4 && start_addr == BOOTROM_SPACE_START_ADDR)
        {
			memcpy(	Fw_Upd_Key,
					resp_buf + 6,
					3		);
            xSemaphoreGive(Fw_Upd_Rqst_Semph);
        }
#else
        return ModbusErrHandler(0x43, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
#endif
	}
	else
	{
		if(size_val == 0 || size_val > 128)
            return ModbusErrHandler(0x43, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

		// start reading from EEPROM
		xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
		ReadEeprom(start_addr, size_val, resp_buf + 5);
		if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
		{
			temp = 0;
		}
		else
		{
			// Reset EEPROM control, set Failure Flag, increment error counter
			SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
			Eeprom_M95xxx.errors++;
			EepromInit();
			temp = 1;
		}
		// response PDU is complete
		xSemaphoreGive(Eep_Mutex);

		if(temp)
            return ModbusErrHandler(0x43, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
	}
    // Copy request 0x43 to response start
    memcpy(resp_buf, rqst_buf, 5);
    return size_val + 5;
}


//----------------------------------- Modbus func 0x65 -----------------------------------------
static uint8_t Modbus0x65Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
	uint8_t size_val;
	uint32_t start_addr, fin_addr;
	uint8_t temp;
#ifdef _BOOTLOADER_ON_
    uint8_t *buf;
	uint16_t crc16 = 0xffff;
#endif

	// check amount of data values & data address range
	size_val = rqst_buf[4];
	start_addr = ((uint32_t)rqst_buf[1])<<16 | ((uint32_t)rqst_buf[2])<<8 | ((uint32_t)rqst_buf[3]);
	fin_addr = start_addr + size_val - 1;

	if(fin_addr > EEPROM_COM_SETTINGS_END_ADDR)
	{
#ifdef _BOOTLOADER_ON_
		if(start_addr < BOOTROM_SPACE_START_ADDR || fin_addr > (BOOTROM_SPACE_START_ADDR + 4 - 1))
            return ModbusErrHandler(0x65, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);

		if(size_val == 0 || size_val > 4)
            return ModbusErrHandler(0x65, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

		// write to virtual ROM
        memcpy( Boot_ROM_Space_Buf + RAM_BOOTROM_SPACE_WRBUF_OFFSET + start_addr - BOOTROM_SPACE_START_ADDR,
                rqst_buf + 5,
                size_val            );

        // action - check confirmation of firmware update key
        if(size_val == 4 && start_addr == BOOTROM_SPACE_START_ADDR)
        {
            // Check key confirmation
			Fw_Upd_Key[3] = 0xf0;
			Fw_Upd_Key[4] = 0x0f;
			crc16 = CalcCrc16(Fw_Upd_Key, 5, crc16);

            buf = Boot_ROM_Space_Buf + RAM_BOOTROM_SPACE_WRBUF_OFFSET + start_addr - BOOTROM_SPACE_START_ADDR;
            if( buf[0] == 0 &&
                buf[1] == (uint8_t)(crc16 >> 8) &&
                buf[2] == (uint8_t)crc16 &&
                buf[3] == (buf[1] ^ buf[2])     )
            {
                xSemaphoreGive(Fw_Upd_Confrm_Semph);
            }
        }
#else
        return ModbusErrHandler(0x65, MODBUS_ILLEGAL_DATA_ADDRESS, resp_buf);
#endif
	}
	else
	{
		if(size_val == 0 || size_val > 128)
            return ModbusErrHandler(0x65, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);

		// start writing to EEPROM
		xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
		WriteEeprom(start_addr, size_val, rqst_buf + 5);
		if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
		{
			temp = 0;
		}
		else
		{
			// Reset EEPROM control, set Failure Flag, increment error counter
			SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
			Eeprom_M95xxx.errors++;
			EepromInit();
			temp = 1;
		}
 		// response PDU is complete
		xSemaphoreGive(Eep_Mutex);

		if(temp)
            return ModbusErrHandler(0x65, MODBUS_SERVER_DEVICE_FAILURE, resp_buf);
	}
    // Copy request 0x65 to response
    memcpy(resp_buf, rqst_buf, 5);
    return 5;
}


//----------------------------------- Modbus func 0x68 -----------------------------------------
static uint8_t Modbus0x68Handler(
                                uint8_t *rqst_buf,
                                uint8_t rqst_size,
                                uint8_t *resp_buf
                                )
{
	// Check command
	if(((rqst_buf[1] << 8) | rqst_buf[2]) != 0x3311)
        return ModbusErrHandler(0x68, MODBUS_ILLEGAL_DATA_VALUE, resp_buf);
    // Reset modules
    xSemaphoreGive(Rst_Semph);
    // Copy request 0x68 from buffer
	memcpy(resp_buf, rqst_buf, rqst_size);
    return 3;
}


//--------------------------------- Modbus PDU handler task --------------------------------------
void vModbusTask(
                void *pvParameters
                )
{
    uint8_t *buf1, *buf2;
    uint8_t *p_buf1_data_size, *p_buf2_data_size;

    buf1 = mb_rqst_buf;
    buf2 = mb_resp_buf;
    p_buf1_data_size = &mb_rqst_size;
    p_buf2_data_size = &mb_resp_size;

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[MODBUS_TASK_NUM] = 0;
        ++task_enter_cnt[MODBUS_TASK_NUM];
#endif
        xSemaphoreTake(*(Modbus.DataRcvd), portMAX_DELAY);
#ifdef _DEBUG_ON_
            task_stage[MODBUS_TASK_NUM] = 1;
#endif
            vTaskSuspendAll();
            *p_buf1_data_size = Modbus.Rqst_PDU_Size;
            memcpy(buf1, Modbus.Rqst_PDU, *p_buf1_data_size);
            xTaskResumeAll();

            switch(buf1[0])
            {
                // -----------------------------------
                case 0x01:  // read flag from flags register
                    *p_buf2_data_size = Modbus0x01Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x02:  // read flag from inputs register
                    *p_buf2_data_size = Modbus0x02Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x03:  // read from storing register
                    *p_buf2_data_size = Modbus0x03Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x04:  // read from inputs register
                    *p_buf2_data_size = Modbus0x04Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x05:  // write flag to flags register
                    *p_buf2_data_size = Modbus0x05Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x06:  // write to storing register
                    *p_buf2_data_size = Modbus0x06Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x0f:  // write to multiple flags
                    *p_buf2_data_size = Modbus0x0fHandler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x10:  // write to multiple registers
                    *p_buf2_data_size = Modbus0x10Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x11:
                case 0x14:
                case 0x42:
                case 0x44:
                case 0x45:
                case 0x46:
                case 0x47:
                case 0x66:
                case 0x67:
                case 0x69:
                    *p_buf2_data_size = ModbusToCpuHandler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x43:
                    *p_buf2_data_size = Modbus0x43Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x65:
                    *p_buf2_data_size = Modbus0x65Handler(buf1, *p_buf1_data_size, buf2);
                    break;
                // -----------------------------------
                case 0x68:
                    if(Rst_Semph != NULL)
                        *p_buf2_data_size = Modbus0x68Handler(buf1, *p_buf1_data_size, buf2);
                    else
                        *p_buf2_data_size = ModbusErrHandler(buf1[0], MODBUS_SERVER_DEVICE_FAILURE, buf2);
                    break;
                // -----------------------------------
                default:
                    *p_buf2_data_size = ModbusErrHandler(buf1[0], MODBUS_ILLEGAL_FUNCTION, buf2);
                    break;
            }
            vTaskSuspendAll();
            Modbus.Resp_PDU_Size = *p_buf2_data_size;
            memcpy(Modbus.Resp_PDU, buf2, *p_buf2_data_size);
            xTaskResumeAll();

            xSemaphoreGive(*(Modbus.DataSent));
    }
}


//---------------------------------------------------------------------------------------------
BaseType_t CreateModbusTask (
                            Modbus_inst_t *pModbus
                            )
{
    if(pModbus != NULL)
    {
        Modbus_Mutex = xSemaphoreCreateMutex();
        Modbus_DataRcvd_Semph = xSemaphoreCreateBinary();
        Modbus_DataSent_Semph = xSemaphoreCreateBinary();
        Modbus_To_Parbus_Rqst_Semph = xSemaphoreCreateBinary();
        Parbus_To_Modbus_Resp_Semph = xSemaphoreCreateBinary();

        Modbus.Mtx = &Modbus_Mutex;
        Modbus.DataRcvd = &Modbus_DataRcvd_Semph;
        Modbus.DataSent = &Modbus_DataSent_Semph;
        Modbus.RqstToApp = &Modbus_To_Parbus_Rqst_Semph;
        Modbus.RespFromApp = &Parbus_To_Modbus_Resp_Semph;

        Modbus.Rqst_PDU = Modbus_Data_Buf + RAM_MODBUS_RQST_DATA_BUF_OFFSET;
        Modbus.Resp_PDU = Modbus_Data_Buf + RAM_MODBUS_RESP_DATA_BUF_OFFSET;
        Modbus.Rqst_PDU_Size = 0;
        Modbus.Resp_PDU_Size = 0;

        return xTaskCreate( vModbusTask,
                            (char *)"Modbus",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            (tskIDLE_PRIORITY + 1),
                            &xModbusHandle		);
    }
    return pdFALSE;
}




