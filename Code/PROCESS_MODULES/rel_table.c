//*********************************************************************************************
//                                       Rel_table.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "rel_table.h"


//---------------------------------------------------------------------------------------------
//                                   Tables of Constants
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern Eeprom_inst_t       	        Eeprom_M95xxx;
extern SemaphoreHandle_t            Eep_Mutex;
extern SemaphoreHandle_t            Eep_Cmplt_Semph;

extern Rel_Tab_struct_t 	    	Modbus_Rel_Tab_Structure;
extern uint8_t				        Modbus_Relationship_Tab[EEPROM_MODBUS_REL_TAB_SIZE];
extern Table_Region_t				Modbus_Region_Tab[MODBUS_REG_TAB_SIZE];

#ifdef _IEC103_ON_
extern Rel_Tab_struct_t 	    	Iec103_Rel_Tab_Structure;
extern uint8_t				    	Iec103_Relationship_Tab[EEPROM_IEC103_REL_TAB_SIZE];
extern Table_Region_t				Iec103_Region_Tab[IEC103_REG_TAB_SIZE];

extern uint8_t                      GI_Tab[EEPROM_INTERVIEW_SET_SIZE];
extern uint8_t                      SE_Tab[EEPROM_INTERVIEW_SET_SIZE];
extern uint8_t                      SE_DIKLVDI_Buf[RAM_VIRT_DAT_SIZE + RAM_DISCRET_DAT_SIZE];
#endif


//#ifdef _TICK_COUNT_ON_
//extern uint32_t 		            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
//#endif



//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//------------------------ Researching of address remapping table -----------------------------
static uint32_t ModbusRemapTabResearch(void)
{
    uint8_t temp[3];
    uint32_t k, cond1, cond2;
    uint8_t modbus_functions[MODBUS_REG_TAB_SIZE] = {0xff, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0f, 0x10};

	for(int j = MODBUS_0x01; j < MODBUS_REG_TAB_SIZE; j++)
	{
		Modbus_Rel_Tab_Structure.Region_Tab[j].Start_Offset = (uint32_t)(-1);
		Modbus_Rel_Tab_Structure.Region_Tab[j].Final_Offset = (uint32_t)(-1);
	}

	k = MODBUS_0x01;
	for(uint32_t offset = 0; offset < EEPROM_MODBUS_REL_TAB_SIZE; offset += REL_ADDR_TABLE_LINE_LEN)
	{
		// copy data (3 bytes)
        for(uint32_t i = 0; i < 3; i++)
            temp[i] = Modbus_Rel_Tab_Structure.Rel_Tab[offset + i * REL_ADDR_TABLE_LINE_LEN];

        // analyse data
        cond1 = (   ((temp[0] == modbus_functions[k]) && (temp[1] == modbus_functions[k]) && (temp[2] == modbus_functions[k]))  ||
                    ((temp[0] == modbus_functions[k]) && (temp[1] == modbus_functions[k]) && (temp[2] == 0xff))                 ||
                    ((temp[0] == modbus_functions[k]) && (temp[2] == modbus_functions[k]) && (temp[1] == 0xff))    );
        if(cond1)
        {
            // set start block addr
            if(Modbus_Rel_Tab_Structure.Region_Tab[k].Start_Offset == (uint32_t)(-1))
				Modbus_Rel_Tab_Structure.Region_Tab[k].Start_Offset = offset;
        }
        else
        {
            cond2 = ((temp[0] == modbus_functions[k]) && (temp[1] == 0xff) && (temp[2] != 0xff) && (temp[2] != modbus_functions[k]));
            if(cond2)
            {
                // set final block addr
                Modbus_Rel_Tab_Structure.Region_Tab[k].Final_Offset = offset;
                // find new function number
                while((temp[2] != modbus_functions[k]) && (k < MODBUS_REG_TAB_SIZE))
                    k++;
                if(k == MODBUS_REG_TAB_SIZE)
                    break;
            }
            else
            {
                if(temp[1] == 0xff && temp[2] == 0xff)
                {
                    // stop analyse end exit loop
                    Modbus_Rel_Tab_Structure.Region_Tab[k].Final_Offset = offset;
                    break;
                }
            }
        }
    }
    for(uint32_t j = MODBUS_0x01; j < MODBUS_REG_TAB_SIZE; j++)
	{
		if(Modbus_Rel_Tab_Structure.Region_Tab[j].Start_Offset != (uint32_t)(-1))
			return RESULT_OK;
	}
	return 1;
}


#ifdef _IEC103_ON_
//------------------------ Researching of address relationship table ---------------------------
static uint32_t Iec103RemapTabResearch(void)
{
	for(int j = IEC103_DISCR_INOUT_STATE; j < IEC103_REG_TAB_SIZE; j++)
	{
		Iec103_Rel_Tab_Structure.Region_Tab[j].Start_Offset = (uint32_t)(-1);
		Iec103_Rel_Tab_Structure.Region_Tab[j].Final_Offset = (uint32_t)(-1);
	}

	for(int k = IEC103_DISCR_INOUT_STATE; k < IEC103_REG_TAB_SIZE; k++)
	{
		Iec103_Rel_Tab_Structure.Region_Tab[k].Start_Offset = (uint32_t)(((uint16_t)Iec103_Rel_Tab_Structure.Rel_Tab[REL_ADDR_TABLE_LINE_LEN * (k - 1) + 2]) * REL_ADDR_TABLE_LINE_LEN);
		Iec103_Rel_Tab_Structure.Region_Tab[k].Final_Offset = Iec103_Rel_Tab_Structure.Region_Tab[k].Start_Offset + (uint32_t)(((uint16_t)Iec103_Rel_Tab_Structure.Rel_Tab[REL_ADDR_TABLE_LINE_LEN * (k - 1) + 4] - 1) * REL_ADDR_TABLE_LINE_LEN);
	}

    for(uint32_t j = IEC103_DISCR_INOUT_STATE; j < IEC103_REG_TAB_SIZE; j++)
	{
		if(Iec103_Rel_Tab_Structure.Region_Tab[j].Start_Offset != (uint32_t)(-1))
			return RESULT_OK;
	}
	return 1;
}
#endif

//--------------------- Initialization of structure for address table -------------------------
static void RemapTabStructInit  (
                                Rel_Tab_struct_t *Rel_Tab_Struct,
                                uint8_t *Relationship_Tab,
                                Table_Region_t *Region_Tab
                                )
{
	Rel_Tab_Struct->Rel_Tab = Relationship_Tab;
	Rel_Tab_Struct->Region_Tab = Region_Tab;
	Rel_Tab_Struct->Flags = 0;
}


//------------------------ Initializing of address relationship table --------------------------
uint32_t RemapTabInit   (
                        Rel_Tab_struct_t *Rel_Tab_struct,
                        uint32_t eep_rel_tab_addr,
                        uint32_t eep_rel_tab_size
                        )
{
    if(Rel_Tab_struct == &Modbus_Rel_Tab_Structure)
        RemapTabStructInit(Rel_Tab_struct, Modbus_Relationship_Tab, Modbus_Region_Tab);
#ifdef _IEC103_ON_
	else if(Rel_Tab_struct == &Iec103_Rel_Tab_Structure)
	    RemapTabStructInit(Rel_Tab_struct, Iec103_Relationship_Tab, Iec103_Region_Tab);
#endif
    else
        return RESULT_ALT;

    Rel_Tab_struct->Flags = 0;

    // reading from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
    ReadEeprom(	eep_rel_tab_addr,
                eep_rel_tab_size,
                Rel_Tab_struct->Rel_Tab);

    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        CLEAR_FLAG(Rel_Tab_struct->Flags, f_Table_Read_Err);
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        SET_FLAG(Rel_Tab_struct->Flags, f_Table_Read_Err);
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);
    // data already has been read from EEPROM and located in RAM buffer

    if( !TEST_FLAG(Rel_Tab_struct->Flags, f_Table_Read_Err) )
    {
        if( (Rel_Tab_struct == &Modbus_Rel_Tab_Structure && ModbusRemapTabResearch() == RESULT_OK)
#ifdef _IEC103_ON_
			|| (Rel_Tab_struct == &Iec103_Rel_Tab_Structure  && Iec103RemapTabResearch() == RESULT_OK)
#endif
                                                                                                    )
        {
            SET_FLAG(Rel_Tab_struct->Flags, f_Table_Ok);
            return RESULT_OK;
        }
    }
    return RESULT_ALT;
}


#ifdef _IEC103_ON_
//---------------------------------------------------------------------------------------------
void GetInterviewSetTab (
                        uint8_t *Interview_Set_Tab,
                        uint32_t eep_set_addr
                        )
{
	uint8_t buf[EEPROM_INTERVIEW_SET_SIZE];
    uint32_t temp = 0;

    // reading from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
    ReadEeprom(eep_set_addr, EEPROM_INTERVIEW_SET_SIZE, buf);
    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        temp = 0;
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        temp = 1;
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);
    // data already has been read from EEPROM and located in RAM buffer

    if(temp == 0)
    {
        // Check & save device function
        if( buf[EEPROM_INTERVIEW_DATA_WRTAG_OFFSET] != 0x5A
            ||
            buf[EEPROM_INTERVIEW_SET_CS_OFFSET] != CalcOrdCRC(buf, EEPROM_INTERVIEW_SET_CS_OFFSET, 0x00))
        {
            // interview data is wrong -> write default settings
            memset(Interview_Set_Tab, 0, EEPROM_INTERVIEW_BYTES_AMOUNT);
        }
        else
        {
            // interview data is right
            // Save data
            memcpy(Interview_Set_Tab, buf, EEPROM_INTERVIEW_BYTES_AMOUNT);
        }
    }
}


//------------------------ Initialization of structure for interviews -------------------------
void InterviewStructInit(
                        Interview_struct_t *Interview_struct,
                        uint8_t *Data_Set_Tab,
                        uint8_t *Old_Data_Buf
                        )
{
	Interview_struct->Flags = 0;
	Interview_struct->Data_Set = Data_Set_Tab;
	if(Old_Data_Buf != 0)
		Interview_struct->Old_Data = Old_Data_Buf;
}


#endif


