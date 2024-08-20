//*********************************************************************************************
//                                         Vers.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "vers.h"


//---------------------------------------------------------------------------------------------
//                                   Tables of Constants
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern Eeprom_inst_t       	        Eeprom_M95xxx;
extern SemaphoreHandle_t            Eep_Mutex;
extern SemaphoreHandle_t            Eep_Cmplt_Semph;

extern Id_Inst_t                    Id_Data;
extern uint8_t				        Device_Name[8];

extern Vers_Inst_t		            Vers_Data;
extern uint8_t                      Frmwr_Vers_Actual_str[EEPROM_VERSION_SIZE];
extern uint8_t                      Frmwr_Vers_Eeprom_str[EEPROM_VERSION_SIZE];
extern uint8_t                      Hrdwr_Vers_Actual_str[EEPROM_VERSION_SIZE];
extern uint8_t                      Hrdwr_Vers_Eeprom_str[EEPROM_VERSION_SIZE];
extern uint8_t                      Btldr_Vers_Actual_str[EEPROM_VERSION_SIZE];
extern uint8_t                      Btldr_Vers_Eeprom_str[EEPROM_VERSION_SIZE];

extern Sync_Inst_t                  Sync_Data;
extern uint8_t                      Sync_Src_Prior_Set[SYNC_SRC_AMOUNT];
//extern uint8_t                      SNTP_IP1[4], SNTP_IP2[4], SNTP_IP3[4];




//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
static void IdStructInit(
                        Id_Inst_t *Dat_Id
                        )
{
	Dat_Id->Flags = 0;
	Dat_Id->Name = Device_Name;
}


//---------------------------------------------------------------------------------------------
static void VersStructInit  (
                            Vers_Inst_t *Dat_Vers
                            )
{
	Dat_Vers->Flags = 0;

    Dat_Vers->Frmwr_Vers_Actual = Frmwr_Vers_Actual_str;
    Dat_Vers->Frmwr_Vers_Eeprom = Frmwr_Vers_Eeprom_str;
	memcpy(Dat_Vers->Frmwr_Vers_Actual, FW_VERS, EEPROM_VERSION_SIZE);

    Dat_Vers->Hrdwr_Vers_Actual = Hrdwr_Vers_Actual_str;
    Dat_Vers->Hrdwr_Vers_Eeprom = Hrdwr_Vers_Eeprom_str;
	memcpy(Dat_Vers->Hrdwr_Vers_Actual, HW_VERS, EEPROM_VERSION_SIZE);

    Dat_Vers->Btldr_Vers_Actual = Btldr_Vers_Actual_str;
    Dat_Vers->Btldr_Vers_Eeprom = Btldr_Vers_Eeprom_str;
// ********************************************************************************
//	memcpy(Dat_Vers->Btldr_Vers_Actual, BL_VERS, EEPROM_VERSION_SIZE);
	memcpy(Dat_Vers->Btldr_Vers_Actual,(uint8_t *)FLASH_BLDR_VERS_ADDR, sizeof(BL_VERS) - 1);
// ********************************************************************************
}


//---------------------------------------------------------------------------------------------
static void SyncStructInit  (
                            Sync_Inst_t *Dat_Sync
                            )
{
	Dat_Sync->Sync_Src_Priority = Sync_Src_Prior_Set;
	Dat_Sync->Flags = 0;
    Dat_Sync->Sync_Period = SYNC_OVERTIME;

//    Dat_Sync->SNTP_1_IP = SNTP_IP1;
//    Dat_Sync->SNTP_2_IP = SNTP_IP2;
//    Dat_Sync->SNTP_3_IP = SNTP_IP3;
}


//---------------------------------------------------------------------------------------------
void IdInit (
            Id_Inst_t *Dat_Id
            )
{
	uint8_t temp;

    IdStructInit(Dat_Id);

    // start reading from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
    ReadEeprom(	EEPROM_ID_DATA_ADDR + EEPROM_DEVFUNC_OFFSET,
                1,
                &temp );

    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        CLEAR_FLAG(Dat_Id->Flags, f_Read_Err);
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        SET_FLAG(Dat_Id->Flags, f_Read_Err);
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);
    // data already has been read from EEPROM and located in RAM buffer
    // Check & save device function
    if(temp < 171 || temp > 174)
    {
        // Device function is wrong -> write 0
        Dat_Id->Func = 0;
        memcpy( Dat_Id->Name, UNKNOWN_IED, 8 );
    }
    else
    {
        // Device function is right
        Dat_Id->Func = temp;
        switch(temp)
        {
            case 171:
                memcpy( Dat_Id->Name, AB3_IED, 8 );
                break;

            case 172:
                memcpy( Dat_Id->Name, B3_IED, 8 );
                break;

            case 173:
                memcpy( Dat_Id->Name, BC3_IED, 8 );
                break;

            case 174:
            default:
                memcpy( Dat_Id->Name, A3_IED, 8 );
                break;
        }
    }
}


//---------------------------------------------------------------------------------------------
void VersInit   (
                Vers_Inst_t *Dat_Vers
                )
{
    uint8_t buf1[EEPROM_VERSION_SIZE * 3];
    uint8_t buf2[EEPROM_VERSION_SIZE * 3];

    VersStructInit(Dat_Vers);

    // start reading from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
    ReadEeprom(	EEPROM_ID_DATA_ADDR + EEPROM_FRMWR_VERSION_OFFSET,
                EEPROM_VERSION_SIZE * 3,
                buf1                    );

    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        CLEAR_FLAG(Dat_Vers->Flags, f_Read_Err);
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        SET_FLAG(Dat_Vers->Flags, f_Read_Err);
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);

    // Compare firmware version with new data
    if( memcmp(	buf1, Dat_Vers->Frmwr_Vers_Actual, EEPROM_VERSION_SIZE) != 0
        ||
        memcmp(	buf1+EEPROM_VERSION_SIZE, Dat_Vers->Hrdwr_Vers_Actual, EEPROM_VERSION_SIZE) != 0
        ||
        memcmp(	buf1+2*EEPROM_VERSION_SIZE, Dat_Vers->Btldr_Vers_Actual, EEPROM_VERSION_SIZE) != 0 )
    // Some version is wrong -> write actual version
    {
        memcpy( buf2,
                Dat_Vers->Frmwr_Vers_Actual,
                EEPROM_VERSION_SIZE   );
        memcpy( buf2+EEPROM_VERSION_SIZE,
                Dat_Vers->Hrdwr_Vers_Actual,
                EEPROM_VERSION_SIZE   );
        memcpy( buf2+2*EEPROM_VERSION_SIZE,
                Dat_Vers->Btldr_Vers_Actual,
                EEPROM_VERSION_SIZE   );

        xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
        WriteEeprom(EEPROM_ID_DATA_ADDR + EEPROM_FRMWR_VERSION_OFFSET,
                    3*EEPROM_VERSION_SIZE,
                    buf2                                            );
        if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
        {
            CLEAR_FLAG(Dat_Vers->Flags, f_Write_Err);
        }
        else
        {
            // Reset EEPROM control, set Failure Flag, increment error counter
            SET_FLAG(Dat_Vers->Flags, f_Write_Err);
            SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
            Eeprom_M95xxx.errors++;
            EepromInit();
        }
        xSemaphoreGive(Eep_Mutex);
    }
    else
    // Version is Ok -> version init is complete
    {
        memcpy( Dat_Vers->Frmwr_Vers_Eeprom,
                buf1,
                EEPROM_VERSION_SIZE   );
        memcpy( Dat_Vers->Hrdwr_Vers_Eeprom,
                buf1+EEPROM_VERSION_SIZE,
                EEPROM_VERSION_SIZE   );
        memcpy( Dat_Vers->Btldr_Vers_Eeprom,
                buf1+2*EEPROM_VERSION_SIZE,
                EEPROM_VERSION_SIZE   );
    }
}


//--------------------------  ---------------------------------
void SyncInit   (
                Sync_Inst_t *Dat_Sync
                )
{
	uint8_t buf[EEPROM_SYNC_SET_SIZE];

    SyncStructInit(Dat_Sync);

    // start reading from EEPROM
    xSemaphoreTake(Eep_Mutex, portMAX_DELAY);
    ReadEeprom(	EEPROM_SYNC_SET_ADDR,
                EEPROM_SYNC_SET_SIZE,
                buf                     );

    if(xSemaphoreTake(Eep_Cmplt_Semph, TIME_TIMEOUT_EEPROM) == pdPASS)
    {
        CLEAR_FLAG(Dat_Sync->Flags, f_Read_Err);
    }
    else
    {
        // Reset EEPROM control, set Failure Flag, increment error counter
        SET_FLAG(Dat_Sync->Flags, f_Read_Err);
        SET_FLAG(Eeprom_M95xxx.Flags_Error_Eeprom, f_Busy_Timeout_Eeprom);
        Eeprom_M95xxx.errors++;
        EepromInit();
    }
    xSemaphoreGive(Eep_Mutex);
    // data already has been read from EEPROM and located in RAM buffer

    // Check & save device function
    if( buf[EEPROM_SYNC_WRTAG_OFFSET] != 0x5A
        ||
        buf[EEPROM_SYNC_SET_CS_OFFSET] != CalcOrdCRC(buf, EEPROM_SYNC_SET_CS_OFFSET, 0x00))
    {
        // Sync system data is wrong -> write default settings
        memset((void*)Dat_Sync, 0, sizeof(Sync_Inst_t) - 4);
        SET_FLAG(Dat_Sync->Sync_Src_Enbls, EEPROM_SYNC_IEC103_2_PRIOR_OFFSET);
        Dat_Sync->Sync_Period = SYNC_OVERTIME;
    }
    else
    {
        // Sync system data is right
        // Save source's priority
        memcpy( Dat_Sync->Sync_Src_Priority,
                buf,
                SYNC_SRC_AMOUNT             );
        // Save source's enables
        Dat_Sync->Sync_Src_Enbls = *((uint32_t *)(buf + EEPROM_SYNC_SRC_ENBL_OFFSET));
        // Save sync period
        Dat_Sync->Sync_Period = *((uint32_t *)(buf + EEPROM_SYNC_PERIOD_OFFSET));
        // Save time zone
        Dat_Sync->Time_Zone = *(buf + EEPROM_SYNC_TIMEZONE_OFFSET);
        // Save summer time autojump
        Dat_Sync->Summer_Time_Auto = *(buf + EEPROM_SYNC_SUMMERTIME_AUTO_OFFSET);
        // Save SNTP servers IPs 1..3
        memcpy( &(Dat_Sync->SNTP_1_IP),
                (buf + EEPROM_SYNC_SNTP_IP1_OFFSET),
                12                           );
    }
}


