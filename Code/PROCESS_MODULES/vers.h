//*********************************************************************************************
//                                         Vers.h
//*********************************************************************************************

#ifndef __VERS_H__
#define __VERS_H__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"

#include "eeprom_m95xxx.h"
#include "crc16.h"



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------

#define SYNC_OVERTIME       (1*60*60)




//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef enum {
	f_Read_Err = 0,
	f_Write_Err,
} Flags_Data_Process_t;


typedef struct {
    uint8_t             *Frmwr_Vers_Eeprom;
    uint8_t             *Hrdwr_Vers_Eeprom;
    uint8_t             *Btldr_Vers_Eeprom;

    uint8_t             *Frmwr_Vers_Actual;
    uint8_t             *Hrdwr_Vers_Actual;
    uint8_t             *Btldr_Vers_Actual;

	uint32_t	        Flags;
} Vers_Inst_t;


typedef struct {
    uint8_t             *Name;
    uint8_t             Func;

	uint32_t	        Flags;
} Id_Inst_t;


typedef enum {
	SYNC_SNTP_1 = 0,
	SYNC_SNTP_2,
	SYNC_SNTP_3,
	SYNC_IEC103_1,
	SYNC_IEC103_2,
	SYNC_IEC104_1,
	SYNC_IEC104_2,
	SYNC_IEC104_3,
	SYNC_IEC104_4,
    SYNC_PPS,
    SYNC_PPS_SERBUS0,
    SYNC_PPS_SERBUS1,
    SYNC_PTP,
    SYNC_IRIGB,
    SYNC_RES1,
    SYNC_RES2,
    SYNC_RES3,
    SYNC_RES4,
    SYNC_RES5,
    SYNC_RES6,
    SYNC_SRC_AMOUNT,
} Sync_Src_Num_t;


typedef  struct {                       //__packed
	uint8_t	    *Sync_Src_Priority;     // 20* (0..31)
    uint32_t	Sync_Src_Enbls;         // 1- Enable, 0- Disable
    uint32_t	Sync_Period;            // 1..70000
    uint32_t	Time_Zone;            	//		int8_t	    Time_Zone;              // -12..11
    uint32_t	Summer_Time_Auto;       //      uint8_t	    Summer_Time_Auto;       // 1- Yes, 0- No
    uint8_t	    SNTP_1_IP[4];           //*SNTP_1_IP;
    uint8_t	    SNTP_2_IP[4];           //*SNTP_2_IP;
    uint8_t	    SNTP_3_IP[4];           //*SNTP_3_IP;

	uint32_t	Flags;
} Sync_Inst_t;

//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                  Function's prototypes
//----------------------------------------------------------------------------------------------
void IdInit(Id_Inst_t *);
void VersInit(Vers_Inst_t *);
void SyncInit(Sync_Inst_t *);


#endif
