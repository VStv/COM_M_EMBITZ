//*********************************************************************************************
//                                         Rel_table.h
//*********************************************************************************************

#ifndef __REL_TABLE_H__
#define __REL_TABLE_H__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "eeprom_m95xxx.h"
#include "crc16.h"




//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef enum {
	f_Table_Read_Err = 0,
	f_Table_Ok,
} Flags_Rel_Tab_t;


typedef enum {
	MODBUS_TAB_HEAD = 0,
	MODBUS_0x01,
	MODBUS_0x02,
	MODBUS_0x03,
	MODBUS_0x04,
	MODBUS_0x05,
	MODBUS_0x06,
	MODBUS_0x0f,
	MODBUS_0x10,

	MODBUS_REG_TAB_SIZE,
} Modbus_Region_Num_t;


typedef enum {
	IEC103_TAB_HEAD = 0,
	IEC103_DISCR_INOUT_STATE,
	IEC103_DISCR_OUT_OFF,
	IEC103_DISCR_OUT_ON,
	IEC103_ANLG_IN_MEAS,

	IEC103_REG_TAB_SIZE,
} Iec103_Region_Num_t;


typedef struct {
    uint32_t            Start_Offset;
    uint32_t            Final_Offset;
} Table_Region_t;


typedef struct {
    uint8_t             *Rel_Tab;
	uint32_t		    Flags;
    Table_Region_t 		*Region_Tab;
} Rel_Tab_struct_t;


typedef struct {
    uint8_t         *Data_Set;
	uint8_t			*Old_Data;
    uint32_t        Flags;
	uint8_t		    Scn;
	uint32_t		Signal_Num;
} Interview_struct_t;


//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                  Function's prototypes
//----------------------------------------------------------------------------------------------
uint32_t RemapTabInit(Rel_Tab_struct_t *, uint32_t, uint32_t);

void InterviewStructInit(Interview_struct_t *, uint8_t *, uint8_t *);
void GetInterviewSetTab (uint8_t *, uint32_t);


#endif
