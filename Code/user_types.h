//*********************************************************************************************

#ifndef __USER_TYPES__
#define __USER_TYPES__

//---------------------------------------------------------------------------------------------
//                                      Include section
//---------------------------------------------------------------------------------------------
#include "stm32f10x.h"
#include "compile_def.h"



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------

#define PROTOCOL_MODBUS				((uint8_t)0)
#define PROTOCOL_103				((uint8_t)1)

#define MODULE_OFF					((uint8_t)0)
#define MODULE_ON					((uint8_t)1)

#define PARITY_BIT_NO				((uint8_t)0)
#define PARITY_BIT_EVEN				((uint8_t)1)
#define PARITY_BIT_ODD				((uint8_t)2)

#define STOP_BIT_1					((uint8_t)0)
#define STOP_BIT_2					((uint8_t)1)

#define BAUDRATE_4800   			((uint8_t)0)
#define BAUDRATE_9600   			((uint8_t)1)
#define BAUDRATE_19200   			((uint8_t)2)
#define BAUDRATE_38400   			((uint8_t)3)
#define BAUDRATE_57600   			((uint8_t)4)
#define BAUDRATE_115200   			((uint8_t)5)

#define SINGLE_CHAR_MSG_DSBL		((uint8_t)0)
#define SINGLE_CHAR_MSG_ENBL		((uint8_t)1)

//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------
//                                      Typedef section
//---------------------------------------------------------------------------------------------
// Error log registers
typedef enum{
    START_TASK_NUM = 0,
	PARBUS_TASK_NUM,
    MODBUS_TASK_NUM,
    SERBUS0_TASK_NUM,
    SERBUS1_TASK_NUM,
#ifdef _IEC103_ON_
    IEC103_0_TASK_NUM,
    IEC103_1_TASK_NUM,
#endif

    PB_FWD_TASK_NUM,
    MB_FWD_TASK_NUM,
    SB0_FWD_TASK_NUM,
    SB1_FWD_TASK_NUM,

#ifdef _IEC103_ON_
    IEC103_0_FWD_TASK_NUM,
    IEC103_1_FWD_TASK_NUM,
#endif

#ifdef _BOOTLOADER_ON_
	FW_UPD_TASK_NUM,
#endif

#ifdef _MONITOR_ON_
	MONITOR_TASK_NUM,
#endif
    TASK_NUM_SIZE
} task_num_t;



// Error log registers
typedef enum{
    RESULT_OK = 0,
	NOT_READY,
    RESULT_ALT,
	WRONG_FRAME_ERR,
	INSTANCE_ERR,
	DMA_ON_TIMEOUT_ERR,
	DMA_OFF_TIMEOUT_ERR,
	INIT_ERR,
    OVERRUN_ERR,
    NOISE_ERR,
    FRAME_ERR,
    PARITY_ERR,
    DMA_ERR,
	TIMEOUT_ERR,
	BUFOVER_ERR,
    INVALID_FUNC_ERR,
    INVALID_ADDR_ERR,
    WRONG_FUNC_SIZE_ERR,
	CRC_ERR,
	OUT_OF_RANGE,
    BUSY_ERR,
	UNKNOWN_PROTOCOL_ERR,
    WRONG_ASDU_ERR,
    REL_TAB_ERR,

    ERR_LOG_SIZE
} err_log_reg_t;





#endif
