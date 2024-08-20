//*********************************************************************************************
//                                    	   Fw_upd.h
//*********************************************************************************************
#ifndef __FW_UPD__
#define __FW_UPD__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "flash.h"
#include "rtc.h"
#include "usart.h"
#include "timer.h"
#include "spi.h"


#include "crc16.h"


//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define FW_UPD_NOT_NEED			(uint16_t)0
#define FW_UPD_NEED				(uint16_t)0x5aa5

#define FW_OK					(uint16_t)0
#define FW_ILLEGAL				(uint16_t)0x3cc3


#define NO_UPDATE			    0
#define UPDATE_PROGRESS			1
#define UPDATE_DONE				2

#define NO_RQST			        0
#define RQST_RECEIVED	        1
#define RQST_CONFIRMED	        2



//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------






//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef struct {
    uint32_t	    Fw_Size;
    uint16_t	    Fw_Crc;
    uint16_t	    Fw_Crc_crrnt;
    uint32_t	    Block_Amount;
    uint32_t	    Block_Index;
    uint32_t	    Page_Index;
    uint32_t	    Page_Data_Size;
    uint32_t        Flags;
    uint32_t        Err_Code;
    uint16_t	    *Page_Buffer;
} Fw_update_struc_t;




//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void BldrEnterKey(rtc_strct_t *);

BaseType_t CreateFwUpdateTask(void);


#endif


