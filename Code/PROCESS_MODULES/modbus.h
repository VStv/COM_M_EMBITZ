//*********************************************************************************************
//                                        Modbus.h
//*********************************************************************************************

#ifndef __MODBUS__
#define __MODBUS__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "crc16.h"

#include "vers.h"
#include "rel_table.h"




//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef enum {
    MODBUS_OK = 0,
	MODBUS_ILLEGAL_FUNCTION,
    MODBUS_ILLEGAL_DATA_ADDRESS,
    MODBUS_ILLEGAL_DATA_VALUE,
    MODBUS_SERVER_DEVICE_FAILURE,
    MODBUS_ACKNOWLEDGE,
    MODBUS_SERVER_DEVICE_BUSY,
} Modbus_Error_t;


typedef struct {
	uint8_t			    Func;
	uint8_t				ASDU[];
} Modbus_PDU_t;


typedef struct {
	uint8_t	            *Rqst_PDU;          // Modbus_PDU_t
	uint8_t     	    *Resp_PDU;
    uint8_t             Rqst_PDU_Size;
    uint8_t             Resp_PDU_Size;

	SemaphoreHandle_t   *Mtx;
	SemaphoreHandle_t   *DataRcvd;
	SemaphoreHandle_t   *DataSent;

	SemaphoreHandle_t   *RqstToApp;         // to Parbus
	SemaphoreHandle_t   *RespFromApp;       // from Parbus

} Modbus_inst_t;




//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                  Function's prototypes
//----------------------------------------------------------------------------------------------
// Link layer
uint16_t ModbusLinkProcess(uint8_t *, uint16_t, uint8_t *, uint8_t);

// Application layer
void vModbusTask(void *);
BaseType_t CreateModbusTask(Modbus_inst_t *);

#endif
