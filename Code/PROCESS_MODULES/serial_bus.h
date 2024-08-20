//*********************************************************************************************
//                                    Serial_bus.h
//*********************************************************************************************

#ifndef __SERIAL_BUS__
#define __SERIAL_BUS__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "gpio.h"
#include "usart.h"
#include "timer.h"

#include "eeprom_m95xxx.h"


//#ifdef _TICK_COUNT_ON_
#include "dwt.h"
//#endif


//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef struct {
	uint32_t On_Off;
	uint32_t Protocol;
	uint32_t Switch_Baud_Rate;
	uint32_t Switch_Parity;
	uint32_t Switch_Stop_Bit;
	uint32_t Address;
    uint32_t Additional_Param;
	uint32_t Pause_1symbol_us;

	uint32_t Flags;
} Serbus_Param_t;


typedef enum {
	f_Err_Init = 0,
	f_Err_Frame_Rx_Start,
    f_Err_Wrong_Frame,
    f_Err_Function_Size_Rx,
	f_Err_Buf_Overflow_Rx,
	f_Err_Crc_Rx,
	f_Err_Execute_Impossible,
	f_Err_Server_Busy,
	f_Err_Settings_Read,
} Serbus_Error_Flags_t;


typedef enum {
	f_Serbus_Init = 0,
    f_New_Frame_Received,
    f_Modbus_Frame_Received,
    f_Iec103_Frame_Received,
} Serbus_State_Flags_t;


typedef struct {
	uint16_t	    Frame_Size;
	uint16_t	    Byte_Counter;
} Serbus_LPCI_t;


typedef struct {
	USART_TypeDef 	*Periph_Usart;
	TIM_TypeDef 	*Periph_Timer;
	Serbus_Param_t	*Periph_Settings;
} Serbus_struc_t;


typedef struct {
	uint8_t	            *Rqst_LPDU;
	uint8_t	            *Resp_LPDU;
    Serbus_LPCI_t       Rqst_LPCI;
    Serbus_LPCI_t       Resp_LPCI;

	SemaphoreHandle_t   *DataRcvd;
	SemaphoreHandle_t   *DataSent;

	SemaphoreHandle_t   *RqstToApp;
	SemaphoreHandle_t   *RespFromApp;

	uint32_t	        Time_Stamp;
    uint32_t            State_Flags;
	uint32_t	        Error_Flags;
	uint32_t 		    *errors;
    Serbus_struc_t      Serbus_Periph;

} Serbus_inst_t;







//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void ReceivedSymbolHandler(Serbus_inst_t *);
void TransmissionFrameEndHandler(Serbus_inst_t *);
void MyUartErrHandler(Serbus_inst_t *);
void DmaTransmissionFrameHandler(Serbus_inst_t *);
void ReceivedFrameHandler(Serbus_inst_t *);
uint32_t FrameRxStartHandler(Serbus_inst_t *);
void FrameTxStartHandler(Serbus_inst_t *);

//void SerbusStructInit(Serbus_inst_t *);
void GetSerbusSettings(Serbus_Param_t *);
//void TurnOnSerbus(Serbus_inst_t *);
void vSerbusTask(void *);
BaseType_t CreateSerbusTask(Serbus_inst_t *);


#endif

