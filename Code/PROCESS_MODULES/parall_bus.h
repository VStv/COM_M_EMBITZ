//*********************************************************************************************
//                                       Parall_bus.h
//*********************************************************************************************
#ifndef __PARALL_BUS__
#define __PARALL_BUS__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "gpio.h"
#include "exti.h"

#include "crc16.h"

#include "modbus.h"

#ifdef _IEC103_ON_
#include "serial_bus.h"
#include "iec60870_103.h"
#endif

//#ifdef _TICK_COUNT_ON_
#include "dwt.h"
//#endif

//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
#define PARALL_BUS_MASK							((uint32_t)(1UL<<DB0_Pin | 1UL<<DB1_Pin	| 	\
															1UL<<DB2_Pin | 1UL<<DB3_Pin | 	\
															1UL<<DB4_Pin | 1UL<<DB5_Pin | 	\
															1UL<<DB6_Pin | 1UL<<DB7_Pin))

#define PARALL_BUS_RX_BYTE()					((uint8_t)(DB_Port->IDR >> DB0_Pin))

#define PARALL_BUS_TX_BYTE(data_8)              (DB_Port->ODR = (DB_Port->ODR & ((uint32_t)~(0x000000ff << DB0_Pin))) | ((uint32_t)data_8 << DB0_Pin))

#define PARBUS_INPUT()                          DB_Port->CRH &= (uint32_t)~((0x0fU<<(4*(DB0_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB1_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB2_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB3_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB4_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB5_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB6_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB7_Pin-8))));  \
                                                DB_Port->CRH |= (uint32_t)(	(0x08U<<(4*(DB0_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB1_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB2_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB3_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB4_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB5_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB6_Pin-8)))|   \
                                                                            (0x08U<<(4*(DB7_Pin-8))))

#define PARBUS_OUTPUT()                         DB_Port->CRH &= (uint32_t)~((0x0fU<<(4*(DB0_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB1_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB2_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB3_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB4_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB5_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB6_Pin-8)))|   \
                                                                            (0x0fU<<(4*(DB7_Pin-8))));  \
                                                DB_Port->CRH |= (uint32_t)(	(0x03U<<(4*(DB0_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB1_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB2_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB3_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB4_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB5_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB6_Pin-8)))|   \
                                                                            (0x03U<<(4*(DB7_Pin-8))))

#define PARBUS_RX_START()                       RESET_PIN(OECOM2_Port, OECOM2_Pin);                 \
                                                Parbus.Resp_LPCI.Frame_Size = 0;                    \
                                                Parbus.Resp_LPCI.Byte_Counter = 0;                  \
                                                EXTI->PR = (uint32_t)(1<<INCPU2_Pin);               \
                                                EXTI->IMR |= (uint32_t)(1<<INCPU2_Pin)


//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef enum {
	f_Err_Crc_Parbus = 0,
	f_End_Receive_Parbus,
    f_Retrans_Rqst_From_Modbus,
} Flags_Control_Parbus_t;


typedef struct {
	uint16_t	    Frame_Size;
	uint16_t	    Byte_Counter;
} Parbus_LPCI_t;


typedef struct {
	uint8_t	            *Rqst_LPDU;     // Parbus_APDU_t
	uint8_t	            *Resp_LPDU;     // Parbus_APDU_t
    Parbus_LPCI_t       Rqst_LPCI;
    Parbus_LPCI_t       Resp_LPCI;

	SemaphoreHandle_t   *Mtx;
	SemaphoreHandle_t   *ProcessEnbld;
	SemaphoreHandle_t   *DataRcvd;

	SemaphoreHandle_t   *RqstToApp;     // to Modbus
	SemaphoreHandle_t   *RespFromApp;   // from Modbus

    uint32_t            State_Flags;

} Parbus_inst_t;


typedef struct {
	uint8_t		    Func;
	uint8_t		    ASDU_Size;
	uint8_t		    ASDU[];
} Parbus_APDU_t;




//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
BaseType_t CreateParbusTask(Parbus_inst_t *);
void vParbusTask(void *);
uint16_t ParbusLinkProcess(uint8_t *, uint16_t, uint8_t *);

void ParbusPeriphInit(void);

void ParbusLinkTxHandler(void);
void ParbusLinkRxHandler(void);


#endif


