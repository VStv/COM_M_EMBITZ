//*********************************************************************************************
//                                          Config.h
//*********************************************************************************************

#ifndef __CONFIG__
#define __CONFIG__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "stm32f10x.h"
#include "compile_def.h"
#include "user_types.h"
#include "common_macro.h"
#include "mem_adr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include <stdio.h>
#include <string.h>
//#include <stddef.h>


// ------------------ Program module control -----------------





//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
// Version constants
#define FW_VERS					        	"    2.02"
#define BL_VERS					        	"    ----"
#define HW_VERS					        	"    RS.0" //"     M.0" //

// Device's names
#define AB3_IED					        	"PC83-AB3" //"HTP-B-02" //
#define B3_IED					        	"PC83-B3 " //"HTP-H-02" //
#define BC3_IED					        	"PC83-BC3" //"HTP-B-03" //
#define A3_IED					        	"PC83-A3 "  //"HTP-B-01" //
#define UNKNOWN_IED					        "UNKNOWN "





//--------------------------------------- Timeouts --------------------------------------------
#define TIME_LOCK_FUNCTION					40		// 20ms
#define PARBUS_RESP_TIMEOUT         		10		// 10ms
#define SERBUS_RESP_TIMEOUT         		300		// 10ms
#define MODBUS_RESP_TIMEOUT					200		// 200ms
#define IEC103_RESP_TIMEOUT					200		// 50ms
#define KEY_RESP_WAIT_TIMEOUT				150		// 150ms

#define TIME_START_PARALL_BUS  				10		// 10ms
#define TIME_TIMEOUT_EEPROM					200		// 20ms
#define TIME_SYNCHRO_MILISECOND				950    	// 950ms
#define TIME_TIMEOUT_IEC103_GEN_CMD         200     // 200ms

//----------------------------------- Descrete signals ----------------------------------------
#define DI_AMOUNT					        40		//
#define KL_AMOUNT					        40		//
#define VDI_AMOUNT					        40		//





//------------------------------------- Use ------------------------------------------










//------------------------------------- Ports'n'Pins ------------------------------------------
//-------------------------------- Parallel Bus Data Lines ------------------------------------
// (<->)
#define DB0_Pin						8
#define DB0_Port					GPIOB
#define DB_Port						DB0_Port

#define DB1_Pin						9
#define DB1_Port					GPIOB

#define DB2_Pin						10
#define DB2_Port					GPIOB

#define DB3_Pin						11
#define DB3_Port					GPIOB

#define DB4_Pin						12
#define DB4_Port					GPIOB

#define DB5_Pin						13
#define DB5_Port					GPIOB

#define DB6_Pin						14
#define DB6_Port					GPIOB

#define DB7_Pin						15
#define DB7_Port					GPIOB

//------------------------------ Parallel Bus Control Lines -----------------------------------
// (->)
#define OUTCPU2_Pin					9
#define OUTCPU2_Port				GPIOC

// (<-)
#define INCPU2_Pin					8
#define INCPU2_Port					GPIOC

// (<-)
#define CSIN2_Pin					7
#define CSIN2_Port					GPIOC

// (->)
#define CSOUT2_Pin					6
#define CSOUT2_Port					GPIOC

// (<-)
#define ENCOM1_Pin					2
#define ENCOM1_Port					GPIOD

// (->)
#define OECOM2_Pin					1
#define OECOM2_Port					GPIOB

// (->)
#define DIRCOM2_Pin					2
#define DIRCOM2_Port				GPIOB

//// (->)
//#define ENCOM2_Pin					10
//#define ENCOM2_Port					GPIOC


//--------------------------------------- LED -------------------------------------------------
// (->)
#define LED_Pin						10
#define LED_Port					GPIOC


//--------------------------------------- PPS -------------------------------------------------
// (->)
#define PPS_Pin						0
#define PPS_Port					GPIOB


//-------------------------------------- UART -------------------------------------------------
// (<-)
#define RX1_Pin						10
#define RX1_Port					GPIOA

// (->)
#define TX1_Pin						9
#define TX1_Port					GPIOA

// (->)
#define ENTX1_Pin					8
#define ENTX1_Port					GPIOA

// (<-)
#define RX2_Pin						3
#define RX2_Port					GPIOA

// (->)
#define TX2_Pin						2
#define TX2_Port					GPIOA

// (->)
#define ENTX2_Pin					1
#define ENTX2_Port					GPIOA


//--------------------------------------- SPI -------------------------------------------------
// (->)
#define MOSI1_Pin					7
#define MOSI1_Port					GPIOA

// (<-)
#define MISO1_Pin					6
#define MISO1_Port					GPIOA

// (->)
#define SCK1_Pin					5
#define SCK1_Port					GPIOA

// (->)
#define CSS1_Pin					4
#define CSS1_Port					GPIOA



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------





#endif

