//*********************************************************************************************
//                                      iec60870_103.h
//*********************************************************************************************

#ifndef __IEC60870_103__
#define __IEC60870_103__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "crc16.h"
#include "eeprom_m95xxx.h"

#include "rtc.h"
#include "message_queue.h"

#include "rel_table.h"

#include "serial_bus.h"
#include "vers.h"



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define FIX_LEN_FRAME_LENGTH 					5


// IEC60870-103 start symbols
#define VAR_LEN_FRAME_START_SYM					0x68
#define FIX_LEN_FRAME_START_SYM 				0x10
#define SINGLE_CHAR_FRAME_START_SYM				0xe5

// IEC60870-103 final symbol
#define FIN_SYM 								0x16

// IEC60870-103 control symbol bits
#define PRM_BIT 								6
// for PRM_BIT = 1
#define FCB_BIT 								5
#define FCV_BIT 								4
// for PRM_BIT = 0
#define ACD_BIT 								5
#define DFC_BIT 								4
// Func. code bits
#define FCODE_BIT0 							    0

#define PRM_MSTR 								(1 << PRM_BIT)
#define PRM_SLV 								(0 << PRM_BIT)

#define FCODE_MASK 							    (0x0f << FCODE_BIT0)

// IEC60870-103 Func. codes
// for PRM_BIT = 1 (0, 3, 4, 9, 10, 11)
#define FCODE_CONTR_RST_CU 					    0	// REQ -> 	<- CON 		FCV = 0		(S2)
#define FCODE_CONTR_DATA_NEED_ACK			    3	// REQ -> 	<- CON 		FCV = 1		(S2)
#define FCODE_CONTR_DATA_NO_ACK			        4	// REQ -> 				FCV = 0		(S1)
#define FCODE_CONTR_RST_FCB					    7	// REQ -> 	<- CON		FCV = 0		(S2)
#define FCODE_CONTR_LINK_STATE_RQST			    9	// REQ -> 	<- RESP 	FCV = 0		(S3)
#define FCODE_CONTR_DATA_1_RQST				    10	// REQ -> 	<- RESP 	FCV = 1		(S3)
#define FCODE_CONTR_DATA_2_RQST				    11	// REQ -> 	<- RESP 	FCV = 1		(S3)

// for PRM_BIT = 0 (0, 1, 8, 9, 11, 14, 15)
#define FCODE_MONIT_POS_ACK 					0	// 		 	<- CON+					(S2)
#define FCODE_MONIT_NEG_ACK					    1	// 		 	<- CON-					(S2)
#define FCODE_MONIT_DATA_RESP					8	// 			<- RESP 				(S3)
#define FCODE_MONIT_NO_DATA_RESP				9	// 			<- RESP 				(S3)
#define FCODE_MONIT_LINK_STATE_RESP			    11	// 			<- RESP 				(S3)
#define FCODE_MONIT_LINK_SRVC_DSBL				14	//
#define FCODE_MONIT_LINK_SRVC_NSUPP			    15	// 			<- CON 					(S2)


// IEC60870-103 Cause of transmission (COT) codes
#define COT_MONIT_SPORAD 						1
#define COT_MONIT_CYCLIC 						2
#define COT_MONIT_RESET_FCB						3
#define COT_MONIT_RESET_CU						4
#define COT_MONIT_START							5
//#define COT_MONIT_POWER_ON						6
//#define COT_MONIT_TEST							7
#define COT_MONIT_SYNC						    8
#define COT_MONIT_GI							9
#define COT_MONIT_GI_FIN						10
//#define COT_MONIT_LOCAL_OP						11
#define COT_MONIT_REMOTE_OP						12
#define COT_MONIT_CMD_ACK						20
#define COT_MONIT_CMD_NACK						21
//#define COT_MONIT_TRANS_FAULT					31
//#define COT_MONIT_GRP_CMD_WR_ACK				40
//#define COT_MONIT_GRP_CMD_WR_NACK				41
//#define COT_MONIT_GRP_CMD_RD_RESP_OK			42
//#define COT_MONIT_GRP_CMD_RD_RESP_NOK			43
//#define COT_MONIT_GRP_WR_ACK					44

#define COT_CONTR_SYNC 						    8
#define COT_CONTR_GI_START 						9
#define COT_CONTR_GEN_CMD 						20
//#define COT_CONTR_TRANS_FAULT					31
//#define COT_CONTR_GRP_CMD_WR					40
//#define COT_CONTR_GRP_CMD_RD					42


// IEC60870-103 Information number codes (INF)
#define INF_TIME_SYNC						    0		//
#define INF_GI_FIN						        0		//
#define INF_RESET_FCB						    2		// TYP = 5   COT = 3
#define INF_RESET_CU						    3		// TYP = 5   COT = 4
#define INF_START							    4		// TYP = 5   COT = 5
#define INF_CMD_KVIT 						    19
#define INF_CMD_SWITCH 						    40
#define INF_CMD_OSC 						    41
#define INF_CMD_KL1 						    200
#define INF_CMD_KL40 						    239


// IEC60870-103 ASDU type ID codes (TYP)
#define TYP_MONIT_TIME_TAG_MSG					1
#define TYP_MONIT_ID_MSG						5
#define TYP_MONIT_SYNC						    6
#define TYP_MONIT_GI_FIN					    8
#define TYP_MONIT_MEASURE_VAL				    9

#define TYP_CONTR_SYNC						    6
#define TYP_CONTR_GI_START					    7
#define TYP_CONTR_GEN_CMD 						20

// IEC60870-103 ASDU type function codes (FUN)
#define FUN_GLB						            255

// IEC60870-103 compatibility level
#define COL_NO_GS						        2
#define COL_GS						            3

// IEC60870-103 MEA value
#define MEA_OV						        	0
#define MEA_ER						        	1
#define MEA_LSB						        	3
#define MEA_S						        	15
#define MAX_MEA						        	0xfff

// IEC60870-103 information elements position
#define INF_EL_POS_DPI					        0
#define INF_EL_POS_SCN					       	0
#define INF_EL_POS_COL					        0
#define INF_EL_POS_SIN					        5
#define INF_EL_POS_RII					        1
#define INF_EL_POS_T4					        1
#define INF_EL_POS_T7					        0
#define INF_EL_POS_MEA					        0
#define INF_EL_POS_DCO					        0
#define INF_EL_POS_IED_ID					    1
#define INF_EL_POS_SW_ID					    9


// ASDU length
#define TYP_MONIT_TIME_TAG_MSG_LEN			    12
#define TYP_MONIT_ID_MSG_LEN					19
#define TYP_MONIT_SYNC_LEN						13
#define TYP_MONIT_GI_FIN_LEN					7



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------

typedef enum {
	NO_RESP = 0,
	SINGLE_CHAR_FRAME,
	FIX_LEN_FRAME,
	VAR_LEN_FRAME
} Iec103_Frame_Type_t;


typedef enum {
	f_Iec103_Init = 0,
	f_Last_FCB,
	f_Repeat_Last_Resp,
//	f_Iec103_Proc_Busy
} Iec103_Flags_t;


typedef struct {
// Data unit ID
	uint8_t			ASDU_TYP;					//
	uint8_t			ASDU_Var_Struc_Class;		//
	uint8_t			ASDU_COT;					//
	uint8_t			ASDU_Addr;					//

// Information object
    // Information object ID
	uint8_t			ASDU_FUN;					//
	uint8_t			ASDU_INF;					//
    // Information elements
	uint8_t			ASDU_Inf_el[10];
} Iec103_ASDU_t;


typedef struct {
    uint16_t        Last_Frame_Size;
	uint8_t	        Data_Size;
	uint8_t	        Ctrl_Byte;
    uint8_t	    	Frame_Type;
} Iec103_APCI_t;


typedef struct {
	uint8_t			*Station_Name;
	uint8_t			Station_Addr;
	uint8_t			Station_Func;
	uint8_t			Single_Char_Msg_Enbl;
	uint32_t        Pause_1symbol_us;
} Iec103_set_t;


typedef struct {
	uint8_t			*RqstToAppBuf;
	uint8_t     	*RespFromAppBuf;
    uint8_t         RqstToAppSize;
    uint8_t         RespFromAppSize;
	uint8_t		    DCO;
	uint8_t		    RII;
	uint8_t		    Cmd_INF;
}Iec103_Gen_Cmd_t;


typedef struct {
	uint8_t	            *Rqst_ASDU;         // Iec103_ASDU_t
	uint8_t     	    *Resp_ASDU;
    Iec103_APCI_t       Rqst_APCI;
    Iec103_APCI_t       Resp_APCI;

//	SemaphoreHandle_t   *Mtx;
//	SemaphoreHandle_t   *ProcessEnbld;
	SemaphoreHandle_t   *DataRcvd;
	SemaphoreHandle_t   *DataSent;

	SemaphoreHandle_t   *RqstToApp;         // to Modbus
	SemaphoreHandle_t   *RespFromApp;       // from Modbus

    SemaphoreHandle_t   *VDI_From_Parbus;
    SemaphoreHandle_t   *DIKL_From_Parbus;

    uint32_t            Time_Stamp;
    uint32_t	        Flags;
	Iec103_set_t        Settings;
    Msg_Queue_t         Queue;

    Iec103_Gen_Cmd_t    Gen_Cmd;
    Interview_struct_t  Spor_Evnt;
    Interview_struct_t  Gen_Intrv;
    uint32_t	        Rst_Status;
    uint32_t	        Sync_Status;
    uint32_t	        GI_Status;
    uint32_t	        GC_Status;
	uint32_t	        CI_Entry_Offset;
} Iec103_inst_t;


typedef enum {
	GC_NO = 0,
    GC_RCVD,
} GC_Status_t;


typedef enum {
	RST_START_STATE = 0,
    RST_CU_STATE,
    RST_FCB_STATE,
    RST_CMPLT_STATE,
} Reset_Status_t;


typedef enum {
	SYNC_WAIT_STATE = 0,
    SYNC_START_STATE,
} Sync_Status_t;


typedef enum {
	GI_INIT_STATE = 0,
    GI_ACTIVE_STATE,
} GI_Status_t;



//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------



//----------------------------------------------------------------------------------------------
//                                  Function's prototypes
//----------------------------------------------------------------------------------------------
// Link layer
//void Iec103StructInit(Iec103_inst_t *);
uint16_t Iec103LinkProcess(Iec103_inst_t *, uint8_t *, uint16_t, uint8_t *, uint8_t, uint32_t);


// Application layer
void vIec103Task(void *);
BaseType_t CreateIec103Task(Iec103_inst_t *);



#endif
