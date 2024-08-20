//*********************************************************************************************
//                                     message_queue.h
//*********************************************************************************************

#ifndef __MESSAGE_QUEUE__
#define __MESSAGE_QUEUE__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define	RAM_MSG_QUEUE_MAX_SIZE		    512 //1024                                                    // maximum quantity messages
#define	RAM_MSG_SIZE			        16                            // sizeof(Iec103_ASDU_t) //  bytes in message
#define	RAM_MSG_QUEUE_BUF_SIZE		    (RAM_MSG_QUEUE_MAX_SIZE * RAM_MSG_SIZE)                 // bytes in message queue buffer



//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef struct {
	uint8_t				*Msg_Queue_Buf;				//
	uint32_t			Msg_Queue_Head_offset;	    // queue head offset from message buffer start
	uint32_t			Msg_Queue_Tail_offset;	    // queue tail offset from message buffer start
	uint32_t			Msg_Queue_Len;	        	// current quantity of messages in buffer
} Msg_Queue_t;







//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------





//----------------------------------------------------------------------------------------------
//                                  Function's prototypes
//----------------------------------------------------------------------------------------------
void MsgQueueClear(Msg_Queue_t *);
uint32_t MsgAddToQueueHead(Msg_Queue_t *, uint8_t *, uint16_t);
uint32_t MsgAddToQueueTail(Msg_Queue_t *, uint8_t *, uint16_t);
uint32_t MsgGetFromQueue(Msg_Queue_t *, uint8_t *, uint16_t);


#endif
