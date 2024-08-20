//*********************************************************************************************
//                                     message_queue.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "message_queue.h"


//---------------------------------------------------------------------------------------------
//                                   Tables of Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------
//								   Message queue processing
// --------------------------------------------------------------------------------------------
// ------------------------------------ Clear message queue------------------------------------
void MsgQueueClear  (
                    Msg_Queue_t *Msg_Queue
                    )
{
    Msg_Queue->Msg_Queue_Len = 0;
    Msg_Queue->Msg_Queue_Head_offset = 0;
    Msg_Queue->Msg_Queue_Tail_offset = 0;
}


// -------------------------------- Add message to queue head ---------------------------------
uint32_t MsgAddToQueueHead  (
                            Msg_Queue_t *Msg_Queue,
                            uint8_t *msg,
                            uint16_t size
                            )
{
	// new offset of queue head
	if(Msg_Queue->Msg_Queue_Head_offset < RAM_MSG_SIZE)
		Msg_Queue->Msg_Queue_Head_offset = RAM_MSG_QUEUE_BUF_SIZE - RAM_MSG_SIZE;
	else
		Msg_Queue->Msg_Queue_Head_offset -= RAM_MSG_SIZE;

    // copy data from message to queue head
	memcpy( Msg_Queue->Msg_Queue_Buf + Msg_Queue->Msg_Queue_Head_offset,
            msg,
            size	);
	
    // new quantity of queue
	if(Msg_Queue->Msg_Queue_Len < RAM_MSG_QUEUE_MAX_SIZE)
		Msg_Queue->Msg_Queue_Len++;
	else
	{
		Msg_Queue->Msg_Queue_Len = RAM_MSG_QUEUE_MAX_SIZE;
		// new offset of queue tail (if queue is full)
		Msg_Queue->Msg_Queue_Tail_offset = Msg_Queue->Msg_Queue_Head_offset;
	}
    return RESULT_OK;
}


// -------------------------------- Add message to queue tail ---------------------------------
uint32_t MsgAddToQueueTail  (
                            Msg_Queue_t *Msg_Queue,
                            uint8_t *msg,
                            uint16_t size
                            )
{
    // copy data from ASDU to queue tail
	memcpy( Msg_Queue->Msg_Queue_Buf + Msg_Queue->Msg_Queue_Tail_offset, 
            msg,
            size	);

    // new offset of queue tail
    Msg_Queue->Msg_Queue_Tail_offset += RAM_MSG_SIZE;
	if(Msg_Queue->Msg_Queue_Tail_offset >= RAM_MSG_QUEUE_BUF_SIZE)
		Msg_Queue->Msg_Queue_Tail_offset = 0;

    // new quantity of queue
	if(Msg_Queue->Msg_Queue_Len < RAM_MSG_QUEUE_MAX_SIZE)
		Msg_Queue->Msg_Queue_Len++;
	else
	{
		Msg_Queue->Msg_Queue_Len = RAM_MSG_QUEUE_MAX_SIZE;
		// new offset of queue head (if queue is full)
		Msg_Queue->Msg_Queue_Head_offset = Msg_Queue->Msg_Queue_Tail_offset;
	}
    return RESULT_OK;
}


// ------------------------------ Get message from queue head ---------------------------------
uint32_t MsgGetFromQueue(
                        Msg_Queue_t *Msg_Queue,
                        uint8_t *msg,
                        uint16_t size
                        )
{
    // if queue is empty - error
	if(Msg_Queue->Msg_Queue_Len == 0)
		return 1;
	
    // copy data from queue head to ASDU
	memcpy( msg,
            Msg_Queue->Msg_Queue_Buf + Msg_Queue->Msg_Queue_Head_offset,
			size	);
	
	// new offset of queue head
    Msg_Queue->Msg_Queue_Head_offset += RAM_MSG_SIZE;
	if(Msg_Queue->Msg_Queue_Head_offset >= RAM_MSG_QUEUE_BUF_SIZE)
		Msg_Queue->Msg_Queue_Head_offset = 0;

    // new quantity of queue
	Msg_Queue->Msg_Queue_Len--;

    return RESULT_OK;
}


