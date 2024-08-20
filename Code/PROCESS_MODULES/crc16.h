//*********************************************************************************************
//                                         CRC16.h
//*********************************************************************************************

#ifndef __CRC16__
#define __CRC16__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "config.h"





//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------




//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------
//                                  Function's prototypes
//----------------------------------------------------------------------------------------------
uint16_t CalcCrc16(uint8_t *, uint16_t, uint16_t);
uint8_t CalcOrdCRC(uint8_t *, uint16_t, uint8_t initCRC);


#endif
