//*********************************************************************************************
//                                          Compile_def.h
//*********************************************************************************************

#ifndef __COMPILE_DEF__
#define __COMPILE_DEF__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------
//                                     Compiling control
//---------------------------------------------------------------------------------------------

#ifdef DEBUG_TARGET

    #define _DEBUG_ON_

    #ifdef _DEBUG_ON_
        #define _MONITOR_ON_
//      #define _TICK_COUNT_ON_
    #endif

    #define _IEC103_ON_


//    #define _BOOTLOADER_ON_



#endif


#ifdef RELEASE_TARGET

    #define _IEC103_ON_
    #define _BOOTLOADER_ON_

    #ifdef _BOOTLOADER_ON_
        #define _FLASH_IRQ_ON_
    #endif

#endif



#endif
