//*********************************************************************************************
//                                       Interrupts.h
//*********************************************************************************************

#ifndef __INTERRUPTS__
#define __INTERRUPTS__

//---------------------------------------------------------------------------------------------
//                                      Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "exti.h"
#include "rtc.h"

#ifdef _FLASH_IRQ_ON_
#include "flash.h"
#endif

#include "parall_bus.h"
#include "serial_bus.h"
#include "vers.h"

//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define SYSHND_CTRL  (*(volatile unsigned int*)  (0xE000ED24u))  // System Handler Control and State Register
#define NVIC_MFSR    (*(volatile unsigned char*) (0xE000ED28u))  // Memory Management Fault Status Register
#define NVIC_BFSR    (*(volatile unsigned char*) (0xE000ED29u))  // Bus Fault Status Register
#define NVIC_UFSR    (*(volatile unsigned short*)(0xE000ED2Au))  // Usage Fault Status Register
#define NVIC_HFSR    (*(volatile unsigned int*)  (0xE000ED2Cu))  // Hard Fault Status Register
#define NVIC_DFSR    (*(volatile unsigned int*)  (0xE000ED30u))  // Debug Fault Status Register
#define NVIC_BFAR    (*(volatile unsigned int*)  (0xE000ED38u))  // Bus Fault Manage Address Register
#define NVIC_AFSR    (*(volatile unsigned int*)  (0xE000ED3Cu))  // Auxiliary Fault Status Register




//---------------------------------------------------------------------------------------------
//                                          Macros
//---------------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------------
//                                      Typedef section
//---------------------------------------------------------------------------------------------
#ifdef _DEBUG_ON_
struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t psr;
} *stack_ptr;
#endif


//----------------------------------------------------------------------------------------------
//                                   Function's prototypes
//----------------------------------------------------------------------------------------------




#endif
