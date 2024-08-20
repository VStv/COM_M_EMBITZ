//*********************************************************************************************
//                                         Flash.h
//*********************************************************************************************

#ifndef __FLASH__
#define __FLASH__

//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
#include "stm32f10x.h"
//#include "mem_adr.h"


//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------

typedef struct {
	uint16_t	    *Data;
    uint32_t        Addr;
    uint32_t        Size;
} Flash_struc_t;




//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
//#define FLASH_KEY1 			(uint32_t)0x45670123
//#define FLASH_KEY2 			(uint32_t)0xCDEF89AB


#define BANK_1 			    (uint32_t)0x08000000
#define BANK_2 			    (uint32_t)0x08080000

//#if defined(STM32F10X_XL) || defined(STM32F10X_CL) || defined(STM32F10X_HD) || defined(STM32F10X_HD_VL)
//#define PAGE_SIZE 			(uint32_t)0x00000800
//#else
//#define PAGE_SIZE 			(uint32_t)0x00000400
//#endif

//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
//---------------------------- Locking MCU flash memory (  us) --------------------------------

#define FLASH_READ(addr)    (*(__IO uint16_t *)(addr))

//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void FlashInit(void);

void FlashReadBuf(Flash_struc_t *, uint16_t *, uint32_t, uint32_t);
uint16_t FlashRead(Flash_struc_t *);

void FlashErasePage(uint32_t);
void FlashWriteBuf(Flash_struc_t *, uint16_t *, uint32_t, uint32_t);

#ifdef _FLASH_IRQ_ON_
void FlashEraseStart(uint32_t);
void FlashWriteStart(Flash_struc_t *, uint16_t *, uint32_t, uint32_t);

void FlashEraseFin(Flash_struc_t *);
void FlashWriteNext(Flash_struc_t *);

void FlashProgHandler(Flash_struc_t *);
#endif

#endif
