//*********************************************************************************************
//                                          Flash.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "flash.h"




//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
#ifdef _FLASH_IRQ_ON_
extern SemaphoreHandle_t            FlashEraseFinSemph;
extern SemaphoreHandle_t            FlashWriteDoneSemph;
#endif



//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//--------------------------- Unlocking MCU flash memory (  us) -------------------------------
static void FlashUnlock (
                        uint32_t address
                        )
{
	if(address < BANK_2)
	{
        while(FLASH->CR & FLASH_CR_LOCK)
        {
            FLASH->KEYR = FLASH_KEY1;
            FLASH->KEYR = FLASH_KEY2;
        }
    }
    else
    {
        while(FLASH->CR2 & FLASH_CR_LOCK)
        {
            FLASH->KEYR2 = FLASH_KEY1;
            FLASH->KEYR2 = FLASH_KEY2;
        }
    }
}


//---------------------------- Locking MCU flash memory (  us) --------------------------------
static void FlashLock   (
                        uint32_t address
                        )
{
	if(address < BANK_2)
	{
        FLASH->CR |= FLASH_CR_LOCK;
    }
    else
    {
        FLASH->CR2 |= FLASH_CR_LOCK;
    }
}




//--------------------------- Unlocking MCU flash memory (  us) -------------------------------
void FlashInit(void)
{
    FlashUnlock(BANK_1);
	FLASH->SR |= FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
	FLASH->CR = 0;
#ifdef _FLASH_IRQ_ON_
	FLASH->CR |= (FLASH_CR_EOPIE | FLASH_CR_ERRIE);
#endif
	FlashLock(BANK_1);

    FlashUnlock(BANK_2);
	FLASH->SR2 |= FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
	FLASH->CR2 = 0;
#ifdef _FLASH_IRQ_ON_
	FLASH->CR2 |= (FLASH_CR_EOPIE | FLASH_CR_ERRIE);
#endif
	FlashLock(BANK_2);

#ifdef _FLASH_IRQ_ON_
	NVIC_SetPriority(FLASH_IRQn, 8);
	FlashEraseFinSemph = xSemaphoreCreateBinary();
	FlashWriteDoneSemph = xSemaphoreCreateBinary();
#endif
}


//--------------------------- Unlocking MCU flash memory (  us) -------------------------------
static void InitFlashStruc  (
                            Flash_struc_t *pFlash,
                            uint16_t *buf,
                            uint32_t address,
                            uint32_t arr_size
                            )
{
    pFlash->Data = buf;
    pFlash->Addr = address;
    pFlash->Size = arr_size;
}



//-------------------------- Reading MCU flash memory page (  us) -----------------------------
void FlashReadBuf	(
                    Flash_struc_t *pFlash,
                    uint16_t *buf,
                    uint32_t address,
                    uint32_t arr_size
                    )
{
    InitFlashStruc(pFlash, buf, address, arr_size);
    while(pFlash->Size)
	{
		*(pFlash->Data) = FLASH_READ(pFlash->Addr);
        pFlash->Addr += 2;
        pFlash->Data++;
        pFlash->Size--;
	}
}


//-------------------------- Reading MCU flash memory page (  us) -----------------------------
uint16_t FlashRead	(
                    Flash_struc_t *pFlash
                    )
{
	return FLASH_READ(pFlash->Addr);
}


//-------------------------- Erasing MCU flash memory page (  us) -----------------------------
void FlashErasePage	(
                    uint32_t address
                    )
{
    FlashUnlock(address);
	if(address < BANK_2)
	{
		while(FLASH->SR & FLASH_SR_BSY);
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = address;
        FLASH->CR |= FLASH_CR_STRT;
        while(FLASH->SR & FLASH_SR_BSY);
        FLASH->CR &= ~FLASH_CR_PER;
        if(FLASH->SR & FLASH_SR_EOP)
            FLASH->SR |= FLASH_SR_EOP;
    }
    else
    {
		while(FLASH->SR2 & FLASH_SR_BSY);
        FLASH->CR2 |= FLASH_CR_PER;
        FLASH->AR2 = address;
        FLASH->CR2 |= FLASH_CR_STRT;
        while(FLASH->SR2 & FLASH_SR_BSY);
        FLASH->CR2 &= ~FLASH_CR_PER;
        if(FLASH->SR2 & FLASH_SR_EOP)
            FLASH->SR2 |= FLASH_SR_EOP;
    }
    FlashLock(address);
}


//----------------------------  --------------------------------
void FlashWriteBuf  (
                    Flash_struc_t *pFlash,
                    uint16_t *buf,
                    uint32_t address,
                    uint32_t arr_size
                    )
{
    InitFlashStruc(pFlash, buf, address, arr_size);
    FlashUnlock(pFlash->Addr);
	if(pFlash->Addr < BANK_2)
	{
        while(FLASH->SR & FLASH_SR_BSY);
        FLASH->CR |= FLASH_CR_PG;
        while(pFlash->Size)
        {
            *(volatile uint16_t *)(pFlash->Addr) = *(pFlash->Data);
            while(FLASH->SR & FLASH_SR_BSY);
            if(FLASH->SR & FLASH_SR_EOP)
                FLASH->SR |= FLASH_SR_EOP;
            pFlash->Addr += 2;
            pFlash->Data++;
            pFlash->Size--;
        }
        FLASH->CR &= ~FLASH_CR_PG;
    }
    else
    {
        while(FLASH->SR2 & FLASH_SR_BSY);
        FLASH->CR2 |= FLASH_CR_PG;
        while(pFlash->Size)
        {
            *(volatile uint16_t *)(pFlash->Addr) = *(pFlash->Data);
            while(FLASH->SR2 & FLASH_SR_BSY);
            if(FLASH->SR2 & FLASH_SR_EOP)
                FLASH->SR2 |= FLASH_SR_EOP;
            pFlash->Addr += 2;
            pFlash->Data++;
            pFlash->Size--;
        }
        FLASH->CR2 &= ~FLASH_CR_PG;
    }
    FlashLock(pFlash->Addr);
}


#ifdef _FLASH_IRQ_ON_
//-------------------------- Erasing flash memory page (  us) -----------------------------
void FlashEraseStart(
                    uint32_t address
					)
{
    __disable_irq();
    FlashUnlock(address);
	if(address < BANK_2)
	{
        FLASH->CR |= FLASH_CR_PER;
        FLASH->AR = address;
        FLASH->CR |= FLASH_CR_STRT;
    }
    else
    {
        FLASH->CR2 |= FLASH_CR_PER;
        FLASH->AR2 = address;
        FLASH->CR2 |= FLASH_CR_STRT;
    }
    NVIC_EnableIRQ(FLASH_IRQn);
    __enable_irq();
}


//-------------------------- Erasing flash memory page (  us) -----------------------------
void FlashEraseFin  (
                    Flash_struc_t *pFlash
                    )
{
	if(pFlash->Addr < BANK_2)
    {
		FLASH->CR &= ~FLASH_CR_PER;
    }
	else
    {
		FLASH->CR2 &= ~FLASH_CR_PER;
    }
    NVIC_DisableIRQ(FLASH_IRQn);
    FlashLock(pFlash->Addr);
    xSemaphoreGiveFromISR(FlashEraseFinSemph, NULL);
}


//------------------------------------------------------------
void FlashWriteStart(
                    Flash_struc_t *pFlash,
                    uint16_t *buf,
                    uint32_t address,
                    uint32_t arr_size
                    )
{
    InitFlashStruc(pFlash, buf, address, arr_size);
    __disable_irq();
    FlashUnlock(pFlash->Addr);
	if(pFlash->Addr < BANK_2)
    {
        FLASH->CR |= FLASH_CR_PG;
    }
    else
    {
        FLASH->CR2 |= FLASH_CR_PG;
    }
    NVIC_EnableIRQ(FLASH_IRQn);
    *(volatile uint16_t *)(pFlash->Addr) = *(pFlash->Data);
    __enable_irq();
}


//------------------------------------------------------------
void FlashWriteNext (
                    Flash_struc_t *pFlash
                    )
{
    pFlash->Addr += 2;
    pFlash->Data++;
    pFlash->Size--;
    if(pFlash->Size)
    {
        *(volatile uint16_t *)(pFlash->Addr) = *(pFlash->Data);
    }
    else
    {
        if(pFlash->Addr < BANK_2)
        {
            FLASH->CR &= ~FLASH_CR_PG;
        }
        else
        {
            FLASH->CR2 &= ~FLASH_CR_PG;
        }
        NVIC_DisableIRQ(FLASH_IRQn);
        FlashLock(pFlash->Addr);
        xSemaphoreGiveFromISR(FlashWriteDoneSemph, NULL);
    }
}


//------------------------------------------------------------
void FlashProgHandler   (
                        Flash_struc_t *pFlash
                        )
{
    if(pFlash->Addr < BANK_2)
    {
        if(FLASH->CR & FLASH_CR_PER)
        {
            FlashEraseFin(pFlash);
        }
        else if(FLASH->CR & FLASH_CR_PG)
        {
            FlashWriteNext(pFlash);
        }
    }
    else
    {
        if(FLASH->CR2 & FLASH_CR_PER)
        {
            FlashEraseFin(pFlash);
        }
        else if(FLASH->CR2 & FLASH_CR_PG)
        {
            FlashWriteNext(pFlash);
        }
    }
}
#endif



//---------------------------------------------------------------------------------------------
//                                    Privat functions
//---------------------------------------------------------------------------------------------

