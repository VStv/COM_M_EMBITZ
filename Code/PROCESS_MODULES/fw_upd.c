//*********************************************************************************************
//                                    	   Fw_upd.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "fw_upd.h"

#ifdef _BOOTLOADER_ON_

//---------------------------------------------------------------------------------------------
//                                    Tables of Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------

extern Flash_struc_t                Flash_struc;
extern uint16_t                     Flash_Buf[FLASH_SAVED_WORDS];

#ifdef _FLASH_IRQ_ON_
extern SemaphoreHandle_t            FlashEraseFinSemph;
extern SemaphoreHandle_t            FlashWriteDoneSemph;
#endif


extern uint8_t                      Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_BUF_SIZE];
extern SemaphoreHandle_t            Fw_Upd_Rqst_Semph;
extern SemaphoreHandle_t            Fw_Upd_Confrm_Semph;

extern TaskHandle_t                 xFwUpdCtrlHandle;


#ifdef _DEBUG_ON_
extern uint32_t                     task_enter_cnt[TASK_NUM_SIZE];
extern uint32_t                     task_stage[TASK_NUM_SIZE];
#endif


//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------
static void FwUpdateStructInit  (
                                Fw_update_struc_t *p_fw_update_struc,
                                uint16_t *page_buf
                                )
{
    if(p_fw_update_struc != NULL && page_buf != NULL)
    {
        p_fw_update_struc->Page_Buffer = page_buf;
        Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 0] = 1;
    }
    else
    {
        Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 0] = 0;
    }
    Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 1] = 0;
    Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 2] = 0;
    Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 3] = 0;
}


//---------------------------------------------------------------------------------------------
static void GoToApplication (
                            uint32_t addr
                            )
{
    uint32_t appJumpAddress;
    void (*GoToApp)(void);

	__disable_irq();
    SCB->VTOR = addr;
    appJumpAddress = *((volatile uint32_t*)(addr + 4));
    GoToApp = (void (*)(void))appJumpAddress;

	//stack pointer (to RAM) for USER app in this address
    __set_MSP(*(volatile uint32_t*) addr);
    GoToApp();
}


//---------------------------------------------------------------------------------------------
void BldrEnterKey   (
                    rtc_strct_t *rtc_fact
                    )
{
    uint16_t temp = rtc_fact->ms_counter;

	Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 2] = (uint8_t)(temp >> 8);
    Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 3] = (uint8_t)temp;
    Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 1] =    Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 2]
                                                                ^
                                                                Boot_ROM_Space_Buf[RAM_BOOTROM_SPACE_RDBUF_OFFSET + 3];
}


//---------------------------- Peripheral deinitialization (  us) -------------------------------
static void DeinitAll(void)
{
    // disable interrupts & turnoff peripheral
	__disable_irq();
	TimerDeinit(MYTIMER1);
    TimerDeinit(MYTIMER2);
    TimerDeinit(TIM1);
    MySpiDeinit();
    MyUartDeinit(MYUART1);
    MyUartDeinit(MYUART2);
    GpioDeinit();
    NVIC_DisableIRQ(EXTI9_5_IRQn);
    NVIC_DisableIRQ(EXTI2_IRQn);

#ifdef _FLASH_IRQ_ON_
    NVIC_DisableIRQ(FLASH_IRQn);
#endif
}


#ifdef _FLASH_IRQ_ON_
//---------------------------------------------------------------------------------------------
static void GoToBootloader(void)
{
    // FLASH_FW_NEED_UPD_FLAG_ADDR <- FW_UPD_NEED
    if(FLASH_READ(FLASH_FW_NEED_UPD_FLAG_ADDR) != FW_UPD_NEED)
    {
        // read update flags & version flash region
        FlashReadBuf(   &Flash_struc,
                        Flash_Buf,
                        FLASH_FW_UPD_FLAGS_ADDR,
                        FLASH_SAVED_WORDS   );

        FlashEraseStart(FLASH_FW_UPD_FLAGS_ADDR);
        xSemaphoreTake(FlashEraseFinSemph, portMAX_DELAY);

        // start write flag
        Flash_Buf[FLASH_FW_NEED_UPD_FLAG_OFFSET/2] = FW_UPD_NEED;
        FlashWriteStart(&Flash_struc,
                        Flash_Buf,
                        FLASH_FW_UPD_FLAGS_ADDR,
                        FLASH_SAVED_WORDS   );
        xSemaphoreTake(FlashWriteDoneSemph, portMAX_DELAY);
    }
    DeinitAll();
    GoToApplication(FLASH_BLDR_ADDR);
}


//---------------------------------------------------------------------------------------------
static void RstFwUpdateFlags(void)
{
    if((FLASH_READ(FLASH_FW_NEED_UPD_FLAG_ADDR) != FW_UPD_NOT_NEED) || (FLASH_READ(FLASH_FW_ILLEGAL_FLAG_ADDR) != FW_OK))
    {
        // read update flags & version flash region
        FlashReadBuf(   &Flash_struc,
                        Flash_Buf,
                        FLASH_FW_UPD_FLAGS_ADDR,
                        FLASH_SAVED_WORDS   );

        FlashEraseStart(FLASH_FW_UPD_FLAGS_ADDR);
        xSemaphoreTake(FlashEraseFinSemph, portMAX_DELAY);

        // start write flag
        Flash_Buf[FLASH_FW_NEED_UPD_FLAG_OFFSET/2] = FW_UPD_NOT_NEED;
        Flash_Buf[FLASH_FW_ILLEGAL_FLAG_OFFSET/2] = FW_OK;
        FlashWriteStart(&Flash_struc,
                        Flash_Buf,
                        FLASH_FW_UPD_FLAGS_ADDR,
                        FLASH_SAVED_WORDS   );
        xSemaphoreTake(FlashWriteDoneSemph, portMAX_DELAY);
    }
}

#else
//---------------------------------------------------------------------------------------------
static void GoToBootloader(void)
{
            if(FLASH_READ(FLASH_FW_NEED_UPD_FLAG_ADDR) != FW_UPD_NEED)
            {
                // read update flags & version flash region
                FlashReadBuf(   &Flash_struc,
                                Flash_Buf,
                                FLASH_FW_UPD_FLAGS_ADDR,
                                FLASH_SAVED_WORDS);

                // update flag
                Flash_Buf[FLASH_FW_NEED_UPD_FLAG_OFFSET/2] = FW_UPD_NEED;

                // stop sheduler
                vTaskSuspendAll();
                __disable_irq();

                // erase page
                FlashErasePage( FLASH_FW_UPD_FLAGS_ADDR );

                // write page
                FlashWriteBuf(  &Flash_struc,
                                Flash_Buf,
                                FLASH_FW_UPD_FLAGS_ADDR,
                                FLASH_SAVED_WORDS   );

                __enable_irq();
                xTaskResumeAll();
            }
            DeinitAll();
            GoToApplication(FLASH_BLDR_ADDR);
}


//---------------------------------------------------------------------------------------------
static void RstFwUpdateFlags(void)
{
    if((FLASH_READ(FLASH_FW_NEED_UPD_FLAG_ADDR) != FW_UPD_NOT_NEED) || (FLASH_READ(FLASH_FW_ILLEGAL_FLAG_ADDR) != FW_OK))
    {
        // read update flags & version flash region
        FlashReadBuf(   &Flash_struc,
                        Flash_Buf,
                        FLASH_FW_UPD_FLAGS_ADDR,
                        FLASH_SAVED_WORDS);

        // update flags
        Flash_Buf[FLASH_FW_NEED_UPD_FLAG_OFFSET/2] = FW_UPD_NOT_NEED;
        Flash_Buf[FLASH_FW_ILLEGAL_FLAG_OFFSET/2] = FW_OK;

        // stop sheduler
        vTaskSuspendAll();
        __disable_irq();

        // erase page
        FlashErasePage( FLASH_FW_UPD_FLAGS_ADDR );

        // write page
        FlashWriteBuf(  &Flash_struc,
                        Flash_Buf,
                        FLASH_FW_UPD_FLAGS_ADDR,
                        FLASH_SAVED_WORDS   );

        __enable_irq();
        xTaskResumeAll();
    }
}
#endif


//---------------- Data (request) from client (Iec103) forwards to server (Modbus) ---------------
static void vFwUpdCtrlTask  (
                            void *pvParameters
                            )
{
    // reset update
#ifdef _DEBUG_ON_
    task_stage[FW_UPD_TASK_NUM] = 3;
#endif

    for(;;)
	{
#ifdef _DEBUG_ON_
        task_stage[FW_UPD_TASK_NUM] = 0;
        ++task_enter_cnt[FW_UPD_TASK_NUM];
#endif
        RstFwUpdateFlags();

        // firmware update request processing
        xSemaphoreTake(Fw_Upd_Rqst_Semph, portMAX_DELAY);
#ifdef _DEBUG_ON_
        task_stage[FW_UPD_TASK_NUM] = 1;
#endif

        if(xSemaphoreTake(Fw_Upd_Confrm_Semph, KEY_RESP_WAIT_TIMEOUT) == pdPASS)
        {
            // firmware update request confirmed
#ifdef _DEBUG_ON_
            task_stage[FW_UPD_TASK_NUM] = 2;
#endif
            GoToBootloader();
        }
    }
}


//---------------------------------------------------------------------------------------------
BaseType_t CreateFwUpdateTask(void)
{
    FlashInit();

    Fw_Upd_Rqst_Semph = xSemaphoreCreateBinary();
    Fw_Upd_Confrm_Semph = xSemaphoreCreateBinary();

    FwUpdateStructInit(NULL, NULL);

	xTaskCreate(vFwUpdCtrlTask,
				(char *)"FwUpdCtrl",
				configMINIMAL_STACK_SIZE,
				NULL,
				(tskIDLE_PRIORITY + 1),
				&xFwUpdCtrlHandle	);

    return pdTRUE;
}


#endif




