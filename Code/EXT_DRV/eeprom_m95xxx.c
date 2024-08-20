//*********************************************************************************************
//                                     Eeprom_M95xxx.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section
//---------------------------------------------------------------------------------------------
#include "eeprom_m95xxx.h"


//---------------------------------------------------------------------------------------------
//                                        Constants
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                     Global variables
//---------------------------------------------------------------------------------------------
extern Eeprom_inst_t                Eeprom_M95xxx;
extern SemaphoreHandle_t            Eep_Mutex;
extern SemaphoreHandle_t            Eep_Cmplt_Semph;

//#ifdef _TICK_COUNT_ON_
//extern uint32_t 		            tmp_cycles_count_arr[DBG_ARR_SIZE], cycles_count_arr_indx, cycles_count_max;
//#endif



//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//------------------------------- SPI initialization (  us) ------------------------------------
void EepromInit(void)
{
	NODE_SELECT();
	Eeprom_M95xxx.Flags_Eeprom = 0;
	Eeprom_M95xxx.Status_Eeprom = 0;
	NODE_RELEASE();
}


// ---------------------------------------------------------------------------------------------
void ReadEeprom	(
                uint32_t eeprom_adress,
                uint32_t data_size,
                uint8_t *ram_buf
                )
{
    Eeprom_M95xxx.Status_Eeprom = 1;
    Eeprom_M95xxx.Addr_Eeprom = eeprom_adress;
    Eeprom_M95xxx.Cnt_Bytes_Eeprom = data_size; 													// number of bytes
    Eeprom_M95xxx.Ram_For_Data_Eeprom = ram_buf;
    CLEAR_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Read_Write_Eeprom);

    NODE_SELECT();
    SPI_TX(M95xxx_RDSR); 																	// read status register
}


// ---------------------------------------------------------------------------------------------
void WriteEeprom(
                uint32_t eeprom_adress,
                uint32_t data_size,
                uint8_t *ram_buf
                )
{
    Eeprom_M95xxx.Status_Eeprom = 1;
    Eeprom_M95xxx.Addr_Eeprom = eeprom_adress;
    Eeprom_M95xxx.Cnt_Bytes_Eeprom = data_size; 													// number of bytes
    Eeprom_M95xxx.Ram_For_Data_Eeprom = ram_buf;
    SET_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Read_Write_Eeprom);

    Eeprom_M95xxx.Page_Eeprom = Eeprom_M95xxx.Addr_Eeprom / EEPROM_PAGE_SIZE; 				// start page in eeprom
    Eeprom_M95xxx.Page_Offset_Eeprom = Eeprom_M95xxx.Addr_Eeprom % EEPROM_PAGE_SIZE; 		// offset in start page

    NODE_SELECT();
    SPI_TX(M95xxx_RDSR); 																	// read status register
}


// ---------------------------------------------------------------------------------------------
void StartEeprom(void)
{
	MySpiInit();

    EepromInit();

    Eep_Mutex = xSemaphoreCreateMutex();
    Eep_Cmplt_Semph = xSemaphoreCreateBinary();
}


////---------------------------------------------------------------------------------------------
//void vEepAccessTask (
//                    void *pvParameters
//                    )
//{
//
//    for(;;)
//	{
//        xSemaphoreTake(Eep_Cmplt_Semph, portMAX_DELAY);
//
//	}
//}



//**********************************************************************************************
//                                   Interrupt procedures
//**********************************************************************************************
//-------------------- SPI transfer handler (executes in SPI interrupt) ------------------------
void ByteMySpiRxHandler(void)
{
	uint8_t m95160_temp = (uint8_t)MYSPI1->DR; 													// read received byte

	if(Eeprom_M95xxx.Status_Eeprom < 7)
	{
        if(Eeprom_M95xxx.Status_Eeprom == 1)
		{
			SPI_TX(M95xxx_NULL); 																// write insignificant byte to read status register
		}
        else if(Eeprom_M95xxx.Status_Eeprom == 2)
		{
            if((m95160_temp & M95xxx_WIP) == 0) 												// check bit "write in progress" in status register
			{
                NODE_RELEASE();
                NODE_SELECT();
                if(!TEST_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Read_Write_Eeprom))
				{
					// if read operation
                    Eeprom_M95xxx.Status_Eeprom++; 												// skip "status == 3" block
                    SPI_TX(M95xxx_READ); 														// "read data" command
                }
				else
				{
					// if write
                    SPI_TX(M95xxx_WREN); 														// set "write enable latch" bit
				}
            }
			else
			{
				// if bit "write in progress" is 1 then check this again
                Eeprom_M95xxx.Status_Eeprom--; 													// correction "status++" operation
                SPI_TX(M95xxx_NULL); 															// check WIP again
            }
        }
		else if(Eeprom_M95xxx.Status_Eeprom == 3)
		{
			// execute this block if write operation
            NODE_RELEASE();
            NODE_SELECT();
            SPI_TX(M95xxx_WRITE); 																// "write data" command
        }
		else if(Eeprom_M95xxx.Status_Eeprom == 4)
		{
            SPI_TX((uint8_t)(Eeprom_M95xxx.Addr_Eeprom >> 8)); 									// write high byte of eeprom adress
		}
        else if(Eeprom_M95xxx.Status_Eeprom == 5)
		{
            if(TEST_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Read_Write_Eeprom)) 						// if write operation
                Eeprom_M95xxx.Status_Eeprom++; 													// skip "status == 6" block
            SPI_TX((uint8_t)Eeprom_M95xxx.Addr_Eeprom); 										// write low byte of eeprom adress
        }
		else if(Eeprom_M95xxx.Status_Eeprom == 6)
		{
            SPI_TX(M95xxx_NULL); 																// only if read operation
		}
        Eeprom_M95xxx.Status_Eeprom++; 															// next step
    }
	else
	{
        if(!TEST_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Read_Write_Eeprom))
		{
			// if read operation
            *Eeprom_M95xxx.Ram_For_Data_Eeprom = m95160_temp; 									// save received byte in RAM;
			Eeprom_M95xxx.Ram_For_Data_Eeprom++;												// increment RAM adress
            if(--Eeprom_M95xxx.Cnt_Bytes_Eeprom != 0)
			{
				// if not all bytes are read
                SPI_TX(M95xxx_NULL); 															// write insignificant byte to read next data byte
            }
			else
			{
                NODE_RELEASE();
                Eeprom_M95xxx.Status_Eeprom = 0;
//              SET_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Read_Ok_Eeprom); 						// set flag
//				STOP_TIMER(TIMER_TIMEOUT_EEPROM);
                xSemaphoreGiveFromISR(Eep_Cmplt_Semph, NULL);
            }
        }
		else
		{
			// if write operation
            if(Eeprom_M95xxx.Cnt_Bytes_Eeprom != 0)
			{
				// if not all bytes are written
                if(Eeprom_M95xxx.Page_Offset_Eeprom == EEPROM_PAGE_SIZE)
				{
					// if pass over a page boundary
                    Eeprom_M95xxx.Page_Offset_Eeprom = 0; 										// reset offset
                    Eeprom_M95xxx.Addr_Eeprom = EEPROM_PAGE_SIZE * (++Eeprom_M95xxx.Page_Eeprom); // calculate adress of next page
                    Eeprom_M95xxx.Status_Eeprom = 1;
                    NODE_RELEASE();
                    NODE_SELECT();
                    SPI_TX(M95xxx_RDSR); 														// at beginning
                }
				else
				{
					// if offset less then page size
                    Eeprom_M95xxx.Page_Offset_Eeprom++; 										// increment offset
                    Eeprom_M95xxx.Cnt_Bytes_Eeprom--;
                    SPI_TX(*Eeprom_M95xxx.Ram_For_Data_Eeprom); 								// write data from RAM to eeprom;
					Eeprom_M95xxx.Ram_For_Data_Eeprom++;										// increment RAM adress
                }
            }
			else
			{
                NODE_RELEASE();
                Eeprom_M95xxx.Status_Eeprom = 0;
//              SET_FLAG(Eeprom_M95xxx.Flags_Eeprom, f_Write_Ok_Eeprom); 						// set flag
//				STOP_TIMER(TIMER_TIMEOUT_EEPROM);
                xSemaphoreGiveFromISR(Eep_Cmplt_Semph, NULL);
            }
        }
    }
}



