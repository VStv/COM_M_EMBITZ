//*********************************************************************************************
//                                      Eeprom_M95xxx.h
//*********************************************************************************************
#ifndef __EEPROM_M95XXX__
#define __EEPROM_M95XXX__

//---------------------------------------------------------------------------------------------
//                                      Include section
//---------------------------------------------------------------------------------------------
#include "config.h"
//#include "dwt.h"

#include "spi.h"




//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------
#define M95160_SIZE 						(0x00000800)
#define M95512_SIZE 						(0x00010000)

#define M95160_PAGE_SIZE 					(32)
#define M95512_PAGE_SIZE 					(128)

#define M95xxx_WREN 						(0x06) 											// write enable
#define M95xxx_WRDI 						(0x04) 											// write disable
#define M95xxx_RDSR 						(0x05) 											// read status register
#define M95xxx_WRSR 						(0x01) 											// write status register
#define M95xxx_READ 						(0x03) 											// read from memory array
#define M95xxx_WRITE 						(0x02) 											// write to memory array
#define M95xxx_NULL 						(0x00)

#define M95xxx_WIP 							(1 << 0) 										// write in progress bit
#define M95xxx_WEL 							(1 << 1) 										// write enable latch bit
#define M95xxx_BP0 							(1 << 2) 										// block protect bit BP0
#define M95xxx_BP1 							(1 << 3) 										// block protect bit BP1
#define M95xxx_SRWD 						(1 << 7) 										// status register write protect bit

#define EEPROM_SIZE 						M95512_SIZE
#define EEPROM_PAGE_SIZE 					M95512_PAGE_SIZE


//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------------
//                                      Typedef section
//---------------------------------------------------------------------------------------------
typedef enum {
	f_Busy_Timeout_Eeprom = 0,
	f_Error_Other,
} Flags_Error_Eeprom_t;


typedef enum {
    f_Read_Write_Eeprom = 0,					    // 0- read, 1- write
} Flags_Eeprom_t;


typedef struct {
	uint8_t				*Ram_For_Data_Eeprom;	    // pointer for data to write/read
	uint32_t			Addr_Eeprom;
	uint32_t			Page_Eeprom;
	uint32_t			Page_Offset_Eeprom;
	uint32_t			Size_Bytes_Eeprom;
	uint32_t			Cnt_Bytes_Eeprom;
	uint32_t		    Flags_Eeprom;
 	uint32_t		    Status_Eeprom;

	uint32_t 			*errors;
	uint32_t	        Flags_Error_Eeprom;
} Eeprom_inst_t;





//----------------------------------------------------------------------------------------------
//                                   Function's prototypes
//----------------------------------------------------------------------------------------------
portBASE_TYPE CreateEepAccessTask(void);

void EepromInit(void);
void ReadEeprom(uint32_t, uint32_t, uint8_t *);
void WriteEeprom(uint32_t, uint32_t, uint8_t *);
void ByteMySpiRxHandler(void);

void StartEeprom(void);

#endif
