//*********************************************************************************************
//                                        Mem_adr.h
//*********************************************************************************************

#ifndef __MEM_ADR__
#define __MEM_ADR__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
//#include "config.h"


//---------------------------------------------------------------------------------------------
//                                     Define section
//---------------------------------------------------------------------------------------------
//-------------------------- Bootloader virtal ROM space addresses ----------------------------
// Boundaries
#define BOOTROM_SPACE_START_ADDR		    (uint32_t)0xffff44
#define BOOTROM_SPACE_END_ADDR		        (uint32_t)0xffffff

// Real RAM space for Bootloader virtal ROM
#define	RAM_BOOTROM_SPACE_RDBUF_SIZE		4
#define	RAM_BOOTROM_SPACE_WRBUF_SIZE		68


#define RAM_BOOTROM_SPACE_RDBUF_OFFSET		0
#define RAM_BOOTROM_SPACE_WRBUF_OFFSET		(RAM_BOOTROM_SPACE_RDBUF_OFFSET + RAM_BOOTROM_SPACE_RDBUF_SIZE)
#define RAM_BOOTROM_SPACE_BUF_SIZE		    (RAM_BOOTROM_SPACE_WRBUF_OFFSET + RAM_BOOTROM_SPACE_WRBUF_SIZE)



//----------------------------------- EEPROM addresses ----------------------------------------
// Boundaries
#define EEPROM_COM_SETTINGS_START_ADDR		(uint32_t)0x00000000
#define EEPROM_COM_SETTINGS_END_ADDR		(uint32_t)(0x10000 - 1)		//(EEPROM_SIZE - 1)

// EEPROM distribution
#define EEPROM_ID_DATA_ADDR					(uint32_t)0x00000000
#define EEPROM_SERBUS_SET_ADDR			    (uint32_t)0x00000030
#define EEPROM_ETHERNET_SET_ADDR			(uint32_t)0x00000070
#define EEPROM_SYNC_SET_ADDR			    (uint32_t)0x00000150
#define EEPROM_INTERVIEW_SET_ADDR			(uint32_t)0x00000180
#define EEPROM_IEC103_REL_TAB_ADDR	        (uint32_t)0x00000200
#define EEPROM_MODBUS_REL_TAB_ADDR	        (uint32_t)0x00002000



// Data ID information
// Version
#define EEPROM_VERSION_SIZE					8                           
#define EEPROM_FRMWR_VERSION_OFFSET			(uint32_t)0x00000000
#define EEPROM_HRDWR_VERSION_OFFSET			(EEPROM_FRMWR_VERSION_OFFSET + EEPROM_VERSION_SIZE)
#define EEPROM_BTLDR_VERSION_OFFSET			(EEPROM_HRDWR_VERSION_OFFSET + EEPROM_VERSION_SIZE)
// Device name & function
//#define EEPROM_DEVNAME_SIZE					8
//#define EEPROM_DEVNAME_OFFSET				(uint32_t)0x00000000
#define EEPROM_DEVFUNC_OFFSET				(EEPROM_BTLDR_VERSION_OFFSET + EEPROM_VERSION_SIZE) // 171 - AB3, 172 - B3, 173 - BC3 



// Serial_Bus0..1 settings
#define EEPROM_SERBUS0_ADDR				    EEPROM_SERBUS_SET_ADDR
#define EEPROM_SERBUS1_ADDR				    (EEPROM_SERBUS_SET_ADDR + 0x20)

#define EEPROM_SERBUS_CONTROL_OFFSET	    (uint32_t)0x00000000								// 0- Off, 1- On
#define EEPROM_SERBUS_PROTOCOL_OFFSET	    (uint32_t)0x00000001								// 0- Modbus, 1- IEC60870-103, 2- DNP3, 3- 
#define EEPROM_SERBUS_BAUDRATE_OFFSET	    (uint32_t)0x00000002								// 0- 4800, 1- 9600, 2- 19200, 3- 38400, 4- 57600, 5- 115200
#define EEPROM_SERBUS_PARITY_OFFSET		    (uint32_t)0x00000003								// 0- No, 1- Even, 2- Odd
#define EEPROM_SERBUS_STOPBIT_OFFSET	    (uint32_t)0x00000004								// 0- 1 bit, 1- 2 bit
#define EEPROM_SERBUS_SATIONADDR_OFFSET	    (uint32_t)0x00000005								// 1..31 (Modbus), 1..255 (IEC60870-103)
#define EEPROM_SERBUS_ADDPARAM_OFFSET	    (uint32_t)0x00000006								// 0 - single char message disable, 1 - single char message enable (IEC60870-103)
#define EEPROM_SERBUS_SETTINGS_SIZE		    (EEPROM_SERBUS_ADDPARAM_OFFSET + 1)



// Ethernet0..1 settings



// Time Synchronization  settings
#define EEPROM_SYNC_SNTP_1_PRIOR_OFFSET	        (uint32_t)0x00000000								// 0..31
#define EEPROM_SYNC_SNTP_2_PRIOR_OFFSET	        (uint32_t)0x00000001								// 0..31
#define EEPROM_SYNC_SNTP_3_PRIOR_OFFSET	        (uint32_t)0x00000002								// 0..31
#define EEPROM_SYNC_IEC103_1_PRIOR_OFFSET       (uint32_t)0x00000003								// 0..31
#define EEPROM_SYNC_IEC103_2_PRIOR_OFFSET       (uint32_t)0x00000004								// 0..31
#define EEPROM_SYNC_IEC104_1_PRIOR_OFFSET       (uint32_t)0x00000005								// 0..31
#define EEPROM_SYNC_IEC104_2_PRIOR_OFFSET       (uint32_t)0x00000006								// 0..31
#define EEPROM_SYNC_IEC104_3_PRIOR_OFFSET       (uint32_t)0x00000007								// 0..31
#define EEPROM_SYNC_IEC104_4_PRIOR_OFFSET       (uint32_t)0x00000008								// 0..31
#define EEPROM_SYNC_PPS_PRIOR_OFFSET            (uint32_t)0x00000009								// 0..31
#define EEPROM_SYNC_PPS_SERBUS0_PRIOR_OFFSET    (uint32_t)0x0000000A								// 0..31
#define EEPROM_SYNC_PPS_SERBUS1_PRIOR_OFFSET    (uint32_t)0x0000000B								// 0..31
#define EEPROM_SYNC_PTP_PRIOR_OFFSET            (uint32_t)0x0000000C								// 0..31
#define EEPROM_SYNC_IRIGB_PRIOR_OFFSET          (uint32_t)0x0000000D								// 0..31

#define EEPROM_SYNC_SRC_ENBL_OFFSET	            (uint32_t)0x00000014								// 4-bytes bitfield: 0- Disable, 1- Enable
#define EEPROM_SYNC_PERIOD_OFFSET	            (uint32_t)0x00000018								// 1..(23*3600)s
#define EEPROM_SYNC_TIMEZONE_OFFSET             (uint32_t)0x0000001C								// -12..11
#define EEPROM_SYNC_SUMMERTIME_AUTO_OFFSET      (uint32_t)0x0000001D								// 0- Disable, 1- Enable
#define EEPROM_SYNC_SNTP_IP1_OFFSET	            (uint32_t)0x0000001E								// x.x.x.x
#define EEPROM_SYNC_SNTP_IP2_OFFSET	            (uint32_t)0x00000022								// x.x.x.x
#define EEPROM_SYNC_SNTP_IP3_OFFSET	            (uint32_t)0x00000026								// x.x.x.x
#define EEPROM_SYNC_WRTAG_OFFSET	            (uint32_t)0x0000002A								// data written tag
#define EEPROM_SYNC_SET_CS_OFFSET	            (uint32_t)0x0000002B								// check sum
#define EEPROM_SYNC_SET_SIZE		            (EEPROM_SYNC_SET_CS_OFFSET + 1)




// General & sporadic interview settings
#define EEPROM_GI_SET_ADDR			            EEPROM_INTERVIEW_SET_ADDR
#define EEPROM_SE_SET_ADDR			            (EEPROM_INTERVIEW_SET_ADDR + 0x20)

#define EEPROM_INTERVIEW_DI_1_8_OFFSET			(uint32_t)0x00000000
#define EEPROM_INTERVIEW_DI_9_16_OFFSET		    (uint32_t)0x00000001
#define EEPROM_INTERVIEW_DI_17_24_OFFSET		(uint32_t)0x00000002
#define EEPROM_INTERVIEW_DI_25_32_OFFSET		(uint32_t)0x00000003
#define EEPROM_INTERVIEW_DI_33_40_OFFSET		(uint32_t)0x00000004
#define EEPROM_INTERVIEW_KL_1_8_OFFSET			(uint32_t)0x00000005
#define EEPROM_INTERVIEW_KL_9_16_OFFSET		    (uint32_t)0x00000006
#define EEPROM_INTERVIEW_KL_17_24_OFFSET		(uint32_t)0x00000007
#define EEPROM_INTERVIEW_KL_25_32_OFFSET		(uint32_t)0x00000008
#define EEPROM_INTERVIEW_KL_33_40_OFFSET		(uint32_t)0x00000009
#define EEPROM_INTERVIEW_VDI_1_8_OFFSET			(uint32_t)0x0000000A
#define EEPROM_INTERVIEW_VDI_9_16_OFFSET		(uint32_t)0x0000000B
#define EEPROM_INTERVIEW_VDI_17_24_OFFSET		(uint32_t)0x0000000C
#define EEPROM_INTERVIEW_VDI_25_32_OFFSET		(uint32_t)0x0000000D
#define EEPROM_INTERVIEW_VDI_33_40_OFFSET		(uint32_t)0x0000000E
#define EEPROM_INTERVIEW_DATA_WRTAG_OFFSET	    (uint32_t)0x0000000F								// data written tag
#define EEPROM_INTERVIEW_SET_CS_OFFSET	        (uint32_t)0x00000010								// check sum
#define EEPROM_INTERVIEW_SET_SIZE		        (EEPROM_INTERVIEW_SET_CS_OFFSET + 1)
#define EEPROM_INTERVIEW_BYTES_AMOUNT           (EEPROM_INTERVIEW_VDI_33_40_OFFSET + 1)

#define EEPROM_INTERVIEW_DI_OFFSET			    EEPROM_INTERVIEW_DI_1_8_OFFSET
#define EEPROM_INTERVIEW_KL_OFFSET			    EEPROM_INTERVIEW_KL_1_8_OFFSET
#define EEPROM_INTERVIEW_VDI_OFFSET			    EEPROM_INTERVIEW_VDI_1_8_OFFSET

#define EEPROM_INTERVIEW_DI_BYTE_SIZE 			(EEPROM_INTERVIEW_KL_1_8_OFFSET - EEPROM_INTERVIEW_DI_1_8_OFFSET)
#define EEPROM_INTERVIEW_KL_BYTE_SIZE 			(EEPROM_INTERVIEW_VDI_1_8_OFFSET - EEPROM_INTERVIEW_KL_1_8_OFFSET)
#define EEPROM_INTERVIEW_VDI_BYTE_SIZE 			(EEPROM_INTERVIEW_DATA_WRTAG_OFFSET - EEPROM_INTERVIEW_VDI_1_8_OFFSET)



// Table of address relationship for IEC60870-103(104)
#define EEPROM_IEC103_REL_TAB_SIZE	            (uint32_t)0x00000A00




// Table of address relationship for Modbus
#define REL_ADDR_TABLE_LINE_LEN			        (uint32_t)8
#define EEPROM_MODBUS_REL_TAB_SIZE			    (uint32_t)0x00008000



//------------------------------------- SRAM addresses ----------------------------------------
// for Data_Buf_Serial_Bus0..1[]
#define	RAM_SERBUS_RXBUF_SIZE			    (256 + 16)
#define	RAM_SERBUS_TXBUF_SIZE			    (256 + 16)


#define RAM_SERBUS_RXBUF_OFFSET			    0
#define RAM_SERBUS_TXBUF_OFFSET			    (RAM_SERBUS_RXBUF_OFFSET + RAM_SERBUS_RXBUF_SIZE)
#define RAM_SERBUS_BUF_SIZE		            (RAM_SERBUS_TXBUF_OFFSET + RAM_SERBUS_TXBUF_SIZE)


// for Iec103_Data_Buf[]
#define	RAM_IEC103_RQST_DATA_BUF_SIZE		256
#define	RAM_IEC103_RESP_DATA_BUF_SIZE		256

#define	RAM_IEC103_RQST_DATA_BUF_OFFSET		(0)
#define	RAM_IEC103_RESP_DATA_BUF_OFFSET		(RAM_IEC103_RQST_DATA_BUF_OFFSET + RAM_IEC103_RQST_DATA_BUF_SIZE)
#define	RAM_IEC103_BUF_SIZE                 (RAM_IEC103_RESP_DATA_BUF_OFFSET + RAM_IEC103_RESP_DATA_BUF_SIZE)

//#define	RAM_MSG_QUEUE_MAX_SIZE		        1024                                                    // maximum quantity messages
//#define	RAM_MSG_SIZE			            16  //sizeof(Iec103_ASDU_t)                                   // bytes in message
//#define	RAM_MSG_QUEUE_BUF_SIZE		        (RAM_MSG_QUEUE_MAX_SIZE * RAM_MSG_SIZE)                 // bytes in message queue buffer


// for Modbus_Data_Buf[]
#define	RAM_MODBUS_RQST_DATA_BUF_SIZE		256
#define	RAM_MODBUS_RESP_DATA_BUF_SIZE		256

#define	RAM_MODBUS_RQST_DATA_BUF_OFFSET		(0)
#define	RAM_MODBUS_RESP_DATA_BUF_OFFSET		(RAM_MODBUS_RQST_DATA_BUF_OFFSET + RAM_MODBUS_RQST_DATA_BUF_SIZE)
#define	RAM_MODBUS_BUF_SIZE                 (RAM_MODBUS_RESP_DATA_BUF_OFFSET + RAM_MODBUS_RESP_DATA_BUF_SIZE)


// for Parbus_Data_Buf[]
#define	RAM_PARBUS_RQST_DATA_BUF_SIZE		256
#define	RAM_PARBUS_RESP_DATA_BUF_SIZE		260

#define	RAM_PARBUS_RQST_DATA_BUF_OFFSET		(0)
#define	RAM_PARBUS_RESP_DATA_BUF_OFFSET		(RAM_PARBUS_RQST_DATA_BUF_OFFSET + RAM_PARBUS_RQST_DATA_BUF_SIZE)
#define	RAM_PARBUS_BUF_SIZE                 (RAM_PARBUS_RESP_DATA_BUF_OFFSET + RAM_PARBUS_RESP_DATA_BUF_SIZE)


// for Ied_Data_Buf[]
#define	RAM_ANALOG_DAT_SIZE		            256 // F33
#define	RAM_DATA_FOR_CPU_SIZE		        256 // F44
#define	RAM_DATA_FROM_CPU_SIZE	            256 // F55
#define	RAM_DISCRET_DAT_SIZE	            256 // F77
#define	RAM_VD_DAT_SIZE			            256 // F88
#define	RAM_DIKL_DAT_SIZE			        16  // F11
#define	RAM_VIRT_DAT_SIZE			        6   // F22

#define	RAM_ANALOG_DAT_OFFSET		        (0)
#define	RAM_DATA_FOR_CPU_OFFSET		        (RAM_ANALOG_DAT_OFFSET + RAM_ANALOG_DAT_SIZE)
#define	RAM_DATA_FROM_CPU_OFFSET	        (RAM_DATA_FOR_CPU_OFFSET + RAM_DATA_FOR_CPU_SIZE)
#define	RAM_DISCRET_DAT_OFFSET		        (RAM_DATA_FROM_CPU_OFFSET + RAM_DATA_FROM_CPU_SIZE)
#define	RAM_VD_DAT_OFFSET		            (RAM_DISCRET_DAT_OFFSET + RAM_DISCRET_DAT_SIZE)
#define	RAM_DIKL_DAT_OFFSET		            (RAM_VD_DAT_OFFSET + RAM_VD_DAT_SIZE)
#define	RAM_VIRT_DAT_OFFSET		            (RAM_DIKL_DAT_OFFSET + RAM_DIKL_DAT_SIZE)
#define	RAM_DATA_FOR_CPU_SIZE_OFFSET		(RAM_VIRT_DAT_OFFSET + RAM_VIRT_DAT_SIZE)
#define	RAM_DATA_FROM_CPU_SIZE_OFFSET		(RAM_DATA_FOR_CPU_SIZE_OFFSET + 2)
#define	RAM_IED_BUF_SIZE		            (RAM_DATA_FROM_CPU_SIZE_OFFSET + 2)
#define	RAM_DI_DAT_OFFSET		            (RAM_DIKL_DAT_OFFSET)
#define	RAM_KL_DAT_OFFSET		            (RAM_DIKL_DAT_OFFSET + 6)


// Table of address relationship
#define RAM_REL_ADDR_TABLE_SIZE				(EEPROM_REL_ADDR_TABLE_SIZE)




//----------------------------------- FLASH addresses -----------------------------------------
#define FLASH_START_ADDR	   				(uint32_t)0x08000000            								// Address start Flash memory MC
#define FLASH_PAGE_SIZE						(uint32_t)0x00000800											// 
#define FLASH_PAGE_VOL						512																// 

#define FLASH_BLDR_ADDR  					(uint32_t)(FLASH_START_ADDR)             						// 
#define FLASH_APP_ADDR  					(uint32_t)(FLASH_BLDR_ADDR + 256 * FLASH_PAGE_SIZE)  				// 


// page with firmware update flags (511)
#define FLASH_FW_UPD_FLAGS_ADDR   			(uint32_t)(FLASH_START_ADDR + (FLASH_PAGE_VOL - 1) * FLASH_PAGE_SIZE)	// 

#define FLASH_FW_NEED_UPD_FLAG_OFFSET		(uint32_t)(0)													// (uint16_t)
#define FLASH_FW_ILLEGAL_FLAG_OFFSET		(uint32_t)(0x02)												// (uint16_t)
#define FLASH_BLDR_VERS_OFFSET  			(uint32_t)(0x10)     											// (uint16_t)

#define FLASH_FW_NEED_UPD_FLAG_ADDR			(uint32_t)(FLASH_FW_UPD_FLAGS_ADDR + FLASH_FW_NEED_UPD_FLAG_OFFSET)	// (uint16_t)
#define FLASH_FW_ILLEGAL_FLAG_ADDR			(uint32_t)(FLASH_FW_UPD_FLAGS_ADDR + FLASH_FW_ILLEGAL_FLAG_OFFSET)	// (uint16_t)
#define FLASH_BLDR_VERS_ADDR  				(uint32_t)(FLASH_FW_UPD_FLAGS_ADDR + FLASH_BLDR_VERS_OFFSET)    // (uint16_t)

#define FLASH_SAVED_BYTES  					(uint32_t)(FLASH_BLDR_VERS_OFFSET + sizeof(BL_VERS) - 1)
#define FLASH_SAVED_WORDS  					(uint32_t)(FLASH_SAVED_BYTES / 2 + FLASH_SAVED_BYTES % 2)
	
#define MAX_FRMW_SIZE						(uint32_t)(FLASH_FW_UPD_FLAGS_ADDR - FLASH_APP_ADDR)			// 
#define FRMW_BLOCK_SIZE						(64)





#endif
