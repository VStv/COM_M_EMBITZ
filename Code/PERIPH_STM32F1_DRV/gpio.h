//*********************************************************************************************
//                                          Gpio.h
//*********************************************************************************************

#ifndef __GPIO__
#define __GPIO__

//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "config.h"


//---------------------------------------------------------------------------------------------
//                                     Typedef section
//---------------------------------------------------------------------------------------------
typedef enum {
	PSH_PULL = 0,
	OPN_DRN,
} outp_t;


typedef enum {
	SLOW = 0x02UL,
	MEDI = 0x01UL,
    FAST = 0x03UL,
} outsp_t;


typedef struct {
	uint32_t state;
	uint32_t cnt;
} Unbounced_In_t;



//---------------------------------------------------------------------------------------------
//                                      Define section
//---------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------
//                                           Macros
//---------------------------------------------------------------------------------------------
//Output control
#define SET_PIN(port, pin)	    port->BSRR |= (uint32_t)(1UL<<pin)
#define RESET_PIN(port, pin)    port->BSRR |= (uint32_t)(1UL<<(pin+16))

//Input polling
#define TEST_PIN(port, pin)		(port->IDR & (uint16_t)(1UL<<pin))

//Pullup - float
#define PIN_PULLUP(port, pin)	(port->ODR | (uint16_t)(1UL<<pin))
#define PIN_FLOAT(port, pin)	(port->ODR & ~((uint16_t)(1UL<<pin)))


//----------------------------------------------------------------------------------------------
//                                    Function's prototypes
//----------------------------------------------------------------------------------------------
void SetAsInput(GPIO_TypeDef *, uint32_t, uint32_t);
void SetAsZInput(GPIO_TypeDef *, uint32_t);
void SetAsPpOutput(GPIO_TypeDef *, uint32_t, uint32_t);
void SetAsOdOutput(GPIO_TypeDef *, uint32_t, uint32_t);
void SetAsAltFuncOutput(GPIO_TypeDef *, uint32_t, outp_t, outsp_t);

void GpioDeinit(void);

void DebounceInput(GPIO_TypeDef *, uint32_t, Unbounced_In_t *, uint32_t);

#endif
