//*********************************************************************************************
//                                      Gpio_control.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "gpio.h"

//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//------------------------ Input with pullup (pulldown) resistor ------------------------------
//!!!!!!!!!!!!!!!!!!!!!!!!!!! Function is hardware independent !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void SetAsInput	(
				GPIO_TypeDef *port, 
				uint32_t pin, 
				uint32_t updown
				)
{
	if(pin < 8)
	{	
		port->CRL &= (uint32_t)~(0x0fU<<(4*pin));
		port->CRL |= (uint32_t)(0x08U<<(4*pin)); 
	}
	else
	{	
		port->CRH &= (uint32_t)~(0x0fU<<(4*(pin-8)));
		port->CRH |= (uint32_t)(0x08U<<(4*(pin-8)));
	}
//	__disable_irq();
	if(updown) 
		port->ODR |= (uint32_t)(1<<pin);
	else
		port->ODR &= (uint32_t)~(1<<pin);
//	__enable_irq();
}																								


//---------------------------------- Input with Z-state ---------------------------------------
//!!!!!!!!!!!!!!!!!!!!!!!!!!! Function is hardware independent !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void SetAsZInput(
				GPIO_TypeDef *port, 
				uint32_t pin 
				)
{
	if(pin < 8)
	{	
		port->CRL &= (uint32_t)~(0x0fU<<(4*pin));
		port->CRL |= (uint32_t)(0x04U<<(4*pin)); 
	}
	else
	{	
		port->CRH &= (uint32_t)~(0x0fU<<(4*(pin-8)));
		port->CRH |= (uint32_t)(0x04U<<(4*(pin-8)));
	}
}																								


//-------------------------------------- Output settings --------------------------------------
// !!!!!!!!!!!!!!!!!!!!!!!!!! Function is hardware independent !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void SetAsPpOutput	(
					GPIO_TypeDef *port, 
					uint32_t pin, 
					uint32_t init
					)
{
	if(init) 
		port->BSRR |= (uint32_t)(1<<pin);
	else 
		port->BSRR |= (uint32_t)(1<<(pin+16));

	if(pin < 8)
	{	
		port->CRL &= (uint32_t)~(0x0fU<<(4*pin));
		port->CRL |= (uint32_t)(0x03U<<(4*pin)); // 0x03U
	}
	else
	{	
		port->CRH &= (uint32_t)~(0x0fU<<(4*(pin-8)));
		port->CRH |= (uint32_t)(0x03U<<(4*(pin-8))); // 0x03U
	}
}																								


//-------------------------------------- Output settings --------------------------------------
// !!!!!!!!!!!!!!!!!!!!!!!!!! Function is hardware independent !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void SetAsOdOutput	(
					GPIO_TypeDef *port, 
					uint32_t pin, 
					uint32_t init
					)
{
	if(init) 
		port->BSRR |= (uint32_t)(1<<pin);
	else 
		port->BSRR |= (uint32_t)(1<<(pin+16));

	if(pin < 8)
	{	
		port->CRL &= (uint32_t)~(0x0fU<<(4*pin));
		port->CRL |= (uint32_t)(0x06U<<(4*pin));  // 0x07U
	}
	else
	{	
		port->CRH &= (uint32_t)~(0x0fU<<(4*(pin-8)));
		port->CRH |= (uint32_t)(0x06U<<(4*(pin-8)));  // 0x07U
	}
}																								


//---------------------------------- Alt.func. settings ---------------------------------------
// !!!!!!!!!!!!!!!!!!!!!!!!!! Function is hardware independent !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void SetAsAltFuncOutput (
						GPIO_TypeDef *port, 
						uint32_t pin, 
						outp_t pp_od,
						outsp_t speed
						)
{
    uint32_t temp = 0;
	
	// CNF1 = 1
	temp = (uint32_t)(0x01UL<<3);
	
    // CNF0 depends of push-pull or open-drain
    if(pp_od == OPN_DRN) 
		temp |= (uint32_t)(0x01UL<<2);

    // speed
    temp |= (uint32_t)speed;

	if(pin < 8)
	{	
		port->CRL &= (uint32_t)~(0x0fUL<<(4*pin));
		port->CRL |= (uint32_t)(temp<<(4*pin));
	}
	else
	{	
		port->CRH &= (uint32_t)~(0x0fUL<<(4*(pin-8)));
		port->CRH |= (uint32_t)(temp<<(4*(pin-8)));
	}
}																								


//--------------------------------- GPIO deinitialization -------------------------------------
void GpioDeinit(void)
{
//	RCC->APB2RSTR |= (  RCC_APB2RSTR_IOPARST |
//                        RCC_APB2RSTR_IOPBRST | 
//                        RCC_APB2RSTR_IOPCRST |
//                        RCC_APB2RSTR_IOPDRST |
//                        RCC_APB2RSTR_AFIORST  );
	RCC->APB2ENR &= ~(  RCC_APB2ENR_IOPAEN |
                        RCC_APB2ENR_IOPBEN | 
                        RCC_APB2ENR_IOPCEN |
                        RCC_APB2ENR_IOPDEN |
                        RCC_APB2ENR_AFIOEN  );
}


//--------------------------------- Unbounced GPIO input -------------------------------------
void DebounceInput	(
					GPIO_TypeDef *port, 
					uint32_t pin, 
					Unbounced_In_t *inp_struc, 
					uint32_t delay
					)
{
	if(inp_struc->state == 0)
	{
		if(port->IDR & (uint16_t)(1UL<<pin))
		{
			inp_struc->cnt++;
		}
		else
		{
			inp_struc->cnt = 0;
		}
		if(inp_struc->cnt >= delay)
		{
			inp_struc->state = 1;
			inp_struc->cnt = 0;
		}
	}
	else
	{
		if(!(port->IDR & (uint16_t)(1UL<<pin)))
		{
			inp_struc->cnt++;
		}
		else
		{
			inp_struc->cnt = 0;
		}
		if(inp_struc->cnt >= delay)
		{
			inp_struc->state = 0;
			inp_struc->cnt = 0;
		}
	}
}



