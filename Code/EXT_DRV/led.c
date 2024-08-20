//*********************************************************************************************
//                                          Led.c
//*********************************************************************************************
//
//---------------------------------------------------------------------------------------------
//                                     Include section 
//---------------------------------------------------------------------------------------------
#include "led.h"


//---------------------------------------------------------------------------------------------
//                                        Functions
//---------------------------------------------------------------------------------------------
//-------------------------------------- LED control ------------------------------------------
void LedCtrl(
			uint8_t state
			)
{
	static uint16_t temp2;
	
	switch(state)
	{
		case LED_ON_MODE:
			LED_ON();
			break;
		
		case LED_BLINK_MODE:
			temp2++;
			if(temp2>=(SYSTICK_IRQ_FREQ/(2*LED_BLINK_FREQ_HZ)))
			{
				if(TEST_PIN(LED_Port, LED_Pin)) 
				{
					LED_OFF();
				}
				else 
				{
					LED_ON();
				}
				temp2 = 0;
			}
			break;
			
		case LED_FLASH_MODE:
			temp2++;
			if(temp2<(SYSTICK_IRQ_FREQ/(LED_FLASH_FREQ_HZ*LED_FLASH_DUTY_RAT))) 
			{
				LED_ON();
			}
			else
			{
				LED_OFF();
				if(temp2>=SYSTICK_IRQ_FREQ/LED_FLASH_FREQ_HZ) 
					temp2 = 0;
			}
			break;
			
		case LED_BLINK_50HZ_MODE:
			temp2++;
			if(temp2>=(SYSTICK_IRQ_FREQ/(2*50)))
			{
				if(TEST_PIN(LED_Port, LED_Pin)) 
				{
					LED_OFF();
				}
				else 
				{
					LED_ON();
				}
				temp2 = 0;
			}
			break;
			
		case LED_BLINK_F_SYSTICK_MODE:
			if(TEST_PIN(LED_Port, LED_Pin)) 
			{
				LED_OFF();
			}
			else 
			{
				LED_ON();
			}			
			break;
			
		default:
			LED_OFF();
			break;
	}
}


