;*******************************************************************************
;* 									proc.s
;*******************************************************************************
NVIC_INT_CTRL_CONST EQU     0xe000ed04

					AREA    text, CODE, READONLY
Default_Handler   	PROC
					EXPORT  Default_Handler
					LDR r3, = NVIC_INT_CTRL_CONST
					LDR r2, [r3, #0]
					UXTB r2, r2
Infinite_Loop		B  Infinite_Loop					
					ENDP

	
;					AREA    DEFAULT_HANDLER_DAT, DATA, READONLY, ALIGN=2
;					EXPORT  NVIC_INT_CTRL_CONST


					END
