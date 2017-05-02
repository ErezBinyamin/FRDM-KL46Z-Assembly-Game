            TTL Program Title for Listing Header Goes Here
;****************************************************************
;The Ulimate Assembler: Text based study game
;Displays question to user, and reads user input. Conditionaly awards points or removes "lives"
;Name:  Erez Binyamin
;Date:  9/26/2016
;Class:  CMPE-250
;Section:  3, Thursday, 2 PM
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;April 3, 2015
;****************************************************************
;Assembler directives
			THUMB
			OPT		64				;Turn on listening macros
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;****************************************************************
; EQUates for the LEDS
;Port D
PTD5_MUX_GPIO  EQU  (1 << PORT_PCR_MUX_SHIFT)
SET_PTD5_GPIO  EQU  (PORT_PCR_ISF_MASK :OR: \
		    PTD5_MUX_GPIO)
;Port E
PTE29_MUX_GPIO  EQU  (1 << PORT_PCR_MUX_SHIFT)
SET_PTE29_GPIO  EQU  (PORT_PCR_ISF_MASK :OR: \
		     PTE29_MUX_GPIO)
		     
POS_RED         EQU  29
POS_GREEN       EQU  5

LED_RED_MASK    EQU  (1 << POS_RED)
LED_GREEN_MASK  EQU  (1 << POS_GREEN)

LED_PORTD_MASK  EQU  LED_GREEN MASK
LED_PORTE_MASK  EQU  LED_RED_MASK  
;****************************************************************
;EQUates continue below
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL46 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
MainLoop

;>>>>>   end main program code <<<<<
;Stay here
            B       .
;>>>>> begin subroutine code <<<<<
;****************************************************************
LED_IRQ
	PUSH	{R0 - R3, LR}
;enable clock for PORT D and E modules
	LDR     R1,=SIM_SCGC5
	LDR     R2,=(SIM_SCGC5_PORTD_MASK :OR: \SIM_SCGC5_PORTE_MASK)
	LDR     R3,[R1,#0]
	ORRS    R3,R3,R2
	STR     R3,[R1,#0]
	
;Select PORT E Pin 29 for GPIO to red LED
	LDR     R0,=PORTE_BASE
	LDR     R1,=SET_PTE29_GPIO
	STR     R1,[R0,#PORTE_PCR29_OFFSET]
	
;Select PORT D Pin 5 for GPIO to green LED
	LDR     R0,=PORTD_BASE
	LDR     R1,=SET_PTD5_GPIO
	STR     R1,[R0,#PORTD_PCR5_OFFSET]
	
	LDR  	R1,=FGPIOD_BASE
	LDR  	R2,=LED_PORTD_MASK
	STR  	R2,[R1,#GPIO_PDDR_OFFSET]
	LDR  	R1,=FGPIOE_BASE
	LDR  	R2,=LED_PORTE_MASK
	STR  	R2,[R1,#GPIO_PDDR_OFFSET]

;Turn off red LED
	LDR  	R1,=FGPIOE_BASE
	LDR  	R2,=LED_RED_MASK
	STR  	R2,[R1,#GPIO_PSOR_OFFSET]
	
;Turn off green LED
	LDR  	R1,=FGPIOD_BASE
	LDR  	R2,=LED_GREEN_MASK
	STR 	R2,[R1,#GPIO_PSOR_OFFSET]
	
;Turn on red LED
	LDR  	R1=FGPIOE_BASE
	LDR  	R2=LED_RED_MASK
	STR  	R2[R1#GPIO_PCOR_OFFSET]

;Turn on green LED
	LDR 	R1=FGPIOD_BASE
	LDR  	R2=LED_GREEN_MASK
	STR  	R2[R1#GPIO_PCOR_OFFSET]	
	
	POP	R0 - R3, LR}
;****************************************************************
Init_PIT_IRQ
			
	PUSH	{R0 - R3, LR}
	; Enable clock for PIT module
	LDR  	R1,=SIM_SCGC6
	LDR   	R2,=SIM_SCGC6_PIT_MASK
	LDR  	R3,[R1,#0]			;current SIM_SCGC6 value
	ORRS  	R3,R3,R2			;only PIT bit set
	STR   	R3,[R1,#0]			;update SIM_SCGC6
	; Disable PIT timer 0 
	LDR	R1, =PIT_CH0_BASE
	LDR	R2, =PIT_TCTRL_TEN_MASK
	LDR	R3, [R1, #PIT_TCTRL_OFFSET]
	BICS	R3, R3, R2
	STR	R3, [R1, #PIT_TCTRL_OFFSET]
	;Set PIT interrupt priority
	LDR   	R1,=PIT_IPR
	LDR   	R2,=NVIC_IPR_PIT_MASK
	LDR	R3, [R1, #0]
	BICS	R3, R3, R2
	STR   	R3,[R1,#0]
	; Clear any pending PIT interrupts
	LDR 	R1, =NVIC_ICPR
	LDR	R2, =NVIC_ICPR_PIT_MASK
	STR	R2, [R1, #0]
	;Unmask PIT interrupts
	LDR   	R1,=NVIC_ISER
	LDR  	R2,=NVIC_ISER_PIT_MASK
	STR   	R2,[R1,#0]
	; Enable PIT module
	LDR   	R1,=PIT_BASE
	LDR   	R2,=PIT_MCR_EN_FRZ
	STR   	R2,[R1,#PIT_MCR_OFFSET]
	; Set PIT timer 0 period for 0.01s s 
	LDR   	R1,=PIT_CH0_BASE
	LDR   	R2,=PIT_LDVAL_10ms
	STR   	R2,[R1,#PIT_LDVAL_OFFSET]
	; Enable PIT timer 0 interrupts
	LDR	R2, =PIT_TCTRL_CH_IE
	STR	R2, [R1, #PIT_TCTRL_OFFSET]
			
	POP	{R0 - R3, PC}
;****************************************************************
; is responseable for starting the stopwatch by setting the 
; RunStopWatch byte to a one.
PIT_ISR
	CPSID	I
	PUSH	{LR}
	PUSH	{R0 - R3}	
	LDR	R0, =RunStopWatch
	LDRB	R1, [R0, #0]
	CMP	R1, #0
	BEQ	EndPIT
	LDR	R0, =WatchCount
	LDR	R1, [R0, #0]
	ADDS	R1, R1, #1
	STR	R1, [R0, #0] 
EndPIT		
;Clear PIT Channel 0 interrupt
	LDR   	R1,=PIT_CH0_BASE		
	LDR   	R2,=PIT_TFLG_TIF_MASK	
	STR   	R2,[R1,#PIT_TFLG_OFFSET]	
	CPSIE	I
	POP	{R0 - R3}
	POP	{PC}				
;****************************************************************						
Init_UART0_IRQ
	PUSH 	{R0 - R3, LR}			
	LDR	R0, =TxQueueBuf
	LDR	R1, =TxQueueRec
	MOVS	R2, #Q_BUF_SZ
	BL	InitQueue
            
	LDR	R0, =RxQueueBuf
	LDR	R1, =RxQueueRec
	MOVS	R2, #Q_BUF_SZ
	BL	InitQueue
	
	LDR 	R0, =UART0_IPR      
	LDR     R2,=NVIC_IPR_UART0_PRI_3    
	LDR     R3,[R0,#0]       
	ORRS    R3,R3,R2    
	STR     R3,[R0,#0] 
;Clear any pending UART0 interrupts    
	LDR     R0,=NVIC_ICPR    
	LDR     R1,=NVIC_ICPR_UART0_MASK    
	STR     R1,[R0,#0] 
;Unmask UART0 interrupts  
	LDR     R0,=NVIC_ISER    
	LDR     R1,=NVIC_ISER_UART0_MASK	
	STR	R1, [R0,#0]
			
	MOVS  	R1,#UART0_C2_T_RI     
	STRB  	R1,[R0,#UART0_C2_OFFSET]
;Select MCGPLLCLK / 2 as UART0 clock source
	LDR 	R0,=SIM_SOPT2
	LDR 	R1,=SIM_SOPT2_UART0SRC_MASK
	LDR 	R2,[R0,#0]
	BICS 	R2,R2,R1
	LDR 	R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
	ORRS 	R2,R2,R1
	STR 	R2,[R0,#0]
;Enable external connection for UART0
	LDR 	R0,=SIM_SOPT5
	LDR 	R1,= SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
	LDR 	R2,[R0,#0]
	BICS 	R2,R2,R1
	STR 	R2,[R0,#0]
;Enable clock for UART0 module
	LDR 	R0,=SIM_SCGC4
	LDR 	R1,= SIM_SCGC4_UART0_MASK
	LDR 	R2,[R0,#0]
	ORRS 	R2,R2,R1
	STR 	R2,[R0,#0]
;Enable clock for Port A module
	LDR	R0,=SIM_SCGC5
	LDR 	R1,= SIM_SCGC5_PORTA_MASK
	LDR 	R2,[R0,#0]
	ORRS 	R2,R2,R1
	STR 	R2,[R0,#0]
;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
	LDR 	R0,=PORTA_PCR1
	LDR 	R1,=PORT_PCR_SET_PTA1_UART0_RX
	STR 	R1,[R0,#0]
;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
	LDR 	R0,=PORTA_PCR2
	LDR 	R1,=PORT_PCR_SET_PTA2_UART0_TX
	STR 	R1,[R0,#0]
;Disable UART0 receiver and transmitter
	LDR 	R0,=UART0_BASE
	MOVS 	R1,#UART0_C2_T_R
	LDRB 	R2,[R0,#UART0_C2_OFFSET]
	BICS 	R2,R2,R1
	STRB 	R2,[R0,#UART0_C2_OFFSET]
	LDR	R0, =UART0_IPR
	LDR	R1, =NVIC_IPR_UART0_PRI_3
	LDR	R3, [R0, #0]
	ORRS	R3, R3, R1
	STR	R1, [R0, #0]

;Set UART0 for 9600 baud, 8N1 protocol
	LDR  	R0,=UART0_BASE
	MOVS 	R1,#UART0_BDH_9600
	STRB 	R1,[R0,#UART0_BDH_OFFSET]
	MOVS 	R1,#UART0_BDL_9600
	STRB 	R1,[R0,#UART0_BDL_OFFSET]
	MOVS 	R1,#UART0_C1_8N1
	STRB 	R1,[R0,#UART0_C1_OFFSET]
	MOVS 	R1,#UART0_C3_NO_TXINV
	STRB 	R1,[R0,#UART0_C3_OFFSET]
	MOVS 	R1,#UART0_C4_NO_MATCH_OSR_16
	STRB 	R1,[R0,#UART0_C4_OFFSET]
	MOVS 	R1,#UART0_C5_NO_DMA_SSR_SYNC
	STRB 	R1,[R0,#UART0_C5_OFFSET]
	MOVS 	R1,#UART0_S1_CLEAR_FLAGS
	STRB 	R1,[R0,#UART0_S1_OFFSET]
	MOVS 	R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
	STRB 	R1,[R0,#UART0_S2_OFFSET] 
;Enable UART0 receiver and transmitter
	MOVS 	R1,#UART0_C2_T_RI
	STRB 	R1,[R0,#UART0_C2_OFFSET]
	POP 	{R0-R3, PC}
;****************************************************************
UART0_ISR
	CPSID	I
	PUSH	{LR}
	PUSH	{R0-R3}	
            
	LDR	R5, =UART0_BASE	
	LDRB	R1,[R5, #UART0_C2_OFFSET]
	MOVS	R2, #UART0_C2_TIE_MASK
	TST	R1, R2
	BEQ	ISRLoop
            
	LDRB	R1, [R5, #UART0_S1_OFFSET]
	MOVS	R2, #UART0_S1_TDRE_MASK
	TST	R1, R2
	BEQ	ISRLoop
            
	LDR	R1, =TxQueueRec
	BL	Dequeue
	BCC	WriteChar
            
	MOVS	R1, #UART0_C2_T_RI
	LDRB	R2, [R5, #UART0_C2_OFFSET]
	ANDS	R2, R2, R1
	STRB	R2, [R5, #UART0_C2_OFFSET]
ISRLoop
	MOVS	R2, #UART0_S1_RDRF_MASK
	LDRB	R3, [R5, #UART_S1_OFFSET]
	TST	R2, R3
	BEQ	BackToMain
			
	LDRB	R0, [R5, #UART0_D_OFFSET]
	LDR	R1, =RxQueueRec
	BL	Enqueue
BackToMain
	POP	{R0 - R3}
	CPSIE	I
	POP	{PC}	
			
WriteChar		
	STRB	R0, [R5, #UART0_D_OFFSET]
	B	ISRLoop
;****************************************************************
;>>>>>   end subroutine code <<<<<
            ALIGN
;>>>>>   end subroutine code <<<<<
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    Dummy_Handler      ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendableSrvReq (pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 xfer complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 xfer complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 xfer complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 xfer complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:command complete; read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:I2C1
            DCD    Dummy_Handler      ;26:SPI0 (all IRQ sources)
            DCD    Dummy_Handler      ;27:SPI1 (all IRQ sources)
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:UART1 (status; error)
            DCD    Dummy_Handler      ;30:UART2 (status; error)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:TPM2
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR	     ;38:PIT (all IRQ sources)
            DCD    Dummy_Handler      ;39:I2S0
            DCD    Dummy_Handler      ;40:USB0
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:Segment LCD
            DCD    Dummy_Handler      ;46:PORTA pin detect
            DCD    Dummy_Handler      ;47:PORTC and PORTD pin detect
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<

;>>>>>   end variables here <<<<<
            ALIGN
            END
