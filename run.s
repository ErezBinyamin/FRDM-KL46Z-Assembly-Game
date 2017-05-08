            TTL Program Title for Listing Header Goes Here
;****************************************************************
;The Ulimate Assembler: Text based study game
;Displays question to user, and reads user input. Conditionaly awards points or removes "lives"
;Name:  Erez Binyamin/Derek Freeman
;Date:  05/02/2017
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
UART0_BDH_9600    EQU 0x01
UART0_BDL_9600    EQU 0x38
UART0_C1_8N1      EQU 0x00
UART0_C2_T_R 	  EQU (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK )
UART0_C3_NO_TXINV EQU 0x00
UART0_C4_OSR_16   EQU 0x0F
UART0_C4_NO_MATCH_OSR_16 EQU UART0_C4_OSR_16
UART0_C5_NO_DMA_SSR_SYNC EQU 0x00
UART0_S1_CLEAR_FLAGS EQU 0x1F
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS EQU 0xC0

PORT_PCR_SET_PTA1_UART0_RX EQU (PORT_PCR_ISF_MASK :OR: \
								PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX EQU (PORT_PCR_ISF_MASK :OR: \
								PORT_PCR_MUX_SELECT_2_MASK)
SIM_SOPT2_UART0SRC_MCGPLLCLK EQU (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
		(SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR EQU (SIM_SOPT5_UART0ODE_MASK :OR: \
			SIM_SOPT5_UART0RXSRC_MASK :OR: SIM_SOPT5_UART0TXSRC_MASK)
SIM_CGC6			EQU	SIM_SCGC6_PIT_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK 
UART0_IRQ_PRIORITY    EQU  3 
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS) 
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS) 
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK 
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
;NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK	
PIT_IRQ_PRIORITY      EQU  0
NVIC_IPR_PIT_MASK     EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0    EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;UART0_IRQ_PRIORITY    EQU  3
;NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
;NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
;NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK	
UART0_C2_T_RI  		  EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R) 
UART0_C2_TI_RI        EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)    
PIT_MCR_EN_FRZ  	  EQU  PIT_MCR_FRZ_MASK
PIT_LDVAL_10ms  	  EQU  239999
PIT_TCTRL_CH_IE  	  EQU  (PIT_TCTRL_TIE_MASK :OR: PIT_TCTRL_TEN_MASK)	
PIT_IRQ_PRI  		  EQU  0 	
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

LED_PORTD_MASK  EQU  LED_GREEN_MASK
LED_PORTE_MASK  EQU  LED_RED_MASK 
;****************************************************************
;EQUates continue below
;EQUates
;<< Queues >>
;Management record structure field displacements 
IN_PTR    EQU    0 
OUT_PTR   EQU    4 
BUF_STRT  EQU    8 
BUF_PAST  EQU    12 
BUF_SIZE  EQU    16 
NUM_ENQD  EQU    17
 
;Queue structure sizes
Q_BUF_SZ  EQU    80        ;Room for 80 characters
Q_REC_SZ  EQU    18        ;Management record size

;<< String I/O >>
;String buffer for user input
MaxString	EQU		79

;Characters
NULL		EQU		0
LF			EQU		10
CR			EQU		13
	
;Division subroutine DIVU
MAXVAL		EQU		32
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
			
			EXPORT 	GetChar
			EXPORT 	PutChar
			
			EXPORT 	GetStringSB
			EXPORT 	PutStringSB
			EXPORT 	SameStringSB

			EXPORT 	Init_UART0_IRQ
			EXPORT 	Init_LED_IRQ
			EXPORT 	Init_PIT_IRQ

			EXPORT 	PutNumHex
			EXPORT 	PutNumUB

			EXPORT 	REDLED_OFF
			EXPORT 	REDLED_ON
			EXPORT 	GREENLED_OFF
			EXPORT 	GREENLED_ON
			
			EXPORT 	GetTime
			EXPORT 	ClearTime
			EXPORT 	StartTime
			EXPORT 	StopTime
			
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
;----------------------------------------------------------------
;LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-LED-
Init_LED_IRQ
LED_IRQ
	PUSH	{R0 - R3, LR}
;enable clock for PORT D and E modules
	LDR     R1, =SIM_SCGC5
	LDR     R2, =(SIM_SCGC5_PORTD_MASK :OR: SIM_SCGC5_PORTE_MASK)
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

	;Turn on red LED
	LDR  	R1, =FGPIOE_BASE
	LDR  	R2, =LED_RED_MASK
	STR  	R2, [R1,#GPIO_PCOR_OFFSET]
	
	;Turn on green LED
	LDR 	R1, =FGPIOD_BASE
	LDR  	R2, =LED_GREEN_MASK
	STR  	R2,[R1,#GPIO_PCOR_OFFSET]	
	
	;Turn off red LED
	LDR  	R1,=FGPIOE_BASE
	LDR  	R2,=LED_RED_MASK
	STR  	R2,[R1,#GPIO_PSOR_OFFSET]	
	
	;Turn off green LED
	LDR  	R1,=FGPIOD_BASE
	LDR  	R2,=LED_GREEN_MASK
	STR 	R2,[R1,#GPIO_PSOR_OFFSET]
		
	POP		{R0 - R3, PC}
;----------------------------------------------------------------
REDLED_OFF
	PUSH	{R1 - R2, LR}
;Turn off red LED
	LDR  	R1,=FGPIOE_BASE
	LDR  	R2,=LED_RED_MASK
	STR  	R2,[R1,#GPIO_PSOR_OFFSET]
	POP	{R1 - R2, PC}
;----------------------------------------------------------------	
GREENLED_OFF	
	PUSH	{R1 - R2, LR}
;Turn off green LED
	LDR  	R1,=FGPIOD_BASE
	LDR  	R2,=LED_GREEN_MASK
	STR 	R2,[R1,#GPIO_PSOR_OFFSET]
	POP	{R1 - R2, PC}
;----------------------------------------------------------------	
REDLED_ON	
	PUSH	{R1 - R2, LR}
;Turn on red LED
	LDR  	R1, =FGPIOE_BASE
	LDR  	R2, =LED_RED_MASK
	STR  	R2, [R1,#GPIO_PCOR_OFFSET]
RedDone	
	POP	{R1 - R2, PC}
;----------------------------------------------------------------
GREENLED_ON
	PUSH	{R1 - R2, LR}
;Turn on green LED
	LDR 	R1, =FGPIOD_BASE
	LDR  	R2, =LED_GREEN_MASK
	STR  	R2,[R1,#GPIO_PCOR_OFFSET]
GreenDone		
	POP	{R1 - R2, PC}
;****************************************************************
;PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-PIT-
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
;----------------------------------------------------------------
;PIT Interrupt Service Routine
;	if (RunStopWatch) {
;	Increment Count
;	}
;	clear interrupt
;	return
PIT_ISR
			CPSID	I
			PUSH	{LR}
			PUSH	{R0-R2}
			
		;If (RunStopWatch)
			LDR		R0,=RunStopWatch
			LDRB		R0,[R0,#0]
			CMP		R0,#0
			BEQ		Clr_PIT_intrpt
			
		;Increment Count
			LDR		R0,=Count
			LDR		R1,[R0,#0]
			ADDS	R1,R1,#1
			STR		R1,[R0,#0]
			
Clr_PIT_intrpt
			LDR 	R1,=PIT_TFLG0
			LDR		R2,=PIT_TFLG_TIF_MASK
			STR 	R2,[R1,#0]
		
			POP		{R0-R2}
			CPSIE	I
			POP		{PC}
;---------------------------------------------------------------
;@PARAM void
;@RETURN R0: int count
;Returns Sytem Time in R0
GetTime
	PUSH	{LR}
	LDR		R0,=Count
	LDR		R0,[R0,#0]
	POP		{PC}
;----------------------------------------------------------------
;@PARAM void
;@RETURN void
;Sets System time to 0
ClearTime
	PUSH	{R0-R1, LR}
	MOVS	R1,#0
	LDR		R0,=Count
	STR		R1,[R0,#0]
	POP		{R0-R1, PC}
;----------------------------------------------------------------
;@PARAM void
;@RETURN void
;Starts PIT timer module
StartTime
	PUSH	{R0-R1, LR}
	MOVS	R1,#1
	LDR		R0,=RunStopWatch
	STR		R1,[R0,#0]
	POP		{R0-R1, PC}
;----------------------------------------------------------------
;@PARAM void
;@RETURN void
;Stops PIT timer module
StopTime
	PUSH	{R0-R1, LR}
	MOVS	R1,#0
	LDR		R0,=RunStopWatch
	STR		R1,[R0,#0]
	POP		{R0-R1, PC}
;****************************************************************
;UART0-UART0-UART0-UART0-UART0-UART0-UART0-UART0-UART0-UART0-UART0				
Init_UART0_IRQ	
				PUSH	{LR,R0-R7}
				LDR		R0,=TxQBuffer
				LDR		R1,=TxRecord
				MOVS	R2,#Q_BUF_SZ
				BL		InitQueue
				LDR		R0,=RxQBuffer
				LDR		R1,=RxRecord
				BL		InitQueue
				
;Set UART0 IRQ priority
			    LDR 	R0,=UART0_IPR
;LDR R1,=NVIC_IPR_UART0_MASK
			    LDR 	R2,=NVIC_IPR_UART0_PRI_3
			    LDR 	R3,[R0,#0]
;BICS R3,R3,R1
			    ORRS 	R3,R3,R2
			    STR 	R3,[R0,#0]
;Clear any pending UART0 interrupts
			    LDR 	R0,=NVIC_ICPR
			    LDR 	R1,=NVIC_ICPR_UART0_MASK
			    STR 	R1,[R0,#0]
;Unmask UART0 interrupts
			    LDR 	R0,=NVIC_ISER
			    LDR 	R1,=NVIC_ISER_UART0_MASK
			    STR 	R1,[R0,#0]
				
;select MCGPLLCLK / 2 as UART0 clock source
				LDR		R0,=SIM_SOPT2							
				LDR		R1,=SIM_SOPT2_UART0SRC_MASK
				LDR		R2,[R0,#0]								;current SIM_SOPT2 value
				BICS	R2,R2,R1								;only UARTSRC bits cleared
				LDR		R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
				ORRS	R2,R2,R1								;only UART0 bits changed
				STR		R2,[R0,#0]								;update SIM_SOPT2
			
		;Enable external connection from UART0
				LDR     R0,=SIM_SOPT5
				LDR     R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
				LDR     R2,[R0,#0]								;current SIM_SOPT5 value
				BICS    R2,R2,R1								;only UART0 bits cleared
				STR     R2,[R0,#0]								;update SIM_SOPT5
				
		;Enable clock for UART0 Module
				LDR     R0,=SIM_SCGC4
				LDR     R1,=SIM_SCGC4_UART0_MASK
				LDR     R2,[R0,#0]								;current SIM_SCGC4 value
				ORRS    R2,R2,R1								;only UART1 bit set
				STR     R2,[R0,#0]								;update SIM_SCGC4
			
		;enable clock for port A module
				LDR     R0,=SIM_SCGC5
				LDR     R1,=SIM_SCGC5_PORTA_MASK
				LDR     R2,[R0,#0]								;current SIM_SCGC5 value
				ORRS    R2,R2,R1								;only PORTA bit set
				STR     R2,[R0,#0]								;update SIM_SCGC5
			
		;connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
				LDR     R0,=PORTA_PCR1
				LDR     R1,=PORT_PCR_SET_PTA1_UART0_RX
				STR     R1,[R0,#0]								;Port A pin 1 connects to UART0 Rc
				LDR     R0,=PORTA_PCR2
				LDR     R1,=PORT_PCR_SET_PTA2_UART0_TX
				STR     R1,[R0,#0]								;Port A pin 2 connects to UART0 Tx
			
		;Disable UART0 receiver and transmitter
				LDR		R0,=UART0_BASE
				MOVS	R1,#UART0_C2_T_R
				LDRB	R2,[R0,#UART0_C2_OFFSET]
				BICS	R2,R2,R1
				STRB	R2,[R0,#UART0_C2_OFFSET]
			
		;Set UART0 for 9600 baud, 8N1 protocol
				MOVS	R1,#UART0_BDH_9600
				STRB	R1,[R0,#UART0_BDH_OFFSET]
				MOVS	R1,#UART0_BDL_9600
				STRB	R1,[R0,#UART0_BDL_OFFSET]
				MOVS	R1,#UART0_C1_8N1
				STRB	R1,[R0,#UART0_C1_OFFSET]
				MOVS	R1,#UART0_C3_NO_TXINV
				STRB	R1,[R0,#UART0_C3_OFFSET]
				MOVS	R1,#UART0_C4_NO_MATCH_OSR_16
				STRB	R1,[R0,#UART0_C4_OFFSET]
				MOVS	R1,#UART0_C5_NO_DMA_SSR_SYNC
				STRB	R1,[R0,#UART0_C5_OFFSET]
				MOVS	R1,#UART0_S1_CLEAR_FLAGS
				STRB	R1,[R0,#UART0_S1_OFFSET]
				MOVS	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
				STRB	R1,[R0,#UART0_S2_OFFSET]
			
		;Enable UART receiver and transmitter
				MOVS	R1,#UART0_C2_T_RI
				STRB	R1,[R0,#UART0_C2_OFFSET]			
				
				POP		{PC,R0-R7}
;---------------------------------------------------------------
;UART0 Interrupt Service Routine
UART0_IRQ_HANDLER
UART0_ISR
			CPSID	I
			PUSH	{LR}
			PUSH 	{R4-R6}
			
		;Check if interrupts are enabled
			LDR		R4,=UART0_BASE
			LDRB	R5,[R4,#UART0_C2_OFFSET]
			MOVS	R6,#UART0_C2_TIE_MASK
			TST		R5,R6
			BEQ		CheckRDRF
			
		;Check if TxInterrupt occured
			LDRB	R5,[R4,#UART0_S1_OFFSET]
			;MOVS	R6,#UART0_S1_TDRE_MASK    ;TDRE mask = 0x80 = TIE mask
			TST		R5,R6
			BEQ		CheckRDRF
			
		;DEQUEUE Tx
			LDR		R1,=TxRecord
			BL		Dequeue
			BCC		WriteToTDRE
			
		;If Decqueue Unsucsessfull Disable Interrups
			MOVS	R5,#UART0_C2_T_RI
			STRB	R5,[R4,#UART0_C2_OFFSET]
CheckRDRF
		;Check if RxInterrupt occured
			LDRB	R5,[R4,#UART0_S1_OFFSET]
			MOVS	R6,#UART0_S1_RDRF_MASK
			TST		R5,R6
			BEQ		MaskPop
			
		;Recieve char and ENQUEUE Rx
			LDRB	R0,[R4,#UART0_D_OFFSET]
			LDR		R1,=RxRecord
			BL		Enqueue
MaskPop
			POP 	{R4-R6}
			CPSIE	I
			POP		{PC}
WriteToTDRE
			STRB	R0,[R4,#UART0_D_OFFSET]
			B		CheckRDRF		
;------------------------------------------------------------------------------------------------------------------
;InitQueue:
;Initializes the queue record structure at the address in R1
;for the empty queue buffer at the address in R0
;of size, (i.e., character capacity), given in R2.
InitQueue
			PUSH	{R0-R3,LR}
			
			MOVS	R3,R0
			STR		R3,[R1,#IN_PTR]
			
	 		STR		R3,[R1,#OUT_PTR]
			
			STR		R3,[R1,#BUF_STRT]
			
			ADDS	R3,R3,R2
			STR		R3,[R1,#BUF_PAST]
			
			STRB	R2,[R1,#BUF_SIZE]
			
			MOVS	R3,#0
			STRB	R3,[R1,#NUM_ENQD]
			
			POP		{R0-R3,PC}
;------------------------------------------------------------------------------------------------------------------
;Enqueue:
;Attempts to put a character in the queue whose queue record structure’s address is in R1—
;if the queue is not full, enqueues the single character from R0 to the queue,
;and returns with the PSRC cleared to report enqueue success;
;otherwise, returns with the PSRC bit set to report enqueue failure. 
Enqueue
			PUSH	{R0-R5,LR}
			
		;Ensure the queue is not Full
			LDRB	R2,[R1,#NUM_ENQD]		;Load NUM_ENQD
            LDRB    R3,[R1,#BUF_SIZE]
			CMP		R2,R3
			BEQ		SetCarryEQ
			
		;store DATA
			LDR		R2,[R1,#IN_PTR]			;Load Word - pointer to IN_PTR
			STRB	R0,[R2,#0]				;Write data to that location
			
		;Increment In Ptr
			ADDS	R2,R2,#1
		;Circular Nature Check (time is a flat circle)
			LDR		R3,[R1,#BUF_PAST]		;Load pointer to last address of queueBuffer
			CMP		R2,R3					;Compare to current position of IN_PTR (after being incremented)
			BGE		CircleEQ
			
DoneCircleEQ
			STR		R2,[R1,#IN_PTR]			;Store new IN_PTR
			
		;Increment Num Enqued
			LDRB	R2,[R1,#NUM_ENQD]		;Load NUM_ENQD	
			ADDS	R2,R2,#1				;NUM_ENQD ++
			STRB	R2,[R1,#NUM_ENQD]		;Store new NUM_ENQD
			
		;Clear Carry Flag
			MOVS	R2,#0
			LSRS	R2,#1
DoneEnQ
			POP		{R0-R5,PC}
;BRANCHES
CircleEQ
			LDR		R2,[R1,#BUF_STRT]
			B		DoneCircleEQ
SetCarryEQ
			MOVS	R2,#1
			LSRS	R2,#1
			B		DoneEnQ
;------------------------------------------------------------------------------------------------------------------
;Dequeue:
;Attempts to get a character from the queue whose record structure’s address is in R1:
;if the queue is not empty, dequeues a single character from the queue to R0,
;and returns with the PSRC bit cleared, (i.e., 0), to report dequeue success; otherwise,
;returns with the PSRC bit set, (i.e., 1) to report dequeue failure.
Dequeue
			PUSH	{R1-R5,LR}
			
		;Ensure the queue is not empty
			LDRB	R2,[R1,#NUM_ENQD]
			CMP		R2,#0
			BEQ		SetCarryDQ
			
		;Get DATA
			LDR		R2,[R1,#OUT_PTR]			;Load Word - OUT_PTR
			LDRB	R0,[R2,#0]					;Get char to POP from queue
			
		;Increment Out Ptr
			ADDS	R2,R2,#1					;Increment OUT_PTR by one byte
		
		;Circular Nature Check
			LDR		R3,[R1,#BUF_PAST]			;Load Word - Max value of OUT_PTR
			CMP		R2,R3
			BGE		CircleDQ
DoneSetDQ
			STR		R2,[R1,#OUT_PTR]			;Store new out pointer
			
		;Decrement Num enqued
			LDRB	R2,[R1,#NUM_ENQD]			;Load NUM_ENQD
			SUBS	R2,R2,#1					;NUM_ENQD --
			STRB	R2,[R1,#NUM_ENQD]			;Store 	NUM_ENQD
			
		;Clear Carry
			MOVS	R2,#0
			LSRS	R2,#1
DoneDeQ
			POP		{R1-R5,PC}
			
;BRANCHES
CircleDQ
			LDR		R2,[R1,#BUF_STRT]			;Load Word - Start value of QBuffer
			B		DoneSetDQ
SetCarryDQ
			MOVS	R2,#1
			LSRS	R2,#1
			B		DoneDeQ
;--------------------------------------------------------------
;@PARAM R0: char
;@RETURN void
;Prints Char in R0, to terminal		
PutChar
			PUSH	{R0-R3, LR}
			LDR 	R1,=TxRecord
PutCEnQ
		;Mask other interrupts
			CPSID	I
		;Try to enqueue
			BL		Enqueue
		;Unmask other interrupts
			CPSIE	I
		;Loop condition
			BCS		PutCEnQ
		;Set UART0_C2 Tx bit to "1"
			LDR		R1,=UART0_BASE
			MOVS	R3,#UART0_C2_TI_RI
			LDRB	R2,[R1,#UART0_C2_OFFSET]
			ORRS	R3,R3,R2
			STRB	R3,[R1,#UART0_C2_OFFSET]
			
			POP		{R0-R3,PC}
;------------------------------------------------------------------------------------------------------------------
;@PARAM void
;@RETURN R0: char
;Gets character from terminal, returns in R0
GetChar		
			PUSH	{R1,LR}	
			LDR		R1,=RxRecord				
GetCEnQ			
			CPSID	I
			BL		Dequeue			;Enqueue character
			CPSIE	I				;Unmask other interrupts
			BCS		GetCEnQ 		;Loop condition
			POP	 	{R1,PC}					
			
;------------------------------------------------------------------------------------------------------------------					
;Get string of characters and store it in memory location of String
;Gets inputted string from terminal
GetStringSB
			PUSH	{R0-R4, LR}
            
            ;Buff Capcaity fuckery
			;MOVS	R1,#MaxString
            ADDS    R1,R1,#1
            
			MOVS 	R2,#1
			MOVS 	R4,R0
GStringLoop
			BL		GetChar
			CMP		R0,#CR
			BEQ		EndGString
			CMP		R2,R1
			BHS		NextLine
			BL		PutChar
			STRB	R0,[R4,#0]
			ADDS	R4,R4,#1
			ADDS	R2,R2,#1
			B		GStringLoop
NextLine	
			BL		GetChar
			CMP		R0,#CR
			BEQ		EndGString
			B	 	NextLine
			
EndGString
			MOVS	R3,#NULL
			STRB	R3,[R4,#0]
			POP		{R0-R4, PC}
;------------------------------------------------------------------------------------------------------------------
;Put the string from the memory location of string into the command line			
PutStringSB		
			PUSH	{LR}			;Save the return address for a function call
			PUSH	{R0-R6}			;Save the Register contents of R0-R6
			MOVS	R1,#MaxString	;Copies the value of MAX_STRING in equates used for the conditional statement
			MOVS	R2,R0			;Save memory location of String into R2
			MOVS	R3,#0			;Initialize the counter and the index displacement to 0
			MOVS	R4,#0			;Initialize the NULL character to be used for a conditional statement
MainPut			
				
			CMP		R3,R1			;Compare to see if the counter is greater than or same as MAX_STRING
			BHS		PutReturn		;If true than branch to Return
			LDRB	R0,[R2,R3]		;Load the data R3 indexes away from the R2(base) into R0
			CMP		R0,R4			;If the character was null 
			BEQ		PutReturn		;Branch to PutReturn
			BL		PutChar			;Branch to PutChar
			ADDS	R3, R3,#1		;Increment the counter
			B		MainPut			;Branch back to the top of loop
				
PutReturn
			POP		{R0-R6}			;Restore the registers R0-R6
			POP		{PC}			;Return to the next instruction address
	
;------------------------------------------------------------------------------------------------------------------
;@PARAM R0: *StringA
;@PARAM R1: *StringB
;@RETURN R0: (boolean)if (StringA .equals (StringB))
SameStringSB
	PUSH	{R1 - R6, LR}
	MOVS	R5, R0
	MOVS	R6, R1
	BL	 	LengthStringSB
	MOVS	R3, R2
	MOVS	R0, R1
	BL	 	LengthStringSB
	CMP		R2, R3
	BNE		Not_Equal
CompareLoop
	SUBS	R2, R2, #1
	LDRB	R3, [R5, R2]
	LDRB	R4, [R6, R2]
	CMP		R3, R4
	BNE		Not_Equal
	CMP		R2, #0
	BNE		CompareLoop
Equal	
	MOVS	R0, #1
	B		GoBack
Not_Equal
	MOVS	R0, #0
GoBack
	POP {R1 - R6, PC}
;------------------------------------------------------------------------------------------------------------------
;Preventing overrun of the buffer capacity specified in R1,
;this subroutine determines how many characters are in the null-terminated string in memory starting at
;the address in R0, and it returns the number of characters in R2 (wiil return 1-BufferCapacity)
LengthStringSB
			PUSH	{R0-R1, R3-R4, LR}
			MOVS	R2,#0				;Initialize counter
			SUBS	R1,R1,#1			;Make Way for Null Char
LengthLoop
			LDRB	R3,[R0,R2]			;Gets Char at index:R2 (Char Array = Byte Array)
;Check if string is done (null terminated)
			CMP		R3,#NULL				;Null char is end of string
			BEQ		LDONE
;Check buffer Capacity
			CMP		R2,R1
			BGE		LDONE
;ELSE iterate counter and loop
			ADDS	R2,R2,#1
			B		LengthLoop
LDONE
			;ADDS	R2,R2,#1			;Account for Neglected NullChar
			POP		{R0-R1, R3-R4, PC}
;------------------------------------------------------------------------------------------------------------------
			;****************************************************************
;MATH-MATH-MATH-MATH-MATH-MATH-MATH-MATH-MATH-MATH-MATH-MATH-MATH
;<<<< DIVISION SUBROUTINE >>>>
									; DIVU
									; dividend / divisor = quotient
									; R1 / R0
									; R2 USED FOR DVISOR
									; R3 USED FOR ITERATOR
									; R4 USED FOR TEMP VALUES IN CALCULATION
									; RETURN QUOTIENT IN R0
									; RETURN REMAINDER IN R1
DIVU								;
			CMP  	R0,#0			;
			BEQ		DivByZero		; Check zero divide
			
			PUSH	{R2-R5, LR}		; Pushing values on the stack to preserve data
									;
			MOVS   	R2,R0			; MAKE R2 HOLD DIVISOR
			MOVS 	R0,#0			; Initialize solution value to 0
			MOVS	R3,#MAXVAL		; Initialize iterator to euqte MAXVAL
ComparisonLoop						;
			SUBS	R3,R3,#1		; Subtract 1 from iterator
									;
			MOVS	R4,R1			;
			LSRS	R4,R4,R3		; Right shift dividend %iterator% bits and store in R4 THIS LINE GAVE ME PAIN A -> L
			CMP		R4,R2			; If shifted value is >= divisor BRANCH to subtract
			BHS		Subtract		;
CheckIterator						;
			CMP 	R3,#0			; If iterator is equal to 0
			BEQ		CleanUp			; End for loop
			LSLS	R0,R0,#1		;
			B		ComparisonLoop	; Else loop againc 
Subtract							;
			ADDS 	R0,R0,#1		; Add 1 to solution
			SUBS	R4,R4,R2		; Subtract divisor from SHIFTED dividend and store in R4
			LSLS 	R4,R4,R3		; Shift Back subtraction result
			
			MOVS	R5,#32			; 
			SUBS	R5,R5,R3		;
			
			LSLS	R1,R1,R5		; Left shift INPUTED divisor 32-iterator times
			LSRS	R1,R1,R5		; Right shift augmented divisor 32-iterator times
			ORRS	R1,R1,R4		; Logical OR subtraction result with augmented divisor
			B 		CheckIterator	; BRANCH back to the loop
CleanUp
			MRS		R2,APSR			;
			MOVS	R3,#0x20		;
			LSLS	R3,R3,#24		;
			BICS	R2,R2,R3		; BIT CLEAR
			MSR		APSR,R2			;
			B 		StoreReturn		;
DivByZero
			MRS 	R0,APSR			; APSR BITS ARE STORED IN R0
			MOVS 	R1,#0x20		; MOVE 32 INTO R1
			LSLS 	R1,R1,#24		; MOVES 1 TO C POSITION. READY TO WRITE TO APSR
			ORRS	R0,R0,R1		; PROTECT OLD APSR DATA, AND WRITE C=1 WITH OR
			MSR		APSR,R0			; LOAD UPDATED APSR.	
			B		ReturnToMain	;
StoreReturn
			POP		{R2-R5, PC}		; return R0 and R5 to their pre-subroutine states
ReturnToMain
			BX		LR
;------------------------------------------------------------------------------------------------------------------
;PutNumHex
;prints to the terminal screen the text hexadecimal representation of the unsigned word value in R0.
;if R0 contains: 0x000012FF
;then 000012FF would be printed to the terminal
PutNumHex
			PUSH 	{R0-R3, LR}
			MOVS	R1,R0				;Copy data out of R0 and into R1 because of PutChar subroutine
			MOVS	R2,#15				;BITMASK 0x0000000F
			MOVS	R3,#28				;ITERATOR 7-0 --> 28-0
ByteLoop
			;Isolate desired value
			MOVS	R0,R1				;Copy DATA
			LSRS	R0,R0,R3			;Move relevant byte to LSByte position
			ANDS 	R0,R0,R2			;Clear out all but LSbyte
			
			;Convert value to ascii value
			CMP		R0,#0x0A			;Check to see if R0 < 0x0A
			BLT		Number
			ADDS	R0,R0,#55			;Capital letter ascii codes are 55 away from their true values
			B		DoneConvert
Number
			ADDS	R0,R0,#'0'			;0-9 ascii codes are 48 away from their true values 
DoneConvert
			;PrintChar in R0 to screen
			BL	PutChar
			
			;Loop Condition
			CMP		R3,#0				;if(R3 <= 0){goto endByteLoop}
			BLE		endByteLoop			;
			
			;Else
			SUBS	R3,R3,#4			;Subtract four each time so iterator can doubble as shifter
			B		ByteLoop			;
endByteLoop	
			POP 	{R0-R3, PC}			;

;------------------------------------------------------------------------------------------------------------------
;PutNumU
;prints to the terminal screen the text decimal representation of the unsigned word value in R0. 
;if R0 contains: 0x00000100
;then 256 would be printed to the terminal
PutNumU
			PUSH 	{R0-R2, LR}
			;Initialize iterator
			MOVS	R2,#0
			
;Convert to decimal and push bytes to stack
DecConvert
			MOVS	R1,R0			;Initialize R1 to be divisor
			MOVS	R0,#10			;Initialize R0 to 10 for dividend
			BL		DIVU 			;R1 / R0 --> QUOTIENT : R0, REMAINDER : R1
			PUSH    {R1}			;Push remainder to stack
			ADDS	R2,R2,#1		;Iterator++
			CMP		R0,#0
			BNE		DecConvert		;If the quotient is NONzero loop

;Print stored Decimal value (iterating backwards through array)
Ploop
			SUBS	R2,R2,#1
			POP		{R0}			;Pop to R0
			ADDS	R0,R0,#'0'		;0-9 ascii codes are 48 away from their true values 
			BL		PutChar			;Print Value
			CMP		R2,#0
			BGT		Ploop			;Print all of the number
			
			POP		{R0-R2, PC}
;------------------------------------------------------------------------------------------------------------------
;PutNumUB
;prints to the terminal screen the text decimal representation of the least significant unsigned byte value in R0.  
;if R0 contains: 0x03021101
;then 01 would be printed to the terminal
;if R0 contains: 0x12345678
;then 78 would be converted to decimal (120) then printed to the terminal
PutNumUB
			PUSH 	{R0-R1, LR}
			MOVS	R1,#0x000000FF		;BitMask to only save last two bytes
			ANDS	R0,R0,R1
			BL		PutNumU				;Call PutNumU to convert to decimal and print
			POP 	{R0-R1, PC}

;---------------------------------------------------------------
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
            DCD    PIT_ISR	      ;38:PIT (all IRQ sources)
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

;PIT variables
RunStopWatch	SPACE	1	;either a zero or a 1
			ALIGN
Count			SPACE	4	;Count is a word

;Queue structures
QBuffer   	SPACE  Q_BUF_SZ  ;Queue contents
QRecord 	SPACE  Q_REC_SZ  ;Queue management record 
    
			ALIGN

;Recieve Queue
RxQBuffer   SPACE  Q_BUF_SZ  ;Queue contents
RxRecord 	SPACE  Q_REC_SZ  ;Queue management record
    
			ALIGN
;Transmit Queue
TxQBuffer   SPACE  Q_BUF_SZ  ;Queue contents
TxRecord 	SPACE  Q_REC_SZ  ;Queue management record
    
			ALIGN
;>>>>>   end variables here <<<<<
            ALIGN
            END
