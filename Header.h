/*********************************************************************/
/* Lab Exercise LASTONE                                              */
/* Ultimate Assembler Game                                           */
/* Name:  Erez Binyamin                                              */
/* Date:  April 24, 2017                                             */
/* Class:  CMPE 250                                                  */
/* Section:  All sections                                            */
/*********************************************************************/

/* Data Types */
typedef char *String;	

/* assembly language subroutines */
char GetChar (void);
void PutChar (char Character);
void GetStringSB (char String[], int StringBufferCapacity);
void PutStringSB (char String[], int StringBufferCapacity);
int SameStringSB (char String_A[], char String_B[]);

void Init_UART0_IRQ (void);
void Init_LED_IRQ (void);
void Init_PIT_IRQ (void);
void Startup (void);

void PutNumHex (int);
void PutNumUB (char);

void REDLED_OFF (void);
void REDLED_ON (void);
void GREENLED_OFF (void);
void GREENLED_ON (void);

int GetTime (void);
void ClearTime (void);
void StartTime (void);
void StopTime (void);
