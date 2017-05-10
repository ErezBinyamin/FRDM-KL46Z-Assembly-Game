/*********************************************************************/
/* Ultimate Assembler Game                                           */
/* Descriptive comment header goes here.                             */
/* Name:  <Erez Binyamin>                                            */
/* Date:  <5/9/2017>		                                             */
/* Class:  CMPE 250                                                  */
/* Section:  <Your lab section, day, and time here>                  */
/*********************************************************************/
/* imports */
#include "Header.h" 

/* Equates */
#define FALSE    	 (0)
#define TRUE      	 (1)
#define MAX_STRING (79)

//Game Dynamics
#define NUM_QUESTIONS (16)
#define NUM_LEVELS (7)
#define NUM_LEVEL_PARTS (2)

/* static constatnts */

const static char rules1[] = "Welcome to the Ultimate Assembler!! \r\n\0";
const static char foo[] = "In this game you will be given the Register Transfer Notaiton, RTN, for an ARM assembly instruction \r\n\0";
const static char foo1[] = "You will then have to type the corresponding ARM assembly instruction and press ENTER \r\n\0";
const static char rules2[] = "Be careful though... \r\n \0";
const static char foo2[] = "There will be a timer running! If you run out of time you Loose a life!\r\n\0";
const static char rules3[] = "In short... \r\n *You'll get some RTN \r\n *You must enter the correct ARM instruction \r\n *You have 3 lives \r\n *Each level the time per question decreaces \r\n\0";

const static char *questions[NUM_QUESTIONS] = {"Ra <- ZeroExt(Imm8) \0", "Ra <- ~Rb \0", "Rj <- RSpecial \0", "RSpecial <- Rj \0", "Rm - Rn \0",
"Ra + Rb \0", "Ra <- 0 - Rb \0", "Rx <- Rx + Ry \0", "Ra <- Rb + Rc \0", "Ra <- Ra + Rb + C \0", "SP <- SP + ZeroExt(Imm9) \0", "Ra <- Rb - Rc \0",
"Ra <- Ra - Rb - C \0", "Ra <- Ra & Rb \0", "Ra <- Ra & ~Rb \0", "Ra <- Ra | Rb \0"};

const static char *answers[NUM_QUESTIONS] = {"MOVS\0", "MVNS\0", "MRS\0", "MSR\0", "CMP\0", "CMN\0", "RSBS\0", 
"ADD\0", "ADDS\0", "ADCS\0", "SUB\0", "SUBS\0", "SBCS\0", "ANDS\0", "BICS\0", "ORRS\0"};

//Time in seconds alloted for each level
const static int times[] = {30, 20, 15, 10, 5, 1, 0};

//Rank achieved for finishing at a given level
const static char *ranks[] = {"Retake DSD1 \0", "TA skillz \0", "Richard Tolleson \0", "Shanchieh Jay Yang \0", "Roy Melton \0",
"Derek Freeman \0", "Erez Binyamin \0"};

const static char newLine[] = "\r\n\0";
const static char wrong[] = "Incorrect!\0";
const static char right[] = "Correct!\0";
const static char outOfTimeError[] = "Out of Time! You lose a life!\0";
const static char levelPrompt[] = "Good luck! You're on level :\0";
const static char gameOver1[] = "Game Over! You made it to level : \0";
const static char gameOver2[] = "And achieved a rank of : \0";
const static char CurLife[] = "Life : \0";

/* static variables */
int life = 3;
int level = 0;
int part_Level = 0;
int current = 0;
int score = 0;

//Timing
int timeLimit;

//User Input
char input[MAX_STRING];

//Temp/loop valriables
int loop = FALSE;
int temp = FALSE;
int same = FALSE;
int outOfTime = FALSE;
int i = 0;
char newChar = '0';
int done = FALSE;

//Solved Array
int solved[NUM_QUESTIONS] = {0};
int place = 0;

//<< Subroutine Code Here >>

void printRules()
{
	PutStringSB ((char *)rules1, sizeof (rules1));
	PutStringSB ((char *)foo, sizeof (foo));
	PutStringSB ((char *)foo1, sizeof (foo1));
	PutStringSB ((char *)rules2, sizeof (rules2));
	PutStringSB ((char *)rules3, sizeof (rules3));
	PutStringSB ((char *)foo2, sizeof (rules3));
}

void timeout()
{
	for(i=0;i<99999;i=i+1){}
}

extern int Count;
int main (void) {   
  
  /* >>>>> Initialization Calls <<<<< */    
  
  __asm("CPSID   I");  /* mask interrupts */
  
	Init_UART0_IRQ ();
	Init_LED_IRQ ();
	Init_PIT_IRQ ();
	
  __asm("CPSIE   I");  /* unmask interrupts */

  /* >>>>> Remaining program code here <<<<< */
	
 printRules();
 StartTime();
   for(;;)
   {
	   if(done == TRUE)
	   {
		   for(;;){}
	   }
	   PutStringSB ((char *)newLine, sizeof (newLine));
	   PutStringSB ((char *)levelPrompt, sizeof (levelPrompt));
	   PutChar (level + '0' + 1);
	   PutStringSB ((char *)newLine, sizeof (newLine));
	   
	   PutStringSB ((char *)CurLife, sizeof (CurLife));
	   PutChar (life + '0');
	   PutStringSB ((char *)newLine, sizeof (newLine));
	   
	   //Get Next question
	   loop = FALSE;
	   while(loop == FALSE)
	   {
		   //RANDOM NUMBER
		   current = Count % NUM_QUESTIONS;
		   
		   //.Contains() method
		   temp = TRUE;
		   for(i=0; i<sizeof(solved); i=i+1)
		   {
			   if(solved[i] == current)
			   {
				   temp = FALSE;
				   break;
			   }
		   }
		   
		   if(temp == TRUE)
		   {
			   *(solved + place) = current;
			   place = place + 1;
			   loop = TRUE;
		   }
	   }
	   
	   //Set Count to 0;
	   timeLimit = times[level];
	   ClearTime();
	   
	   //User input loop
	   while(Count < timeLimit * 100)
	   {
		   PutStringSB ((char *)questions[current], MAX_STRING);
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   
		   //Get input
		   GetStringSB ((char *) input, MAX_STRING);
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   
		   //Check Answer
		   same = SameStringSB((char *)answers[current], input);
		   REDLED_OFF();
		   GREENLED_OFF();
		   
		   
		   //INCORRECT
		   if(same == FALSE)
		   {
			   REDLED_ON();
			   PutStringSB ((char *)wrong, sizeof (wrong));
			   life = life - 1;
		   }
		   //CORRECT
		   else
		   {
			   GREENLED_ON();
			   PutStringSB ((char *)right, sizeof (right));
			   same = TRUE;
			   break;
		   }
		   PutStringSB ((char *)newLine, sizeof (newLine));
	   }
	   
	   //Out of Time
	   if(same == FALSE)
	   {
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   //timeout();
		   PutStringSB ((char *)outOfTimeError, sizeof (outOfTimeError));
		   //timeout();
		   life = life - 1;
		   
		   PutStringSB ((char *)newLine, sizeof (newLine));
	   }
	   
	   part_Level = part_Level + 1;
	   if(part_Level > NUM_LEVEL_PARTS)
	   {
		   part_Level = 0;
		   level = level + 1;
	   }
	   
	   if(life <= 0)
	   {
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   PutStringSB ((char *)gameOver1, sizeof (gameOver1));
		   PutChar (level + '0' + 1);
		   PutStringSB ((char *)newLine, sizeof (newLine));
		  
		   PutStringSB ((char *)gameOver2, sizeof (gameOver2));
		   PutStringSB ((char *)ranks[level], MAX_STRING);
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   done = TRUE;
	   }
   }
  
  return (0);
} 
/* main */

