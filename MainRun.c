/*********************************************************************/
/* Ultimate Assembler Game                                           */
/* Descriptive comment header goes here.                             */
/* Name:  <Erez Binyamin>                                            */
/* Date:  <5/9/2017>		                                        		 */
/* Class:  CMPE 250                                                  */
/* Section:  <Your lab section, day, and time here>                  */
/*********************************************************************/
/* imports */
#include "Header.h" 

/* Equates */
#define FALSE    	 (0)
#define TRUE      	 (1)
#define MAX__STRING (10)

//Game Dynamics
#define NUM_QUESTIONS (16)
#define NUM_LEVELS (7)
#define NUM_LEVEL_PARTS (3)

/* static constatnts */
const static String rules1 = "Welcome to the Ultimate Assembler!! \r\n In this game you will be given the Register Transfer Notaiton (RTN) for an ARM assembly instruction. \r\n \r\n You will then have to type the corresponding ARM assembly instruction and press 'ENTER' \r\n ";
const static String rules2 = "Be careful though... \r\n If you get the answer wrong you will lose a life! \r\n and there will be a timer running! \r\n";
const static String rules3 = "In short... \r\n *You'll get some RTN \r\n *You must enter the correct ARM instruction \r\n *You have 5 lives \r\n *Each level the time per question decreaces";

const static String questions[] = {"Ra <- ZeroExt(Imm8)", "Ra <- ~Rb", "Rj <- RSpecial", "RSpecial <- Rj", "Rm - Rn",
"Ra + Rb", "Ra <- 0 - Rb", "Rx <- Rx + Ry", "Ra <- Rb + Rc", "Ra <- Ra + Rb + C", "SP <- SP + ZeroExt(Imm9)", "Ra <- Rb - Rc",
"Ra <- Ra - Rb - C", "Ra <- Ra & Rb", "Ra <- Ra & ~Rb", "Ra <- Ra | Rb"};

const static String answers[] = {"MOVS", "MVNS", "MRS", "MSR", "CMP", "CMN", "RSBS", "ADD", "ADDS", "ADCS", "SUB", 
"SUBS", "SBCS", "ANDS", "BICS", "ORRS"};

//Time in seconds alloted for each level
const static int times[] = {60, 40, 20, 10, 5, 1, 0};

//Rank achieved for finishing at a given level
const static String ranks[] = {"Retake DSD1", "TA skillz", "Richard Tolleson", "Shanchieh Jay Yang", "Roy Melton",
"Derek Freeman", "Erez Binyamin"};

const static String newLine = "\r\n";
const static String wrong = "Incorrect! You a life!";
const static String right = "Correct!";
const static String outOfTimeError = "Out of Time! You lose a life!";
const static String levelPrompt = "Good luck! You're on level :";
//const static String levelPrompt = "Game Over! You made it to level :";

/* static variables */
int life = 5;
int level = 0;
int part_Level = 0;
int current = 0;
int score = 0;

//Timing
int sysTime;
int timeLimit;

//User Input
String input;

//Temp/loop valriables
int loop = FALSE;
int temp = FALSE;
int same = FALSE;
int outOfTime = FALSE;
int i = 0;

//Solved Array
int solved[] = {0};
int place = 0;

//Timer
int systemTime = 0;

//<< Subroutine Code Here >>

void printRules()
{
	PutStringSB ((char *)rules1, sizeof (rules1));
	PutStringSB ((char *)rules2, sizeof (rules2));
	PutStringSB ((char *)rules3, sizeof (rules3));
}

  
int main (void) {   
  
  /* >>>>> Initialization Calls <<<<< */    
  
  __asm("CPSID   I");  /* mask interrupts */
  Startup ();
  Init_UART0_IRQ ();
	Init_LED_IRQ ();
	Init_PIT_IRQ ();
  __asm("CPSIE   I");  /* unmask interrupts */

  /* >>>>> Remaining program code here <<<<< */
  printRules();
   do 
   {
	   PutStringSB ((char *)levelPrompt, sizeof (levelPrompt));
	   PutChar (level + '0');
	   PutStringSB ((char *)newLine, sizeof (newLine));
	   
	   //Get Next question
	   while(loop == FALSE)
	   {
		   systemTime = GetTime();
		   current = systemTime % NUM_QUESTIONS;
		   
		   //.Contains() method
		   temp = TRUE;
		   for(i=0; i<sizeof(solved); i=i+1)
		   {
			   if(solved[i] == current)
			   {
				   temp = FALSE;
			   }
		   }
		   
		   if(temp == TRUE)
		   {
			   solved[place] = current;
			   place = place + 1;
			   loop = TRUE;
		   }
	   }
	   
	   
	   //Figure out how to get SysTime
	   //TODODODODODODODODODODODODODO
	   sysTime = GetTime();
	   
	   sysTime = 0;
	   timeLimit = times[level];
	   
	   while(sysTime < timeLimit)
	   {
		   //Prompt user
		   PutStringSB ((char *)questions[current], sizeof (questions[current]));
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   
		   //Get input
		   GetStringSB ((char *)input, MAX__STRING);
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   
		   //Check Answer
		   same = SameStringSB((char *)answers[current], (char *)input);
		   
		   //INCORRECT
		   if(same == FALSE)
		   {
			   PutStringSB ((char *)wrong, sizeof (wrong));
			   life = life - 1;
		   }
		   //CORRECT
		   else
		   {
			   PutStringSB ((char *)right, sizeof (right));
			   same = TRUE;
		   }
		   PutStringSB ((char *)newLine, sizeof (newLine));
	   }
	   
	   //Out of Time
	   if(same == FALSE)
	   {
		   PutStringSB ((char *)newLine, sizeof (newLine));
		   
		   PutStringSB ((char *)outOfTimeError, sizeof (outOfTimeError));
		   life = life - 1;
		   
		   PutStringSB ((char *)newLine, sizeof (newLine));
	   }
	   
	   part_Level = part_Level + 1;
	   if(part_Level > NUM_LEVEL_PARTS)
	   {
		   part_Level = 0;
		   level = level + 1;
	   }
   }
  while (life > 0);
  
  PutStringSB ((char *)ranks[level], sizeof (ranks[level]));
  PutChar (level + '0');
 
  PutStringSB ((char *)newLine, sizeof (newLine));
  
  return (0);
} 
/* main */

