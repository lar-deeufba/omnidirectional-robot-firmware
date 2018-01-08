/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ     80000000L
#define FCY          SYS_FREQ

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

/* Custom oscillator configuration funtions, reset source evaluation
functions, and other non-peripheral microcontroller initialization functions
go here. */


/*
 * system.h
 *
 *  Created on: 18/11/2013
 *      Author: jovelino.torres
 */

#define EnterCriticalSection INTDisableInterrupts();;
#define OutCriticalSection INTEnableInterrupts();



typedef struct{
	unsigned char Vel_1[2];
	unsigned char Vel_2[2];
	unsigned char Vel_3[2];
	unsigned char Vel_Med_1[2];
	unsigned char Vel_Med_2[2];
	unsigned char Vel_Med_3[2];
	unsigned char Current_1[2];
	unsigned char Current_2[2];
	unsigned char Current_3[2];
	unsigned char Current_Media_1[2];
	unsigned char Current_Media_2[2];
	unsigned char Current_Media_3[2];
	unsigned char Acer_x[2];
	unsigned char Acer_y[2];
	unsigned char Girosc[2];
        unsigned char Compass[2];
        unsigned char Dir;

}sPortaSerial;

extern sPortaSerial PortaSerial;


int CheckSum(unsigned char *pBuffer, int length);
