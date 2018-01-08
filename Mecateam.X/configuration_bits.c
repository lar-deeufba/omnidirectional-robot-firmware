/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* Refer to 'C32 Configuration Settings' under the Help > Contents            */
/* > C32 Toolchain in MPLAB X IDE for available PIC32 Configurations.  For    */
/* additional information about what the hardware configurations mean in      */
/* terms of device operation, refer to the device datasheet 'Special Features'*/
/* chapter.                                                                   */
/*                                                                            */
/******************************************************************************/

/* Fill in your configuration bits here.  The general style is shown below.
The Debug Configuration bit is handline by MPLAB and should not be embedded
in the configuration macro.*/

/* TODO Fill in your configuration bits here.  The general style is below:    */

#if 1

/* Example Syntax: */
#pragma config FPLLIDIV=DIV_2, FPLLMUL=MUL_20, FPLLODIV=DIV_1, FNOSC=PRIPLL
#pragma config POSCMOD=HS, FPBDIV=DIV_1, FCKSM=CSECME, FWDTEN=OFF
#pragma config DEBUG = OFF            // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)

//7381209021
#endif
