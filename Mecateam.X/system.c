/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "system.h"          /* variables/params used by system.c             */

/******************************************************************************/
/* System Level Functions                                                     */
/*                                                                            */
/* Custom oscillator configuration funtions, reset source evaluation          */
/* functions, and other non-peripheral microcontroller initialization         */
/* functions get placed in system.c                                           */
/*                                                                            */
/******************************************************************************/

/* <Initialize variables in system.h and put code for system algorithms here.>*/



int CheckSum(unsigned char *pBuffer, int length)
{
	unsigned int n;
	unsigned int result;

	result = 0;

	for (n = 0; n < length; n++){
		result	= result + *(pBuffer+n);
	}




	return 0xFF - (result & 0x00FF);

}