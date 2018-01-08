#include "ZigbeeTransmit.h"
#include "system.h"
#include "uart.h"
#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "user.h"            /* variables/params used by user.c               */

//#include "timer.h"

unsigned char BufferContinuousSend[127];

/**************************************************************************************************
 * @fn      	ZigbeeTransmitRead()
 *
 * @Descrição   Monta o frame de envio com o checksum
 *
 *
 * @parametros unsigned char *pCabecalho - ponteiro para a estrutura que possui o cabeçalho do frame
 * 			   unsigned char LengthCabecalho - tamanho da estrutura que contém o cabeçalho
 * 			   unsigned char *pData - Ponteiro para o campo de dados
 * 			   unsigned char LengthData - tamanho do campo de dados
 * 			   unsigned char *pDestination - Ponteiro para o buffer de destino
 *
 * @retorno  	unsigned char  - tamanho do frame que será enviado ao zigbee
 **************************************************************************************************/
unsigned char ZigbeeTransmitRead(unsigned char *pCabecalho, unsigned char LengthCabecalho, unsigned char *pData, unsigned char LengthData, unsigned char *pDestination)
{
	unsigned int i;
	unsigned char Length;

	for (Length = 0; Length < LengthCabecalho; Length++) {			// Armazena o frame no buffer de envio
		*(pDestination +Length) = *(pCabecalho +Length);
	}


	if(LengthData){													// Se o campo de dados não está vazio, armazena no buffer
		i = 0;
		for (Length = LengthCabecalho; Length < LengthData + LengthCabecalho; Length++) {
			*(pDestination + Length) = *(pData + i);
			i++;
		}
	}

	Length = LengthCabecalho+LengthData;							// Pega o tamnho do frame
	*(pDestination + Length) = (char)CheckSum((pDestination+3),LengthCabecalho+LengthData-3); // Calcula o cecksum

	return Length;
}

/**************************************************************************************************
 * @fn      	ZigbeeSend()
 *
 * @Descrição   Coloca o frame no buffer de transmissão da porta UART do zigbee
 *
 *
 * @parametros unsigned char *pData - ponteiro para o frame que será enviado
 * 			   unsigned char length - tamanho do frame
 *
 * @retorno  	nenhum
 **************************************************************************************************/
void ZigbeeSend(unsigned char *pData, unsigned char length){

	for (uartBufferTxSize = 0; uartBufferTxSize <= length; uartBufferTxSize++) {
		//uartBufferTx[uartBufferTxSize] = *(pData + uartBufferTxSize);
                putcUART3(*(pData + uartBufferTxSize));

	}

	//uart1ConterTx = 0;

//	UCA1IE |= UCTXIE;

}

/**************************************************************************************************
 * @fn      	ZigbeeTransmitContinuous()
 *
 * @Descrição   Realiza o envio continuo do diametro do fio para o sistema supervisório.
 * 				Esta função incrementa o contador TransmitContinuousCount até 54000, o que
 * 				corresponde a 3 horas de opração.
 * 				A cada chamada desta função ela envia um frame e ativa um tic do timer para
 * 				200 ms para ser chamada novamente.
 *
 *
 * @parametros nenhum
 *
 * @retorno  	nenhum
 **************************************************************************************************/
void ZigbeeTransmitContinuous(void){

		ZigbeeSend(BufferContinuousSend, 28);
		
}
