/*
 * ZigbeeReceive.c
 *
 *  Created on: 14/11/2013
 *      Author: jovelino.torres
 */
#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition   */

#include "ZigbeeReceive.h"
#include "ZigbeeTransmit.h"
#include "system.h"
#include "uart.h"
//#include "timer.h"
#include "Usual.h"
#include "user.h"


unsigned int FrameLength;
unsigned char BufferProcessing[127];
unsigned char BufferTransmit[127];

struct tCabecalhoDoFrame CabecalhoDoFrame;

/**************************************************************************************************
 * @fn      	FrameProcessing()
 *
 * @Descrição   Esta função retira o frame da pilha, verifica o checksum e chama função de tratamento
 * 				adequada de acordo com o Frame Type
 *
 *
 * @parametros  nenhum
 *
 * @retorno  	nenhum
 **************************************************************************************************/

void FrameProcessing(void)
{
	unsigned int n;

	int resultchecksum;

	/*
	 * Entra na seção critica
	 */
	EnterCriticalSection;
	FrameLength = uartBufferZigbeeRx[uartPilhaZigbee-1][1]*256 + uartBufferZigbeeRx[uartPilhaZigbee-1][2] +3;

	for (n = 0; n <= FrameLength; n++) {

		 BufferProcessing[n] = uartBufferZigbeeRx[uartPilhaZigbee-1][n];		// Pega o frame da pilha
	}
				uartPilhaZigbee--;

	OutCriticalSection;
	/*
	 * Fim da seção critica
	 */

	/*
	 * Verifica o checksum, se estiver correto chama a função de tratamento de acordo com
	 * o frame Identifier
	 */

	resultchecksum = CheckSum(&BufferProcessing[3],FrameLength-3);
	if (resultchecksum == BufferProcessing[FrameLength] ) {


	switch (BufferProcessing[3]) {

					/*
					 * Resposta a um comando AT
					 */
					case 0x88:

						break;
					/*
					 * Status do modem
					 */
					case 0x8A:

						break;

					/*
					 * Status de um dado transmitido
					 */
					case 0x8B:
							
						break;
					/*
					 * Recepção de um dado
					 */
					case 0x90:
                                                fSend =1;
						FrameReceivedProcessing(BufferProcessing,FrameLength);
						break;
					default:
						break;
				}
	}

}

/**************************************************************************************************
 * @fn      	FrameReceivedProcessing()
 *
 * @Descrição   Esta função trata o frame de acordo com o campo command, que fica no campo data. Esta
 * 				command é um comando feito especificamente para o projeto Bratac.
 * 				Esta função envia uma resposta ao remetente de acondo com o campo command.
 * 				adequada de acordo com o Frame Type
 *
 *
 * @parametros unsigned char *pBuffer - ponteiro par ao buffer com o dado que foi retirado da pilha
 * 			   unsigned char length - tamanho do buffer com o dado que foi retirado da pilha
 *
 * @retorno  	nenhum
 **************************************************************************************************/
void FrameReceivedProcessing(unsigned char *pBuffer, unsigned char length)
{
	unsigned char count;
        char direcao1, direcao2, direcao3;
	/*
	 * Cabeçalho do frame que será enviado como resposta.
	 * Este cabeçalho é padrão, apenas os campos de dado, comando, status e
	 * tamanho que variam com a resposta.
	 */
	CabecalhoDoFrame.StarDelimiter = 0x7E;
	CabecalhoDoFrame.FrameType = 0x10;
	CabecalhoDoFrame.FrameId = 0x00;
	CabecalhoDoFrame.NtwAddress[0] = 0xFF;
	CabecalhoDoFrame.NtwAddress[1] = 0xFE;
	CabecalhoDoFrame.BroadCast = 0x00;
	CabecalhoDoFrame.Options = 0x00;


	CabecalhoDoFrame.Command = *(pBuffer + 15);							// Guarda o comando recebido
	util_memcpy(CabecalhoDoFrame.ExterAddress, (pBuffer+4),8);	// Guarda o endereço do zigbee que enviou.

	switch (CabecalhoDoFrame.Command) {
		/*
		 * Envio das leituras.
		 * Os campos com as infomrações já devem estar preenchidas.
		 */
		case 0x01:

                    CabecalhoDoFrame.Length[0] = 0x00;
                    CabecalhoDoFrame.Length[1] = 0x21;

                    length = ZigbeeTransmitRead(&CabecalhoDoFrame.StarDelimiter, 18,Serial.Vel_1, 18,BufferTransmit);

                    ZigbeeSend(BufferTransmit,length);

                    //fInit =1;

                    break;
		/*
		 * Envio contínuo de leituras
		 */
		case 0x02:

                    fPID = 0;
                    fLQR = 0;
                    fOtimo = 0;
                    Motor_1.Direcao = (*(pBuffer + 19) & BV(0));
                    Motor_2.Direcao = (*(pBuffer + 19) & BV(1));
                    Motor_3.Direcao = (*(pBuffer + 19) & BV(2));
                    
                    if (Motor_1.Direcao == 0x01)mPORTEClearBits(BIT_3);
                    else mPORTESetBits(BIT_3);

                    if (Motor_2.Direcao == 0x02)mPORTEClearBits(BIT_2);
                    else mPORTESetBits(BIT_2);

                    if (Motor_3.Direcao == 0x04)mPORTEClearBits(BIT_4);
                    else mPORTESetBits(BIT_4);


                    OC3RS = *(pBuffer + 16);
                    OC2RS = *(pBuffer + 17);
                    OC4RS = *(pBuffer + 18);

                    break;
		/*
		 * Referencias do PID
		 */
		case 0x03:

                    fPID = 1;
                    fLQR = 0;
                    fOtimo = 0;

                    Motor_1.Direcao = (*(pBuffer + 22) & BV(0));
                    Motor_2.Direcao = (*(pBuffer + 22) & BV(1));
                    Motor_3.Direcao = (*(pBuffer + 22) & BV(2));

                    Motor_1.VReferencia = *(pBuffer + 16)*256 + *(pBuffer + 17);
                    Motor_2.VReferencia = *(pBuffer + 18)*256 + *(pBuffer + 19);
                    Motor_3.VReferencia = *(pBuffer + 20)*256 + *(pBuffer + 21);
                    if (Motor_1.Direcao == 0x01)Motor_1.VReferencia = -1*Motor_1.VReferencia;
                    if (Motor_2.Direcao == 0x02)Motor_2.VReferencia = -1*Motor_2.VReferencia;
                    if (Motor_3.Direcao == 0x04)Motor_3.VReferencia = -1*Motor_3.VReferencia;

                    break;
		/*
		 * Parametros do PID
		 */
		case 0x04:

                    fPID = 1;
                    fLQR = 0;
                    fOtimo = 0;

                    Motor_1.Kp = (float)*(pBuffer + 16)/100.00;
                    Motor_2.Kp = (float)*(pBuffer + 16)/100.00;
                    Motor_3.Kp = (float)*(pBuffer + 16)/100.00;
                    Motor_1.Ki = (float)*(pBuffer + 17)/100.00;
                    Motor_2.Ki = (float)*(pBuffer + 17)/100.00;
                    Motor_3.Ki = (float)*(pBuffer + 17)/100.00;
                    Motor_1.Kd = (float)*(pBuffer + 18)/100.00;
                    Motor_2.Kd = (float)*(pBuffer + 18)/100.00;
                    Motor_3.Kd = (float)*(pBuffer + 18)/100.00;

                    break;
		/*
		 * Referencias do LQR Smith
		 */
		case 0x05:

                    fPID = 0;
                    fLQR = 1;
                    fOtimo = 0;

                    direcao1 = (*(pBuffer + 22) & BV(0));
                    direcao2 = (*(pBuffer + 22) & BV(1));
                    direcao3 = (*(pBuffer + 22) & BV(2));

                    V_ref = (float)(((float)*(pBuffer + 16))*256 + ((float)*(pBuffer + 17)))/100.0;
                    Vn_ref = (float)(((float)*(pBuffer + 18))*256 + ((float)*(pBuffer + 19)))/100.0;
                    W_ref = (float)(((float)*(pBuffer + 20))*256 + ((float)*(pBuffer + 21)))/100.0;

                    if(direcao1 == 0x01) V_ref = -1*V_ref;
                    if(direcao2 == 0x02) Vn_ref = -1*Vn_ref;
                    if(direcao3 == 0x04) W_ref = -1*W_ref;

                    break;
		/*
		 * Configuração do comprimento do fushi
		 */
		case 0x06:

                    fPID = 0;
                    fLQR = 0;
                    fOtimo = 1;

                    direcao1 = (*(pBuffer + 22) & BV(0));
                    direcao2 = (*(pBuffer + 22) & BV(1));
                    direcao3 = (*(pBuffer + 22) & BV(2));

                    V_ref = (float)(((float)*(pBuffer + 16))*256 + ((float)*(pBuffer + 17)))/100.0;
                    Vn_ref = (float)(((float)*(pBuffer + 18))*256 + ((float)*(pBuffer + 19)))/100.0;
                    W_ref = (float)(((float)*(pBuffer + 20))*256 + ((float)*(pBuffer + 21)))/100.0;

                    if(direcao1 == 0x01) V_ref = -1*V_ref;
                    if(direcao2 == 0x02) Vn_ref = -1*Vn_ref;
                    if(direcao3 == 0x04) W_ref = -1*W_ref;

			break;
		/*
		 * Configuração do relógio
		 */
		case 0x07:
			break;
		/*
		 * Configuração do nome do sensor
		 */
		case 0x08:
			break;
		/*
		 * Calibração
		 */
		case 0x09:
			break;


		default:

			break;
	}
}


