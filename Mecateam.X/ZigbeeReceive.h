/*
 * ZigbeeReceive.h
 *
 *  Created on: 14/11/2013
 *      Author: jovelino.torres
 */

extern unsigned int FrameLength;
extern unsigned char BufferProcessing[127];
extern unsigned char BufferTransmit[127];
struct tCabecalhoDoFrame{
	unsigned char StarDelimiter ;			// Indica o inicio do frame
	unsigned char Length[2];				// Tamanho do frame
	unsigned char FrameType;				// Indica que o frame � de transmiss�o de dado para outro dispositivo
	unsigned char FrameId;					// Solicita um ACK
	unsigned char ExterAddress[8]; 			// Endere�o de 64 bits do destino
	unsigned char NtwAddress[2]; 			// Endere�o de 16 dit do destino. 0xFFFE indica que n�o � conhecido
	unsigned char BroadCast;				// Configura o n�mero m�ximo de pulos de um dado broadcast
	unsigned char Options;					// Configura algumas op��es da rede, como a critografia
	unsigned char Command;					// Indica o comando recebido. Verificar manua descritivo da fase basica.
	unsigned char checksum;					// Checksum do frame
};
extern struct tCabecalhoDoFrame CabecalhoDoFrame;
/**************************************************************************************************
 * @fn      	FrameProcessing()
 *
 * @Descri��o   Esta fun��o retira o frame da pilha, verifica o checksum e chama fun��o de tratamento
 * 				adequada de acordo com o Frame Type
 *
 *
 * @parametros  nenhum
 *
 * @retorno  	nenhum
 **************************************************************************************************/
void FrameProcessing(void);
/**************************************************************************************************
 * @fn      	FrameReceivedProcessing()
 *
 * @Descri��o   Esta fun��o trata o frame de acordo com o campo command, que fica no campo data. Esta
 * 				command � um comando feito especificamente para o projeto Bratac.
 * 				Esta fun��o envia uma resposta ao remetente de acondo com o campo command.
 * 				adequada de acordo com o Frame Type
 *
 *
 * @parametros unsigned char *pBuffer - ponteiro par ao buffer com o dado que foi retirado da pilha
 * 			   unsigned char length - tamanho do buffer com o dado que foi retirado da pilha
 *
 * @retorno  	nenhum
 **************************************************************************************************/
void FrameReceivedProcessing(unsigned char *pBuffer, unsigned char length);
