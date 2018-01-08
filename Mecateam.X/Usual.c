/**************************************************************************************************
  Nome do arquivo:      Usual.c
  Revisado:    		    $Data:$
  Versão:       		$Revisão:$

  Descrição:    		Este arquivo contém funções auxiliares de propsito geral
  Notas:          		Está biblioteca é generica para C, não utiliza nenhum item especifico de qualquer
   	   	   	   	   	    Microcontrolador, portanto é potável a qualquer plataforma



  **************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "Usual.h"
/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/



/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/


/**************************************************************************************************
 *                                        EXTERNAL VARIABLES
 **************************************************************************************************/
unsigned char Modo_de_operacao;
unsigned char Modo_de_operacao_Zigbee;
unsigned char Modo_de_operacao_GSM;
const unsigned char Sucess = 1;
const unsigned char Fail = 0;
/**************************************************************************************************
 *                                        		Funções
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      util_memcmp
 *
 * @Descrição	Compara dois vetores. Caso seja iguais retorna 1, caso contrário retorna 0.
 * 				A comparação se dá do primeiro para o último elemento
 *
 *
 *
 * @param   src1 - endereço da primeira posição do primeiro vetor
 * @param   src2 - endereço da primeira posição do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser comparado
 *
 * @return  Sucess (1) - igual, Fail (0) - diferente
 */
unsigned char util_memcmp(const unsigned char *src1,unsigned char *src2,unsigned char len )
{
 unsigned int count = 0;

  while ( len-- )
  {
    if( *(src1+count) != *(src2+count) )return Fail;
    count++;
  }
  return Sucess;
}

/**************************************************************************************************
 * @fn      util_memcmp_inv
 *
 * @Descrição	Compara dois vetores. Caso seja iguais retorna 1, caso contrário retorna 0.
 * 				A comparação se dá do último elemento até o elemento apontado pelo tamanho que deseja-se
 * 				averiguar
 *
 *
 *
 * @param   src1 - endereço da primeira posição do primeiro vetor
 * @param   src2 - endereço da primeira posição do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser comparado
 *
 * @return  Sucess (1) - igual, Fail (0) - diferente
 */
unsigned char util_memcmp_inv(const unsigned char *src1,unsigned char *src2,unsigned char len )
{
  while ( len-- )
  {
    if( *(src1+len) != *(src2+len) )return Fail;
  }
  return Sucess;
}

/**************************************************************************************************
 * @fn      util_memcpy
 *
 * @Descrição	Copia o conteudo de um vetor para outro.
 * 				A cópia se dá da primeira até a última posição apontada pelo tamanho que se deseja copiar.
 *
 *
 *
 * @param   src1 - endereço da primeira posição do primeiro vetor
 * @param   src2 - endereço da primeira posição do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser copiado
 *
 * @return  Sucess (1)
 */
unsigned char util_memcpy(unsigned char *pDestination, const unsigned char *pSource, unsigned char len)
{


	unsigned int count = 0;
	while ( len-- )
	{
		*(pDestination+count) = *(pSource+count);
		count++;
	}

	return Sucess;
}



unsigned char util_memcpyAcrescentaZero(unsigned char *pDestination, const unsigned char *pSource, unsigned char len)
{


	unsigned int count = 0;
	while ( len-- )
	{
		if ( *(pSource+count) == 0)*(pDestination+count) = 0x30;
		else *(pDestination+count) = *(pSource+count);
		count++;
	}

	return Sucess;
}


/**************************************************************************************************
 * @fn      util_memcpy_inv
 *
 * @Descrição	Copia o conteudo de um vetor para outro.
 * 				A cópia se dá da última até a primeira posição apontada pelo tamanho que se deseja copiar.
 *
 *
 *
 * @param   src1 - endereço da primeira posição do primeiro vetor
 * @param   src2 - endereço da primeira posição do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser copiado
 *
 * @return  Sucess (1)
 */
unsigned char util_memcpy_mirror(unsigned char *pDestination, const unsigned char *pSource, unsigned char len)
{

	int count =0;

	while ( len-- )
	{
		*(pDestination+count) = *(pSource+len);
		count++;

	}

	return Sucess;
}

unsigned char util_memcpy_inv(unsigned char *pDestination, const unsigned char *pSource, unsigned char len)
{



	while ( len-- )
	{
		*(pDestination+len) = *(pSource+len);

	}

	return Sucess;
}

/**************************************************************************************************
 * @fn      util_clear
 *
 * @Descrição
 *
 *   Coloca Zero da primeira posição do vetor até a posição apontada pelo amanho passado
 *
 * @param   src1 - endereço da primeira posição do vetor
 * @param   src2 - source 2 address
 * @param   len - tamanho do vetor, ou do bloco a ser apagado
 *
 * @return  Sucess (1)
 */
unsigned char util_clear(unsigned char *pDestination, unsigned char len)
{



	while ( len-- )
	{
		*(pDestination+len) = 0;

	}

	return Sucess;
}
/**************************************************************************************************
 * @fn      util_search
 *
 * @Descrição
 *
 *   Busca num vetor um trecho igual ao outro vetor
 *
 * @param   src1 - endereço da primeira posição do vetorque se deseja realizar a busca
 * @param   src2 - endereço da primeira posição do vetor utilizado como referencia na busca
 * @param   len - tamanho do vetor, utilizado como eferencia da comparação
 *
 *  * @return  Sucess (1) - Se encontrou , Fail (0) - se não encontrou
 */
unsigned char util_search(unsigned char *src1,const unsigned char *src2, int len1,int len2){


	unsigned char j;



	for (j = 0; j <= (len1-len2);  j++) {




		    if(util_memcmp((src2),(src1+j),len2) == Sucess)return Sucess;


	}

	return Fail;
}
