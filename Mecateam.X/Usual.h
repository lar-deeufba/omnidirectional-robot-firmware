/**************************************************************************************************
  Nome do arquivo:      Usual.c
  Revisado:    		    $Data:$
  Vers�o:       		$Revis�o:$

  Descri��o:    		Este arquivo cont�m fun��es auxiliares de propsito geral
  Notas:          		Est� biblioteca � generica para C, n�o utiliza nenhum item especifico de qualquer
   	   	   	   	   	    Microcontrolador, portanto � pot�vel a qualquer plataforma



  **************************************************************************************************/


#ifndef USUAL_H_
#define USUAL_H_


/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/
#define BV(n)      (1 << (n))
#define TRUE        1
#define FALSE       0


#define IDENTIFICACAO_MODO       0
//#define PERIFERICOS 			 1
#define COORDENADOR              1
#define SENSOR_A                 2
#define SENSOR_B                 3
#define ROUTER                   4
#define MODO_SLEEP               5
#define MODO_NAO_IDENTICADO      6
#define MODO_MANUTENCAO          7
#define PREPARACAO_SLEEP         8
#define MODO_RTC_MANUTENCAO      9
#define MODO_RTC_SLEEP          10


#define Comando_01               1
#define Comando_02               2
#define Comando_03               3
#define Comando_04               4
#define Comando_05               5


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/


/**************************************************************************************************
 *                                        EXTERNAL VARIABLES
 **************************************************************************************************/

extern unsigned char Modo_de_operacao;
extern unsigned char Modo_de_operacao_Zigbee;
extern unsigned char Modo_de_operacao_GSM;
extern const unsigned char Sucess;
extern const unsigned char Fail;


/**************************************************************************************************
 *                                        		Fun��es
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      util_memcmp
 *
 * @Descri��o	Compara dois vetores. Caso seja iguais retorna 1, caso contr�rio retorna 0.
 * 				A compara��o se d� do primeiro para o �ltimo elemento
 *
 *
 *
 * @param   src1 - endere�o da primeira posi��o do primeiro vetor
 * @param   src2 - endere�o da primeira posi��o do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser comparado
 *
 * @return  Sucess (1) - igual, Fail (0) - diferente
 */
unsigned char util_memcmp(const unsigned char *src1,unsigned char *src2,unsigned char len );

/**************************************************************************************************
 * @fn      util_memcmp_inv
 *
 * @Descri��o	Compara dois vetores. Caso seja iguais retorna 1, caso contr�rio retorna 0.
 * 				A compara��o se d� do �ltimo elemento at� o elemento apontado pelo tamanho que deseja-se
 * 				averiguar
 *
 *
 *
 * @param   src1 - endere�o da primeira posi��o do primeiro vetor
 * @param   src2 - endere�o da primeira posi��o do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser comparado
 *
 * @return  Sucess (1) - igual, Fail (0) - diferente
 */
unsigned char util_memcmp_inv(const unsigned char *src1,unsigned char *src2,unsigned char len );

unsigned char util_memcpy_mirror(unsigned char *pDestination, const unsigned char *pSource, unsigned char len);

/**************************************************************************************************
 * @fn      util_memcpy
 *
 * @Descri��o	Copia o conteudo de um vetor para outro.
 * 				A c�pia se d� da primeira at� a �ltima posi��o apontada pelo tamanho que se deseja copiar.
 *
 *
 *
 * @param   src1 - endere�o da primeira posi��o do primeiro vetor
 * @param   src2 - endere�o da primeira posi��o do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser copiado
 *
 * @return  Sucess (1)
 */
unsigned char util_memcpy(unsigned char *pDestination, const unsigned char *pSource, unsigned char len);


unsigned char util_memcpyAcrescentaZero(unsigned char *pDestination, const unsigned char *pSource, unsigned char len);

/**************************************************************************************************
 * @fn      util_memcpy_inv
 *
 * @Descri��o	Copia o conteudo de um vetor para outro.
 * 				A c�pia se d� da �ltima at� a primeira posi��o apontada pelo tamanho que se deseja copiar.
 *
 *
 *
 * @param   src1 - endere�o da primeira posi��o do primeiro vetor
 * @param   src2 - endere�o da primeira posi��o do segundo vetor
 * @param   len - tamanho dos vetores, ou do bloco a ser copiado
 *
 * @return  Sucess (1)
 */
unsigned char util_memcpy_inv(unsigned char *pDestination, const unsigned char *pSource, unsigned char len);

/**************************************************************************************************
 * @fn      util_clear
 *
 * @Descri��o
 *
 *   Coloca Zero da primeira posi��o do vetor at� a posi��o apontada pelo amanho passado
 *
 * @param   src1 - endere�o da primeira posi��o do vetor
 * @param   src2 - source 2 address
 * @param   len - tamanho do vetor, ou do bloco a ser apagado
 *
 * @return  Sucess (1)
 */
unsigned char util_clear(unsigned char *pDestination, unsigned char len);

/**************************************************************************************************
 * @fn      util_search
 *
 * @Descri��o
 *
 *   Busca num vetor um trecho igual ao outro vetor
 *
 * @param   src1 - endere�o da primeira posi��o do vetorque se deseja realizar a busca
 * @param   src2 - endere�o da primeira posi��o do vetor utilizado como referencia na busca
 * @param   len - tamanho do vetor, utilizado como eferencia da compara��o
 *
 *  * @return  Sucess (1) - Se encontrou , Fail (0) - se n�o encontrou
 */
unsigned char util_search(unsigned char *src1,const unsigned char *src2, int len1,int len2);


#endif /* USUAL_H_ */
