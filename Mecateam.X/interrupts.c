/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <sys/attribs.h>     /* For __ISR definition                          */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "user.h"
#include "uart.h"
#include "system.h"
/******************************************************************************/
/* Interrupt Vector Options                                                   */
/******************************************************************************/
/*                                                                            */
/* VECTOR NAMES:                                                              */
/*                                                                            */
/* _CORE_TIMER_VECTOR          _COMPARATOR_2_VECTOR                           */
/* _CORE_SOFTWARE_0_VECTOR     _UART_2A_VECTOR                                */
/* _CORE_SOFTWARE_1_VECTOR     _I2C_2A_VECTOR                                 */
/* _EXTERNAL_0_VECTOR          _SPI_2_VECTOR                                  */
/* _TIMER_1_VECTOR             _SPI_2A_VECTOR                                 */
/* _INPUT_CAPTURE_1_VECTOR     _I2C_4_VECTOR                                  */
/* _OUTPUT_COMPARE_1_VECTOR    _UART_3_VECTOR                                 */
/* _EXTERNAL_1_VECTOR          _UART_2_VECTOR                                 */
/* _TIMER_2_VECTOR             _SPI_3A_VECTOR                                 */
/* _INPUT_CAPTURE_2_VECTOR     _I2C_3A_VECTOR                                 */
/* _OUTPUT_COMPARE_2_VECTOR    _UART_3A_VECTOR                                */
/* _EXTERNAL_2_VECTOR          _SPI_4_VECTOR                                  */
/* _TIMER_3_VECTOR             _I2C_5_VECTOR                                  */
/* _INPUT_CAPTURE_3_VECTOR     _I2C_2_VECTOR                                  */
/* _OUTPUT_COMPARE_3_VECTOR    _FAIL_SAFE_MONITOR_VECTOR                      */
/* _EXTERNAL_3_VECTOR          _RTCC_VECTOR                                   */
/* _TIMER_4_VECTOR             _DMA_0_VECTOR                                  */
/* _INPUT_CAPTURE_4_VECTOR     _DMA_1_VECTOR                                  */
/* _OUTPUT_COMPARE_4_VECTOR    _DMA_2_VECTOR                                  */
/* _EXTERNAL_4_VECTOR          _DMA_3_VECTOR                                  */
/* _TIMER_5_VECTOR             _DMA_4_VECTOR                                  */
/* _INPUT_CAPTURE_5_VECTOR     _DMA_5_VECTOR                                  */
/* _OUTPUT_COMPARE_5_VECTOR    _DMA_6_VECTOR                                  */
/* _SPI_1_VECTOR               _DMA_7_VECTOR                                  */
/* _I2C_3_VECTOR               _FCE_VECTOR                                    */
/* _UART_1A_VECTOR             _USB_1_VECTOR                                  */
/* _UART_1_VECTOR              _CAN_1_VECTOR                                  */
/* _SPI_1A_VECTOR              _CAN_2_VECTOR                                  */
/* _I2C_1A_VECTOR              _ETH_VECTOR                                    */
/* _SPI_3_VECTOR               _UART_4_VECTOR                                 */
/* _I2C_1_VECTOR               _UART_1B_VECTOR                                */
/* _CHANGE_NOTICE_VECTOR       _UART_6_VECTOR                                 */
/* _ADC_VECTOR                 _UART_2B_VECTOR                                */
/* _PMP_VECTOR                 _UART_5_VECTOR                                 */
/* _COMPARATOR_1_VECTOR        _UART_3B_VECTOR                                */
/*                                                                            */
/* Refer to the device specific .h file in the C32 Compiler                   */
/* pic32mx\include\proc directory for a complete Vector and IRQ mnemonic      */
/* listings for the PIC32 device.                                             */
/*                                                                            */
/* PRIORITY OPTIONS:                                                          */
/*                                                                            */
/* (default) IPL0AUTO, IPL1, IPL2, ... IPL7 (highest)                         */
/*                                                                            */
/* Example Shorthand Syntax                                                   */
/*                                                                            */
/* void __ISR(<Vector Name>,<PRIORITY>) user_interrupt_routine_name(void)     */
/* {                                                                          */
/*     <Clear Interrupt Flag>                                                 */
/* }                                                                          */
/*                                                                            */
/* For more interrupt macro examples refer to the C compiler User Guide in    */
/* the C compiler /doc directory.                                             */
/*                                                                            */
/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* TODO Add interrupt routine code here. */
// UART 2 interrupt handler
// it is set at priority level 2

//#include "user.h"           /* User funct/params, such as InitApp             */


char a;

volatile unsigned char uartBufferRx[127];
volatile unsigned char uartConter;


volatile unsigned char uartBufferZigbeeRx[2][127];
volatile unsigned char uartZigbeeConter;
volatile unsigned char uartZigbeeRxSize;
volatile unsigned char uartPilhaZigbee;

static unsigned char PosicaoAtualNaPilha;

unsigned char uartBufferTx[127];

unsigned char uart1BufferRx[127];
unsigned char uart1ConterRx;
unsigned char uart1BufferRxSize;

unsigned char uartBufferTxSize;
unsigned char uart1ConterTx;

unsigned char EstadoRecepcaoZigbee;

unsigned char uartFlags;




//unsigned int channel3;	// conversion result as read from result buffer
//unsigned int channel4;	// conversion result as read from result buffer
//unsigned int channel5;	// conversion result as read from result buffer

void __ISR(_UART_3_VECTOR, ipl7) IntUART3Handler(void)
{
    unsigned char rx;
    // Is this an RX interrupt?
    if((IFS1bits.U3RXIF == 1)){
        // Clear the RX interrupt Flag
        IFS1CLR = _IFS1_U3RXIF_MASK;
        // Echo what we just received.
        rx = (char)ReadUART3();
        //putcUART3(rx);
      /*
        // VERIFICA SE ESTA NO INICO DO PACOTE E VERIFICA SE É O CARACTTER QUE INDICA O INICIO DO PACOTE
        if(RXCount == 0 && rx == '@'){
           RXBuffer[0] = rx;
           RXCount++;
           }
        else if(RXCount > 0){
            RXBuffer[RXCount] = rx;
            RXCount++;
            }

        if(RXCount == 23){
            Flag_RXReceived = TRUE;
            RXCount = 0;
            }*/

         switch (uart1ConterRx) {
				case 0:

					uart1BufferRx[0] = rx;										// Armazena o dado recebido pela porta serial
					if (uart1BufferRx[0] == 0x7E) {										// O primeiro byte recebido tem que ser 0x7E

						 //PTimerAOutON(25, Tic_ZigbeeReceive);							// Liga o timerout com 25 ms para esperar a recepção de todo o pacote.

						uart1ConterRx++;
					}
					break;
				case 1:
					uart1BufferRx[1] = rx;										// Armazena o primeiro byte que contem o tamanho do frame
					uart1ConterRx++;
					break;
				case 2:
					uart1BufferRx[2] = rx;										// Armazena o segundo byte que contem o tamanho do frame
					uart1BufferRxSize = (256*uart1BufferRx[1] + uart1BufferRx[2]) + 3; 	// Guarda o tamanho do Frame
					uart1ConterRx++;
					break;
				default:
					uart1BufferRx[uart1ConterRx] = rx;
					if (uart1ConterRx == uart1BufferRxSize) {							// Se o frame foi totalmente recebido, configura as flags e pula para próxima posição da pilha.

						 //PTimerAOutOFF(Tic_ZigbeeReceive);								// Desliga o timerout.

						/*
						 * Entra na seção critica
						 */
						EnterCriticalSection;
                                                 INTDisableInterrupts();
						PosicaoAtualNaPilha = uartPilhaZigbee;							// Pega a posição na pilha para armazenar o dado
						if (uartPilhaZigbee >= 2) PosicaoAtualNaPilha = 0;				// Se a pilha estiver cheia, sobrescreve a primeira posição;
						for (uart1ConterRx = 0; uart1ConterRx <= uart1BufferRxSize; uart1ConterRx++) {
							uartBufferZigbeeRx[PosicaoAtualNaPilha][uart1ConterRx] = uart1BufferRx[uart1ConterRx]; // Armazena o frame na pilha a espera de tratamento.

						}
						uartPilhaZigbee++;

						OutCriticalSection;
						/*
						 * Fim da seção critica
						 */
						uart1ConterRx = 0;


					}

					else uart1ConterRx++;
					break;
			}



     }

       



    // We don't care about TX interrupt
    if ( INTGetFlag(INT_SOURCE_UART_TX(UART3)) ){
        INTClearFlag(INT_SOURCE_UART_TX(UART3));
    }
    
}

// Timer Interrupt
void __ISR(_TIMER_1_VECTOR, ipl6) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

            //
//                  mPORTGToggleBits(BIT_0);
                ContBussola++;
                FlagTempoDeAmostragem = TRUE;
          
      
}

/*
 * Interrupção Bussola
 */
void __ISR(_EXTERNAL_2_VECTOR, ipl4) Extern2Handler(void)
{
    // clear the interrupt flag
//                  mPORTGToggleBits(BIT_0);

   /* if(INTCONbits.INT2EP ==1){
    ContBussola=0;
    bussolaInical = TMR1;
    INTCONbits.INT2EP = 0;


    }*/
    //else{
     //   INTCONbits.INT2EP =1;
        bussolaFinal = TMR1;
        if(ContBussola){
        bussola = bussolaFinal + (12500-bussolaInical) + (ContBussola-1)*12500;
            bussola = (8*bussola - 660000)/100;
        }
        else{
            bussola = bussolaFinal - bussolaInical;
            bussola = (8*bussola - 660000)/100;

        }
        bussolaInical =bussolaFinal;
        ContBussola = 0;


    //}

     mINT2ClearIntFlag();

}




/*
 * Interrupção do Motor 1
 */
void __ISR(_EXTERNAL_4_VECTOR, ipl1) Extern4Handler(void)
{
    // clear the interrupt flag

    if(Motor_1.Sincronismo){

    Motor_1.Velocidade = TMR3;
   // Motor_1.Velocidade = ((281250/Motor_1.Velocidade)/304)*60;
   // Motor_1.Velocidade = ((312500*60)/304)/Motor_1.Velocidade;
   // Motor_1.Velocidade = Motor_1.Velocidade * 10.;
      Motor_1.Velocidade = 616776.31/Motor_1.Velocidade;
    TMR3 = 0;

    Motor_1.DirecaoAtual = PORTGbits.RG12;

    if( !Motor_1.DirecaoAtual ){
        Motor_1.Velocidade = (-1.0)*Motor_1.Velocidade;
    }
    }
    Motor_1.Sincronismo = TRUE;
    Motor_1.Estouro = 0;
   
     mINT4ClearIntFlag();
    // .. things to do
    // .. in this case, toggle the LED
    //mPORTGToggleBits(BIT_0);
}



/*
 * Interrupção Motor 2
 */
void __ISR(_EXTERNAL_3_VECTOR, ipl2) Extern3Handler(void)
{
    // clear the interrupt flag
    
    if(Motor_2.Sincronismo){
    Motor_2.Velocidade = TMR4;
   // Motor_2.Velocidade = ((281250/Motor_2.Velocidade)/304)*60;
   // Motor_2.Velocidade = ((312500*60)/304)/Motor_2.Velocidade;
   // Motor_2.Velocidade =Motor_2.Velocidade * 10.;
    Motor_2.Velocidade = 616776.31/Motor_2.Velocidade;
    TMR4 = 0;

    Motor_2.DirecaoAtual = PORTGbits.RG14;

    if( !Motor_2.DirecaoAtual )Motor_2.Velocidade = (-1.0)*Motor_2.Velocidade;

    }
    Motor_2.Sincronismo = TRUE;
    Motor_2.Estouro = 0;
   
    mINT3ClearIntFlag();
    // .. things to do
    // .. in this case, toggle the LED
    //mPORTGToggleBits(BIT_0);
}

/*
 * Interrupção Motor 3
 */
void __ISR(_EXTERNAL_0_VECTOR, ipl3) Extern0Handler(void)
{
    // clear the interrupt flag
    if(Motor_3.Sincronismo){
    Motor_3.Velocidade = TMR5;
    //Motor_3.Velocidade = ((281250/Motor_3.Velocidade)/304)*60;
    //Motor_3.Velocidade = ((312500*60)/304)/Motor_3.Velocidade;
    //Motor_3.Velocidade =Motor_3.Velocidade * 10.;
    Motor_3.Velocidade = 616776.31/Motor_3.Velocidade;
    TMR5 = 0;

    Motor_3.DirecaoAtual = PORTGbits.RG13;

    if( !Motor_3.DirecaoAtual )Motor_3.Velocidade = (-1.0)*Motor_3.Velocidade;

    }
    Motor_3.Sincronismo = TRUE;
    Motor_3.Estouro = 0;
    mINT0ClearIntFlag();
    // .. things to do
    // .. in this case, toggle the LED
    //mPORTGToggleBits(BIT_0);
}
