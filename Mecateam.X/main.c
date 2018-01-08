/******************************************************************************/
/*  Files to Include                                                          */
/******************************************************************************/

#include <plib.h>           /* Include to use PIC32 peripheral libraries      */
#include <stdint.h>         /* For uint32_t definition                        */
#include <stdbool.h>        /* For true/false definition                      */

#include "system.h"         /* System funct/params, like osc/periph config    */
#include "user.h"           /* User funct/params, such as InitApp             */

#include "uart.h"
#include "ZigbeeTransmit.h"
#include "ZigbeeReceive.h"
#include <dsplib_dsp.h>
#include <math.h>

#include <time.h>

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/

/* i.e. uint32_t <variable_name>; */

/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/
#define GetSystemClock() (80000000L)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())
#define DESIRED_BAUDRATE    	(115200)      //The desired BaudRate
#define T1_TICK               (12500)  //(6250) //(2500) //(12500)
#define T3_TICK                 (0xFFFF)
//#define I2C_CLOCK_FREQ           (100000)
//#define Fsck        375000
//#define BRG_VAL     (GetPeripheralClock()/2/Fsck)
long int corrente1,corrente2,corrente3;
char n;

int16 vout[256],vin[256],c;
int u;

int32_t main(void) {

#ifndef PIC32_STARTER_KIT
    /*The JTAG is on by default on POR.  A PIC32 Starter Kit uses the JTAG, but
    for other debug tool use, like ICD 3 and Real ICE, the JTAG should be off
    to free up the JTAG I/O */
    DDPCONbits.JTAGEN = 0;
#endif

    /*Refer to the C32 peripheral library compiled help file for more
    information on the SYTEMConfig function.

    This function sets the PB divider, the Flash Wait States, and the DRM
    /wait states to the optimum value.  It also enables the cacheability for
    the K0 segment.  It could has side effects of possibly alter the pre-fetch
    buffer and cache.  It sets the RAM wait states to 0.  Other than
    the SYS_FREQ, this takes these parameters.  The top 3 may be '|'ed
    together:

    SYS_CFG_WAIT_STATES (configures flash wait states from system clock)
    SYS_CFG_PB_BUS (configures the PB bus from the system clock)
    SYS_CFG_PCACHE (configures the pCache if used)
    SYS_CFG_ALL (configures the flash wait states, PB bus, and pCache)*/

    /* TODO Add user clock/system configuration code if appropriate.  */
    SYSTEMConfig(SYS_FREQ, SYS_CFG_ALL);


    UARTConfigure(UART3, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART3, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART3, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART3, GetPeripheralClock(), DESIRED_BAUDRATE);
    UARTEnable(UART3, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART3 RX Interrupt
    INTEnable(INT_SOURCE_UART_RX(UART3), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART3), INT_PRIORITY_LEVEL_7);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART3), INT_SUB_PRIORITY_LEVEL_0);

    // Config Timer
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_64, T1_TICK);
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, T3_TICK);
    OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_256, T3_TICK);
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_256, T3_TICK);
    // set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_6);
    ConfigIntTimer3(T3_INT_OFF);
    ConfigIntTimer4(T4_INT_OFF);
    ConfigIntTimer5(T5_INT_OFF);

    //Pino de teste
    TRISGbits.TRISG0 = 0;


    /* Motor 1
     * Interrup��o 4
     * Seta Pino INT3 (RA15) como entrada
     * Seta PINO RG12 como entrada
     */

    TRISAbits.TRISA15 = 1;
    TRISGbits.TRISG12 = 1;

    INTEnable(INT_SOURCE_EX_INT(4), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_EX_INT(4), INT_PRIORITY_LEVEL_1);
    INTSetVectorSubPriority(INT_VECTOR_EX_INT(4), INT_SUB_PRIORITY_LEVEL_1);


    /* Motor 2
     * Interrup��o 3
     * Seta Pino INT3 (RA14) como entrada
     * Seta PINO RG14 como entrada
     */

    TRISAbits.TRISA14 = 1;
    TRISGbits.TRISG14 = 1;

    INTEnable(INT_SOURCE_EX_INT(3), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_EX_INT(3), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_EX_INT(3), INT_SUB_PRIORITY_LEVEL_2);


    /* Motor 3
     * Interrup��o 0
     * Seta Pino INT0 (RD0) como entrada
     * Seta PINO RG13 como entrada
     */

    TRISDbits.TRISD0 = 1;
    TRISGbits.TRISG13 = 1;

    INTEnable(INT_SOURCE_EX_INT(0), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_EX_INT(0), INT_PRIORITY_LEVEL_3);
    INTSetVectorSubPriority(INT_VECTOR_EX_INT(0), INT_SUB_PRIORITY_LEVEL_3);

    /* Gyrosc�pio
     * Interrup��o 2
     * Seta Pino INT2 (RE9) como entrada
     */

    TRISEbits.TRISE9 = 1;

    INTEnable(INT_SOURCE_EX_INT(2), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_EX_INT(2), INT_PRIORITY_LEVEL_4);
    INTSetVectorSubPriority(INT_VECTOR_EX_INT(2), INT_SUB_PRIORITY_LEVEL_3);
    INTCONbits.INT2EP =1;

    ContBussola =0;

    /*Configure Multivector Interrupt Mode.  Using Single Vector Mode
    is expensive from a timing perspective, so most applications
    should probably not use a Single Vector Mode*/
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    // enable interrupts
    INTEnableInterrupts();

    /* Initialize I/O and Peripherals for application */

    InitApp();

    SobreCorrente1=0;
    SobreCorrente2=0;
    SobreCorrente3=0;
    // a=4;
    while(1) {

        if(uartPilhaZigbee){

            FrameProcessing();

        }

        if (FlagTempoDeAmostragem == TRUE) {

            FlagTempoDeAmostragem = FALSE;

            Motor_1.Corrente[Temporizador] =  readADC(2);
            Motor_2.Corrente[Temporizador] =  readADC(12);
            Motor_3.Corrente[Temporizador] =  readADC(4);

            AcelerometroX[Temporizador] =  readADC(14);
            AcelerometroY[Temporizador] =  readADC(15);

            Giroscopio[Temporizador] =  readADC(13);

            Motor_1.MediaMovelCorrente = (Motor_1.Corrente[0] + Motor_1.Corrente[1] +
                    Motor_1.Corrente[2] + Motor_1.Corrente[3] +
                    Motor_1.Corrente[4])/ 5.;

            Motor_2.MediaMovelCorrente = (Motor_2.Corrente[0] + Motor_2.Corrente[1] +
                    Motor_2.Corrente[2] + Motor_2.Corrente[3] +
                    Motor_2.Corrente[4])/ 5.;

            Motor_3.MediaMovelCorrente = (Motor_3.Corrente[0] + Motor_3.Corrente[1] +
                    Motor_3.Corrente[2] + Motor_3.Corrente[3] +
                    Motor_3.Corrente[4])/ 5.;

            AcelerometroXMovelCorrente = (AcelerometroX[0] + AcelerometroX[1] + AcelerometroX[2] +
                    AcelerometroX[3] + AcelerometroX[4])/ 5.;

            AcelerometroYMovelCorrente = (AcelerometroY[0] + AcelerometroY[1] + AcelerometroY[2] +
                    AcelerometroY[3] + AcelerometroY[4])/ 5.;

            GiroscopioMovelCorrente = (Giroscopio[0] + Giroscopio[1] + Giroscopio[2] +
                    Giroscopio[3] + Giroscopio[4])/ 5.;

            Motor_1.Estouro++;
            Motor_2.Estouro++;
            Motor_3.Estouro++;

            if (Motor_1.Estouro >= 18){
                Motor_1.Velocidade = 0;
                Motor_1.Estouro = 0;
            }

            if (Motor_2.Estouro >= 18){
                Motor_2.Velocidade = 0;
                Motor_2.Estouro = 0;
            }

            if (Motor_3.Estouro >= 18){
                Motor_3.Velocidade = 0;
                Motor_3.Velocidade = 0;
            }

            Motor_1.VelocidadeMedia[Temporizador]=Motor_1.Velocidade;
            Motor_1.MediaMovel = (Motor_1.VelocidadeMedia[0] + Motor_1.VelocidadeMedia[1] +
            Motor_1.VelocidadeMedia[2] + Motor_1.VelocidadeMedia[3] +
            Motor_1.VelocidadeMedia[4])/ 5;

            Motor_2.VelocidadeMedia[Temporizador]=Motor_2.Velocidade;
            Motor_2.MediaMovel = (Motor_2.VelocidadeMedia[0] + Motor_2.VelocidadeMedia[1] +
            Motor_2.VelocidadeMedia[2] + Motor_2.VelocidadeMedia[3] +
            Motor_2.VelocidadeMedia[4])/ 5;

            Motor_3.VelocidadeMedia[Temporizador]=Motor_3.Velocidade;
            Motor_3.MediaMovel = (Motor_3.VelocidadeMedia[0] + Motor_3.VelocidadeMedia[1] +
            Motor_3.VelocidadeMedia[2] + Motor_3.VelocidadeMedia[3] +
            Motor_3.VelocidadeMedia[4])/ 5;

            switch (Temporizador){
                case 0:
                    Motor_1.Corrente[2] = Motor_1.MediaMovelCorrente;
                    Motor_2.Corrente[2] = Motor_2.MediaMovelCorrente;
                    Motor_3.Corrente[2] = Motor_3.MediaMovelCorrente;
                    AcelerometroX[2] = AcelerometroXMovelCorrente;
                    AcelerometroY[2] = AcelerometroYMovelCorrente;
                    Giroscopio[2] = GiroscopioMovelCorrente;
                    Motor_1.VelocidadeMedia[2] = Motor_1.MediaMovel;
                    Motor_2.VelocidadeMedia[2] = Motor_2.MediaMovel;
                    Motor_3.VelocidadeMedia[2] = Motor_3.MediaMovel;
                    break;
                case 1:
                    Motor_1.Corrente[3] = Motor_1.MediaMovelCorrente;
                    Motor_2.Corrente[3] = Motor_2.MediaMovelCorrente;
                    Motor_3.Corrente[3] = Motor_3.MediaMovelCorrente;
                    AcelerometroX[3] = AcelerometroXMovelCorrente;
                    AcelerometroY[3] = AcelerometroYMovelCorrente;
                    Giroscopio[3] = GiroscopioMovelCorrente;
                    Motor_1.VelocidadeMedia[3] = Motor_1.MediaMovel;
                    Motor_2.VelocidadeMedia[3] = Motor_2.MediaMovel;
                    Motor_3.VelocidadeMedia[3] = Motor_3.MediaMovel;
                    break;
                case 2:
                    Motor_1.Corrente[4] = Motor_1.MediaMovelCorrente;
                    Motor_2.Corrente[4] = Motor_2.MediaMovelCorrente;
                    Motor_3.Corrente[4] = Motor_3.MediaMovelCorrente;
                    AcelerometroX[4] = AcelerometroXMovelCorrente;
                    AcelerometroY[4] = AcelerometroYMovelCorrente;
                    Giroscopio[4] = GiroscopioMovelCorrente;
                    Motor_1.VelocidadeMedia[4] = Motor_1.MediaMovel;
                    Motor_2.VelocidadeMedia[4] = Motor_2.MediaMovel;
                    Motor_3.VelocidadeMedia[4] = Motor_3.MediaMovel;
                    break;
                case 3:
                    Motor_1.Corrente[0] = Motor_1.MediaMovelCorrente;
                    Motor_2.Corrente[0] = Motor_2.MediaMovelCorrente;
                    Motor_3.Corrente[0] = Motor_3.MediaMovelCorrente;
                    AcelerometroX[0] = AcelerometroXMovelCorrente;
                    AcelerometroY[0] = AcelerometroYMovelCorrente;
                    Giroscopio[0] = GiroscopioMovelCorrente;
                    Motor_1.VelocidadeMedia[0] = Motor_1.MediaMovel;
                    Motor_2.VelocidadeMedia[0] = Motor_2.MediaMovel;
                    Motor_3.VelocidadeMedia[0] = Motor_3.MediaMovel;
                    break;
                case 4:
                    Motor_1.Corrente[1] = Motor_1.MediaMovelCorrente;
                    Motor_2.Corrente[1] = Motor_2.MediaMovelCorrente;
                    Motor_3.Corrente[1] = Motor_3.MediaMovelCorrente;
                    AcelerometroX[1] = AcelerometroXMovelCorrente;
                    AcelerometroY[1] = AcelerometroYMovelCorrente;
                    Giroscopio[3] = GiroscopioMovelCorrente;
                    Motor_1.VelocidadeMedia[1] = Motor_1.MediaMovel;
                    Motor_2.VelocidadeMedia[1] = Motor_2.MediaMovel;
                    Motor_3.VelocidadeMedia[1] = Motor_3.MediaMovel;
                    break;
            }

            // Prote��o de sobre corrente
            if(Motor_1.MediaMovelCorrente>1000){
                SobreCorrente1++;
                if(SobreCorrente1>5){OC3RS=0;}
            }
            else {
                SobreCorrente1=0;
            }

            if(Motor_2.MediaMovelCorrente>1000){
                SobreCorrente2++;
                if(SobreCorrente2>5){OC2RS=0;}
            }
            else {
                SobreCorrente2=0;
            }

            if(Motor_3.MediaMovelCorrente>1000){
                SobreCorrente3++;
                if(SobreCorrente3>5){OC4RS=0;}
            }
            else {
                SobreCorrente3=0;
            }

            if (fPID){
                PID();
            }

            if(Temporizador >= 4){
                Temporizador = 0;
            }
            else{
                Temporizador++;
            }

            Temp++;
        }

        if (Temp >= 6){ //2 para 20ms 6 para 60ms

            mPORTGToggleBits(BIT_0);

            Temp =0;

            if(fLQR) {
                LQR_smith();  //a cada 60ms
            }
            else if(fOtimo){
                LQR_otimo();  //a cada 60ms
            }
        }

        if(fSend){
            fSend =0;
            EnviarDado();
        }

    }

    return 0;
}
