/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <plib.h>            /* Include to use PIC32 peripheral libraries     */
#include <stdint.h>          /* For uint32_t definition                       */
#include <stdbool.h>         /* For true/false definition                     */
#include "user.h"            /* variables/params used by user.c               */
#include "uart.h"
#include "ZigbeeReceive.h"
#include "ZigbeeTransmit.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* TODO Initialize User Ports/Peripherals/Project here */

#define AINPUTS 0xFF00  // pinos de entrada anal�gica para os sensores de corrente

unsigned char RXBuffer[23];
unsigned char RXCount;

unsigned char FlagTempoDeAmostragem;
unsigned char Temporizador;
unsigned char Temp;
//unsigned char Temp2;
unsigned char TempMotor1,TempMotor2,TempMotor3;
unsigned char fSend;
unsigned char fPID;
unsigned char fLQR;
unsigned char ContBussola;
long int bussola;
long int bussolaInical;
long int bussolaFinal;

int fi1=0,fi2=0,fi3=0;
int output_1, output_2, output_3;
int saida_1, saida_2, saida_3;
float erro_old_11=0,erro_old_12=0,erro_old_21=0,erro_old_22=0,erro_old_31=0,erro_old_32=0;
float erro_1=0,erro_2=0,erro_3=0;
float integral_1=0,integral_2=0,integral_3=0;
float derivativo_1,derivativo_2,derivativo_3;
float uk1=0,uk1_1=0;
float uk2=0,uk2_1=0;
float uk3=0,uk3_1=0;


unsigned int AcelerometroX[5];
unsigned int AcelerometroY[5];
unsigned int Giroscopio[5];
long int AcelerometroXMovelCorrente;
long int AcelerometroYMovelCorrente;
long int GiroscopioMovelCorrente;

unsigned char Flag_RXReceived;

float V_ref;
float Vn_ref;
float W_ref;

struct Motor Motor_1;
struct Motor Motor_2;
struct Motor Motor_3;
struct tSerial Serial;

int SobreCorrente1,SobreCorrente2,SobreCorrente3;

void InitApp(void) {
    /* Setup analog functionality and port direction */
    /*
     * Configura os pinos de controle de dire��o como saida.
     * Motor 1 E2
     * Motor 2 E3
     * Motor 3 E4
     */
    TRISEbits.TRISE2 = 0;
    TRISEbits.TRISE3 = 0;
    TRISEbits.TRISE4 = 0;


    /* Initialize peripherals */
    RXCount = 0;
    initPWM();

    initADC( AINPUTS);
    uart1ConterRx = 0;
    fSend =0;
    fPID = 0;
    fLQR = 0;

    Motor_1.Kp = 0.24;
    Motor_2.Kp = 0.24;
    Motor_3.Kp = 0.24;
    Motor_1.Ki = 0.051;
    Motor_2.Ki = 0.051;
    Motor_3.Ki = 0.051;
    Motor_1.Kd = 0.00;
    Motor_2.Kd = 0.00;
    Motor_3.Kd = 0.00;
    Motor_1.VReferencia = 0;
    Motor_2.VReferencia = 0;
    Motor_3.VReferencia = 0;

    V_ref = 0;
    Vn_ref = 0;
    W_ref = 0;

}


int initADC( int amask)

{
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB15 = 1;
    TRISBbits.TRISB14 = 1;
    TRISBbits.TRISB13 = 1;
    TRISBbits.TRISB12 = 1;

    AD1PCFGbits.PCFG2 = 0;
    AD1PCFGbits.PCFG3 = 0;    // seleciona pinos de entrada
    AD1PCFGbits.PCFG4 = 0;
    AD1PCFGbits.PCFG5 = 0;
    AD1PCFGbits.PCFG12 = 0;
    AD1PCFGbits.PCFG13 = 0;
    AD1PCFGbits.PCFG14 = 0;
    AD1PCFGbits.PCFG15 = 0;


/*
    AD1CON1bits.ON = 0;
    AD1CON1bits.SIDL = 0;
    AD1CON1bits.FORM = 0b000;
    AD1CON1bits.SSRC = 0b111;
    AD1CON1bits.CLRASAM = 0;
    AD1CON1bits.ASAM = 0;

    AD1CON2bits.VCFG = 0b000;
    AD1CON2bits.OFFCAL = 0;
    AD1CON2bits.CSCNA = 1;      // Scan Inputs
    AD1CON2bits.BUFS = 0;
    AD1CON2bits.SMPI = 7;
    AD1CON2bits.BUFM = 0;
    AD1CON2bits.ALTS = 0;

    AD1CON3bits.ADRC = 0;//1
    AD1CON3bits.SAMC = 6;
    AD1CON3bits.ADCS = 7;

    AD1CHSbits.CH0NA = 0;
    AD1CSSLbits.CSSL &= BIT_3 + BIT_4 + BIT_5 + BIT_12 + BIT_13 + BIT_14 + BIT_15;

    AD1CON1bits.ON = 1;
*/

    AD1CON1 = 0x00E0;   // auto convers�o apos a amostragem

    AD1CSSL = 0;        // n�o requer escaneamento

    AD1CON2 = 0;        // usa MUXA, AVss/AVdd e Vref+/-

    AD1CON3 = 0x1F3F;   // maximo tempo de amostragem = 31Tad

    AD1CON1SET = 0x8000;// liga ADC


 /*   AD1PCFG = amask;    // seleciona pinos de entrada
    AD1CON1 = 0x00E0;   // auto convers�o apos a amostragem
    AD1CSSL = 0;        // n�o requer escaneamento
    AD1CON2 = 0;        // usa MUXA, AVss/AVdd e Vref+/-
    AD1CON3 = 0x1F3F;   // maximo tempo de amostragem = 31Tad
    AD1CON1SET = 0x8000;// liga ADC
*/
} //initADC

int readADC( int ch){

    AD1CHSbits.CH0SA = ch;      // Seleciona o canal de entrada

    AD1CON1bits.SAMP = 1;       // inicia a amostragem

    while (!AD1CON1bits.DONE);  // wespera completar a convers�o
    AD1CON1bits.SAMP = 0;

    return ADC1BUF0;            // retorna o valor lido

} // readADC


/**********************************************************************************/

// Configura��o dos PWMs OCS1 OCS2 OCS3

void initPWM(void) {

    //Configura��o OSC1
    OC4CON = 0x0000; // Turn off OC1 while doing setup.
    OC4R = 0; // Initialize primary Compare Register
    OC4RS = 0; // Initialize secondary Compare Register
    OC4CON = 0x0006; // Configure for PWM mode, Fault pin Disabled

    //Configura��o OSC2
    OC2CON = 0x0000; // Turn off OC1 while doing setup.
    OC2R = 0; // Initialize primary Compare Register
    OC2RS = 0; // Initialize secondary Compare Register
    OC2CON = 0x0006; // Configure for PWM mode, Fault pin Disabled

    //Configura��o OSC3
    OC3CON = 0x0000; // Turn off OC1 while doing setup.
    OC3R = 0; // Initialize primary Compare Register
    OC3RS = 0; // Initialize secondary Compare Register
    OC3CON = 0x0006; // Configure for PWM mode, Fault pin Disabled

    //Configura��o do timer 2
    PR2 = 255; // Set period
    IFS0 &= ~0x00000080; // Clear the OC1 interrupt flag
    T2CONbits.ON = 1; // Enable timer2
    T2CONbits.TCKPS = 4; //resistrador do timer 2 que configura o prescale

    //Habilitando os PWMs
    OC4CON |= 0x8000; // turn on OC1 module
    OC2CON |= 0x8000; // turn on OC1 module
    OC3CON |= 0x8000; // turn on OC1 module
}

void EnviarDado(void) {

    unsigned char length;
    Serial.Direcao = 0;
    long int velocidade;
    long int Ganho; // da corrente

    Ganho = (3300/1024)/1.58;
    Serial.DutyCycle_1 = OC3RS;
    Serial.DutyCycle_2 = OC2RS;
    Serial.DutyCycle_3 = OC4RS;

    if (Motor_1.Velocidade >= 0){
        Serial.Direcao = 1;

        Serial.Vel_1[0] = (Motor_1.Velocidade >> 8) & 0xFF;
        Serial.Vel_1[1] = Motor_1.Velocidade  & 0xFF;
    }

    else{
        //Motor_1.Direcao = 0;

        velocidade = -1 * Motor_1.Velocidade;

        Serial.Vel_1[0] = (velocidade >> 8) & 0xFF;
        Serial.Vel_1[1] = velocidade  & 0xFF;
    }

    // Motor 2
    if (Motor_2.Velocidade >= 0){
        Serial.Direcao = Serial.Direcao + 2;

        Serial.Vel_2[0] = (Motor_2.Velocidade >> 8) & 0xFF;
        Serial.Vel_2[1] = Motor_2.Velocidade  & 0xFF;
    }

    else{
        //Motor_2.Direcao = 0;

        velocidade = -1 * Motor_2.Velocidade;

        Serial.Vel_2[0] = (velocidade >> 8) & 0xFF;
        Serial.Vel_2[1] = velocidade  & 0xFF;
    }

    // Motor 3
    if (Motor_3.Velocidade >= 0){
        Serial.Direcao = Serial.Direcao +4;

        Serial.Vel_3[0] = (Motor_3.Velocidade >> 8) & 0xFF;
        Serial.Vel_3[1] = Motor_3.Velocidade  & 0xFF;
    }

    else {
        //Motor_3.Direcao = 0;

        velocidade = -1 * Motor_3.Velocidade;

        Serial.Vel_3[0] = (velocidade >> 8) & 0xFF;
        Serial.Vel_3[1] = velocidade  & 0xFF;

    }
    /*
     * Referencia
     */

    Serial.DirecaoRef = 0;
    if (Motor_1.VReferencia >= 0){
        Serial.DirecaoRef = Serial.DirecaoRef +1;

        Serial.Setpoint_1[0] = (Motor_1.VReferencia >> 8) & 0xFF;
        Serial.Setpoint_1[1] = Motor_1.VReferencia  & 0xFF;
    }

    else {

        velocidade = -1 * Motor_1.VReferencia;

        Serial.Setpoint_1[0] = (velocidade >> 8) & 0xFF;
        Serial.Setpoint_1[1] = velocidade  & 0xFF;

    }

    // Motor 2
    if (Motor_2.VReferencia >= 0){
        Serial.DirecaoRef = Serial.DirecaoRef + 2;

        Serial.Setpoint_2[0] = (Motor_2.VReferencia >> 8) & 0xFF;
        Serial.Setpoint_2[1] = Motor_2.VReferencia  & 0xFF;
    }

    else {

        velocidade = -1 * Motor_2.VReferencia;

        Serial.Setpoint_2[0] = (velocidade >> 8) & 0xFF;
        Serial.Setpoint_2[1] = velocidade  & 0xFF;

    }

    // Motor 3
    if (Motor_3.VReferencia >= 0){
        Serial.DirecaoRef = Serial.DirecaoRef +4;

        Serial.Setpoint_3[0] = (Motor_3.VReferencia >> 8) & 0xFF;
        Serial.Setpoint_3[1] = Motor_3.VReferencia  & 0xFF;
    }

    else {

        velocidade = -1 * Motor_3.VReferencia;

        Serial.Setpoint_3[0] = (velocidade >> 8) & 0xFF;
        Serial.Setpoint_3[1] = velocidade  & 0xFF;

    }

    Motor_1.MediaMovelCorrente = Motor_1.MediaMovelCorrente*Ganho;
    Serial.Corrent_1[0] = (Motor_1.MediaMovelCorrente >> 8) & 0xFF;
    Serial.Corrent_1[1] = (Motor_1.MediaMovelCorrente ) & 0xFF;

    Motor_2.MediaMovelCorrente = Motor_2.MediaMovelCorrente*Ganho;
    Serial.Corrent_2[0] = (Motor_2.MediaMovelCorrente >> 8) & 0xFF;
    Serial.Corrent_2[1] = (Motor_2.MediaMovelCorrente ) & 0xFF;

    Motor_3.MediaMovelCorrente = Motor_3.MediaMovelCorrente*Ganho;
    Serial.Corrent_3[0] = (Motor_3.MediaMovelCorrente >> 8) & 0xFF;
    Serial.Corrent_3[1] = (Motor_3.MediaMovelCorrente ) & 0xFF;

    //Serial.Acer_X[0] = (AcelerometroXMovelCorrente>>8)  & 0xFF;
    //Serial.Acer_X[1] = (AcelerometroXMovelCorrente)  & 0xFF;

    Serial.Acer_Y[0] = (AcelerometroYMovelCorrente>>8)  & 0xFF;
    Serial.Acer_Y[1] = (AcelerometroYMovelCorrente)  & 0xFF;

    Serial.Giroscop[0] = (GiroscopioMovelCorrente>>8) & 0xFF;
    Serial.Giroscop[1] = (GiroscopioMovelCorrente) & 0xFF;

    Serial.Compass[0] =(bussola>>8) & 0xFF;
    Serial.Compass[1] = bussola & 0xFF;


    CabecalhoDoFrame.Length[0] = 0x00;
    CabecalhoDoFrame.Length[1] = 0x2E;
    CabecalhoDoFrame.Command = 0x01;
    length = ZigbeeTransmitRead(&CabecalhoDoFrame.StarDelimiter, 18,Serial.Vel_1, 31,BufferTransmit);

    ZigbeeSend(BufferTransmit,length);

}

void PID(void){

    if(SobreCorrente1<=5){

 		// algoritmo velocidade

        if(output_1<255&&output_1>-255){
            erro_old_11=erro_1;
            fi1=fi1+1;
        }

        if(fi1=2){
            erro_old_12=erro_old_11;
            fi1=0;
        }

	    erro_1=Motor_1.VReferencia-Motor_1.Velocidade;

		uk1=uk1_1 + Motor_1.Kp*erro_1 + Motor_1.Kd*erro_1 + Motor_1.Ki*erro_old_11 - Motor_1.Kp*erro_old_11 - 2*Motor_1.Kd*erro_old_11 + Motor_1.Kd*erro_old_12;

		uk1_1=uk1;

		if(uk1<0){
			saida_1=-1*uk1;
            output_1=-1*uk1;
			mPORTEClearBits(BIT_3);//	E=0;
		}
		else {
            saida_1=uk1;
            output_1=uk1;
            mPORTESetBits(BIT_3);//E=2;
		}

		if(saida_1>254){
            saida_1=254;
		}

	 	OC3RS=saida_1;

    }

    if(SobreCorrente2<=5){

 		// algoritmo velocidade

	    if(output_2<255&&output_2>-255){

            erro_old_21=erro_2;

            fi2=fi2+1;

        }

        if(fi2=2){

            erro_old_22=erro_old_21;

            fi2=0;

        }

		erro_2=Motor_2.VReferencia-Motor_2.Velocidade;

		uk2=uk2_1 + Motor_2.Kp*erro_2 + Motor_2.Kd*erro_2 + Motor_2.Ki*erro_old_21 - Motor_2.Kp*erro_old_21 - 2*Motor_2.Kd*erro_old_21 + Motor_2.Kd*erro_old_22;

		uk2_1=uk2;

		if(uk2<0){
            saida_2=-1*uk2;
            output_2=-1*uk2;
            mPORTEClearBits(BIT_2);//C=0;
		}
        else {
            saida_2=uk2;
            output_2=uk2;
			mPORTESetBits(BIT_2);//C=8;
		}

		if(saida_2>254){
            saida_2=254;
		}

		OC2RS=saida_2;

    }

    if(SobreCorrente3<=5){

 		// algoritmo velocidade

		if(output_3<255&&output_3>-255){
            erro_old_31=erro_3;
            fi3=fi3+1;
        }

        if(fi3=2){
            erro_old_32=erro_old_31;
            fi3=0;
        }

		erro_3=Motor_3.VReferencia-Motor_3.Velocidade;

		uk3=uk3_1 + Motor_3.Kp*erro_3 + Motor_3.Kd*erro_3 + Motor_3.Ki*erro_old_31 - Motor_3.Kp*erro_old_31 - 2*Motor_3.Kd*erro_old_31 + Motor_3.Kd*erro_old_32;

		uk3_1=uk3;

		if(uk3<0){
            saida_3=-1*uk3;

            output_3=-1*uk3;

			mPORTEClearBits(BIT_4);//A=0;
		}
        else {
            saida_3=uk3;

            output_3=uk3;

			mPORTESetBits(BIT_4);//A=32;
        }

		if(saida_3>254){
            saida_3=254;
		}

        OC4RS=saida_3;
    }

    //PORTE= E + C +A;// seta o brake e dir dos motores
}

void LQR_smith(void){

/*----------------------------------------------------------------------------*/

    //Para matrizes de pondera��o: Q 100*I e R = I
    const float Kdlqr[3][6] = {
        { 0.0000,   -7.3404,    0.1982,    0.0000,   -5.0455,    0.3356},
        { 6.3570,    3.6702,    0.1982,    4.3695,    2.5227,    0.3356},
        {-6.3570,    3.6702,    0.1982,   -4.3695,    2.5227,    0.3356}};

    /*Referentes ao preditor de Smith*/
    const float Ad[3][3] = {
        {0.8218,         0,         0},
        {     0,    0.8218,         0},
        {     0,         0,    0.5888}};

    const float Bd[3][3] = {
        {        0,    0.0400,   -0.0400},
        {    -0.0461,    0.0231,    0.0231},
        {    0.9868,    0.9868,    0.9868}};

    const float alpha = 0.85;

/*----------------------------------------------------------------------------*/

    static float Vref[3] = {0,0,0};

    static float x[3] = {0,0,0};
    static float xFiltrado[3] = {0,0,0};

    static float U[3] = {0,0,0};

    static float velm[3] = {0,0,0};

/*----------------------------------------------------------------------------*/

    static float vel[3];
    static float velrad[3];

    float old_x[3];
    float difvel[3];
    float new_velm[3];

    float dU[3];

/*----------------------------------------------------------------------------*/

    int aux;

/*----------------------------------------------------------------------------*/

    Vref[0] = V_ref;
    Vref[1] = Vn_ref;
    Vref[2] = W_ref;

    //O calculo da velocidade � RPM *10 -->
    //dividindo por 10 e dividindo por 60 para converter em voltas por seg
    //Depois multiplica por 2*PI para achar em rad/s
    //Velocidade escalar das rodas em rad/s
    velrad[0] = (Motor_1.Velocidade)*0.01047197;
    velrad[1] = (Motor_2.Velocidade)*0.01047197;
    velrad[2] = (Motor_3.Velocidade)*0.01047197;

    //Aqui converter para o sistema de coordenadas do rob�
    //vel[0] = 0*vel[0] + 0.5774*vel[1] - 0.5774*vel[2];
    //vel[1] = -0.6667*vel[0] + 0.3333*vel[1] + 0.3333*vel[2];
    //vel[2] = 3.3333*vel[0] + 3.3333*vel[1] + 3.3333*vel[2];
    //Multiplica a matriz acima comentada por 0,0505
    //para obter a vel linear a partir de velrad
    vel[0] = 0*velrad[0] + 0.0292*velrad[1] - 0.0292*velrad[2];
    vel[1] = -0.0337*velrad[0] + 0.0168*velrad[1] + 0.0168*velrad[2];
    vel[2] = 0.1683*velrad[0] + 0.1683*velrad[1] + 0.1683*velrad[2];

    old_x[0] = x[0];
    old_x[1] = x[1];
    old_x[2] = x[2];

    //Encontrando o erro entre os valores lidos e os valores do modelo
    //difvel[0] = vel[0] - velm[0];
    //difvel[1] = vel[1] - velm[1];
    //difvel[2] = vel[2] - velm[2];

    //Calculando Novas Velocidades do modelo
    new_velm[0] = Ad[0][0]*velm[0] + Ad[0][1]*velm[1] + Ad[0][2]*velm[2];
    new_velm[0] += Bd[0][0]*U[0] + Bd[0][1]*U[1] + Bd[0][2]*U[2];

    new_velm[1] = Ad[1][0]*velm[0] + Ad[1][1]*velm[1] + Ad[1][2]*velm[2];
    new_velm[1] += Bd[1][0]*U[0] + Bd[1][1]*U[1] + Bd[1][2]*U[2];

    new_velm[2] = Ad[2][0]*velm[0] + Ad[2][1]*velm[1] + Ad[2][2]*velm[2];
    new_velm[2] += Bd[2][0]*U[0] + Bd[2][1]*U[1] + Bd[2][2]*U[2];

    velm[0] = new_velm[0];
    velm[1] = new_velm[1];
    velm[2] = new_velm[2];

    //Encontrando o erro entre os valores lidos e os valores do modelo
    difvel[0] = vel[0] - velm[0];
    difvel[1] = vel[1] - velm[1];
    difvel[2] = vel[2] - velm[2];


    //Filtro
    xFiltrado[0] = difvel[0]*(1-alpha) + alpha*xFiltrado[0];
    xFiltrado[1] = difvel[1]*(1-alpha) + alpha*xFiltrado[1];
    xFiltrado[2] = difvel[2]*(1-alpha) + alpha*xFiltrado[2];

    //Encontrando o vetor de estados ap�s filtragem
    x[0] = velm[0] + xFiltrado[0];
    x[1] = velm[1] + xFiltrado[1];
    x[2] = velm[2] + xFiltrado[2];

    //Calculando a a��o de controle
    dU[0] = -Kdlqr[0][0]*(x[0]-old_x[0]) - Kdlqr[0][1]*(x[1]- old_x[1]) - Kdlqr[0][2]*(x[2]-old_x[2]);
    dU[0] += Kdlqr[0][3]*(Vref[0] - x[0]) + Kdlqr[0][4]*(Vref[1] - x[1]) + Kdlqr[0][5]*(Vref[2] - x[2]);

    dU[1] = -Kdlqr[1][0]*(x[0]-old_x[0]) - Kdlqr[1][1]*(x[1]- old_x[1]) - Kdlqr[1][2]*(x[2]-old_x[2]);
    dU[1] += Kdlqr[1][3]*(Vref[0] - x[0]) + Kdlqr[1][4]*(Vref[1] - x[1]) + Kdlqr[1][5]*(Vref[2] - x[2]);

    dU[2] = -Kdlqr[2][0]*(x[0]-old_x[0]) - Kdlqr[2][1]*(x[1]- old_x[1]) - Kdlqr[2][2]*(x[2]-old_x[2]);
    dU[2] += Kdlqr[2][3]*(Vref[0] - x[0]) + Kdlqr[2][4]*(Vref[1] - x[1]) + Kdlqr[2][5]*(Vref[2] - x[2]);

    U[0] = dU[0] + U[0];
    U[1] = dU[1] + U[1];
    U[2] = dU[2] + U[2];

    if(SobreCorrente1<=5){

        //Aqui eh preciso converter de tens�o para PWM
        aux = (int) 255-(255/6)*(6-U[0]);

        if(aux<0){

            aux=-1*aux;
            mPORTEClearBits(BIT_3);//E=0;

        }
        else {

            mPORTESetBits(BIT_3);//E=0;

        }

	if(aux>254){
            aux=254;
        }

        OC3RS=aux;

    }

    if(SobreCorrente2<=5){

        //Aqui eh preciso converter de tens�o para PWM
        aux = (int) 255-(255/6)*(6-U[1]);

        if(aux<0){

            aux=-1*aux;
            mPORTEClearBits(BIT_2);//C=0;

        }
        else {

            mPORTESetBits(BIT_2);//C=8;

        }

	if(aux>254){
            aux=254;
        }

        OC2RS=aux;

    }

    if(SobreCorrente3<=5){

        //Aqui eh preciso converter de tens�o para PWM
        aux = (int) 255-(255/6)*(6-U[2]);

        if(aux<0){

            aux=-1*aux;
            mPORTEClearBits(BIT_4);//A=0;

        }
        else {

            mPORTESetBits(BIT_4);//A=32;

        }

	if(aux>254){
            aux=254;
        }

        OC4RS=aux;

    }

}

void LQR_otimo(void){

/*----------------------------------------------------------------------------*/

    //Para matrizes de pondera��o: Q 100*I e R = I
    const float Kdlqr[3][6] = {
        { 0.0000,   -7.3404,    0.1982,    0.0000,   -5.0455,    0.3356},
        { 6.3570,    3.6702,    0.1982,    4.3695,    2.5227,    0.3356},
        {-6.3570,    3.6702,    0.1982,   -4.3695,    2.5227,    0.3356}};

    /*Referentes ao preditor Otimo*/
    const float Ag[6][6] = {
        {0.8218,         0,         0,         0,         0,         0},
        {     0,    0.8218,         0,         0,         0,         0},
        {     0,         0,    0.5888,         0,         0,         0},
        {0.8218,         0,         0,    1.0000,         0,         0},
        {     0,    0.8218,         0,         0,    1.0000,         0},
        {     0,         0,    0.5888,         0,         0,    1.0000}};

    const float Bg[6][3] = {
        {      0,    0.0400,   -0.0400},
        {-0.0461,    0.0231,    0.0231},
        { 0.9868,    0.9868,    0.9868},
        {      0,    0.0400,   -0.0400},
        {-0.0461,    0.0231,    0.0231},
        { 0.9868,    0.9868,    0.9868}};

/*----------------------------------------------------------------------------*/

    static float Vref[3] = {0,0,0};

    static float vel[3] = {0,0,0};

    static float U[3] = {0,0,0};

/*----------------------------------------------------------------------------*/

    static float old_vel[3];
    static float velrad[3];

    static float x[6];

    float dU[3];

/*----------------------------------------------------------------------------*/

    int aux;

/*----------------------------------------------------------------------------*/

    //if(Vref[0] < 0.5)
    //    Vref[0] += 0.0;//V_ref;

    //Vref[1] = 0;//Vn_ref;

    //if(Vref[2] < 3.14)
    //    Vref[2] += 3.14;//V_ref;

    Vref[0] = V_ref;
    Vref[1] = Vn_ref;
    Vref[2] = W_ref;

    old_vel[0]= vel[0];
    old_vel[1]= vel[1];
    old_vel[2]= vel[2];

    //O calculo da velocidade � RPM *10 -->
    //dividindo por 10 e dividindo por 60 para converter em voltas por seg
    //Depois multiplica por 2*PI para achar em rad/s
    //Velocidade escalar das rodas em rad/s
    velrad[0] = (Motor_1.MediaMovel)*0.01047197;
    velrad[1] = (Motor_2.MediaMovel)*0.01047197;
    velrad[2] = (Motor_3.MediaMovel)*0.01047197;

    //Aqui converter para o sistema de coordenadas do rob�
    //vel[0] = 0*vel[0] + 0.5774*vel[1] - 0.5774*vel[2];
    //vel[1] = -0.6667*vel[0] + 0.3333*vel[1] + 0.3333*vel[2];
    //vel[2] = 3.3333*vel[0] + 3.3333*vel[1] + 3.3333*vel[2];
    //Multiplica a matriz acima comentada por 0,0505
    //para obter a vel linear a partir de velrad
    vel[0] = 0*velrad[0] + 0.0292*velrad[1] - 0.0292*velrad[2];
    vel[1] = -0.0337*velrad[0] + 0.0168*velrad[1] + 0.0168*velrad[2];
    vel[2] = 0.1683*velrad[0] + 0.1683*velrad[1] + 0.1683*velrad[2];

    //Encontrando o vetor de estados x'(k+1|k)
    x[0] = Ag[0][0]*(vel[0]-old_vel[0]) + Ag[0][1]*(vel[1]- old_vel[1]) + Ag[0][2]*(vel[2]-old_vel[2]);
    x[0] += Ag[0][3]*(vel[0]) + Ag[0][4]*(vel[1]) + Ag[0][5]*(vel[2]);
    x[0] += Bg[0][0]*dU[0] + Bg[0][1]*dU[1] + Bg[0][2]*dU[2];

    x[1] = Ag[1][0]*(vel[0]-old_vel[0]) + Ag[1][1]*(vel[1]- old_vel[1]) + Ag[1][2]*(vel[2]-old_vel[2]);
    x[1] += Ag[1][3]*(vel[0]) + Ag[1][4]*(vel[1]) + Ag[1][5]*(vel[2]);
    x[1] += Bg[1][0]*dU[0] + Bg[1][1]*dU[1] + Bg[1][2]*dU[2];

    x[2] = Ag[2][0]*(vel[0]-old_vel[0]) + Ag[2][1]*(vel[1]- old_vel[1]) + Ag[2][2]*(vel[2]-old_vel[2]);
    x[2] += Ag[2][3]*(vel[0]) + Ag[2][4]*(vel[1]) + Ag[2][5]*(vel[2]);
    x[2] += Bg[2][0]*dU[0] + Bg[2][1]*dU[1] + Bg[2][2]*dU[2];

    x[3] = Ag[3][0]*(vel[0]-old_vel[0]) + Ag[3][1]*(vel[1]- old_vel[1]) + Ag[3][2]*(vel[2]-old_vel[2]);
    x[3] += Ag[3][3]*(vel[0]) + Ag[3][4]*(vel[1]) + Ag[3][5]*(vel[2]);
    x[3] += Bg[3][0]*dU[0] + Bg[3][1]*dU[1] + Bg[3][2]*dU[2];

    x[4] = Ag[4][0]*(vel[0]-old_vel[0]) + Ag[4][1]*(vel[1]- old_vel[1]) + Ag[4][2]*(vel[2]-old_vel[2]);
    x[4] += Ag[4][3]*(vel[0]) + Ag[4][4]*(vel[1]) + Ag[4][5]*(vel[2]);
    x[4] += Bg[4][0]*dU[0] + Bg[4][1]*dU[1] + Bg[4][2]*dU[2];

    x[5] = Ag[5][0]*(vel[0]-old_vel[0]) + Ag[5][1]*(vel[1]- old_vel[1]) + Ag[5][2]*(vel[2]-old_vel[2]);
    x[5] += Ag[5][3]*(vel[0]) + Ag[5][4]*(vel[1]) + Ag[5][5]*(vel[2]);
    x[5] += Bg[5][0]*dU[0] + Bg[5][1]*dU[1] + Bg[5][2]*dU[2];

    //Calculando a acao de controle
    dU[0] = -Kdlqr[0][0]*x[0] - Kdlqr[0][1]*x[1] - Kdlqr[0][2]*x[2];
    dU[0] += Kdlqr[0][3]*(Vref[0] - x[3]) + Kdlqr[0][4]*(Vref[1] - x[4]) + Kdlqr[0][5]*(Vref[2] - x[5]);

    dU[1] = -Kdlqr[1][0]*x[0] - Kdlqr[1][1]*x[1] - Kdlqr[1][2]*x[2];
    dU[1] += Kdlqr[1][3]*(Vref[0] - x[3]) + Kdlqr[1][4]*(Vref[1] - x[4]) + Kdlqr[1][5]*(Vref[2] - x[5]);

    dU[2] = -Kdlqr[2][0]*x[0] - Kdlqr[2][1]*x[1] - Kdlqr[2][2]*x[2];
    dU[2] += Kdlqr[2][3]*(Vref[0] - x[3]) + Kdlqr[2][4]*(Vref[1] - x[4]) + Kdlqr[2][5]*(Vref[2] - x[5]);

    U[0] = dU[0] + U[0];
    U[1] = dU[1] + U[1];
    U[2] = dU[2] + U[2];

    if(SobreCorrente1<=5){

        //Aqui eh preciso converter de tens�o para PWM
        aux = (int) 255-(255/6)*(6-U[0]);

        if(aux<0){

            aux=-1*aux;
            mPORTEClearBits(BIT_3);//E=0;

        }
        else {

            mPORTESetBits(BIT_3);//E=0;

        }

	if(aux>254){
            aux=254;
        }

        OC3RS=aux;

    }

    if(SobreCorrente2<=5){

        //Aqui eh preciso converter de tens�o para PWM
        aux = (int) 255-(255/6)*(6-U[1]);

        if(aux<0){

            aux=-1*aux;
            mPORTEClearBits(BIT_2);//C=0;

        }
        else {

            mPORTESetBits(BIT_2);//C=8;

        }

	if(aux>254){
            aux=254;
        }

        OC2RS=aux;

    }

    if(SobreCorrente3<=5){

        //Aqui eh preciso converter de tens�o para PWM
        aux = (int) 255-(255/6)*(6-U[2]);

        if(aux<0){

            aux=-1*aux;
            mPORTEClearBits(BIT_4);//A=0;

        }
        else {

            mPORTESetBits(BIT_4);//A=32;

        }

	if(aux>254){
            aux=254;
        }

        OC4RS=aux;

    }

}
