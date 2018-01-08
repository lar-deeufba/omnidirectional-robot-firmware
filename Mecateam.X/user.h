/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/

/* TODO Application specific user parameters used in user.c may go here */

/******************************************************************************/
/* User Function Prototypes                                                    /
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */
#define WriteUART3(data)	U3TXREG = (data)
#define putcUART3(c)           do{while(!U3STAbits.TRMT); WriteUART3((int)(c));}while(0)
#define ReadUART3()		(U3RXREG)

#define BV(n)      (1 << (n))


#define SENSOR_3   3	//	AN3

#define SENSOR_2   4    // 	AN4

#define SENSOR_1   5    // 	AN5

#define Giro       14    // 	AN15

#define Acelerometro_X 12 //    AN14

#define Acelerometro_Y 13 //    AN13



//#define FALSE       (0)
//#define TRUE

unsigned char RXBuffer[23];
unsigned char RXCount;
unsigned char FlagTempoDeAmostragem;
unsigned char Temporizador;
unsigned char Temp;
unsigned char Temp2;
unsigned char TempMotor1,TempMotor2,TempMotor3;
unsigned char fSend;
unsigned char fPID;
unsigned char fLQR;
unsigned char fOtimo;

float V_ref;
float Vn_ref;
float W_ref;

unsigned char ContBussola;

long int bussolaInical;
long int bussolaFinal;

long int bussola;


 unsigned char  Flag_RXReceived;

 struct Motor{
     unsigned char Controle;
     unsigned char Direcao;
     unsigned char DirecaoAtual;
     unsigned char Sincronismo;
     unsigned char Duty;
     unsigned char Estouro;
     float Kp;
     float Ki;
     float Kd;
     long int VReferencia;
     long int Velocidade;
     long int VelocidadeMedia[5];
     unsigned int VMotorLow;
     unsigned int VMotorHigh;
     long int MediaMovel;
     long int Corrente[5];
     long int MediaMovelCorrente;
    };

    extern struct Motor Motor_1;
    extern struct Motor Motor_2;
    extern struct Motor Motor_3;

    struct tSerial{
        unsigned char Vel_1[2];
        unsigned char Vel_2[2];
        unsigned char Vel_3[2];
        unsigned char Corrent_1[2];
        unsigned char Corrent_2[2];
        unsigned char Corrent_3[2];
        unsigned char Acer_X[2];
        unsigned char Acer_Y[2];
        unsigned char Giroscop[2];
        unsigned char Compass[2];
        unsigned char Direcao;
        unsigned char DutyCycle_1;
        unsigned char DutyCycle_2;
        unsigned char DutyCycle_3;
        unsigned char Setpoint_1[2];
        unsigned char Setpoint_2[2];
        unsigned char Setpoint_3[2];
        unsigned char DirecaoRef;


    };

extern struct tSerial Serial;


    extern unsigned char FlagTempoDeAmostragem;
    extern unsigned char Temporizador;
    extern unsigned int AcelerometroX[5];
    extern unsigned int AcelerometroY[5];
    extern unsigned int Giroscopio[5];
    extern long int AcelerometroXMovelCorrente;
    extern long int AcelerometroYMovelCorrente;
    extern long int GiroscopioMovelCorrente;

int SobreCorrente1,SobreCorrente2,SobreCorrente3;


void InitApp(void);         /* I/O and Peripheral Initialization */

int initADC( int amask);

int readADC( int ch);

void initPWM( void );

void EnviarDado(void);

void PID(void);

void LQR_smith(void);

void LQR_otimo(void);

