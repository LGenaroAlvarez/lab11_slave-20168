/*
 * File:   lab11_main-20168.c
 * Author: Luis Genaro Alvarez Sulecio 20168
 * 
 * Programa: Comunicacion master->slave con envio de datos de un potenciometro
 * leidos en el canal 1 (AN1), utilizando RA0 como selector de modo (maestro/esclavo)
 * y mostrando valores recibidos en puerto D
 *
 * Created on May 11, 2022, 5:17 AM
 */
// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF           // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF          // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF          // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF             // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF            // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF          // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF           // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF          // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF            // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V       // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF            // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

//DEFINICION DE FRECUENCIA PARA DELAY
#define _XTAL_FREQ 1000000          // FRECUENCIA PARA DELAYS (1MHz)

//DEFINICIONES GLOBALES
#define FLAG_SPI 0xFF
#define POT_MIN 0                   // VALOR MINIMO DE ENTRADA AL ADC POR POTENCIOMETRO
#define POT_MAX 255                 // VALOR MAXIMO DE ENTRADA AL ADC POR POTENCIOMETRO
#define PWM_MIN 62                  // VALOR MINIMO PARA PWM (0.4mS) PARA SERVO
#define PWM_MAX 125                 // VALOR MAXIMO PARA PWM (2.4mS) PARA SERVO

//DEFINICION DE ALIAS PARA PINES
#define incB PORTBbits.RB0
#define decB PORTBbits.RB1

//VARIABLES GLOBALES
uint8_t pwm_va;;                    // VARIABLE PARA VALOR DEL POTENCIOMETRO
unsigned short CCPR = 0;            // VARIABLE PARA CCPR1

//PROTO FUNCIONES
void setup(void);                   // FUNCION DE SETUP

// FUNCION PARA INTERPOLACION
unsigned short interpol(uint8_t val, uint8_t pot_min, uint8_t pot_max,
        unsigned short pwm_min, unsigned short pwm_max);

//INTERRUPCIONES
void __interrupt() isr(void){
    //----------------------------------SLAVE---------------------------------------------
    if (PIR1bits.SSPIF){            // REVISAR INTERRUPCION DE RECEPCION DE DATOS
        CCPR = interpol(SSPBUF, POT_MIN, POT_MAX, PWM_MIN, PWM_MAX);    // EJECUCION DE INTERPOLACION PARA ANCHO DE PULSO
        CCPR1L = (uint8_t)(CCPR>>2);                                    // ASIGNAR AL CPR1L LOS 8 BITS MAS SIGNIFICATIVOS
        CCP1CONbits.DC1B = CCPR & 0b11;                                 // ASIGNAR AL DC1B LOS 2 BITS MENOS SIGNIFICATIVOS
        
        PIR1bits.SSPIF = 0;         // LIMPIEZA DE BANDERA DE INTERRUPCION DE SPI
    }
   
    //------------------------------------------------------------------------------------
    return;
}

void main(void) {
    //EJECUCION CONFIG
    setup();

    while(1){
                                    // 
    }
    return;
}

//CONFIGURACION PRINCIPAL
void setup(void){
    ANSEL = 0;                      // I/O DIGITALES
    ANSELH = 0;                     // I/O DIGITALES

    TRISA = 0b00100000;             // RA5 COMO ENTRADA
    PORTA = 0;                      // LIMPIEZA DE PORTA

    //OSCCONFIG
    OSCCONbits.IRCF = 0b0100;       // FRECUENCIA DE OSCILADOR INTERNO (1MHz)
    OSCCONbits.SCS  = 1;            // RELOJ INTERNO

    //---------------------------------ESCLAVO----------------------------------

    TRISC = 0b00011000;             // ENTRADA DE DATOS Y SINCRONIZADOR DE RELOJ COMO ENTRADA, SALIDA DE DATOS COMO SALIDA
    PORTC = 0;                      // LIMPIEZA DE PORTD

    //SSPCON <5:0>
    SSPCONbits.SSPM = 0b0100;       // SS HABILITADO, SPI EN MODO ESCLAVO
    SSPCONbits.CKP = 0;             // RELOJ INACTIVO EN 0
    SSPCONbits.SSPEN = 1;           // HABILITACION DE PINES DE SPI
    //SSPSTAT <7:6>
    SSPSTATbits.CKE = 1;            // ENVIO DE DATO EN FLANCO DE SUBIDA
    SSPSTATbits.SMP = 0;            // DATO AL FINAL DE PULSO DE RELOJ (EN 0 DEBIDO A MODO ESCLAVO)

    //CONFIG PWM
    TRISCbits.TRISC2 = 1;           // CCP1 COMO ENTRADA (SALIDA DESABILITADA)
    PR2 = 255;                      // PERIODO DE TMR2 EN 16mS
    
    //CCP CONFIG
    CCP1CON = 0;                    // CCP1 APAGADO
    CCP1CONbits.P1M = 0;            // CAMBIO DE MODO A "SINGLE OUTPUT"
    CCP1CONbits.CCP1M = 0b1100;     // PWM PARA CCP1

    CCPR1L = 61;                    // CONFIGURACION DE ANCHO DE PULSO PARA CCP1
    CCP1CONbits.DC1B = 61 & 0b11;   // ANCHO DE PULSO BASE EN 1mS

    T2CONbits.T2CKPS = 0b11;        // RPESCALER DEL TMR2 EN 1:16
    PIR1bits.TMR2IF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCION DE TMR2
    T2CONbits.TMR2ON = 1;           // TMR2 ENCENDIDO
    while(!PIR1bits.TMR2IF);        // CICLO INDIVIDUAL DE TMR2 EN ESPERA
    PIR1bits.TMR2IF = 0;            // LIMPIEZA DE BANDERA DE INTERRUPCION DE TMR2

    TRISCbits.TRISC2 = 0;           // HABILITAR SALIDA DE PWM EN RC2

    //CONFIG DE INTERRUPCIONES
    PIR1bits.SSPIF = 0;             // LIMPIAR BANDERA DE INTERRUPCIONES DE SP1
    PIE1bits.SSPIE = 1;             // ACTIVAR INTERRUPCIONES DE SPI
    INTCONbits.GIE = 1;             // ACTIVAR INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;            // ACTIVAR INTERRUPCIONES DE PERIFERICOS
}

//INTERPOLACION DE VALORES
unsigned short interpol(uint8_t val, uint8_t pot_min, uint8_t pot_max,
        unsigned short pwm_min, unsigned short pwm_max){

    return (unsigned short)(pwm_min+((float)(pwm_max-pwm_min)/(pot_max-pot_min))
            *(val-pot_min));

}