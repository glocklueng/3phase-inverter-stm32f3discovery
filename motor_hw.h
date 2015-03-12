#ifndef __MOTOR_HW_H__
#define __MOTOR_HW_H__

/*!
 * \defgroup MotorHW MotorHW Module
 * @{
 * \author Lukas Otava
 * \date 2014
 *
 *
 * \brief Motor control related HW layer
 *
 *  Motor control related HW layer
 *  Initialization functions, update functions and ADC interface
 *
 *
 * Chyby v SW a HW
 * --------------
 *
 * Na co si dát pøíštì pozor?
 *
 * - TIM2/3/4/5 jde pøes dìliè dvìma -> 36 MHz
 * - na encoderu byl prescaler 1 -> polovièní poèet pulzù
 * - jenom jeden TRGOx výstup mùže být aktivní
 *
 * - enkodér musí být modulo %4096 pak je rozsah do 4095, to odpovídá logické operaci &4095
 *
 * - vstupy pøevodníku musejí být mapovány na ananlogové piny, ne na výstupy OA, pokud jsou v režimu s vnìjší ZV
 *
 * Propojení periferií
 * -------------------
 *
 * - TIM8 advanced 3-ph PWM periferie èasovaèe  TODO: repetition timer, kdy nastává update PWM registrù
 * 	-  output compare 1-3 complementar PWM výstupy s deadtime
 * 	-  output compare 4 - TRGO událost
 * 		-  událost TRGO
 * 			-  TIM2 input capture polohy
 * 			-  ADC kompletní pøerušení (data pøes DMA)
 * 					-  výpoèet regulátorù proudu
 *
 * - TIM2 režim encoder mode (CH1, CH2 vstupy generují hodiny)
 * 	-  Input capture 1 jednotka - direct vstup (A encoderu) - rising edge A bude generovat CC1IF
 * 		-  událost TRGO
 * 			- TIM3 timer rychlosti
 * 	-  Input capture 3 jednotka - direct vstup Index pulse
 * 	-  Input capture 4 jednotka - TRC od TIM8 - bude zachytávat polohu v dobì mìøení proudu
 *
 * - TIM3 timer na plné frekvenci pro vysoké rozlišení otáèek, rychlé reakci zastavení
 * 	- input capture rychlosti
 * 	- reset s TRC po delay
 *
 * Pøipojení HW
 * ------------
 * Následující øádky shrnují zapojení vývdù mikrokontroléru a periferií:
 *
 * pinout PWM
 * - phase a : top PC6 (PWM TIM8_CH1); bottom PC10 (PWM TIM8_CH1N)
 * - phase b : top PC7 (PWM TIM8_CH2); bottom PC11 (PWM TIM8_CH2N)
 * - phase c : top PC8 (PWM TIM8_CH3); bottom PC12 (PWM TIM8_CH3N)
 *
 *  all Alternative function AF4
 *
 * pinout Encoder
 * - PD3-A-TIM2_CH1
 * - PD4-B-TIM2_CH2
 * - PD7-index-TIM2_CH3
 * ALTFUNC2
 *
 * pinout OPAMP
 *
 * pinout ADC
 * - A		PA2		ADC1_IN3
 * - DCB	PA4		ADC2_IN1
 * - C		PB1		ADC3_IN1
 * - B		PB12	ADC4_IN3
 *
 * Testy
 * -----
 * - Kontrola deadtime - Zmìøeno 156ns deadtime (nastaveno 11); 1.39us odpovídá 100 nastaveno
 * - Kontrola start-upu pomocí log analyzátoru - jsou tam zezaèátku nìjaké glitch - divné chování - proè?
 * - kontrola update události - stihne se ještì zapsat akèní zásah?
 * -
 *
 *  TODO
 *	----
	- Nahodilé pøerušení mìøení rychlosti - proè - vyøešit! - glitch na výstupu enkodéru!
	- Nulování rychlosti pøi pøeteèení èasovaèe rychlosti

	- Float nadstavba
 *		- Implementace SVM (vstup napìtí alfa a beta, korekce dle napìtí DC bus, filtr napìtí DCB)
 *		- Implementace mìøení proudu v alfa a beta souøadnicích, selekce výbìru fází vhodných k výpoètu ( napø. dle aplikaèní poznámky freescale pro qoriva)

	- Podpora pøevodníkù v asynchronním modu (další kanály mimo periodu vzorkování)
	- Podpora GPIO vhodných pro motor control - související v poruchami

 *	- Implementace mechanismu, který bude vybírat aktuální dobu mìøení proudu pøi inicializaci
 *	- API pro inicializaci, deinicializaci (podpora režimu spánku)
 *	- API pro nastavení HW chybového stavu, cybové události, simulace chybové události, obnovení z chybové události
 *		- pøidat komparátory na vstup brzdy (treshold nastaví DAC) -> over-current protection, over-voltage protection
 *
 *  \f{equation*}{ \alpha \f}
 *
 * */

#include <stdint.h>
#include <stdio.h>


/*! \brief This function initializes all motor control related hardware
 * \param samplerate sample rate in Hz
 * \return PWM resolution
 *   */
uint16_t initMotorHW( uint16_t samplerate);

/*! debug init */
void initDebugGPIO( void);

/*!
 * Function providing alignment and position calibration
 * */
void motorAlign( void);


/*!
 * This function sets duty cycles for corresponding outputs
 *
 * \param a Phase a duty cycle
 * \param b Phase b duty cycle
 * \param c Phase c duty cycle
 * */
inline void motorSetPWMCompare( uint16_t a, uint16_t b, uint16_t c);


/*! This function do SVM algorithm and write PWM registers
 * values are in range <-MotorControlPWMResolution; + MotorControlPWMResolution>
 * \param alfa duty cycle in \f$ \alpha \f$ coordinate
 * \param beta duty cycle in \f$ \beta \f$ coordinate
 * */
inline void setAlfaBeta(int16_t alfa, int16_t beta);

/*! this function returns mechanical position as 32 bit integer */
inline int32_t motorGetPosition( void);

/*! this function returns actual electrical position */
inline int32_t motorGetPositionEl( void);

/*! this function returns period tick count representing speed */
inline int16_t motorGetSpeedPeriod( void);


/*! This function force interrupt ADC measure complete interrupt off or on
 * \param offon values: 0 - off, 1 - on
 * \param controlFinction function which will be called with ADC measure complete interrupt
 * */
void motorSetSampleInterrupt( uint8_t offon, void (* controlFinction));


/*! Structure for holding ADC values */
struct ADCvaluesType
{
	int iA, iB, iC, //< phase currents ADC value
	uDCB; //< DC bus voltage ADC value
};

/*! variable that holds ADC readings */
struct ADCvaluesType ADCvalues;

//! Variable holding PWM resolution
//! TODO: \warning This is timer frequency dependent value,
int MotorControlPWMResolution;



/*! @} */
#endif//__MOTOR_HW_H__
