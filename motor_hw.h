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
 * Na co si d�t p��t� pozor?
 *
 * - TIM2/3/4/5 jde p�es d�li� dv�ma -> 36 MHz
 * - na encoderu byl prescaler 1 -> polovi�n� po�et pulz�
 * - jenom jeden TRGOx v�stup m��e b�t aktivn�
 *
 * - enkod�r mus� b�t modulo %4096 pak je rozsah do 4095, to odpov�d� logick� operaci &4095
 *
 * - vstupy p�evodn�ku musej� b�t mapov�ny na ananlogov� piny, ne na v�stupy OA, pokud jsou v re�imu s vn�j�� ZV
 *
 * Propojen� periferi�
 * -------------------
 *
 * - TIM8 advanced 3-ph PWM periferie �asova�e  TODO: repetition timer, kdy nast�v� update PWM registr�
 * 	-  output compare 1-3 complementar PWM v�stupy s deadtime
 * 	-  output compare 4 - TRGO ud�lost
 * 		-  ud�lost TRGO
 * 			-  TIM2 input capture polohy
 * 			-  ADC kompletn� p�eru�en� (data p�es DMA)
 * 					-  v�po�et regul�tor� proudu
 *
 * - TIM2 re�im encoder mode (CH1, CH2 vstupy generuj� hodiny)
 * 	-  Input capture 1 jednotka - direct vstup (A encoderu) - rising edge A bude generovat CC1IF
 * 		-  ud�lost TRGO
 * 			- TIM3 timer rychlosti
 * 	-  Input capture 3 jednotka - direct vstup Index pulse
 * 	-  Input capture 4 jednotka - TRC od TIM8 - bude zachyt�vat polohu v dob� m��en� proudu
 *
 * - TIM3 timer na pln� frekvenci pro vysok� rozli�en� ot��ek, rychl� reakci zastaven�
 * 	- input capture rychlosti
 * 	- reset s TRC po delay
 *
 * P�ipojen� HW
 * ------------
 * N�sleduj�c� ��dky shrnuj� zapojen� v�vd� mikrokontrol�ru a periferi�:
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
 * - Kontrola deadtime - Zm��eno 156ns deadtime (nastaveno 11); 1.39us odpov�d� 100 nastaveno
 * - Kontrola start-upu pomoc� log analyz�toru - jsou tam zeza��tku n�jak� glitch - divn� chov�n� - pro�?
 * - kontrola update ud�losti - stihne se je�t� zapsat ak�n� z�sah?
 * -
 *
 *  TODO
 *	----
	- Nahodil� p�eru�en� m��en� rychlosti - pro� - vy�e�it! - glitch na v�stupu enkod�ru!
	- Nulov�n� rychlosti p�i p�ete�en� �asova�e rychlosti

	- Float nadstavba
 *		- Implementace SVM (vstup nap�t� alfa a beta, korekce dle nap�t� DC bus, filtr nap�t� DCB)
 *		- Implementace m��en� proudu v alfa a beta sou�adnic�ch, selekce v�b�ru f�z� vhodn�ch k v�po�tu ( nap�. dle aplika�n� pozn�mky freescale pro qoriva)

	- Podpora p�evodn�k� v asynchronn�m modu (dal�� kan�ly mimo periodu vzorkov�n�)
	- Podpora GPIO vhodn�ch pro motor control - souvisej�c� v poruchami

 *	- Implementace mechanismu, kter� bude vyb�rat aktu�ln� dobu m��en� proudu p�i inicializaci
 *	- API pro inicializaci, deinicializaci (podpora re�imu sp�nku)
 *	- API pro nastaven� HW chybov�ho stavu, cybov� ud�losti, simulace chybov� ud�losti, obnoven� z chybov� ud�losti
 *		- p�idat kompar�tory na vstup brzdy (treshold nastav� DAC) -> over-current protection, over-voltage protection
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
