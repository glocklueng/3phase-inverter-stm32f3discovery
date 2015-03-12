/*! \file motor_hw.c */
#include "motor_hw.h"

#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_opamp.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_dma.h"

#include "stm32f30x_misc.h"

#include "dataAcq.h"

// private variables

static void (* sampleCallback)( void);//< function pointer to calback function

// function prototypes private
static void motorInitSpeedTimer( void);
inline void motorSetADCSamplingPoint( uint16_t duty);

void ADC1_2_IRQHandler( void);


/*! \brief This function initializes internal OPAMPS in stand-alone mode and DC bus measurement pin into analog mode
 *
 * ingroup MotorHW
 * \warning Hardware dependent */
static void motorInitOA( void)
{

	GPIO_InitTypeDef        GPIO_InitStructure;
	OPAMP_InitTypeDef       OPAMP_InitStructure;

	/* GPIOA GPIO B Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	/*  OPAMP1 GPIO in analog mode - Phase A */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*  OPAMP3 GPIO in analog mode - Phase C */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*  OPAMP4 GPIO in analog mode - Phase B */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* GPIO DC bus analog mode ADC2_IN1 na PA4 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* OPAMP Peripheral clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* OPAMP1 config */
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
	OPAMP_InitStructure.OPAMP_InvertingInput =  OPAMP_InvertingInput_IO2;
	OPAMP_Init(OPAMP_Selection_OPAMP1, &OPAMP_InitStructure);

	/* Enable OPAMP1 */
	OPAMP_Cmd(OPAMP_Selection_OPAMP1, ENABLE);

	/* OPAMP3 config */
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO4;
	OPAMP_InitStructure.OPAMP_InvertingInput =  OPAMP_InvertingInput_IO2;
	OPAMP_Init(OPAMP_Selection_OPAMP3, &OPAMP_InitStructure);

	/* Enable OPAMP3 */
	OPAMP_Cmd(OPAMP_Selection_OPAMP3, ENABLE);

	/* OPAMP4 config */
	OPAMP_InitStructure.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO2;
	OPAMP_InitStructure.OPAMP_InvertingInput =  OPAMP_InvertingInput_IO1;
	OPAMP_Init(OPAMP_Selection_OPAMP4, &OPAMP_InitStructure);

	/* Enable OPAMP4 */
	OPAMP_Cmd(OPAMP_Selection_OPAMP4, ENABLE);

}

/*! Array holding ADC measured values updated by DMA
 *   ingroup MotorHW
 *  */
int ADCdata[4] = {0,0,0,0};

#define ADC1_DR_ADDRESS     0x50000040
#define ADC2_DR_ADDRESS     0x50000140
#define ADC3_DR_ADDRESS     0x50000440
#define ADC4_DR_ADDRESS     0x50000540


/*! \brief This function initializes ADC modules and DMA
 *
 * Trigger for AD conversion is compare4 of PWM timer module
 *
 *	doba pøevodu je 7.5 + 12.5 adc clock cycles 20 cycles pøi 36 MHz => 0.5e-6
 *
 * \ingroup MotorHW
 * \warning Hardware dependent */
static void motorInitADC( void)
{
	uint32_t calibration_value;
	uint32_t iterator;

	ADC_InitTypeDef        ADC_InitStructure;
	ADC_CommonInitTypeDef  ADC_CommonInitStructure;

	DMA_InitTypeDef        DMA_InitStructure;

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

	/* Configure the ADC clock => 36 MHz  */
	RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
	RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div2);

	/* Enable ADC1-4 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);


	/* DMA configuration */
	/* DMA1 Channel1 Init Test */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADCvalues.iA;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC2_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADCvalues.uDCB;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADCvalues.iC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC4_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADCvalues.iB;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_Init(DMA2_Channel2, &DMA_InitStructure);


	/* ADC Calibration procedure */
	ADC_VoltageRegulatorCmd(ADC1, ENABLE);
	ADC_VoltageRegulatorCmd(ADC2, ENABLE);
	ADC_VoltageRegulatorCmd(ADC3, ENABLE);
	ADC_VoltageRegulatorCmd(ADC4, ENABLE);


	ADC_StructInit(&ADC_InitStructure);


	//! TODO: this delay should be realised differen way, independent of system clock
	iterator = 0xFFF00000;
	do { iterator++; } while (iterator != 0);

	//! TODO: kalibrace se resetuje s režimem spánku :-/
	ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC1);

	ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC2);

	ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC3);

	ADC_SelectCalibrationMode(ADC4, ADC_CalibrationMode_Single);
	ADC_StartCalibration(ADC4);

	while(ADC_GetCalibrationStatus(ADC1) != RESET );
	calibration_value = ADC_GetCalibrationValue(ADC1);

	while(ADC_GetCalibrationStatus(ADC2) != RESET );
	calibration_value = ADC_GetCalibrationValue(ADC2);

	while(ADC_GetCalibrationStatus(ADC3) != RESET );
	calibration_value = ADC_GetCalibrationValue(ADC3);

	while(ADC_GetCalibrationStatus(ADC4) != RESET );
	calibration_value = ADC_GetCalibrationValue(ADC4);

	// zadostiuèinìní pro kompilátor
	calibration_value = calibration_value;

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_Circular;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;

	ADC_CommonInit(ADC1, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC3, &ADC_CommonInitStructure);
	ADC_CommonInit(ADC4, &ADC_CommonInitStructure);


	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_RisingEdge;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
	ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
	ADC_InitStructure.ADC_NbrOfRegChannel = 1;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_7; // TIM8 TRGO str 223

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);

	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_4; // TIM8 TRGO str 223
	ADC_Init(ADC3, &ADC_InitStructure);
	ADC_Init(ADC4, &ADC_InitStructure);

	/* ADC1 regular channel configuration - toto funguje asi v jiných pøípadech použití OPAMPù
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);// DCbus
	ADC_RegularChannelConfig(ADC3, ADC_Channel_17, 1, ADC_SampleTime_7Cycles5);
	ADC_RegularChannelConfig(ADC4, ADC_Channel_17, 1, ADC_SampleTime_7Cycles5);*/

	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5);// Ia
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);// DCbus
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);// Ic
	ADC_RegularChannelConfig(ADC4, ADC_Channel_3, 1, ADC_SampleTime_7Cycles5);// Ib


	/* Configures and enable the ADC DMA */
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC3, ADC_DMAMode_Circular);
	ADC_DMAConfig(ADC4, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_DMACmd(ADC2, ENABLE);
	ADC_DMACmd(ADC3, ENABLE);
	ADC_DMACmd(ADC4, ENABLE);

	// temporary interrupt
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_Cmd(ADC4, ENABLE);


	/* wait for ADRDY */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));
	while(!ADC_GetFlagStatus(ADC4, ADC_FLAG_RDY));

	/* Start ADC1 Software Conversion */
	ADC_StartConversion(ADC1);
	ADC_StartConversion(ADC2);
	ADC_StartConversion(ADC3);
	ADC_StartConversion(ADC4);

	/* Enable the DMA channel */
	DMA_Cmd(DMA1_Channel1, ENABLE);//ADC1
	DMA_Cmd(DMA2_Channel1, ENABLE);//ADC2
	DMA_Cmd(DMA2_Channel5, ENABLE);//ADC3
	DMA_Cmd(DMA2_Channel2, ENABLE);//ADC4
}



/*! \brief This initialize advanced timer TIM8 on STM32F3
 *
 *  Example usage:
 *  \code{c}
	uint16_t resolution;
	resolution = motorInitPWM( 20000, 11);
	\endcode
 * \param freq desired PWM frequency in Hz
 * \param dt dead time setting as register value *not* in seconds!
 * \return PWM resolution
 *  ingroup MotorHW
 * \warning Hardware dependent, LOCK mode selected - only one call after reset */
static uint16_t motorInitPWM( const uint16_t freq, const uint16_t dt)
{
	//! temporary code - timer initialization
	//! GPIO alternative function initialization
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
	NVIC_InitTypeDef    NVIC_InitStructure;

	uint16_t TimerPeriod = 0;
	uint16_t ChannelDefPulse = 0;

	/* GPIOA and GPIOB clocks enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* TIM8 clock enable je na APB2 na 72 MHz */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	/* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIOA Configuration: channel 1N, 2N, 3N  as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM pins to AF4 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_4);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_4);


	// Compare 4 AF 4 na PC9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_4);

	TimerPeriod = ((SystemCoreClock>>1) / freq) - 1; // sysclk je dìlen dvema protože center aligned

	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
	ChannelDefPulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 100); // výchozí duty cycle 50%

	/*! Dealing with PWM modes
	11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
	interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
	both when the counter is counting up or down.
	Note: It is not allowed to switch from edge-aligned mode to center-aligned mode as long as
	the counter is enabled (CEN=1)
	 */

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //<! toto nastavení umožòuje snížit periodu regulátorù rychlosti (n+1), nastavení 0 odpovídá dvounásobné frekvenci, nemá proto význam

	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	/* Channel 1, 2 and 3 Configuration in PWM mode, channel 4 synchronizuje ADC pøevodník */
	TIM_OCInitStructure.TIM_Pulse = ChannelDefPulse; //default duty cycle
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);


	TIM_OCInitStructure.TIM_OutputNState = TimerPeriod;
	TIM_OCInitStructure.TIM_Pulse = TimerPeriod*900/1000;
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);

	//TIM_OC5Init(TIM8, &TIM_OCInitStructure);

	/* TIM events and interrupts */
	//TIM_SelectOutputTrigger2(TIM8,TIM_TRGO2Source_OC4Ref); //< toto je událost, která zahájí vzorkování AD pøevodníku
	TIM_SelectOutputTrigger(TIM8,TIM_TRGOSource_OC4Ref); //< toto je událost která zajistí capture polohy

	/*! Dealing with deadtime
	 * This bit-field defines the duration of the dead-time inserted between the complementary
	outputs. DT correspond to this duration.
	DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
	DTG[7:5]=10x => DT=(64+DTG[5:0])x tdtg with Tdtg=2xtDTS.
	DTG[7:5]=110 => DT=(32+DTG[4:0])x tdtg with Tdtg=8xtDTS.
	DTG[7:5]=111 => DT=(32+DTG[4:0])x tdtg with Tdtg=16xtDTS.
	Example if TDTS=125ns (8MHz), dead-time possible values are:
	0 to 15875 ns by 125 ns steps,
	16 us to 31750 ns by 250 ns steps,
	32 us to 63us by 1 us steps,
	64 us to 126 us by 2 us steps
	*/

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = dt;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);


	// NVIC interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// NVIC interrupt
	/*NVIC_InitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);*/

	/* Main Output Enable */
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	/* TIM8 counter enable */
	TIM_Cmd(TIM8, ENABLE);


	return TimerPeriod;
}

static void motorInitENC( void)
{
	//NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* GPIOD clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	/* TIM2 GPIO configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect TIM pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_2);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF; // maximum
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// both channels  =>  4096 pulzù/ot
	TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);


	// tento kanál je pro index pulse - udìlá timer capture -> sledujeme jestli se mìní offset senzoru polohy
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	// pøipojení èasovaèe TIM8 (ITR1 vstup TIM2 èasovaèe) pøes TRC na input compare
	// Table 62. TIMx internal trigger connection
	TIM_SelectInputTrigger(TIM2,TIM_TS_ITR1);

	// tento kanál bude bude zachytávat polohu v dobì mìøení proudu
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	// tento kanál "invisible" -> propojí jeden ze vstupù na triger výstup TRGO - viz další
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x7;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	// výstup TRGO èasovaèe TIM2 dle capture jednotky 1
	// Compare Pulse - The trigger output send a positive pulse when the CC1IF flag is to be
	// set (even if it was already high), as soon as a capture or a compare match occurred. strana 494
	// even if it was already high => netøeba mazat pøíznak pøerušení
	TIM_SelectOutputTrigger(TIM2,TIM_TRGOSource_OC1);

	/* Enable the TIM2 global Interrupt */
	/*NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM Interrupts enable
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);*/

	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

/* timer 3 je použit jako èasová základna pro rychlostní regulátor a mìøení rychlosti */
static void motorInitSpeedTimer( void)
{
	//uint32_t TimerPeriod;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/*// TIM3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TimerPeriod = ((SystemCoreClock>>1) / freq) - 1; // sys core clk je dìlen dvema protože APB je dìlen dvìma

	//TIM3 timebase
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//Table 62. TIMx internal trigger connection
	//každý update èasovaèe zpùsobí input capture senzoru polohy
	TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);

	// TIM3 enable counter
	TIM_Cmd(TIM3, ENABLE);*/

	// další èasovaè na mìøení periody TIM4
	// TIM4 clock enable

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//TIM4 timebase
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 4;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// tento kanál pro periodu vstupem pro capture je TRC
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	// výbìr trc
	// pøipojení èasovaèe TIM2 na input compare èasovaèe TIM4
	// Table 62. TIMx internal trigger connection
	TIM_SelectInputTrigger(TIM4,TIM_TS_ITR1);

	// zpoždìní reakce na triger
	TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);

	// nastavení resetu na TRC
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);

	// NVIC konfigurace
	//Enable the TIM global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// interrupt na capture
	//TIM Interrupts enable
	//TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	// TIM4 enable counter
	TIM_Cmd(TIM4, ENABLE);

}

uint16_t initMotorHW( uint16_t samplerate)
{
	//! sensors initialization
	motorInitENC();
	motorInitSpeedTimer();
	motorInitOA();
	motorInitADC();

	ClearBuffer();

	//! output to actuator initialization
	MotorControlPWMResolution = motorInitPWM( samplerate, 11);
	motorSetADCSamplingPoint(MotorControlPWMResolution*900/1000);


	return MotorControlPWMResolution;
}

void initDebugGPIO( void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//GPIO_SetBits(GPIOD,GPIO_Pin_15);
	//GPIO_ResetBits(GPIOD,GPIO_Pin_15);
}

void motorAlign( void)
{
	volatile uint32_t iterator;

	// alignment - nastavíme pozici rotoru s osou alpha
	setAlfaBeta(200,0);

	// delay
	iterator = 0xFF000000;
	do { iterator++; } while (iterator != 0);

	// initial position - nastavíme výchozí úhel - tam kde je osa alpha
	TIM2->CNT = 1023;
	setAlfaBeta(0,0);
}


inline void motorSetPWMCompare( uint16_t a, uint16_t b, uint16_t c)
{
	TIM8->CCR1 = a;
	TIM8->CCR2 = b;
	TIM8->CCR3 = c;
}

inline void motorSetADCSamplingPoint( uint16_t duty)
{
	TIM8->CCR4 = duty;
}

inline int32_t motorGetPositionEl( void)
{
	return (((TIM2->CCR1)&0xFFF)-1023)*3;
}

inline int32_t motorGetPosition( void)
{
	return (TIM2->CNT)-1023;
}

uint32_t theta_before = 1023 ;


inline int16_t motorGetSpeedPeriod( void)
{
	uint32_t theta_temp = TIM2->CNT;
	int16_t result = 0;

	if (TIM4->SR & TIM_IT_CC1)
	{
		TIM4->SR = (uint16_t)~TIM_IT_CC1;
		result = TIM4->CCR1;
	}
	else if( TIM4->SR & TIM_IT_Update)
	{
		TIM4->SR = (uint16_t)~TIM_IT_Update;
		TIM4->CCR1 = 0;
		result = 0;
	}

	if (theta_temp < theta_before )
	{
		result *= -1;
	}

	theta_before = theta_temp;



	return result;
}

void motorSetSampleInterrupt( uint8_t offon, void (* controlFinction))
{
	if (offon>0)
	{
		ADC_ITConfig(ADC2,ADC_IT_EOC, ENABLE);
		sampleCallback = controlFinction;
	}
	else
	{
		ADC_ITConfig(ADC2,ADC_IT_EOC, DISABLE);
		sampleCallback = 0;
	}
}

void ADC1_2_IRQHandler( void)
{
	GPIOD->BSRR = GPIO_Pin_15;
	if (sampleCallback)
		sampleCallback();
	GPIOD->BRR = GPIO_Pin_15;

	DumpTrace();

	//ADC_ClearITPendingBit(ADC2,ADC_IT_EOC);
	ADC2->ISR |= ADC_IT_EOC;

}

inline void setAlfaBeta(int16_t alfa, int16_t beta)
{
    int32_t Ua;
    int32_t Ub;
    int32_t X, Y, Z;
    int32_t a,b,c;
    int16_t sector;

    // todo: je toto korektní, nebo je to nepøesné sqrt(3)?
    Ua = (222 * alfa) /128;
    Ub = - beta;

    X = Ub;
    Y = (Ua+Ub)/2;
    Z = (Ub-Ua)/2;

    //! volba sektoru
    sector = -1;
    if (Y<0)
    {
        if(Z<0)
        {
            sector = 5;
        }
        else
        {
            if (X>0)
            {
                sector = 3;
            }
            else
            {
                sector = 4;
            }
        }
    }
    else
    {
        if(Z<0)
        {
            if(X>0)
            {
                sector = 1;
            }
            else
            {
                sector = 6;
            }
        }
        else
        {
            sector = 2;
        }
    }

    //! stanovení jednotlivých Duty cycle
    // prohozeno y[1] a y[2], oproti originálu, proè?
    switch(sector)
    {
        case 1:
        case 4:
            a = MotorControlPWMResolution/4 + (MotorControlPWMResolution/2 + X - Z)/2;
            c = a + Z;
            b = c - X;
            break;
        case 2:
        case 5:
            a = MotorControlPWMResolution/4 + (MotorControlPWMResolution/2 + Y - Z)/2;
            c = a + Z;
            b = a - Y;
            break;
        case 3:
        case 6:
            a = MotorControlPWMResolution/4 + (MotorControlPWMResolution/2 + Y - X)/2;
            b = a - Y;
            c = b + X;
            break;
        default:
            a = MotorControlPWMResolution/2;
            b = MotorControlPWMResolution/2;
            c = MotorControlPWMResolution/2;
            break;
    }

    // pøenesení do PWM registrù
    //motorSetPWMCompare( a, b, c);
    TIM8->CCR1 = a;
    TIM8->CCR2 = b;
    TIM8->CCR3 = c;
}

