/*
 * Bit_Position.h
 *
 *  Created on: Aug 26, 2020
 *      Author: 91726
 */

#ifndef BIT_POSITION_H_
#define BIT_POSITION_H_

#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR					3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI					8
#define SPI_CR1_SSM					9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_DFF					11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7

#define	SPI_SR_RXNE					0
#define	SPI_SR_TXE					1
#define	SPI_SR_CHSIDE				2
#define	SPI_SR_UDR					3
#define	SPI_SR_CRCERR				4
#define	SPI_SR_MODF					5
#define	SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define	SPI_SR_FRE					8

/******************************************BIT POSTIONS FOR SYSTICK TIMERS*************************/

//READ-WRITE
#define SYST_CSR_ENABLE						0
#define SYST_CSR_TICKINT					1
#define SYST_CSR_CLKSOURCE					2
#define SYST_CSR_COUNTFLAG					16

//READ-WRITE
#define SYST_RVR_RELOAD						0

//READ-WRITE (CURRENT VALUE REGISTER) (WRITING ANY VALUE CLEARS THE REGISTER AND CLEARS THE SYST_CSR COUNTFLAG BIT TO 0)
#define SYST_CVR_CURRENT					0

//READ ONLY
#define SYST_CALIB_TENMS					0
#define SYST_CALIB_SKEW						30
#define SYST_CALIB_NOREF					31

/*************************************************************************************************/

/***********************************************I2C BIT POSITIONS***********************************************/

#define I2C_CR1_PE						0
#define I2C_CR1_SMBUS					1
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_ENARP					4
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENGC					6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_POS						11
#define I2C_CR1_PEC						12
#define I2C_CR1_ALERT					13
#define I2C_CR1_SWRST					15


#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12

#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_SR1_SMBALERT				15

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC						8

#define I2C_CCR_CCR						0
#define I2C_CCR_DUTY					14
#define I2C_CCR_FS						15

/***************************************************************************************************************/

/************************************************USART BIT POSITIONS*****************************************/
#define USART_SR_PE						0
#define USART_SR_FE						1
#define USART_SR_NF						2
#define USART_SR_ORE					3
#define USART_SR_IDLE					4
#define USART_SR_RXNE					5
#define USART_SR_TC						6
#define USART_SR_TXE					7
#define USART_SR_LBD					8
#define USART_SR_CTS					9

#define USART_DATA_REG					0

#define USART_BRR_DIV_FRACTION			0
#define USART_BRR_DIV_MANTISSA			4

#define USART_CR1_SBK					0
#define USART_CR1_RWU					1
#define USART_CR1_RE					2
#define USART_CR1_TE					3
#define USART_CR1_IDLEIE				4
#define USART_CR1_RXNEIE				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE					8
#define USART_CR1_PS					9
#define USART_CR1_PCE					10
#define USART_CR1_WAKE					11
#define USART_CR1_M						12
#define USART_CR1_UE					13			//14 is skipped because it was RESERVED
#define USART_CR1_OVER8					15

#define USART_CR2_ADD					0
#define USART_CR2_LBDL					5			//skipped bits are RESERVED
#define USART_CR2_LBDIE					6
#define USART_CR2_LBCL					8
#define USART_CR2_CPHA					9
#define USART_CR2_CPOL					10
#define USART_CR2_CLKEN					11
#define USART_CR2_STOP					12
#define USART_CR2_LINEN					14

#define USART_CR3_EIE					0
#define USART_CR3_IREN					1
#define USART_CR3_IRLP					2
#define USART_CR3_HDSEL					3
#define USART_CR3_NACK					4
#define USART_CR3_SCEN					5
#define USART_CR3_DMAR					6
#define USART_CR3_DMAT					7
#define USART_CR3_RTSE					8
#define USART_CR3_CTSE					9
#define USART_CR3_CTSIE					10
#define USART_CR3_ONEBIT				11


/*************************************************************************************************************/

/**************************************************ADC********************************************/
#define ADC_SR_AWD						0
#define ADC_SR_EOC						1
#define ADC_SR_JEOC						2
#define ADC_SR_JSTRT					3
#define ADC_SR_STRT						4
#define ADC_SR_OVR						5

#define ADC_CR1_AWDCH					0
#define ADC_CR1_EOCIE					5
#define ADC_CR1_AWDIE					6
#define ADC_CR1_JEOCIE					7
#define ADC_CR1_SCAN					8
#define ADC_CR1_AWDSGL					9
#define ADC_CR1_JAUTO					10
#define ADC_CR1_DISCEN					11
#define ADC_CR1_JDISCEN					12
#define ADC_CR1_DISCNUM					13
#define ADC_CR1_JAWDEN					22
#define ADC_CR1_AWDEN					23
#define ADC_CR1_OVRIE					26

#define ADC_CR2_ADON					0
#define ADC_CR2_CONT					1
#define ADC_CR2_DMA						8
#define ADC_CR2_DDS						9
#define ADC_CR2_EOCS					10
#define ADC_CR2_ALIGN					11
#define ADC_CR2_JEXTSEL					16
#define ADC_CR2_JEXTEN					20
#define ADC_CR2_JSWSTART				22
#define ADC_CR2_EXTSEL					24
#define ADC_CR2_EXTEN					28
#define ADC_CR2_SWSTART					30

#define ADC_DataReg							0

#define ADC_CSR_ADC1_AWD					0
#define ADC_CSR_ADC1_EOC					1
#define ADC_CSR_ADC1_JEOC					2
#define ADC_CSR_ADC1_JSTRT					3
#define ADC_CSR_ADC1_STRT					4
#define ADC_CSR_ADC1_OVR					5

#define ADC_CSR_ADC2_AWD					8
#define ADC_CSR_ADC2_EOC					9
#define ADC_CSR_ADC2_JEOC					10
#define ADC_CSR_ADC2_JSTRT					11
#define ADC_CSR_ADC2_STRT					12
#define ADC_CSR_ADC2_OVR					13

#define ADC_CSR_ADC3_AWD					16
#define ADC_CSR_ADC3_EOC					17
#define ADC_CSR_ADC3_JEOC					18
#define ADC_CSR_ADC3_JSTRT					19
#define ADC_CSR_ADC3_STRT					20
#define ADC_CSR_ADC3_OVR					21

#define ADC_CCR_MULTI						0
#define ADC_CCR_DELAY						8
#define ADC_CCR_DDS							13
#define ADC_CCR_DMA							14
#define ADC_CCR_ADCPRE						16
#define ADC_CCR_VBATE						22
#define ADC_CCR_TSVREFE						23

#define ADC_CDR_DATA1						0
#define ADC_CDR_DATA2						16

/****************************************************************************************************/

#endif /* BIT_POSITION_H_ */
