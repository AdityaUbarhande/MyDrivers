//Device specific
/*
 * stm32f4xx.h
 *
 *  Created on: May 9, 2020
 *  Author: Aditya Ubarhande
 */

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_


#include<stdint.h>

#define __vol volatile

//THE CLOCK IS
#define FCLCK_HSI							16000000		//16Mhz is converted to 16000000Hz (formula = multiply by 1000000 (six zeroes))W

/************************************************PROCESSOR CORTEX-M4 REGARDING DETAILS**************************************************/
/*****INTERRUPTS*******/

//NVIC ISER (INTERRUPT SET-ENABLE REGISTER) THIS IS A PROCESSOR SIDE REGISTER
#define NVIC_ISER0							((__vol uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vol uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vol uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vol uint32_t*)0xE000E10C)


//NVIC ICER (INTERRUPT CLEAR-ENABLE REGISTER) THIS IS A PROCESSOR SIDE REGISTER BASE ADDRESSES
#define NVIC_ICER0							((__vol uint32_t*)0XE000E180)
#define NVIC_ICER1							((__vol uint32_t*)0XE000E184)
#define NVIC_ICER2              			((__vol uint32_t*)0XE000E188)
#define NVIC_ICER3							((__vol uint32_t*)0XE000E18C)

//NVIC INTERRUPT PRIORITY REGISTER BASE ADDRESSE
#define NVIC_IPR							((__vol uint32_t*)0xE000E400)

#define IMPLEMENTED_BITS					4

/*******************/
/*************SYSTICK TIMERS***************/

#define SYST_CSR 					((__vol uint32_t*)0xE000E010)
#define SYST_RVR			 		((__vol uint32_t*)0xE000E014)
#define SYST_CVR 					((__vol uint32_t*)0xE000E018)
#define SYST_CALIB					((__vol uint32_t*)0xE000E01C)

/******************************************/

/*********************************************************************************************************************************/

//Base Addresses of Memory Elements of stm32f446re NUCLEO
#define EMBEDDED_FLASH_BASE_ADDR			0x08000000UL		//Base Address of Main Memory or FLASH
#define SRAM1_BASE_ADDR						0x20000000UL		//Base Address of SRAM_1
#define SRAM								0x20000000UL		//Base Address of Main RAM (SRAM_1)
#define SRAM2_BASE_ADDR						0x2001C000UL		//Base Address of SRAM_2
#define ROM_BASE_ADDR						0x1FFF0000UL 		//Base Address of System Memory or ROM

//Base Addresses of Buses of stm32f446re NUCLEO
#define PERIPHERAL_BASE_ADDR				0x40000000UL		 //Base Address of all Peripherals
#define APB1_BASE_ADDR						PERIPHERAL_BASE_ADDR //Base Address of APB_1 Bus
#define APB2_BASE_ADDR						0x40010000UL		 //Base Address of APB_2 Bus
#define AHB1_BASE_ADDR						0x40020000UL		 //Base Address of AHB_1 Bus.
#define AHB2_BASE_ADDR						0x50000000UL		 //Base Address of AHB_2 Bus
#define AHB3_BASE_ADDR						0xA0001000UL		 //Base Address of AHB_3 Bus

//Base Addresses of AHB_1 Peripherals
#define GPIOA_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x0000)   //Base Address of GPIO A Peripheral
#define GPIOB_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x0400)   //Base Address of GPIO B Peripheral
#define GPIOC_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x0800)   //Base Address of GPIO C Peripheral
#define GPIOD_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x0C00)	//Base Address of GPIO D Peripheral
#define GPIOE_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x1000)	//Base Address of GPIO E Peripheral
#define GPIOF_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x1400)	//Base Address of GPIO F Peripheral
#define GPIOG_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x1800)	//Base Address of GPIO G Peripheral
#define GPIOH_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x1C00)	//Base Address of GPIO H Peripheral
#define GPIOI_BASE_ADDR						(AHB1_BASE_ADDR	+ 0x2000)	//Base Address of GPIO I Peripheral


//Base Addresses of APB_1 Peripherals
#define SPI_2_BASE_ADDR						(APB1_BASE_ADDR	+ 0x3800)	//Base Address of SPI_2 Peripheral
#define	SPI_3_BASE_ADDR						(APB1_BASE_ADDR	+ 0x3C00)	//Base Address of SPI_3 Peripheral
#define USART_2_BASE_ADDR					(APB1_BASE_ADDR	+ 0x4400)	//Base Address of USART_2 Peripheral
#define USART_3_BASE_ADDR					(APB1_BASE_ADDR	+ 0x4800)	//Base Address of USART_3 Peripheral
#define UART_4_BASE_ADDR					(APB1_BASE_ADDR	+ 0x4C00)	//Base Address of UART_4 Peripheral
#define UART_5_BASE_ADDR					(APB1_BASE_ADDR	+ 0x5000)	//Base Address of UART_5 Peripheral
#define I2C_1_BASE_ADDR						(APB1_BASE_ADDR	+ 0x5400)	//Base Address of I2C_1 Peripheral
#define I2C_2_BASE_ADDR						(APB1_BASE_ADDR	+ 0x5800)	//Base Address of I2C_2 Peripheral
#define I2C_3_BASE_ADDR						(APB1_BASE_ADDR	+ 0x5C00)	//Base Address of I2C_3 Peripheral

//Base Addresses of APB_2 Peripherals
#define	USART_1_BASE_ADDR					(APB2_BASE_ADDR	+ 0x1000)	//Base Address of USART_1 Peripheral
#define USART_6_BASE_ADDR					(APB2_BASE_ADDR	+ 0x1400)	//Base Address of USART_2 Peripheral
#define	SPI_1_BASE_ADDR						(APB2_BASE_ADDR	+ 0x3000)	//Base Address of SPI_1 Peripheral
#define SPI_5_BASE_ADDR						(APB2_BASE_ADDR	+ 0x5000)	//Base Address of SPI_5 Peripheral
#define SPI_6_BASE_ADDR						(APB2_BASE_ADDR	+ 0x5400)	//Base Address of SPI_6 Peripheral
#define	SPI_4_BASE_ADDR						(APB2_BASE_ADDR	+ 0x3400)	//Base Address of SPI_4 Peripheral
#define SYSCFG_BASE_ADDR					(APB2_BASE_ADDR	+ 0x3800)	//Base Address of SYSCFG Peripheral
#define EXTI_BASE_ADDR 						(APB2_BASE_ADDR	+ 0x3C00)	//Base Address of EXTI Peripheral
#define ADC1_BASE_ADDR						(APB2_BASE_ADDR + 0x2000)	//Base Address of ADC1 Peripheral
#define ADC2_BASE_ADDR						(APB2_BASE_ADDR + 0x2100)	//Base Address of ADC2 Peripheral
#define ADC3_BASE_ADDR						(APB2_BASE_ADDR + 0x2200)	//Base Address of ADC3 Peripheral


//RCC engine Base Address
#define RCC_BASE_ADDR						0x40023800UL



//DUMMY MACROS
#define DUMMY_GPIO_CALL 							((GPIO_REGISTERS_t*)123456)    //to give a dummy call to a function

typedef struct
{
	//Configuring
	__vol uint32_t RCC_CR;
	__vol uint32_t RCC_PLLCFGR;
	__vol uint32_t RCC_CFGR;
	__vol uint32_t RCC_CIR;

	//Reset Register
	__vol uint32_t RCC_AHB1RSTR;
	__vol uint32_t RCC_AHB2RSTR;
	__vol uint32_t RCC_AHB3RSTR;

	      uint32_t RESERVED_1;

	__vol uint32_t RCC_APB1RSTR;
	__vol uint32_t RCC_APB2RSTR;

		  uint32_t RESERVED_2[2];

	//Enable Register
	__vol uint32_t RCC_AHB1ENR;
	__vol uint32_t RCC_AHB2ENR;
	__vol uint32_t RCC_AHB3ENR;

		 uint32_t RESERVED_3;

	__vol uint32_t RCC_APB1ENR;
	__vol uint32_t RCC_APB2ENR;

		 uint32_t RESERVED_4[2];


	//Low Power Enable Register
	__vol uint32_t RCC_AHB1LPENR;
	__vol uint32_t RCC_AHB2LPENR;
	__vol uint32_t RCC_AHB3LPENR;

		  uint32_t RESERVED_5;

	__vol uint32_t RCC_APB1LPENR;
	__vol uint32_t RCC_APB2LPENR;

		  uint32_t RESERVED_6[2];


	//Configuring
	__vol uint32_t RCC_BDCR;
	__vol uint32_t RCC_CSR;

	  	  uint32_t RESERVED_7[2];

	__vol uint32_t RCC_SSCGR;
	__vol uint32_t RCC_PLLI2SCFGR;

}RCC_REGISTERS_t;



//CLOCK ENABLE AND DISABLE FUNCTIONS AND MACROS FOR SYSCFG

#define SYSCFG_DI								0
#define SYSCFG_EN								1


typedef struct
{
	__vol uint32_t GPIOx_MODER;			//GPIO port mode register					OFFSET: 0x00
	__vol uint32_t GPIOx_OTYPER;		//GPIO port output type register			OFFSET: 0x04
	__vol uint32_t GPIOx_OSPEEDR;		//GPIO port output speed register			OFFSET: 0x08
	__vol uint32_t GPIOx_PUPDR;			//GPIO port pull-up/pull-down register		OFFSET: 0x0C
	__vol uint32_t GPIOx_IDR;			//GPIO port input data register				OFFSET: 0x10
	__vol uint32_t GPIOx_ODR;			//GPIO port output data register			OFFSET: 0x14
	__vol uint32_t GPIOx_BSRR;			//GPIO port bit set/reset register			OFFSET: 0x18
	__vol uint32_t GPIOx_LCKR;			//GPIO port configuration lock register		OFFSET: 0x1C
	__vol uint32_t GPIOx_AFRL;			//GPIO alternate function low register		OFFSET: 0x20
	__vol uint32_t GPIOx_AFRH;			//GPIO alternate function high register		OFFSET: 0x24

	//AFRL and AFRH can be written as AFR[2];
}GPIO_REGISTERS_t;


typedef struct
{
	__vol uint32_t EXTI_IMR;
	__vol uint32_t EXTI_EMR;
	__vol uint32_t EXTI_RTSR;
	__vol uint32_t EXTI_FTSR;
	__vol uint32_t EXTI_SWIER;
	__vol uint32_t EXTI_PR;
}EXTI_REGISTERS_t;


typedef struct
{
	__vol uint32_t SYSCFG_MEMRMP;
	__vol uint32_t SYSCFG_PMC;
	__vol uint32_t SYSCFG_EXTICR1;
	__vol uint32_t SYSCFG_EXTICR2;
	__vol uint32_t SYSCFG_EXTICR3;
	__vol uint32_t SYSCFG_EXTICR4;
		  uint32_t RESERVED[2];   //The 8 bytes are not in used in the SYSCFG REGISTERS thus [2] of 4 bytes each = 8 bytes
	__vol uint32_t SYSCFG_CMPCR;
}SYSCFG_REGISTERS_t;

/************************************************SPI PERIPHERAL**********************************************************************************/

typedef struct
{
	__vol uint32_t SPI_CR1;
	__vol uint32_t SPI_CR2;
	__vol uint32_t SPI_SR;
	__vol uint32_t SPI_DR;
	__vol uint32_t SPI_CRCPR;
	__vol uint32_t SPI_RXCRCR;
	__vol uint32_t SPI_TXCRCR;
	__vol uint32_t SPI_I2SCFGR;
	__vol uint32_t SPI_I2SPR;
}SPI_REGISTERS_t;

//SPI PERIPHERALS MACROS
#define SPI1 	((SPI_REGISTERS_t*)SPI_1_BASE_ADDR)
#define SPI2    ((SPI_REGISTERS_t*)SPI_2_BASE_ADDR)
#define SPI3 	((SPI_REGISTERS_t*)SPI_3_BASE_ADDR)
#define SPI4 	((SPI_REGISTERS_t*)SPI_4_BASE_ADDR)
#define SPI5 	((SPI_REGISTERS_t*)SPI_5_BASE_ADDR)
#define SPI6 	((SPI_REGISTERS_t*)SPI_6_BASE_ADDR)

#define SPI_MASTER		1
#define SPI_SLAVE		0

#define SPI_FULL_DUPLEX					0	//Full Duplex bus confguration
#define SPI_HALF_DUPLEX					1	//Half Duplex bus confguration
#define SPI_SIMPLEX_RXONLY				2	////Simplex (receive only) bus confguration

#define SPI_BAUD_DIV2					0
#define SPI_BAUD_DIV4					1
#define SPI_BAUD_DIV8					2
#define SPI_BAUD_DIV16					3
#define SPI_BAUD_DIV32					4
#define SPI_BAUD_DIV64					5
#define SPI_BAUD_DIV128					6
#define SPI_BAUD_DIV256					7

#define SPI_DFF_8BITS					0 		//to set the SPI DATA FRAME FORMAT (as 8 bits)
#define SPI_DFF_16BITS					1  		//to set the SPI DATA FRAME FORMAT (as 16 bits)

#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

#define SPI_SSM_HWM						0		//to set as hardware management
#define SPI_SSM_SWM						1		//to set as software management


//SPI INTERRUPTS
#define SPI1_IRQPRI						42
#define SPI2_IRQPRI						43
#define SPI3_IRQPRI						58

#define SPI1_IRQNUM						35
#define SPI2_IRQNUM						36
#define SPI3_IRQNUM						51

/**********************************************************************************************************************************/

/*********************************************************I2C PERIPHERAL*************************************************************************/

typedef struct
{
	__vol uint32_t I2C_CR1;
	__vol uint32_t I2C_CR2;
	__vol uint32_t I2C_OAR1;
	__vol uint32_t I2C_OAR2;
	__vol uint32_t I2C_DR;
	__vol uint32_t I2C_SR1;
	__vol uint32_t I2C_SR2;
	__vol uint32_t I2C_CCR;
	__vol uint32_t I2C_TRISE;
	__vol uint32_t I2C_FLTR;
}I2C_REGISTERS_t;


#define I2C1							((I2C_REGISTERS_t*)I2C_1_BASE_ADDR)
#define I2C2							((I2C_REGISTERS_t*)I2C_2_BASE_ADDR)
#define I2C3							((I2C_REGISTERS_t*)I2C_3_BASE_ADDR)

//SERIAL CLOCK SPEED
#define I2C_SM				 	 100000
#define I2C_FM_2K 			 	 200000
#define I2C_FM_4K			 	 300000

//I2C ACK CONTROL
#define ACK_ENABLE					1
#define ACK_DISABLE					0

//I2C FM DUTY CYCLE
#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1

//I2C ADDRESS BIT COUNT SELECTION (APPLICABLE ONLY WHEN THE DEVICE IS IN SLAVE MODE)
#define OWN_ADDRESS_7			0
#define OWN_ADDRESS_10			1

//I2C IRQ NUMBERS AND PRIORITIES
#define I2C1_EV_IT_PRI			38
#define I2C1_ER_IT_PRI			39
#define I2C2_EV_IT_PRI			40
#define I2C2_ER_IT_PRI			41

#define I2C1_EV_IT_NUM			31
#define I2C1_ER_IT_NUM			32
#define I2C2_EV_IT_NUM			33
#define I2C2_ER_IT_NUM			34

/******************************************************************************************************************************/
/****************************************************UART PERIPHERAL****************************************************/
typedef struct
{
	__vol uint32_t USART_SR;
	__vol uint32_t USART_DR;
	__vol uint32_t USART_BRR;
	__vol uint32_t USART_CR1;
	__vol uint32_t USART_CR2;
	__vol uint32_t USART_CR3;
	__vol uint32_t USART_GTPR;

}USART_REGISTERS_t;

//FOR ENABLING/DISABLING THE CLOCK FOR UART 4 AND UART 5 [only for USART_CLK_EN() and USART_CLK_DI()]
#define UART4_CLK_EN			19
#define UART5_CLK_EN			20

#define UART4_CLK_DI			19
#define UART5_CLK_DI			20

//FOR DE-INIT API
#define UART4_RST			19
#define UART5_RST			20

//USART HW_FLOW_CONTROL MACROS
#define RTS_ENABLE			1
#define CTS_ENABLE			2
#define RTS_CTS_ENABLE		3

//USART MODE
#define USART_MODE_TX				0
#define USART_MODE_RX				1
#define USART_MODE_TX_RX			2

//UART WORD LENGTH
#define WORD_LEN_8				8
#define WORD_LEN_9				9

//UART STOP BIT COUNT
#define STOP_BIT_0_5			1
#define STOP_BIT_1				0
#define STOP_BIT_1_5			3
#define STOP_BIT_2				2

//UART PARITY BIT
#define NO_PARITY				2
#define EVEN_PARITY				0
#define ODD_PARITY				1

/********************************************************************************************************************************/

/****************************************************************ADC***********************************************************/
typedef struct
{
	__vol uint32_t ADC_SR;
	__vol uint32_t ADC_CR1;
	__vol uint32_t ADC_CR2;
	__vol uint32_t ADC_SMPR1;
	__vol uint32_t ADC_SMPR2;
	__vol uint32_t ADC_JOFR1;
	__vol uint32_t ADC_JOFR2;
	__vol uint32_t ADC_JOFR3;
	__vol uint32_t ADC_JOFR4;
	__vol uint32_t ADC_HTR;
	__vol uint32_t ADC_LTR;
	__vol uint32_t ADC_SQR1;
	__vol uint32_t ADC_SQR2;
	__vol uint32_t ADC_SQR3;
	__vol uint32_t ADC_JSQR;
	__vol uint32_t ADC_JDR1;
	__vol uint32_t ADC_JDR2;
	__vol uint32_t ADC_JDR3;
	__vol uint32_t ADC_JDR4;
	__vol uint32_t ADC_DR;
}ADC_REGISTERS_t;

typedef struct	   //THIS STRUCT IS FOR THE COMMON REGISTERS OF ADC WHICH ARE OF 4 BYTES EACH WITH ADDRESS: BASE ADDRESS OF ADC1 + 0X300
{
	__vol uint32_t ADC_CSR;
	__vol uint32_t ADC_CCR;
	__vol uint32_t ADC_CDR;
}ADC_COMMON_REGISTERS_t;

//ADC PERIPHERALS:
#define ADC1						((ADC_REGISTERS_t*)ADC1_BASE_ADDR)
#define ADC2						((ADC_REGISTERS_t*)ADC2_BASE_ADDR)
#define ADC3						((ADC_REGISTERS_t*)ADC3_BASE_ADDR)

//ADC COMMON REGISTERS:
#define ADC_COMMON_BASE_ADDR		(ADC1_BASE_ADDR + 0x300)	//Base Address for the ADC Peripheral Common Registers
#define ADC_COMMON					((ADC_COMMON_REGISTERS_t*)ADC_COMMON_BASE_ADDR)

//ADC FLAGS:
#define	ADC_FLAG_EOC				(1 << 1)
#define	ADC_FLAG_STRT				(1 << 4)

//ADC CHANNELS:
#define ADC_IN0						0
#define ADC_IN1						1
#define ADC_IN2						2
#define ADC_IN3						3
#define ADC_IN4						4
#define ADC_IN5						5
#define ADC_IN6						6
#define ADC_IN7						7
#define ADC_IN8						8
#define ADC_IN9						9
#define ADC_IN10					10
#define ADC_IN11					11
#define ADC_IN12					12
#define ADC_IN13					13
#define ADC_IN14					14
#define ADC_IN15					15
#define ADC_IN16					16
#define ADC_IN17					17
#define ADC_IN18					18

//SAMPLING TIME MACROS:
#define CYCLES_3					0
#define CYCLES_15					1
#define CYCLES_28					2
#define CYCLES_56					3
#define CYCLES_84					4
#define CYCLES_112					5
#define CYCLES_144					6
#define CYCLES_480					7

/********************************************************************************************************************************/

//PERIPHERAL MACROS
#define Rcc_Registers    					((RCC_REGISTERS_t*)RCC_BASE_ADDR)
#define EXTI 								((EXTI_REGISTERS_t*)EXTI_BASE_ADDR)
#define SysCfg								((SYSCFG_REGISTERS_t*)SYSCFG_BASE_ADDR)

//Ready-to-use GPIO Peripherals
#define GPIOA ((GPIO_REGISTERS_t*)GPIOA_BASE_ADDR)
#define GPIOB ((GPIO_REGISTERS_t*)GPIOB_BASE_ADDR)
#define GPIOC ((GPIO_REGISTERS_t*)GPIOC_BASE_ADDR)
#define GPIOD ((GPIO_REGISTERS_t*)GPIOD_BASE_ADDR)
#define GPIOE ((GPIO_REGISTERS_t*)GPIOE_BASE_ADDR)
#define GPIOF ((GPIO_REGISTERS_t*)GPIOF_BASE_ADDR)
#define GPIOG ((GPIO_REGISTERS_t*)GPIOG_BASE_ADDR)
#define GPIOH ((GPIO_REGISTERS_t*)GPIOH_BASE_ADDR)
#define GPIOI ((GPIO_REGISTERS_t*)GPIOI_BASE_ADDR)


//Generic Macros

#define ENABLE 1
#define DISABLE 0

#define SET 1
#define RESET 0

#define ON  1
#define OFF 0

#define LOOP 	0x0

#define PRESSED  1
#define NOT_PRESSED 0

//Output type Macros
#define PUSH_PULL  0
#define OPEN_DRAIN 1



//Mode Macros
#define INPUT				0
#define OUTPUT  			1
#define ALTERNATE_FN 		2
#define ANALOG				3
#define INTERRUPT_IT_FT 	4	//Falling interrupt mode
#define INTERRUPT_IT_RT 	5	//Rising interrupt mode
#define INTERRUPT_IT_RFT 	6	//Both Rising interrupt mode and Falling  interrupt mode

//Speed Macros
#define SPEED_LOW			0
#define SPEED_MEDIUM		1
#define SPEED_HIGH			2
#define SPEED_VERY_HIGH		3

//Pull_up and Pull_down Macros
#define GPIO_PIN_NO_PUPD		0
#define GPIO_PIN_PULL_UP		1
#define GPIO_PIN_PULL_DOWN		2
#define GPIO_PIN_RESERVED		3

/************************************************PIN MACROS START**************************************************/

//GPIO PORT A
#define PA0							0
#define PA1							1
#define PA2							2
#define PA3							3
#define PA4							4
#define PA5							5
#define PA6							6
#define PA7							7
#define PA8							8
#define PA9							9
#define PA10						10
#define PA11						11
#define PA12						12
#define PA13						13
#define PA14						14
#define PA15						15

//GPIO PORT B
#define PB0							0
#define PB1							1
#define PB2							2
#define PB3							3
#define PB4							4
#define PB5							5
#define PB6							6
#define PB7							7
#define PB8							8
#define PB9							9
#define PB10						10
#define PB11						11
#define PB12						12
#define PB13						13
#define PB14						14
#define PB15						15

//GPIO PORT C
#define PC0							0
#define PC1							1
#define PC2							2
#define PC3							3
#define PC4							4
#define PC5							5
#define PC6							6
#define PC7							7
#define PC8							8
#define PC9							9
#define PC10						10
#define PC11						11
#define PC12						12
#define PC13						13
#define PC14						14
#define PC15						15


//GPIO PORT D
#define PD0							0
#define PD1							1
#define PD2							2
#define PD3							3
#define PD4							4
#define PD5							5
#define PD6							6
#define PD7							7
#define PD8							8
#define PD9							9
#define PD10						10
#define PD11						11
#define PD12						12
#define PD13						13
#define PD14						14
#define PD15						15

//GPIO PORT E
#define PE0							0
#define PE1							1
#define PE2							2
#define PE3							3
#define PE4							4
#define PE5							5
#define PE6							6
#define PE7							7
#define PE8							8
#define PE9							9
#define PE10						10
#define PE11						11
#define PE12						12
#define PE13						13
#define PE14						14
#define PE15						15

//GPIO PORT F
#define PF0							0
#define PF1							1
#define PF2							2
#define PF3							3
#define PF4							4
#define PF5							5
#define PF6							6
#define PF7							7
#define PF8							8
#define PF9							9
#define PF10						10
#define PF11						11
#define PF12						12
#define PF13						13
#define PF14						14
#define PF15						15

//GPIO PORT G
#define PG0							0
#define PG1							1
#define PG2							2
#define PG3							3
#define PG4							4
#define PG5							5
#define PG6							6
#define PG7							7
#define PG8							8
#define PG9							9
#define PG10						10
#define PG11						11
#define PG12						12
#define PG13						13
#define PG14						14
#define PG15						15

//GPIO PORT H
#define PH0							0
#define PH1							1
#define PH2							2
#define PH3							3
#define PH4							4
#define PH5							5
#define PH6							6
#define PH7							7
#define PH8							8
#define PH9							9
#define PH10						10
#define PH11						11
#define PH12						12
#define PH13						13
#define PH14						14
#define PH15						15

//GPIO PORT I (ONLY 12 PINS)
#define PI0							0
#define PI1							1
#define PI2							2
#define PI3							3
#define PI4							4
#define PI5							5
#define PI6							6
#define PI7							7
#define PI8							8
#define PI9							9
#define PI10						10
#define PI11						11

/********************************************PIN MACROS END*******************************************/

//Alternate Functionality Macros
#define AF0		0
#define AF1		1
#define AF2		2
#define AF3		3
#define AF4		4
#define AF5		5
#define AF6		6
#define AF7		7
#define AF8		8
#define AF9		9
#define AF10	10
#define AF11	11
#define AF12	12
#define AF13	13
#define AF14	14
#define AF15	15

//SYSCFG Used Macros
#define GPIOA_INTERRUPT			0
#define GPIOB_INTERRUPT			1
#define GPIOC_INTERRUPT			2
#define GPIOD_INTERRUPT			3
#define GPIOE_INTERRUPT			4
#define	GPIOF_INTERRUPT			5
#define GPIOG_INTERRUPT			6
#define GPIOH_INTERRUPT			7
#define GPIOI_INTERRUPT			8

//INTERRUPT MACROS FOR EXTI IRQ NUMBERS
#define IRQNUM_EXTI_0					6
#define IRQNUM_EXTI_1					7
#define IRQNUM_EXTI_2					8
#define IRQNUM_EXTI_3					9
#define IRQNUM_EXTI_4					10
#define IRQNUM_EXTI_5_9					23
#define IRQNUM_EXTI_10_15				40


//INTERRUPT MACROS FOR EXTI IRQ PRIORITIES
#define IRQPRI_EXTI_0					13
#define IRQPRI_EXTI_1					14
#define IRQPRI_EXTI_2					15
#define IRQPRI_EXTI_3					16
#define IRQPRI_EXTI_4                   17
#define IRQPRI_EXTI_5_9					30
#define IRQPRI_EXTI_10_15               47

//EXTI VALUE MACROS
#define EXTI0							0
#define EXTI1							1
#define EXTI2							2
#define EXTI3							3
#define EXTI4							4
#define EXTI5							5
#define EXTI6							6
#define EXTI7							7
#define EXTI8							8
#define EXTI9							9
#define EXTI10							10
#define EXTI11							11
#define EXTI12							12
#define EXTI13							13
#define EXTI14							14
#define EXTI15							15

#endif /* INC_STM32F407XX_H_ */
