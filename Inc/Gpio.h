/*
 * Gpio_driver.h
 *
 *  Created on: June 1, 2020
 *  Author: Aditya Technology
 *
 *  NOTE FOR USER:
 *
 *  1] THERE ARE TWO HEADER FILES FOR THE GPIO DRIVER THE MCU SPECIFIC HEADER FILE AND THE DRIVER SPECIFIC.
 *  2] PLEASE HAVE A LOOK AT THE APIs BEFORE CODING ON THE BOARD.
 *  3] INTERRUPTS HAVE THEIR SPECIAL APIs SO COFIGURE THE APIs BEFORE USING INTERRUPTS
 *  4] WHEN YOU ENABLE ANY INTERRUPTS, PLEASE CLEAR THE PENDING REGISTER BY THE IRQ_HANDLER API.
 *  5] DO NOT MAKE ANY CHANGES IN THE FILE WHAT SO EVER. ADITYA TECH WILL NOT BE HELD RESPONSIBLE FOR ANY PROBLEMS OCCURED IF CHANGES ARE MADE.
 *  6] PLEASE REFER THE DEBUG.txt TO GET SOME DEBUG TIPS FOR THIS DRIVER HEADER FILE.
 *
 *  !!!HAPPY CODING!!!
 *
 *  ALL RIGHTS RESERVED ADITYA TECHNOLOGY
 *  COPYRIGHT RESERVED (c) ADITYA TECHNOLOGY
 *
 */
///*************************************A NOTE FROM FOUNDER**********************************************///
//																										//
//YOU MIGHT GET DEBUGGING ERRORS IN YOUR CODING TO SOLVE THOSE WE HAVE PROVIDED HELP FROM OUR SIDE		//
//ALWAYS WHILE DEBUGGING OPEN THE ITM DATA CONSOLE TO GET TIPS ON DEBUGGING 							//
//ALSO CODE IS PROVIDED WITH A STEP BY STEP UPDATER IN THE ITM DATA CONSOLE 							//
//  																				//
///******************************************************************************************************///


#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include"stm32f407.h"
#include<stdio.h>

//DEBUG MACROS
#define INTERRUPT_FALLING_EDGE_CHECK		0x11
#define INTERRUPT_RISING_EDGE_CHECK  		0x12

//WHOLE PORT SETTING MACROS
int i, j;
//#define WS_INPUT				0
//#define WS_OUTPUT				for(j = 0 ; j <= 15; j++) { i = 2 * j;
//															GPIO->REG1 |= (1 << i);}

#define WS_ALTFN
#define WS_ANALOG				3

#define WS_PUSH_PULL			0
#define WS_OPEN_DRAIN			1

#define WS_LOW					0
#define WS_MEDIUM				1
#define WS_HIGH					2
#define WS_VERY_HIGH		 	3

#define WS_NO_PUPD				0
#define WS_PULL_UP				1
#define WS_PULL_DOWN			2
#define WS_RESERVED				3

#define WS_AF0					0
#define WS_AF1					1
#define WS_AF2					2
#define WS_AF3					3
#define WS_AF4					4
#define WS_AF5					5
#define WS_AF6					6
#define WS_AF7					7
#define WS_AF8					8
#define WS_AF9					9
#define WS_AF10					10
#define WS_AF11					11
#define WS_AF12					12
#define WS_AF13					13
#define WS_AF14					14
#define WS_AF15					15


 typedef struct
{

	uint32_t gpiox_moder;
	uint32_t gpiox_otyper;
	uint32_t gpiox_speed;
	uint32_t gpiox_pupdr;
	uint32_t gpiox_lckr;
	uint32_t pin_number;
	uint32_t alt_func_en_or_di;
	uint32_t gpiox_alt_choice;

}GPIO_PIN_CONFIG_t;


typedef struct
{
	GPIO_REGISTERS_t *Gpio_Registers;
	GPIO_PIN_CONFIG_t Gpio_Config;

}GPIO_HANDLE_t;

void delay( int num); 				//delay prototype (software delay)
void delay(int num)
{
	for( int i = 0; i <= num; i++)
	{
		for( int a = 0; a <= i; a++);
	}
}
/***************************************Clock Enable AND Disable Macros***********************************************/

void GPIO_PCLCK_EN(GPIO_REGISTERS_t *Gpio_Registers); //to enable the clock
void GPIO_PCLCK_DI(GPIO_REGISTERS_t *Gpio_Registers); //to disable the clock
void SYSCFG_CLCK_EN();
void SYSCFG_CLCK_DI();

/**************************************Init AND De-Init Macros*******************************************************/
void GPIO_INIT(GPIO_HANDLE_t *pGpio_Handle);         	//set the configurations like the output speed pull-up and pull-down etc
void GPIO_DEINIT(GPIO_REGISTERS_t *Gpio_Registers); 	//to erase the mode and configurations

/**************************************INTERRUPT Macros*******************************************************/
void IRQ_EDGE_SETTING(uint16_t edge_selection, uint8_t pin_num);
void IRQ_CONFIG(uint16_t GPIOx_INTERRUPT, uint8_t pin);
void IRQ_NVIC_CONFIG(uint16_t IRQ_num, uint8_t EN_or_DI);
void IRQ_PRIORITY_CONFIG(uint16_t IRQ_num, uint16_t IRQ_priority);
void IRQ_HANDLING(uint8_t PinNumber);



/************************************** Pin-Read AND Write Macros *******************************************************/
void WRITE_DATA_OUTPIN(GPIO_REGISTERS_t *Gpio_Registers, uint32_t data, uint32_t pin_number);
void WRITE_DATA_OUTPORT(GPIO_REGISTERS_t *Gpio_Registers, uint32_t data);
void DATA_BSRR(GPIO_REGISTERS_t *Gpio_Registers, uint8_t data, uint8_t pin);

void TOGGLE_PIN(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin_number);

int READ_DATA_INPIN(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin_number);
int READ_DATA_INPORT(GPIO_REGISTERS_t *Gpio_Registers);

/**************************************Register Clearing Macros*******************************************************/
void CLEAR_OUTPUT_REG(GPIO_REGISTERS_t *Gpio_Registers);

/**************************************Featured Macros*****************************************************************/
void LED_BLINK(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin,  uint32_t blink_count, uint32_t blink_time);

int INTERRUPTS_DEBUG( uint16_t debug_argument, uint16_t pin);
/**********************************************************************************************************************/
/*****************************************WHOLE PORT SETTING MACROS********************************************************/
void WHOLEPORT_SET(GPIO_REGISTERS_t *Gpio_Registers, uint8_t mode, uint8_t otype, uint8_t ospeed, uint8_t pupd, uint16_t altfn);
/**************************************************************************************************************************/


void SYSCFG_CLCK_EN()
{
	Rcc_Registers->RCC_APB2ENR |= (1 << 14);
	printf("\n\nSYSCFG CLOCK ENABLED \n\n");

}

void SYSCFG_CLCK_DI()
{
	Rcc_Registers->RCC_APB2ENR &= ~(1 << 14);
	printf("\n\nSYSCFG CLOCK DISABLED\n\n");

}

void GPIO_PCLCK_EN(GPIO_REGISTERS_t *Gpio_Registers)
{

	//for the GPIOs:

	if(Gpio_Registers == GPIOA)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<0);
	}

	else if(Gpio_Registers == GPIOB)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<1);

	}
	else if(Gpio_Registers == GPIOC)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<2);

	}
	else if(Gpio_Registers == GPIOD)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<3);

	}
	else if(Gpio_Registers == GPIOE)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<4);
	}
	else if(Gpio_Registers == GPIOF)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<5);

	}
	else if(Gpio_Registers == GPIOG)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<6);

	}
	else if(Gpio_Registers == GPIOH)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<7);
	}
	else if(Gpio_Registers == GPIOI)
	{
		Rcc_Registers->RCC_AHB1ENR |= (1<<8);
	}

}

void GPIO_PCLCK_DI(GPIO_REGISTERS_t *Gpio_Registers)
{

	if(Gpio_Registers == GPIOA)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<0);
	}
	else if(Gpio_Registers == GPIOB)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<1);

	}
	else if(Gpio_Registers == GPIOC)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<2);

	}
	else if(Gpio_Registers == GPIOD)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<3);

	}
	else if(Gpio_Registers == GPIOE)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<4);

	}
	else if(Gpio_Registers == GPIOF)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<5);

	}
	else if(Gpio_Registers == GPIOG)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<6);

	}
	else if(Gpio_Registers == GPIOH)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<7);

	}
	else if(Gpio_Registers == GPIOI)
	{
		Rcc_Registers->RCC_AHB1ENR &= ~(1<<8);

	}
}

void GPIO_INIT(GPIO_HANDLE_t *pGpio_Handle)
{

	if(pGpio_Handle->Gpio_Config.gpiox_moder <= ANALOG)
	{
		if(pGpio_Handle->Gpio_Config.gpiox_moder == INPUT)
		{
			pGpio_Handle->Gpio_Registers->GPIOx_MODER &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); // clear the bits
		}
		else if(pGpio_Handle->Gpio_Config.gpiox_moder == OUTPUT)
		{
			pGpio_Handle->Gpio_Registers->GPIOx_MODER &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); // clear the bits
		    pGpio_Handle->Gpio_Registers->GPIOx_MODER |= (1 << (pGpio_Handle->Gpio_Config.pin_number * 2));
		}
		else if(pGpio_Handle->Gpio_Config.gpiox_moder == ALTERNATE_FN)
		{
			pGpio_Handle->Gpio_Registers->GPIOx_MODER &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); // clear the bits
			 pGpio_Handle->Gpio_Registers->GPIOx_MODER |= (2 <<  (pGpio_Handle->Gpio_Config.pin_number * 2));

		}
		else if(pGpio_Handle->Gpio_Config.gpiox_moder == ANALOG)
		{
			pGpio_Handle->Gpio_Registers->GPIOx_MODER &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); // clear the bits
			pGpio_Handle->Gpio_Registers->GPIOx_MODER |= (3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); // clear the bits


		}
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\nERR-001: NO_MODE_SET\n");
		printf("\nGiving a Call to The Interrupt APIs\n");
		IRQ_EDGE_SETTING(pGpio_Handle->Gpio_Config.gpiox_moder, pGpio_Handle->Gpio_Config.pin_number);
	}


//	SPEED CONFIGURATIONS

	if(pGpio_Handle->Gpio_Config.gpiox_speed == SPEED_LOW)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); //clear the bits
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_speed == SPEED_MEDIUM)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); //clear the bits
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR |= (1 << (pGpio_Handle->Gpio_Config.pin_number * 2));
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_speed == SPEED_HIGH)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); //clear the bits
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR |= (2 << (pGpio_Handle->Gpio_Config.pin_number * 2)); //10 in binary
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_speed == SPEED_VERY_HIGH)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2)); //clear the bits
		pGpio_Handle->Gpio_Registers->GPIOx_OSPEEDR |= (3 << (pGpio_Handle->Gpio_Config.pin_number * 2));//11 in binary
	}


	//PUPDR CONFIGURATIONS

	if(pGpio_Handle->Gpio_Config.gpiox_pupdr == GPIO_PIN_NO_PUPD)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2));
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_pupdr == GPIO_PIN_PULL_UP)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2));
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR |= (1 << (pGpio_Handle->Gpio_Config.pin_number * 2));
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_pupdr == GPIO_PIN_PULL_DOWN)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2));
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR |= (2 << (pGpio_Handle->Gpio_Config.pin_number * 2));
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_pupdr == GPIO_PIN_RESERVED)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR &= ~(3 << (pGpio_Handle->Gpio_Config.pin_number * 2));
		pGpio_Handle->Gpio_Registers->GPIOx_PUPDR |= (3 << (pGpio_Handle->Gpio_Config.pin_number * 2));
	}



//	OTYPER CONFIGURATIONS
	if(pGpio_Handle->Gpio_Config.gpiox_otyper == PUSH_PULL)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_OTYPER &= ~(1 << pGpio_Handle->Gpio_Config.pin_number);
	}
	else if(pGpio_Handle->Gpio_Config.gpiox_otyper == OPEN_DRAIN)
	{
		pGpio_Handle->Gpio_Registers->GPIOx_OTYPER |= (1 << pGpio_Handle->Gpio_Config.pin_number);
	}


//	ALTERNATE FUNCTION CONFIGURATIONS
	if(pGpio_Handle->Gpio_Config.gpiox_moder == ALTERNATE_FN)
	{
		if(pGpio_Handle->Gpio_Config.pin_number <= 7)
		{
			pGpio_Handle->Gpio_Registers->GPIOx_AFRL &= ~(0xF << (pGpio_Handle->Gpio_Config.pin_number * 4));
			pGpio_Handle->Gpio_Registers->GPIOx_AFRL |= (pGpio_Handle->Gpio_Config.gpiox_alt_choice << (pGpio_Handle->Gpio_Config.pin_number * 4));

		}
		else if(pGpio_Handle->Gpio_Config.pin_number > 7)
		{
			pGpio_Handle->Gpio_Registers->GPIOx_AFRH &= ~(0xF << (pGpio_Handle->Gpio_Config.pin_number * 4));
			pGpio_Handle->Gpio_Registers->GPIOx_AFRH |= (pGpio_Handle->Gpio_Config.gpiox_alt_choice << ((pGpio_Handle->Gpio_Config.pin_number * 4)%32));
		}
	}
}

void GPIO_DEINIT(GPIO_REGISTERS_t *Gpio_Registers)
{
	if(Gpio_Registers == GPIOA)
	{

		Rcc_Registers->RCC_AHB1RSTR |= (1<<0);
	}
	else if(Gpio_Registers == GPIOB)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<1);

	}
	else if(Gpio_Registers == GPIOC)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<2);

	}
	else if(Gpio_Registers == GPIOD)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<3);

	}
	else if(Gpio_Registers == GPIOE)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<4);
	}
	else if(Gpio_Registers == GPIOF)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<5);
	}
	else if(Gpio_Registers == GPIOG)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<6);

	}
	else if(Gpio_Registers == GPIOH)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<7);
	}
	else if(Gpio_Registers == GPIOI)
	{
		Rcc_Registers->RCC_AHB1RSTR |= (1<<8);
	}

}


	void WRITE_DATA_OUTPIN(GPIO_REGISTERS_t *Gpio_Registers, uint32_t data, uint32_t pin_number)
	{
		if(data == ON)
		{
			Gpio_Registers->GPIOx_ODR |= (data << pin_number);
		}
		else if(data == OFF)
		{
			Gpio_Registers->GPIOx_ODR &= ~(1 << pin_number);
		}
	}

	void WRITE_DATA_OUTPORT(GPIO_REGISTERS_t *Gpio_Registers, uint32_t data)
	{
		Gpio_Registers->GPIOx_ODR = data;
	}

	void TOGGLE_PIN(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin_number)
	{
		Gpio_Registers->GPIOx_ODR ^= (1 << pin_number);
	}

	int READ_DATA_INPIN(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin_number)
	{
		uint8_t data;
		uint8_t return_val = 0;
		data = (Gpio_Registers->GPIOx_IDR & (1 << pin_number));
		if(data > 0)
		{
			return_val = 1;
		}
		else if(data < 1)
		{
			return_val = 0;
		}
		return return_val;
	}

	int READ_DATA_INPORT(GPIO_REGISTERS_t *Gpio_Registers)
	{
		uint32_t data = 0;
		data = Gpio_Registers->GPIOx_IDR;
		return data;
	}

	void CLEAR_OUTPUT_REG(GPIO_REGISTERS_t *Gpio_Registers)
	{
		Gpio_Registers->GPIOx_BSRR |= 0xFFFF0000; //clear the ODR in the BSRR registers
	}

	void CLEAR_OUTPUT_PIN(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin_number)
	{
		Gpio_Registers->GPIOx_ODR &= ~(1 << pin_number);	//clear the corresponding bit position
	}

	void BSRR_SEND_DATA(GPIO_REGISTERS_t *Gpio_Registers, uint8_t data, uint8_t pin)
	{
		if(data == OFF)
		{
			Gpio_Registers->GPIOx_BSRR &= ~(1 << (pin + 16));
		}
		else if(data == ON)
		{
			Gpio_Registers->GPIOx_BSRR |= (1 << pin );
		}
	}

	void IRQ_HANDLING(uint8_t PinNumber)
	{
		if(EXTI->EXTI_PR & (1 << PinNumber))
		{
			EXTI->EXTI_PR |= (1 << PinNumber); //clear the bit position
		}
	}

//my APIs
	void IRQ_EDGE_SETTING(uint16_t edge_selection, uint8_t pin_num)
	{

		EXTI->EXTI_IMR |= (1 << pin_num); //Enable the interrupt
		if(edge_selection == INTERRUPT_IT_FT)
		{
			EXTI->EXTI_RTSR &= ~(1 << pin_num);
			EXTI->EXTI_FTSR |= (1 << pin_num);
		}

		else if(edge_selection == INTERRUPT_IT_RT)
		{
			EXTI->EXTI_FTSR &= ~(1 << pin_num);
			EXTI->EXTI_RTSR |= (1 << pin_num);
		}

		else if(edge_selection == INTERRUPT_IT_RFT)
		{

			EXTI->EXTI_FTSR |= (1 << pin_num);
			EXTI->EXTI_RTSR |= (1 << pin_num);

		}
	}

	void IRQ_CONFIG(uint16_t GPIOx_INTERRUPT, uint8_t pin) //Here use the SYSCFG used macros
	{


		if(pin <= 3 && pin >= 0) //less than equal to 3 and greater than 0
		{
			SysCfg->SYSCFG_EXTICR1 |= (GPIOx_INTERRUPT << (pin*4));
		}

		else if(pin <= 7 && pin >= 4)//less than 8 and greater than 4
		{
			SysCfg->SYSCFG_EXTICR2 |= (GPIOx_INTERRUPT << ((pin % 4)*4));
		}

		else if(pin <= 11 && pin >= 8)
		{
			SysCfg->SYSCFG_EXTICR3 |= (GPIOx_INTERRUPT << ((pin % 4)*4));
		}

		else if(pin <= 15 && pin >= 12)
		{
			SysCfg->SYSCFG_EXTICR4 |= (GPIOx_INTERRUPT << ((pin % 4)*4));
		}

	}

	void IRQ_NVIC_CONFIG(uint16_t IRQ_num, uint8_t EN_or_DI)
	{
		if(EN_or_DI == ENABLE)
		{
			if(IRQ_num <= 31)
			{
				*NVIC_ISER0 |= (1 << IRQ_num);
			}

			else if(IRQ_num >= 32 && IRQ_num <=63)
			{
				*NVIC_ISER1 |= (1 << IRQ_num % 32);
			}

			else if(IRQ_num >= 64 && IRQ_num < 96)
			{
				*NVIC_ISER2 |= (1 << IRQ_num % 64);
			}
		}

		else if(EN_or_DI == DISABLE)
		{
			if(IRQ_num <= 31)
			{
				*NVIC_ICER0 |= (1 << IRQ_num);
			}

			else if(IRQ_num >= 32 && IRQ_num <=63)
			{
				*NVIC_ICER1 |= (1 << IRQ_num % 32);
			}

			else if(IRQ_num >= 64 && IRQ_num <= 96)
			{
				*NVIC_ICER2 |= (1 << IRQ_num % 64);
			}
		}
	}

	void IRQ_PRIORITY_CONFIG(uint16_t IRQ_num, uint16_t IRQ_priority)
	{
		uint8_t ipr_reg_no = IRQ_num / 4;
		uint8_t ipr_reg_section = ipr_reg_no % 4;

		uint16_t shift_value = (8 * ipr_reg_section) + (8 - IMPLEMENTED_BITS);
		*(NVIC_IPR + (ipr_reg_no * 4)) |= (IRQ_priority << shift_value);
	}

/************************************************Featured Macros***********************************************/
	void LED_BLINK(GPIO_REGISTERS_t *Gpio_Registers, uint8_t pin, uint32_t blink_count, uint32_t blink_time)
	{
		if(blink_count > 0)
		{
			for(int i = 1; i <= blink_count; i++)
			{
				Gpio_Registers->GPIOx_ODR |= (1 << pin);
				delay(blink_time);
				Gpio_Registers->GPIOx_ODR &= ~(1 << pin);
				delay(blink_time);
			}
		}
		else if(blink_count == LOOP)
		{
			while(1)
			{
				Gpio_Registers->GPIOx_ODR |= (1 << pin);
				delay(blink_time);
				Gpio_Registers->GPIOx_ODR &= ~(1 << pin);
				delay(blink_time);
			}
		}
	}

	int INTERRUPTS_DEBUG( uint16_t debug_argument, uint16_t pin)
	{
		int return_val = 0;
		if(debug_argument == INTERRUPT_FALLING_EDGE_CHECK)
		{
			if((EXTI->EXTI_FTSR & (1 << pin)) == 0)
			{
				return_val = INTERRUPT_FALLING_EDGE_CHECK;
			}
		}
		else if(debug_argument == INTERRUPT_RISING_EDGE_CHECK)
		{
			if((EXTI->EXTI_RTSR & (1 << pin)) == 0)
			{
				return_val = INTERRUPT_RISING_EDGE_CHECK;
			}
		}

		return return_val;
	}

//void WHOLEPORT_SET(GPIO_REGISTERS_t *Gpio_Registers, uint8_t mode, uint8_t otype, uint8_t ospeed, uint8_t pupd, uint16_t altfn)
//{
//	if(mode >= WS_INPUT || mode <= WS_ANALOG)			//IF THE SETTING PARAMETERS ARE NOT OUT OF NOWHERE THEN ONLY SET THE MODE
//	{
//		Gpio_Registers->GPIOx_MODER = mode;
//	}
//
//	if(otype == WS_PUSH_PULL || otype == WS_OPEN_DRAIN)
//	{
//		Gpio_Registers->GPIOx_OTYPER = otype;
//	}
//
//	if(ospeed >= WS_SPEED_LOW || ospeed <= WS_SPEED_VERY_HIGH)
//	{
//		Gpio_Registers->GPIOx_OSPEEDR = ospeed;
//	}
//
//	if(pupd >= WS_GPIO_PIN_NO_PUPD || pupd <= WS_GPIO_PIN_RESERVED)
//	{
//		Gpio_Registers->GPIOx_PUPDR = pupd;
//	}
//
//	if(altfn >= WS_AF0 || altfn <= WS_AF15)
//	{
//		Gpio_Registers->GPIOx_AFRL = altfn;
//		Gpio_Registers->GPIOx_AFRH = altfn;
//	}
//}


#endif /* GPIO_DRIVER_H_ */
