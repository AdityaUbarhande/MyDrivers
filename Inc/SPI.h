/*
 * Spi_Driver.h
 *
 *  Created on: Aug 26, 2020
 *      Author: Aditya Ubarhande
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include"stm32f407.h"
#include"Gpio_Driver.h"
#include<string.h>
#include<stdio.h>
#include"Bit_Position.h"
#include<stddef.h>

//Generic Macros
#define __weak				__attribute__((weak))
#define RANDOM 				0x87824



/********APPLICATION MACROS****/
#define SPI_TX_BUSY			0x11
#define SPI_RX_BUSY			0x12
#define SPI_READY			0x13
#define SPI_MODF_ERROR		0x14
#define SPI_OVR_ERROR		0x15

//APPLICATION STATUS MACROS
#define SPI_TX_COMPLETE						0x1
#define SPI_RX_COMPLETE						0x2
#define SPI_MODF_ERROR_SOLVED				0x3
#define SPI_OVRRUN_ERROR_SOLVED				0x4

/***************************/

#define TX_ONLY 		1
#define RX_ONLY			0

#define FLAG_SET		1
#define FLAG_RESET		0

#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

typedef struct
{
	uint16_t Master_Slave_Selection;			//for Master, Slave selection
	uint16_t Bus_Config;						//for DUPLEX selection
	uint16_t Baud_Rate;							//Baud Rate
	uint16_t Data_Frame_Format;					//Selection of data frame (8 and 16 bits)
	uint16_t Spi_Cpha;
	uint16_t Spi_Cpol;
	uint16_t Spi_SSM;							//Slave Select Management for selecting hardware/software management
}SPI_CONFIG_t;

typedef struct
{
	SPI_REGISTERS_t *Spi_Registers;
	SPI_CONFIG_t Spi_Config;
	uint8_t *TxBuffer;
	uint8_t *RxBuffer;
	uint8_t TxLen;
	uint8_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

}SPI_HANDLE_t;

/***************************************CLOCK APIs*******************************************/
void SPI_CLCK_EN(SPI_REGISTERS_t *Spi_Registers);
void SPI_CLCK_DI(SPI_REGISTERS_t *Spi_Registers);

/***************************************INIT, DE-INIT AND ENABLE APIs***************************/
void SPI_INIT(SPI_HANDLE_t* Spi_Handle);
void SPI_DEINIT(SPI_HANDLE_t* Spi_Handle);
void SPI_EN_DI(SPI_REGISTERS_t* Spi_Registers, uint8_t enordi);
void SPI_SSI_CONFIG(SPI_REGISTERS_t *Spi_Registers, uint8_t enordi);
void SPI_SSOE_CONFIG(SPI_REGISTERS_t *Spi_Registers, uint8_t enordi);
void TX_RX_ONLY_CONIFG(SPI_REGISTERS_t *Spi_Registers, uint8_t TXorRX);

/****************************************DATA TRANSCEIVING APIs*********************************/
void SPI_SEND_DATA(SPI_REGISTERS_t *Spi_Registers, uint8_t *pTxBuffer, uint32_t length);
void SPI_RECEIVE_DATA(SPI_REGISTERS_t *Spi_Registers, uint8_t *pRxBuffer, uint32_t length);


/*******************************************SPI INTERRUPTS APIs*********************************/
//PROCESSOR CONFIGURATION APIs
void SPI_IRQ_NVIC_CONFIG(uint16_t IRQ_num, uint8_t EN_or_DI);
void SPI_IRQ_PRIORITY_CONFIG(uint16_t IRQ_num, uint16_t IRQ_priority);

//DATA TRANSCEIVING
int SPI_SEND_DATA_IT(SPI_HANDLE_t *Spi_Handle, uint8_t *pTxBuffer, uint32_t length);
int SPI_RECEIVE_DATA_IT(SPI_HANDLE_t *Spi_Handle, uint8_t *pRxBuffer, uint32_t length);

//INTERRUPT CATEGORISING
void SPI_IRQ_HANDLING(SPI_HANDLE_t *Spi_Handle);

//HANDLER APIs  [CANNOT BE CHANGED BY THE USER OR CANT BE USED TOO]
static void SPI_TX_HANDLER(SPI_HANDLE_t* Spi_Handle);
static void SPI_RX_HANDLER(SPI_HANDLE_t* Spi_Handle);
static void SPI_ERROR_HANDLER(SPI_HANDLE_t *handle, uint8_t error_type);

static void SPI_END_RECEPTION(SPI_HANDLE_t* Spi_Handle);
static void SPI_END_TRANSMISSION(SPI_HANDLE_t* Spi_Handle);
static void SPI_OVR_CLEAR(SPI_HANDLE_t* Spi_Handle);

/******************************************************************************************/

/*****************************************STATUS APIs******************************************/
int SPI_FLAG_STAT(SPI_REGISTERS_t *Spi_Registers, uint8_t flagname);

void Application_CALL_BACK(SPI_HANDLE_t* Spi_Handle, uint8_t Event);
/*********************************************************************************************/

void SPI_CLCK_EN(SPI_REGISTERS_t *Spi_Registers)
{
	if(Spi_Registers == SPI1)
	{
		Rcc_Registers->RCC_APB2ENR |= (1 << 12);
	}
	else if(Spi_Registers == SPI2)
	{
		Rcc_Registers->RCC_APB1ENR |= (1 << 14);

	}
	else if(Spi_Registers == SPI3)
	{
		Rcc_Registers->RCC_APB1ENR |= (1 << 14);

	}
	else if(Spi_Registers == SPI4)
	{
		Rcc_Registers->RCC_APB2ENR |= (1 << 3);
	}

}


void SPI_CLCK_DI(SPI_REGISTERS_t *Spi_Registers)
{
	if(Spi_Registers == SPI1)
	{
		Rcc_Registers->RCC_APB2ENR &= ~(1 << 12);
	}

	else if(Spi_Registers == SPI2)
	{
		Rcc_Registers->RCC_APB1ENR &= ~(1 << 14);

	}
	else if(Spi_Registers == SPI3)
	{
		Rcc_Registers->RCC_APB1ENR &= ~(1 << 14);

	}
	else if(Spi_Registers == SPI4)
	{
		Rcc_Registers->RCC_APB2ENR &= ~(1 << 3);
	}
}

void SPI_INIT(SPI_HANDLE_t* Spi_Handle)
{
	/****************************************MASTER / SLAVE SELECTION************************************/
	if(Spi_Handle->Spi_Config.Master_Slave_Selection == SPI_MASTER || Spi_Handle->Spi_Config.Master_Slave_Selection == SPI_SLAVE)
	{
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(Spi_Handle->Spi_Config.Master_Slave_Selection << 2);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (Spi_Handle->Spi_Config.Master_Slave_Selection << 2); //set mode as master
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\n\nPLEASE SELECT A APPROPIATE MASTER-SLAVE CONFIGURATION \n\n");
	}
	/*******************************************************************************************************/


	/****************************************BUS MODE SELECTION************************************/
	if(Spi_Handle->Spi_Config.Bus_Config == SPI_FULL_DUPLEX)
	{
		//bidi bit must be cleared
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 15);
	}
	else if(Spi_Handle->Spi_Config.Bus_Config == SPI_HALF_DUPLEX)
	{
		//bidi bit must be set
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 15);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (1 << 15);
	}
	else if(Spi_Handle->Spi_Config.Bus_Config == SPI_SIMPLEX_RXONLY)
	{
        //bidi bit must be cleared and RXONLY to be set
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 15);
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 10);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (1 << 10);
	}
	/*******************************************************************************************************/


	/**************************************************BAUD RATE**********************************************/
	if(Spi_Handle->Spi_Config.Baud_Rate >= SPI_BAUD_DIV2  &&  Spi_Handle->Spi_Config.Baud_Rate <= SPI_BAUD_DIV256)
	{
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(0x7 << 3);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (Spi_Handle->Spi_Config.Baud_Rate << 3);
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\n\nPLEASE SELECT A APPROPIATE BAUD RATE DIVISOR\n\n");
	}

	/*******************************************************************************************************/

	/**************************************************DATA FRAME FORMAT**********************************************/
	if(Spi_Handle->Spi_Config.Data_Frame_Format == SPI_DFF_8BITS || Spi_Handle->Spi_Config.Data_Frame_Format == SPI_DFF_16BITS)
	{
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 11);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (Spi_Handle->Spi_Config.Data_Frame_Format << 11);
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\n\nPLEASE SELECT A APPROPIATE DATA FRAME FORMAT\n\n");
	}

	/*******************************************************************************************************/

	/**************************************************SPI CPHA**********************************************/
	if(Spi_Handle->Spi_Config.Spi_Cpha == SPI_CPHA_HIGH || Spi_Handle->Spi_Config.Spi_Cpha == SPI_CPHA_LOW)
	{
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 0);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (Spi_Handle->Spi_Config.Spi_Cpha << 0);
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\n\nPLEASE SELECT A APPROPIATE CPHA CONFIGURATION\n\n");
	}
	/*******************************************************************************************************/

	/***************************************************SPI CPOL*********************************************/
	if(Spi_Handle->Spi_Config.Spi_Cpol == SPI_CPOL_HIGH || Spi_Handle->Spi_Config.Spi_Cpol == SPI_CPOL_LOW)
	{
		Spi_Handle->Spi_Registers->SPI_CR1 &=	~(1 << 1);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (Spi_Handle->Spi_Config.Spi_Cpol << 1);
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\n\nPLEASE SELECT A APPROPIATE CPOL CONFIGURATION\n\n");
	}

	/*******************************************************************************************************/

	/*****************************************************SPI SLAVE SELECT MANAGEMENT (HARDWARE/SOFTWARE)***********************************/
	if(Spi_Handle->Spi_Config.Spi_SSM == SPI_SSM_HWM || Spi_Handle->Spi_Config.Spi_SSM == SPI_SSM_SWM)
	{
		Spi_Handle->Spi_Registers->SPI_CR1 &= ~(1 << 9);
		Spi_Handle->Spi_Registers->SPI_CR1 |= (Spi_Handle->Spi_Config.Spi_SSM << 9);
	}
	else
	{
		printf("\nCAUTION\n");
		printf("\n\nPLEASE SELECT A APPROPIATE SSM CONFIGURATION\n\n");
	}

	/*******************************************************************************************************/
}

void SPI_EN_DI(SPI_REGISTERS_t* Spi_Registers, uint8_t enordi)
{
	if(enordi == ENABLE)
	{
		Spi_Registers->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		Spi_Registers->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSI_CONFIG(SPI_REGISTERS_t *Spi_Registers, uint8_t enordi)
{
	if(enordi == ENABLE)
	{
		Spi_Registers->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}
	else if(enordi == DISABLE)
	{
		Spi_Registers->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOE_CONFIG(SPI_REGISTERS_t *Spi_Registers, uint8_t enordi)
{
	if(enordi == ENABLE)
	{
		Spi_Registers->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}
	else if(enordi == DISABLE)
	{
		Spi_Registers->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

void TX_RX_ONLY_CONIFG(SPI_REGISTERS_t *Spi_Registers, uint8_t TXorRX)
{
	if(TXorRX == TX_ONLY)
	{
		Spi_Registers->SPI_CR1 |= (1 << SPI_CR1_BIDIOE);
	}
	else if(TXorRX == RX_ONLY)
	{
		Spi_Registers->SPI_CR1 &= ~(1 << SPI_CR1_BIDIOE);
	}
}

void SPI_SEND_DATA(SPI_REGISTERS_t *Spi_Registers, uint8_t *pTxBuffer, uint32_t length)
{

	while(length > 0)
	{

			while(!(Spi_Registers->SPI_SR & (1 << SPI_SR_TXE))); 	//CHECK WHETHER THE TX BUFFER IS EMPTY

			if((Spi_Registers->SPI_CR1 & (1 << SPI_CR1_DFF)))        //16 bit transmission
			{
				Spi_Registers->SPI_DR = *((uint16_t*)pTxBuffer);
				length--;  //2 times decreased because of 16 bits is there
				length--;

				(uint16_t*)pTxBuffer++;   //to move to the next character (for 16 bits)
			}
			else													//8 bit transmission
			{
				Spi_Registers->SPI_DR = *pTxBuffer;
				length--;

				pTxBuffer++;   //to move to the next character (for 8 bits)



			}


	}
}

void SPI_RECEIVE_DATA(SPI_REGISTERS_t *Spi_Registers, uint8_t *pRxBuffer, uint32_t length)
{
	while(length > 0)
	{
		if(!(Spi_Registers->SPI_SR & (1 << SPI_SR_RXNE)))      		//CHECK IF RX BUFFER IS EMPTY OR NOT (if empty return 1 as per st)
		{
			if((Spi_Registers->SPI_CR1 & (1 << SPI_CR1_DFF)))        //16 bit transmission
			{
				*(pRxBuffer) = Spi_Registers->SPI_DR;

				length--;  //2 times decreased because of 16 bits is there
				length--;

				pRxBuffer++;   //to move to the next character (for 16 bits)
			}

			else													//8 bit transmission
			{
				*((uint8_t*)pRxBuffer) = Spi_Registers->SPI_DR;
				length--;

				(uint8_t*)pRxBuffer++;   //to move to the next character (for 8 bits)

			}

		}
	}
}

void SPI_IRQ_NVIC_CONFIG(uint16_t IRQ_num, uint8_t EN_or_DI)
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

void SPI_IRQ_PRIORITY_CONFIG(uint16_t IRQ_num, uint16_t IRQ_priority)
{

	//FOR CALCULATING THE REGISTER NUMBER AND THE REGISTER SECTION
	uint8_t ipr_reg_no = IRQ_num / 4;
	uint8_t ipr_reg_section = ipr_reg_no % 4;


	uint16_t shift_value = (8 * ipr_reg_section) + (8 - IMPLEMENTED_BITS);
	/*THIS IS TO CALCULATE THE SHIFT VALUE OR THE VALUE BY WHICH TO LEFT/RIGHT SHIFT (EXAMPLE: (1 << SHIFT) WHEN SHIFT = 32.
	 * HOPE IT IS CLEAR
	 */

	*(NVIC_IPR + (ipr_reg_no * 4)) |= (IRQ_priority << shift_value); 	//SHIFT THE REGISTER
}



int SPI_SEND_DATA_IT(SPI_HANDLE_t *Spi_Handle, uint8_t *pTxBuffer, uint32_t length)
{
	uint8_t state = Spi_Handle->TxState;

	if(state != SPI_TX_BUSY)
	{

		//STORE LENGTH AND TX BUFFER ADDRESS IN GLOBAL VARIABLES

		Spi_Handle->TxBuffer = pTxBuffer;
		Spi_Handle->TxLen = length;

		Spi_Handle->TxState = SPI_TX_BUSY;			//SET SPI STATE AS BUSY IN TRANSMISSION

		Spi_Handle->Spi_Registers->SPI_CR2 |= (1 << SPI_CR2_TXEIE);  		//ENABLING THE TXEIE INTERRRUPT

	}
	return state;

}

int SPI_RECEIVE_DATA_IT(SPI_HANDLE_t *Spi_Handle, uint8_t *pRxBuffer, uint32_t length)
{
	uint8_t state = Spi_Handle->RxState;

	if(state != SPI_RX_BUSY)
	{

		//STORE LENGTH AND TX BUFFER ADDRESS IN GLOBAL VARIABLES

		Spi_Handle->RxBuffer = pRxBuffer;
		Spi_Handle->RxLen = length;

		Spi_Handle->RxState = SPI_RX_BUSY;			//SET SPI STATE AS BUSY IN TRANSMISSION

		Spi_Handle->Spi_Registers->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);  		//ENABLING THE TXEIE INTERRRUPT

	}
	return state;
}

void SPI_IRQ_HANDLING(SPI_HANDLE_t *Spi_Handle)
{
	//CHECK THE FLAGS.
	int tx_check1;
	int tx_check2;

	int rx_check1;
	int rx_check2;

	int error_check1;
	int error_modf;
	int error_ovr;

	tx_check1 = Spi_Handle->Spi_Registers->SPI_SR & (1 << SPI_SR_TXE);				//CHECK WHETHER THE TX BUFFER IS EMPTY
	tx_check2 = Spi_Handle->Spi_Registers->SPI_CR2 & (1 << SPI_CR2_TXEIE);			//CHECK WHETHER THE TX INTERRUPT FLAG IS SET

	rx_check1 = Spi_Handle->Spi_Registers->SPI_SR & (1 << SPI_SR_RXNE);				//CHECK WHETHER THE RX BUFFER IS EMPTY
	rx_check2 = Spi_Handle->Spi_Registers->SPI_CR2 & (1 << SPI_CR2_RXNEIE);			//CHECK WHETHER THE RX INTERRUPT FLAG IS SET

	error_check1 = Spi_Handle->Spi_Registers->SPI_CR2 & (1 << SPI_CR2_ERRIE);		//CHECK WHETHER THE ERROR INTERRUPTS ARE ENABLED
	error_modf = Spi_Handle->Spi_Registers->SPI_SR & (1 << SPI_SR_MODF);			//CHECK WHETHER THE MODE FAULT ERROR HAS OCCURED
	error_ovr =  Spi_Handle->Spi_Registers->SPI_SR & (1 << SPI_SR_OVR);				//CHECK WHETHER THE OVER RUN ERROR HAS OCCURED

	//TX CHECKER
	if(tx_check1 && tx_check2 )
	{
		SPI_TX_HANDLER(Spi_Handle);
	}

	//RX CHECKER
	if(rx_check1 && rx_check2)
	{
		SPI_RX_HANDLER(Spi_Handle);
	}

	//MODF ERROR CHECKER
	if(error_check1 && error_modf)
	{
		SPI_ERROR_HANDLER(Spi_Handle, SPI_MODF_ERROR);
	}

	//OVR ERROR CHECKER
	if(error_check1 && error_ovr)
	{
		SPI_ERROR_HANDLER(Spi_Handle, SPI_OVR_ERROR);
	}

}

static void SPI_TX_HANDLER(SPI_HANDLE_t* Spi_Handle)
{
	//TRANSMIT DATA

		if((Spi_Handle->Spi_Registers->SPI_CR1 & (1 << SPI_CR1_DFF)))        //16 bit transmission
		{
			Spi_Handle->Spi_Registers->SPI_DR = *((uint16_t*)Spi_Handle->TxBuffer);

			Spi_Handle->TxLen--;
			Spi_Handle->TxLen--;

			(uint16_t*)Spi_Handle->TxBuffer++;   //to move to the next character (for 16 bits)
		}
		else													//8 bit transmission
		{
			Spi_Handle->Spi_Registers->SPI_DR = *((uint16_t*)Spi_Handle->TxBuffer);
			Spi_Handle->TxLen--;

			Spi_Handle->TxBuffer++;   //to move to the next character (for 8 bits)
		}



	if(Spi_Handle->TxLen == 0)
	{
		SPI_END_TRANSMISSION(Spi_Handle);
		Application_CALL_BACK(Spi_Handle, SPI_TX_COMPLETE);
	}
}

static void SPI_RX_HANDLER(SPI_HANDLE_t* Spi_Handle)
{
	//RECEIVE DATA

		if((Spi_Handle->Spi_Registers->SPI_CR1 & (1 << SPI_CR1_DFF)))        //16 bit transmission
		{
			*((uint16_t*)Spi_Handle->RxBuffer) = Spi_Handle->Spi_Registers->SPI_DR;

			Spi_Handle->RxLen--;  //2 times decreased because of 16 bits is there
			Spi_Handle->RxLen--;  //2 times decreased because of 16 bits is there

			Spi_Handle->RxBuffer++;   //to move to the next character (for 16 bits)
		}

		else													//8 bit transmission
		{
			*((uint8_t*)Spi_Handle->RxBuffer) = Spi_Handle->Spi_Registers->SPI_DR;
			Spi_Handle->RxLen--;  //1 time decreased because of 8 bits is there

			(uint8_t*)Spi_Handle->RxBuffer++;   //to move to the next character (for 8 bits)

		}

		if(Spi_Handle->RxLen == 0)
		{
			SPI_END_RECEPTION( Spi_Handle);
			Application_CALL_BACK(Spi_Handle, SPI_RX_COMPLETE);
		}
}

static void SPI_ERROR_HANDLER(SPI_HANDLE_t *handle, uint8_t error_type)
{
	if(error_type == SPI_MODF_ERROR)
	{
		//CHECK THE SLAVE SETTING (SWM OR HWM) AND:

		if(handle->Spi_Config.Spi_SSM == SPI_SSM_HWM)
		{
			//SET THE SSOE BIT
			SPI_SSOE_CONFIG(SPI2, ENABLE);
		}
		else if(handle->Spi_Config.Spi_SSM == SPI_SSM_SWM)
		{
			//SET THE SSI BIT
			SPI_SSI_CONFIG(SPI2, ENABLE);
		}

	}

	else if(error_type == SPI_OVR_ERROR)
	{
		if(handle->TxState == SPI_READY)
		{
			SPI_OVR_CLEAR(handle);
		}
		//2. INFORM THE APPLICATION
		Application_CALL_BACK(handle, SPI_OVRRUN_ERROR_SOLVED);

	}
}

static void SPI_END_RECEPTION(SPI_HANDLE_t* Spi_Handle)
{
	Spi_Handle->Spi_Registers->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE);
	Spi_Handle->RxLen = 0;
	Spi_Handle->RxState = SPI_READY;
	Spi_Handle->RxBuffer = NULL;
}

static void SPI_END_TRANSMISSION(SPI_HANDLE_t* Spi_Handle)
{
	Spi_Handle->Spi_Registers->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE); 	//DISABLE THE INTERRUPTS (WILL BE ENABLED AGAIN IF THE USER CALLS THE SEND DATA IT FUNCTION)
	Spi_Handle->TxBuffer = NULL;
	Spi_Handle->TxLen = 0;
	Spi_Handle->TxState = SPI_READY;
}

static void SPI_OVR_CLEAR(SPI_HANDLE_t* handle)
{
	uint8_t temp1 = 0;
	printf("%d",temp1);			//THIS STATEMENT WONT BE PRINTED ( DUE TO \n) IT IS JUST FOR THE SAKE OF REMOVING THE WARNING UNUSED VARIABLE
	temp1 = handle->Spi_Registers->SPI_DR;
	temp1 = 0;
	temp1 = handle->Spi_Registers->SPI_SR;
	temp1 = 0;
}

int SPI_FLAG_STAT(SPI_REGISTERS_t *Spi_Registers, uint8_t flagname)
{
	if(Spi_Registers->SPI_SR & flagname)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


__weak void Application_CALL_BACK(SPI_HANDLE_t* Spi_Handle, uint8_t Event)				//WILL BE IMPLEMENTED BY THE USER
{
	/*THIS FUNCTION IS BASICALLY FOR GETTING A SIGNAL WHEN THE TX/RX/OVR/MODF INTERRUPTS ARE COMPLETED
	 * IF USER WISHES TO IMPLEMENT THIS, THEN HE/SHE WILL GET THE CURRENT STATUS OF TX/RX/OVR/MODF INTERRUPTS
	 * WHICH WILL HELP HIM/HER TO PROCEED WITH THE FURTHER CODE...
	 */
	//IMPLEMENTED BY USER IN THE ".c" FILE....
}


#endif /* SPI_DRIVER_H_ */
