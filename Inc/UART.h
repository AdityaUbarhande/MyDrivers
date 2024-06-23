/*
 * USART_Driver.h
 *
 *  Created on: 29-Dec-2020
 *      Author: Aditya Ubarhande
 */

//WHY INCREMENT THE TXBUFFER TWICE?????


#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_

#include"stm32f407.h"
#include<stdint.h>
#include"Bit_Position.h"
#include"Rcc_Driver.h"

static int Parity_Choice;

typedef struct
{
	uint32_t Baud_Rate; 				//THIS VARIABLE IF OF 32 BITS (4-BYTE) BECAUSE THE BAUD RATES CAN BE HIGH.
	uint8_t UART_Mode; 					//CAN BE CONFIGURED AS MASTER OR SLAVE (dedicated macros are created)
	uint8_t Word_Length; 				//CAN BE CONFIGURED AS 8-BIT MODE OR 9-BIT MODE
	uint8_t Stop_Bit_Count; 			//CAN BE CONFIGURED AS: 1 stop bit, 0.5 stop bit, 2 stop bits, 1.5 stop bits.
	uint8_t Parity_Bit; 				//CAN BE CONFIGURED AS EVEN PARITY OR ODD PARITY (by dedicated macros)
	uint8_t HW_Flow_Ctrl; 				//CAN BE CONFIGURED FOR CTS (CLEAR TO SEND) AND RTS (REQUEST TO SEND)
}USART_CONFIG_t;

//NOTE FOR UART_CONFIG_t: THE STOP BIT CONFIGURATIONS 0.5 AND 1.5 CANNOT BE USED IN UART..

typedef struct
{
	USART_REGISTERS_t *Uart_Register;			//THIS HAS A POINTER BECAUSE WE HAVE TO POINT TO THE UART TO BE USED.
	USART_CONFIG_t Usart_Configurations;
}USART_HANDLE_t;


//CLOCK ENABLING AND DISABLING APIs
void USART_CLK_EN(uint8_t UART_CLK);			//DEDICATED MACROS TO BE PASSED
void USART_CLK_DI(uint8_t UART_CLK);			//DEDICATED MACROS TO BE PASSED

//INIT AND DE-INIT APIs
void USART_INIT(USART_HANDLE_t *usart_handle);
void USART_DEINIT(uint8_t UART_RST);			//DEDICATED MACROS TO BE PASSED

//DATA TRANSCEIVING APIs
void USART_DATA_SEND(USART_REGISTERS_t *Uart_Register, uint8_t len, uint8_t *TxBuffer);
uint8_t USART_DATA_RECEIVE();

//USART BAUD RATE CALCULATION
void SET_BAUDRATE(USART_HANDLE_t *usart_handle);

//FOR ENABLING UART
void UART_ENABLE(USART_REGISTERS_t *Uart_Register);

void USART_CLK_EN(uint8_t UART_CLK)
{
	Rcc_Registers->RCC_APB1ENR |= (1 << UART_CLK);			//ENABLE THE CLOCK
}

void USART_CLK_DI(uint8_t UART_CLK)
{
	Rcc_Registers->RCC_APB1ENR &= ~(1 << UART_CLK);			//DISABLE THE CLOCK
}

void USART_INIT(USART_HANDLE_t *usart_handle)
{
	//ENABLE USART CLOCK
	usart_handle->Uart_Register->USART_CR1 |= (1 << USART_CR1_UE);

	//ENABLE THE USART TRANSMITTER/RECEIVER OR BOTH
	if(usart_handle->Usart_Configurations.UART_Mode == USART_MODE_TX)
	{
		usart_handle->Uart_Register->USART_CR1 &= ~(1 << USART_CR1_RE);		//DIABLE THE TRANSMITTER
		usart_handle->Uart_Register->USART_CR1 |= (1 << USART_CR1_TE);		//ENABLE THE TRANSMITTER
	}
	else if(usart_handle->Usart_Configurations.UART_Mode == USART_MODE_RX)
	{
		usart_handle->Uart_Register->USART_CR1 &= ~(1 << USART_CR1_TE);		//DIABLE THE TRANSMITTER
		usart_handle->Uart_Register->USART_CR1 |= (1 << USART_CR1_RE);		//ENABLE THE RECEIVER
	}
	else if(usart_handle->Usart_Configurations.UART_Mode == USART_MODE_TX_RX)
	{
		usart_handle->Uart_Register->USART_CR1 |= (1 << USART_CR1_TE);		//ENABLE THE TRANSMITTER
		usart_handle->Uart_Register->USART_CR1 |= (1 << USART_CR1_RE);		//ENABLE THE RECEIVER
	}

	//CONFIGURE THE WORD LENGTH
	usart_handle->Uart_Register->USART_CR1 |= (usart_handle->Usart_Configurations.Word_Length << USART_CR1_M);

	//CONFIGURE THE PARITY BITS
	if(usart_handle->Usart_Configurations.Parity_Bit == EVEN_PARITY || usart_handle->Usart_Configurations.Parity_Bit == ODD_PARITY)
	{
		//ENABLE THE PARITY CONTROL
		usart_handle->Uart_Register->USART_CR1 |= (1 << USART_CR1_PCE);

		//CONFIGURE THE APPROPIATE PARITY
		usart_handle->Uart_Register->USART_CR1 |= (usart_handle->Usart_Configurations.Parity_Bit << USART_CR1_PS);

		//SET THE PARITY CHOICE IN A VARIABLE.
		Parity_Choice = usart_handle->Usart_Configurations.Parity_Bit;
	}

	//CONFIGURE THE NUMBER OF STOP BITS
	usart_handle->Uart_Register->USART_CR2 |= (usart_handle->Usart_Configurations.Stop_Bit_Count << USART_CR2_STOP);

	//CONFIGURE THE HARDWARE FLOW CONTROL
	usart_handle->Uart_Register->USART_CR3 |= (usart_handle->Usart_Configurations.HW_Flow_Ctrl << USART_CR3_RTSE);
}

void USART_DEINIT(uint8_t UART_RST)
{
	Rcc_Registers->RCC_APB1RSTR |= (1 << UART_RST);			//RESET THE PERIPHERAL
}


void UART_ENABLE(USART_REGISTERS_t *Uart_Register)
{
	Uart_Register->USART_CR1 |= (1 << USART_CR1_UE);		//ENABLE THE USART PERIPHERAL.
}

void USART_SEND_DATA(USART_HANDLE_t *Uart_Handle, uint8_t len, uint8_t *TxBuffer)
{
	while(len != 0)
	{
		while(!(Uart_Handle->Uart_Register->USART_SR |= (1 << USART_SR_TXE)));	//CHECK IF THE TXE FLAG IS ENABLED OR NOT.

		if(Uart_Handle->Usart_Configurations.Word_Length == WORD_LEN_8)
		{
			Uart_Handle->Uart_Register->USART_DR = (*TxBuffer & 0xFF);
			len--;
			TxBuffer++;
		}
		else if(Uart_Handle->Usart_Configurations.Word_Length == WORD_LEN_9)
		{
			if(Parity_Choice == NO_PARITY)
			{
				Uart_Handle->Uart_Register->USART_DR = (uint16_t)(*TxBuffer & 0x1FF);
				TxBuffer++;
				TxBuffer++;
				len--;
			}
			else
			{
				Uart_Handle->Uart_Register->USART_DR = (uint16_t)(*TxBuffer & 0x1FF);
				len--;
				TxBuffer++;
			}
		}
	}
}

void SET_BAUDRATE(USART_HANDLE_t *usart_handle)
{
	float USARTDIV;
	uint32_t FCLCK_FREQ = GET_APB2_FREQENCY;
	uint32_t Baud_Rate = usart_handle->Usart_Configurations.Baud_Rate;
	uint8_t OVER8 = (usart_handle->Uart_Register->USART_CR1 >> 15) & 1;		//MASK THE VALUE OF OVER8

	//APPLY THE FORMULA OF FINDING USARTDIV HERE (derived from original formula: TX/RX BAUD = FCLCK/8 * (2 - OVER8) * USARTDIV (ON REFERENCE MANUAL PAGE - 978)
	USARTDIV = FCLCK_FREQ/8*(2 - OVER8)*Baud_Rate;

	uint16_t DIV_Mantissa;
	DIV_Mantissa = USARTDIV;		//THIS WILL EXTRACT ONLY THE MANTISSA (as in integer, the part after the decimal is not taken.)

	uint16_t Fraction;
	Fraction = (USARTDIV * 100) - DIV_Mantissa;		//THIS WILL EXTRACT THE FRACTION PART

	uint16_t DIV_Fraction = ((OVER8 * Fraction) * 100 + 50)/100;		//THIS WILL CALCULATE THE ROUNDED OFF DIV_FRACTION (ready to put in the BRR)

	if(OVER8 = 1)
	{
		DIV_Fraction = DIV_Fraction & 7;   //7 = 0b111  		//THIS IS DONE TO KEEP THE BIT3 CLEAR AS IN REFERENCE MANUAL.
	}

	//SET THE VALUE OF DIV_FRACTION IN THE RIGHT PLACE
	usart_handle->Uart_Register->USART_BRR |= (DIV_Fraction << 0);

	//SET THE VALUE OF DIV_MANTISSA
	usart_handle->Uart_Register->USART_BRR |= (DIV_Mantissa << 4);
}


#endif /* USART_DRIVER_H_ */
