///*
// * I2C_Driver.h
// *
// *  Created on: Aug 26, 2020
// *      Author: 91726
// */
//
//#ifndef I2C_DRIVER_H_
//#define I2C_DRIVER_H_
//
//#include"stm32f407.h"
//#include"Bit_Position.h"
//#include"Gpio_Driver.h"
//#include"Rcc_Driver.h"
//#include<stdio.h>
//#include<string.h>
//#include<stddef.h>
//
//#define SM_MODE		0
//#define FM_MODE		1
//
//#define FLAG_SET		1
//#define FLAG_RESET		0
//
//#define BIT_7			1
//#define BIT_8			2
//#define BIT_10			3
//
//#define NO_RE_START		0
//#define RE_START		1
//
//#define READ 			1
//#define WRITE			0
//
//#define I2C_FLAG_SB						(1 << I2C_SR1_SB)
//#define I2C_FLAG_ADDR					(1 << I2C_SR1_ADDR)
//#define I2C_FLAG_BTF					(1 << I2C_SR1_BTF)
//#define I2C_FLAG_RXNE					(1 << I2C_SR1_RXNE)
//#define I2C_FLAG_TXE					(1 << I2C_SR1_TXE)
//#define I2C_FLAG_BERR					(1 << I2C_SR1_BERR)
//#define I2C_FLAG_ARLO					(1 << I2C_SR1_ARLO)
//#define I2C_FLAG_AF						(1 << I2C_SR1_AF)
//#define I2C_FLAG_OVR					(1 << I2C_SR1_OVR)
//#define I2C_FLAG_PECERR					(1 << I2C_SR1_PECERR)
//#define I2C_FLAG_TIMEOUT				(1 << I2C_SR1_TIMEOUT)
//
//#define I2C_BUSY_TX						0
//#define I2C_BUSY_RX						1
//
//uint8_t SM_FM_MODE;
//uint32_t CCR_VAL;
//
//typedef struct
//{
//	uint32_t clock_speed;
//	uint16_t own_address;
//	uint8_t Address_Mode_select;
//	uint8_t ACK_control;
//	uint8_t FM_duty_cycle;
//}I2C_CONFIG_t;
//
//typedef struct
//{
//	I2C_REGISTERS_t *I2C_Registers;
//	I2C_CONFIG_t I2C_Config;
//	uint8_t *pTxBufferAddr;
//	uint8_t *pRxBufferAddr;
//	uint8_t TxLen;
//	uint8_t RxLen;
//	uint8_t TxRxState;
//	uint16_t DevAddr;
//	uint8_t RE_Start_OnOff;
//	uint32_t RxSize;
//}I2C_HANDLE_t;
//
//
///***************************************CLOCK APIs************************************/
//void I2C_CLCK_EN(I2C_REGISTERS_t *I2C_Registers);
//void I2C_CLCK_DI(I2C_REGISTERS_t *I2C_Registers);
///***************************************************************************************/
//
///***************************************INIT, DE-INIT AND ENABLE APIs***************************/
//void I2C_INIT(I2C_HANDLE_t* I2C_HANDLE);
//void I2C_DEINIT(I2C_HANDLE_t* I2C_HANDLE);
//void I2C_EN_DI(I2C_REGISTERS_t* I2C_Registers, uint8_t enordi);
///**************************************************************************************************/
//
///*******************************************I2C INTERRUPTS APIs*****************************/
////PROCESSOR CONFIGURATION APIs
//void I2C_IRQ_NVIC_CONFIG(uint16_t IRQ_num, uint8_t EN_or_DI);
//void I2C_IRQ_PRIORITY_CONFIG(uint16_t IRQ_num, uint16_t IRQ_priority);
///*******************************************************************************************/
//
///******************************************DATA TRANSCEIVING APIs**************************/
//void I2C_SEND_DATA_MASTER(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI);
//void I2C_RECEIVE_DATA_MASTER(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t *RxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len);
//void I2C_SEND_DATA_MASTERIT(I2C_HANDLE_t* I2C_HANDLE, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI);
//void I2C_RECEIVE_DATA_MASTERIT(I2C_HANDLE_t* I2C_HANDLE, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI);
///****************************************************************************************/
//
///******************************************INTERRUPTS APIs*********************************************/
//void I2C_SEND_DATA_MASTERIT(I2C_HANDLE_t* I2C_HANDLE, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI);
//void I2C_RECEIVE_DATA_MASTERIT(I2C_HANDLE_t* I2C_HANDLE, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI);
//void I2C_INTERRUPT_HANDLE(I2C_HANDLE_t* I2C_HANDLE);
///******************************************************************************************************/
//
///*********************************HELPER FUNCTIONS**********************/
//static void I2C_generate_start(I2C_REGISTERS_t* I2C_REGISTERS);
//static void I2C_clear_EV5_Write(I2C_REGISTERS_t* I2C_REGISTERS, uint16_t SLAVE_ADDRESS);
//static void I2C_clear_EV5_Read(I2C_REGISTERS_t* I2C_REGISTERS, uint16_t SLAVE_ADDRESS);
//static void I2C_clear_EV6(I2C_REGISTERS_t* I2C_REGISTERS);
//static void I2C_start_data_sending(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t* TxBuffer, uint32_t len);
//static void I2C_generate_stop(I2C_REGISTERS_t* I2C_REGISTERS);
//static void I2C_start_data_receiving(I2C_REGISTERS_t* I2C_REGISTERS, uint32_t len, uint8_t *RxBuffer);
//static void I2C_start_data_receiving_1BYTE(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t* RxBuffer);
//
////FOR INTERRUPTS
//static void I2C_startTX_IT(I2C_HANDLE_t* I2C_HANDLE);
//static void I2C_startRX_IT(I2C_HANDLE_t* I2C_HANDLE);
//static void I2C_close_reception(I2C_HANDLE_t* I2C_HANDLE);
//static void I2C_close_data_transmit(I2C_HANDLE_t* I2C_HANDLE);
///**************************************************************************/
//
///********************************EXTRA APIs*******************/
//void I2C_SEND_SLAVE_ADDR(I2C_REGISTERS_t* I2C_REGISTERS, uint16_t SLAVE_ADDRESS, uint8_t READ_or_WRITE);
//void I2C_START_GENERATER(I2C_REGISTERS_t* I2C_REGISTERS);
//
///*****************************************STATUS APIs*****************************/
//
//int I2C_FLAG_STAT(I2C_REGISTERS_t *I2C_REGISTERS, uint16_t flagname);
//
///**********************************************************************************/
//
///*****************************************ADDRESS SCANNING API***********************************/
//
//int I2C_SCAN_ADDR(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t ADDR_MODE);		//ADDR MODE CAN BE 7BIT, 8BIT OR 10BIT ONLY
//
///***********************************************************************************************/
//void I2C_CLCK_EN(I2C_REGISTERS_t *I2C_Registers)
//{
//	if(I2C_Registers == I2C1)
//	{
//		Rcc_Registers->RCC_APB1ENR |= (1 << 21);
//	}
//	else if(I2C_Registers == I2C2)
//	{
//		Rcc_Registers->RCC_APB1ENR |= (1 << 22);
//	}
//	else if(I2C_Registers == I2C3)
//	{
//		Rcc_Registers->RCC_APB1ENR |= (1 << 23);
//	}
//}
//
//void I2C_CLCK_DI(I2C_REGISTERS_t *I2C_Registers)
//{
//	if(I2C_Registers == I2C1)
//	{
//		Rcc_Registers->RCC_APB1ENR &= ~(1 << 21);
//	}
//	else if(I2C_Registers == I2C2)
//	{
//		Rcc_Registers->RCC_APB1ENR &= ~(1 << 22);
//	}
//	else if(I2C_Registers == I2C3)
//	{
//		Rcc_Registers->RCC_APB1ENR &= ~(1 << 23);
//	}
//}
//
//
//
//void I2C_INIT(I2C_HANDLE_t* I2C_HANDLE)
//{
//	int temp = 0;
//
//	/*******************************************ACK CONTROL************************************************/
//	if(I2C_HANDLE->I2C_Config.ACK_control == ACK_ENABLE || I2C_HANDLE->I2C_Config.ACK_control == ACK_DISABLE)
//	{
//		I2C_HANDLE->I2C_Registers->I2C_CR1 &= ~(1 << I2C_CR1_ACK);								   		//CLEAR
//		I2C_HANDLE->I2C_Registers->I2C_CR1 |= (I2C_HANDLE->I2C_Config.ACK_control << I2C_CR1_ACK); 		//SET
//	}
//
//	/******************************************************************************************************/
//
//	/*******************************************OWN ADDRESS REGISTER (OAR1) CONFIGURATION************************************************/
//	if(I2C_HANDLE->I2C_Config.Address_Mode_select == OWN_ADDRESS_7)
//	{
//		I2C_HANDLE->I2C_Registers->I2C_OAR1 |= (OWN_ADDRESS_7 << 15); 								//SET THE ADDRESS MODE AS 7 BIT ADDRESS
//		I2C_HANDLE->I2C_Registers->I2C_OAR1 |= ((I2C_HANDLE->I2C_Config.own_address) << 1);			//SET THE ADDRESS
//		I2C_HANDLE->I2C_Registers->I2C_OAR1 |= (1 << 14);									//REFER PG-864 OF REFERENCE MANUAL (RM407) IT IS A NEED TO SET 14TH BIT
//	}
//	else if(I2C_HANDLE->I2C_Config.Address_Mode_select == OWN_ADDRESS_10)
//	{
//		I2C_HANDLE->I2C_Registers->I2C_OAR1 |= (OWN_ADDRESS_10 << 15); 								//SET THE ADDRESS MODE AS 7 BIT ADDRESS
//		I2C_HANDLE->I2C_Registers->I2C_OAR1 |= ((I2C_HANDLE->I2C_Config.own_address) << 0);			//SET THE ADDRESS
//		I2C_HANDLE->I2C_Registers->I2C_OAR1 |= (1 << 14);									//REFER PG-864 OF REFERENCE MANUAL (RM407) IT IS A NEED TO SET 14TH BIT
//	}
//	/************************************************************************************************************************************/
//
//	/*******************************************FREQ FIELD CONFIGURATION************************************************/
//		temp = GET_APB_FREQUENCY();				//THIS WILL GET US THE VALUE OF APB FREQUENCY
//
//		uint32_t FREQ_VAL = (temp / 1000000U);				//THIS WILL MAKE THE 16000000 (16MHz) AS 16(10000 in binary) TO BE STORED IN THE FREQ FIELD
//
//		I2C_HANDLE->I2C_Registers->I2C_CR2 |= (FREQ_VAL << I2C_CR2_FREQ);		//SET THE VALUE
//
//	/*******************************************************************************************************************/
//
//	/*******************************************CLOCK CONTROL CONFIGURATIONS*********************************************************************/
//	if(I2C_HANDLE->I2C_Config.clock_speed > 100000)				//FAST MODE
//	{
//		SM_FM_MODE = FM_MODE;
//		I2C_HANDLE->I2C_Registers->I2C_CCR |= (FM_MODE << I2C_CCR_FS); 		//SET THE MODE AS FAST MODE
//	}
//	else if(I2C_HANDLE->I2C_Config.clock_speed <= 100000)		//STANDARD MODE
//	{
//		SM_FM_MODE = SM_MODE;
//		I2C_HANDLE->I2C_Registers->I2C_CCR &= ~(1 << I2C_CCR_FS); 		//SET THE MODE AS STANDAED MODE
//	}
//
//	//CCR FORMULA AND SETTING THE I2C SCL VALUE
//
//	if(SM_FM_MODE == SM_MODE)
//	{
//		CCR_VAL = temp / (I2C_HANDLE->I2C_Config.clock_speed * 2);		//temp IS THE APB1_BUS PCLCK (SEE FREQ_FIELD_CONFIGURATION)
//
//		if(CCR_VAL < 0x04)
//		{
//			printf("\n\nCAUTION: THE CCR VALUE IS OUT OF MINIMUM LIMIT!!\n\n");
//			CCR_VAL = 0x08;			//SET TO LIMIT OF STANDARD MODE (0x08) = 10000000 = 10KHz
//		}
//
//	}
//
//	else if(SM_FM_MODE == FM_MODE)
//	{
//		//SET THE DUTY CYCLE
//		if((I2C_HANDLE->I2C_Config.FM_duty_cycle == I2C_FM_DUTY_2) || (I2C_HANDLE->I2C_Config.FM_duty_cycle == I2C_FM_DUTY_16_9))
//		{
//			temp = I2C_HANDLE->I2C_Config.FM_duty_cycle;
//			I2C_HANDLE->I2C_Registers->I2C_CCR |= ((I2C_HANDLE->I2C_Config.FM_duty_cycle) << 14);
//		}
//
//		//CALCULATE THE CCR AND SET THE CCR VALUE
//		if(temp == I2C_FM_DUTY_2)
//		{
//			//THE CCR FORMULA WILL BE:
//			CCR_VAL = temp / (I2C_HANDLE->I2C_Config.clock_speed * 3);		//temp IS THE APB1_BUS PCLCK (SEE FREQ_FIELD_CONFIGURATION)
//			if(CCR_VAL < 0x01)
//			{
//				printf("\n\nCAUTION: THE CCR VALUE IS OUT OF MINIMUM LIMIT!!\n\n");
//				CCR_VAL = 0x01;			//SET TO LIMIT OF FAST MODE IN 2*T_high DUTY CYCLE (0x01) = 50000000 = 5KHz
//			}
//		}
//		else if(temp == I2C_FM_DUTY_16_9)
//		{
//			//THE CCR FORMULA WILL BE:
//			CCR_VAL = temp / (I2C_HANDLE->I2C_Config.clock_speed * 25);		//temp IS THE APB1_BUS PCLCK (SEE FREQ_FIELD_CONFIGURATION)
//			if(CCR_VAL < 0x01)
//			{
//				printf("\n\nCAUTION: THE CCR VALUE IS OUT OF MINIMUM LIMIT!!\n\n");
//				CCR_VAL = 0x01;			//SET TO LIMIT OF FAST MODE IN 16/9*T_high DUTY CYCLE (0x01) = 50000000 = 5KHz
//			}
//		}
//	}
//
//	I2C_HANDLE->I2C_Registers->I2C_CCR |= (CCR_VAL << 0); 			//SET THE CCR VALUE
//	/********************************************************************************************************************************************/
//
//	/*************************************TRISE CONFIGURATIONS*******************************************/
//	uint8_t TRISE = 0;
//	if(SM_FM_MODE == SM_MODE)
//	{
//		//Standard Mode
//		//FORMULA OF TRISE IN STANDARD MODE
//
//		//((Fpclck * TRISE(max))/1000000) + 1
//
//		 TRISE = (((temp * 1)/1000000) + 1);		//temp = Fpclck
//	}
//	else if(SM_FM_MODE == FM_MODE)
//	{
//		//Fast Mode
//		//FORMULA OF TRISE IN FAST MODE
//
//		//((Fpclck * TRISE(max))/1000000) + 1
//
//		 TRISE = (((temp * 300)/1000000) + 1 );		//temp = Fpclck
//	}
//
//	I2C_HANDLE->I2C_Registers->I2C_TRISE |= (TRISE << 0);	//SET THE TRISE VALUE
//
//	/*********************************************************************************************/
//
//}
//
//
//
//void I2C_EN_DI(I2C_REGISTERS_t* I2C_Registers, uint8_t enordi)
//{
//	if(enordi == ENABLE)
//	{
//		I2C_Registers->I2C_CR1 |= (1 << I2C_CR1_PE);
//	}
//	else
//	{
//		I2C_Registers->I2C_CR1 &= ~(1 << I2C_CR1_PE);
//	}
//}
//
//
//
//void I2C_SEND_DATA_MASTER(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI)
//{
//	//STEP 1: GENERATE THE START CONDITION
//
//	I2C_generate_start(I2C_REGISTERS);
//
//	//STEP 2: CLEAR EV5 EVENT AND SEND THE ADDRESS
//	//EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
//
//	while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_SB)));			//WAIT TILL THE SB BIT POSITION IS SET
//	I2C_clear_EV5_Write(I2C_REGISTERS, SLAVE_ADDRESS);
//
//	//STEP 3: CLEAR THE ADDR FLAG THE EV6
//	//EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
//
//	while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_ADDR)));			//WAIT TILL THE ADDR BIT POSITION IS SET
//	I2C_clear_EV6(I2C_REGISTERS);
//
//	//STEP 4: SEND THE DATA BYTES
//	//EV8_1: TxE=1, shift register empty, data register empty, write Data1 in DR.
//
//	while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_TXE)));			//WAIT TILL THE TX BUFFER IS EMPTY
//	I2C_start_data_sending(I2C_REGISTERS, TxBuffer, len);
//
//	//STEP 5: GENERATE THE STOP CONDITION
//	//EV8_2: TxE=1, BTF = 1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition
//
//	if(RE_START_ENDI == NO_RE_START)		//IF USER DOESNT WANT TO DO REPEATED START
//	{
//		while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_BTF))); 			//WAIT TILL THE BTF IS SET
//		while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_TXE)));			//WAIT TILL THE TX BUFFER IS EMPTY
//		I2C_generate_stop(I2C_REGISTERS);
//	}
//}
//
//void I2C_SEND_DATA_MASTERIT(I2C_HANDLE_t* I2C_HANDLE, uint8_t *TxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len, uint8_t RE_START_ENDI)
//{
//	//STEP 1: CHECK THE STATE OF THE I2C BUS
//	uint8_t status = I2C_HANDLE->TxRxState;
//
//	//STEP 2: CHECK IF THE STATE OF THE I2C BUS IS BUSY OR NOT (a-b)
//	if(status != I2C_BUSY_TX && status != I2C_BUSY_RX)
//	{
//		//STEP 3: SET I2C IS GONNA BE BUSY IN TX
//		I2C_HANDLE->TxRxState = I2C_BUSY_TX;
//
//		//STEP 4: SET THE DEVICE ADDRESS
//		I2C_HANDLE->DevAddr = SLAVE_ADDRESS;
//
//		//STEP 5: SET THE Tx BUFFER ADDRESS (address of data to be sent)
//		I2C_HANDLE->pTxBufferAddr = TxBuffer;
//
//		//STEP 6: SET THE Tx LENGTH
//		I2C_HANDLE->TxLen = len;
//
//		//STEP 7: SET THE REPEATED START CONDITION
//		I2C_HANDLE->RE_Start_OnOff = RE_START_ENDI;
//
//		//STEP 8: GENERATE THE START CONDITION
//		I2C_generate_start((I2C_HANDLE->I2C_Registers));
//
//		//STEP 9 A: ENABLE THE BUFFER INTERRUPTS
//		I2C_HANDLE->I2C_Registers->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
//
//		//STEP 9 B: ENABLE THE ERROR INTERRUPTS
//		I2C_HANDLE->I2C_Registers->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
//
//		//STEP 9 C: ENABLE THE EVENT INTERRUPTS
//		I2C_HANDLE->I2C_Registers->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
//
//	}
//
//}
//
//void I2C_INTERRUPT_HANDLE(I2C_HANDLE_t* I2C_HANDLE)
//{
//	int EV, BUF, CON;
//	EV = I2C_HANDLE->I2C_Registers->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
//	BUF = I2C_HANDLE->I2C_Registers->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
//
//	//STEP 1: CHECK FOR THE SB EVENT INTERRUPT
//	CON = I2C_HANDLE->I2C_Registers->I2C_SR1 & (1 << I2C_SR1_SB);		//CHECK IF THE SB FLAG IS SET OR NOT.
//
//	if(EV && CON)		//IF THE EV INTERRRUPT IS ENABLED AND THE SB IS SET (clear EV5)
//	{
//		if(I2C_HANDLE->TxRxState == I2C_BUSY_TX)
//			I2C_clear_EV5_Write(I2C_HANDLE->I2C_Registers, I2C_HANDLE->DevAddr);
//
//		else if(I2C_HANDLE->TxRxState == I2C_BUSY_RX)
//			I2C_clear_EV5_Read(I2C_HANDLE->I2C_Registers, I2C_HANDLE->DevAddr);
//	}
//
//	//STEP 2: CHECK FOR THE ADDR EVENT INTERRUPT
//	CON = I2C_HANDLE->I2C_Registers->I2C_SR1 & (1 << I2C_SR1_ADDR);		//CHECK IF THE ADDR FLAG IS SET OR NOT.
//
//	if(EV && CON)  		//IF THE EV INTERRUPT IS ENABLED AND THE ADDR IS SET (clear EV6)
//	{
//		I2C_clear_EV6(I2C_HANDLE->I2C_Registers);		//CLEAR EV6
//	}
//
//	//STEP 3.1: CHECK FOR THE TxE EVENT AND BUFFER INTERRUPT
//	CON = I2C_HANDLE->I2C_Registers->I2C_SR1 & (1 << I2C_SR1_TXE);			//CHECK IF THE TxE FLAG IS SET OR NOT
//
//	if(EV && BUF && CON)			//IF THE EVENT INTERRUPT AND THE BUFFER INTERRUPT IS ENABLED AND THE TxE FLAG IS SET
//	{
//		I2C_startTX_IT(I2C_HANDLE);
//	}
//
//	//STEP 3.2: CHECK FOR THE RXNE EVENT AND BUFFER INTERRUPT
//	CON = I2C_HANDLE->I2C_Registers->I2C_SR1 & (1 << I2C_SR1_RXNE);			//CHECK IF THE RXNE FLAG IS SET OR NOT
//
//	if(EV && BUF && CON)			//IF THE EVENT INTERRUPT AND THE BUFFER INTERRUPT IS ENABLED AND THE RXNE FLAG IS SET
//	{
//		I2C_startRX_IT(I2C_HANDLE);
//	}
//
//	//STEP 4: GENERATE THE STOP CONDITION IF THE BTF AND THE TxE IS SET (no need to generate the STOP condition in the RX as we have done in the I2C_startRX_IT() already)
//	CON = I2C_HANDLE->I2C_Registers->I2C_SR1 & (1 << I2C_SR1_BTF);			//CHECK IF THE BTF FLAG IS SET OR NOT
//
//	if(EV && BUF && CON)		//THIS CHECKS WHETHER THE BTF (EV), TxE (BUF and EV) and the BTF bit is set or not and the interrupts are enabled or not
//	{
//		if(I2C_HANDLE->I2C_Registers->I2C_SR1 & (1 << I2C_SR1_TXE))		//DOUBLE CHECK FOR THE TxE FLAG
//		{
//			//GENERATE THE STOP (only for Transmission)
//			I2C_HANDLE->I2C_Registers->I2C_CR1 |= (1 << I2C_CR1_STOP);	//GENERATE THE STOP
//		}
//	}
//	else  //FOR STOPPING THE RECEPTION OF DATA YOU DON'T NEED TO DO ANYTHING
//	{
//		;
//	}
//
//	//DONE
//}
//
//void I2C_RECEIVE_DATA_MASTER(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t* RxBuffer, uint16_t SLAVE_ADDRESS, uint32_t len)
//{
//	//STEP 1: GENERATE THE START CONDITION
//	 I2C_generate_start(I2C_REGISTERS);
//
//	 //STEP 2: CLEAR EV5 EVENT AND SEND THE ADDRESS
//	//EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
//
//	while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_SB)));			//WAIT TILL THE SB BIT POSITION IS SET
//	I2C_clear_EV5_Read(I2C_REGISTERS, SLAVE_ADDRESS);
//
//	while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_ADDR)));			//WAIT TILL THE ADDR BIT POSITION IS SET
//
//	if(len == 1)				//IF ONLY 1 BYTE TO BE SENT (THIS IS TO BE DONE BEFORE CLEARING THE ADDR FLAG AS THE FLAG WILL BE SET IN THE FUNCTION
//	{
//		I2C_start_data_receiving_1BYTE(I2C_REGISTERS, RxBuffer);		//NO NEED TO SEND LENGTH BECAUSE ONLY 1 BYTE TO BE SENT
//		len--;
//	}
//	else if(len > 1)
//	{
//		//STEP 3: CLEAR THE ADDR FLAG THE EV6
//		//EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
//		I2C_clear_EV6(I2C_REGISTERS);
//
//		//STEP 4: START RECEIVING THE DATA SENT BY THE SLAVE
//		//EV7: RXNE=1, cleared by reading DR register
//
//		//NOTE: WE DONT HAVE TO WAIT UNTIL THE ADDR FLAG IS SET, BECAUSE WE HAVE TO DISABLE ACKING AND LET THE ADDR FLAG STRETCH THE SCL UNTIL THEN
//
//		while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_RXNE)));			//WAIT TILL THE RX BUFFER IS FILLED (RXNE IS SET)
//		I2C_start_data_receiving(I2C_REGISTERS, len, RxBuffer);
//	}
//
//
//	//RE-ENABLE THE ACKING
////	if(!(I2C_REGISTERS->I2C_CR1 & (1 << I2C_CR1_ACK)))
//		I2C_REGISTERS->I2C_CR1 |= (1 << I2C_CR1_ACK);		//ENABLE THE ACK
//
//}
//
//
//void I2C_generate_start(I2C_REGISTERS_t* I2C_REGISTERS)
//{
//	//GENERATE THE START
//	I2C_REGISTERS->I2C_CR1 |= (1 << I2C_CR1_START);
//}
//
//static void I2C_clear_EV5_Write(I2C_REGISTERS_t* I2C_REGISTERS, uint16_t SLAVE_ADDRESS)
//{
//	uint32_t temp_1;
//	//THE STEPS TO CLEAR EV5 ARE:
//
//	// 1] READ THE SR1 (STATUS REGISTER 1)
//	temp_1 = I2C_REGISTERS->I2C_SR1;
//	printf("%ld", temp_1);		//THIS IS A DUMMY PRINTF TO REMOVE THE UN-USED VARIABLE WARNING
//
//	// 2] WRITING THE DR (DATA REGISTER) WITH THE ADDRESS
//	SLAVE_ADDRESS = SLAVE_ADDRESS << 1;   	//THIS WILL DO: ADD = 0b1010101 as 0b10101010 thus creating a extra space at the LSB for Read-Write Bit
//
//	//NOW AS THIS IS SENDING DATA WHICH IS WRITING THE READ-WRITE BIT WILL BE 0
//	//SO WE GET:
//	SLAVE_ADDRESS &= ~(1 << 0); //HERE, WE ARE JUST CLEARING THE 0TH BIT OF THE SLAVE_ADDRESS
//
//	//NOW PUT THIS ADDRESS WITH THE READ-WRITE (R/W) BIT INTO THE DATA REGISTER
//	I2C_REGISTERS->I2C_DR |= (SLAVE_ADDRESS << 0);
//}
//
//static void I2C_clear_EV5_Read(I2C_REGISTERS_t* I2C_REGISTERS, uint16_t SLAVE_ADDRESS)
//{
//	uint32_t temp_1;
//	//THE STEPS TO CLEAR EV5 ARE:
//
//	// 1] READ THE SR1 (STATUS REGISTER 1)
//	temp_1 = I2C_REGISTERS->I2C_SR1;
//	printf("%ld", temp_1);		//THIS IS A DUMMY PRINTF TO REMOVE THE UN-USED VARIABLE WARNING
//
//	// 2] WRITING THE DR (DATA REGISTER) WITH THE ADDRESS
//	SLAVE_ADDRESS = SLAVE_ADDRESS << 1;   	//THIS WILL DO: ADD = 0b1010101 as 0b10101010 thus creating a extra space at the LSB for Read-Write Bit
//
//	//NOW AS THIS IS SENDING DATA WHICH IS WRITING THE READ-WRITE BIT WILL BE 1
//	//SO WE GET:
//	SLAVE_ADDRESS |= (1 << 0); //HERE, WE ARE JUST SETTING THE 0TH BIT OF THE SLAVE_ADDRESS
//
//	//NOW PUT THIS ADDRESS WITH THE READ-WRITE (R/W) BIT INTO THE DATA REGISTER
//	I2C_REGISTERS->I2C_DR |= (SLAVE_ADDRESS << 0);
//}
//
//static void I2C_clear_EV6(I2C_REGISTERS_t* I2C_REGISTERS)
//{
//	uint32_t temp;
//	/*STEPS TO CLEAR THE EV6:
//	 * 1. READ THE SR1
//	 * 2. READ THE SR2
//	 */
//	temp = I2C_REGISTERS->I2C_SR1;			//1. READ THE SR1
//	temp = I2C_REGISTERS->I2C_SR2;			//2. READ THE SR2
//	(void)temp;
//}
//
//static void I2C_start_data_sending(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t* TxBuffer, uint32_t len)
//{
//	while(len != 0)
//	{
//		while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_TXE)));			//WAIT TILL THE TX BUFFER IS EMPTY
//		I2C_REGISTERS->I2C_DR = *TxBuffer;
//		len--;
//		TxBuffer++;
//	}
//}
//
//static void I2C_start_data_receiving(I2C_REGISTERS_t* I2C_REGISTERS, uint32_t len, uint8_t *RxBuffer)
//{
//	while(len != 0)
//	{
//		while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_RXNE)));		//CHECK WHETHER THE RX BUFFER IS EMPTY OR NOT
//		*RxBuffer = (uint8_t)I2C_REGISTERS->I2C_DR;					//READ THE DATA
//		len--;		//3 , 2
//		RxBuffer++;
//
//		if(len == 2)		//IF LENGTH IS 1, DISABLE THE ACK
//		{
//			I2C_REGISTERS->I2C_CR1 &= ~(1 << I2C_CR1_ACK);		//DISABLE THE ACKING
//			I2C_REGISTERS->I2C_CR1 |= (1 << I2C_CR1_STOP);		//SEND STOP TO STOP THE COMMUNICATION
//		}
//	}
//}
//
//static void I2C_start_data_receiving_1BYTE(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t *RxBuffer)
//{
//	//AS THE DATA TO BE SENT IS ONLY OF 1 BYTE, THE STEPS ARE
//	//STEP 1: DISABLE THE ACKING
//	I2C_REGISTERS->I2C_CR1 &= ~(1 << I2C_CR1_ACK);			//DISABLE THE ACKING
//
//	//STEP 2: CLEAR THE ADDR FLAG (CLEAR EV6)
//	//EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
//	I2C_clear_EV6(I2C_REGISTERS);
//
//	//STEP 3: WAIT TILL THE RXNE IS SET (RECEIVE BUFFER IN NOT EMPTY)
//	while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_RXNE)));		//CHECK WHETHER THE RX BUFFER IS EMPTY OR NOT
//
//	//STEP 3: GENERATE THE STOP CONDITION
//	I2C_generate_stop(I2C_REGISTERS);
//
//	//STEP 4: READ THE DATA IN THE BUFFER
//	//EV7: RxNE=1 cleared by reading DR register
//	RxBuffer = (uint8_t)I2C_REGISTERS->I2C_DR;				//READ THE DATA
//
//}
//
//static void I2C_startTX_IT(I2C_HANDLE_t* I2C_HANDLE)
//{
//	I2C_HANDLE->I2C_Registers->I2C_DR = *(I2C_HANDLE->pTxBufferAddr);		//SEND THE DATA
//	I2C_HANDLE->TxLen--;				//DECREMENT THE LENGTH
//	(I2C_HANDLE->pTxBufferAddr)++;		//MOVE TO THE NEXT ADDRESS OF THE TX BUFFER
//
//	if(I2C_HANDLE->TxLen == 0)		//IF THE LENGTH IS ZERO (data transmission is completed)
//		I2C_close_data_transmit(I2C_HANDLE);
//}
//
//static void I2C_startRX_IT(I2C_HANDLE_t* I2C_HANDLE)
//{
//	I2C_HANDLE->pRxBufferAddr = I2C_HANDLE->I2C_Registers->I2C_DR;			//READ THE DATA
//	I2C_HANDLE->RxLen--;				//DECREMENT THE LENGTH
//	(I2C_HANDLE->pRxBufferAddr)++;		//MOVE TO THE NEXT ADDRESS OF THE RX BUFFER
//
//	if(I2C_HANDLE->RxLen == 2)
//	{
//		I2C_HANDLE->I2C_Registers &= ~(1 << I2C_CR1_ACK);		//DISABLE THE ACKing
//		I2C_HANDLE->I2C_Registers |= (1 << I2C_CR1_STOP);		//GENERATE THE START CONDITION
//	}
//
//	if(I2C_HANDLE->RxLen == 0)
//	{
//		I2C_close_reception(I2C_HANDLE);
//	}
//}
//
//static void I2C_close_reception(I2C_HANDLE_t* I2C_HANDLE)
//{
//	I2C_HANDLE->RxLen = 0;
//	I2C_HANDLE->RxSize = 0;
//	I2C_HANDLE->pRxBufferAddr = NULL;
//	I2C_HANDLE->TxRxState = I2C_READY;
//
//	//DISABLE THE INTERRUPTS
//	I2C_HANDLE->I2C_Registers->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);		//DISABLE THE BUFFER INTERRUPTS
//	I2C_HANDLE->I2C_Registers->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);		//DISABLE THE EVENT INTERRUPTS
//
//	//ENABLE THE ACKING IF IT IS DISABLED AND THE USER WANTS IT ENABLED
//	if((I2C_HANDLE->I2C_Config.ACK_control == ACK_ENABLE) && (!(I2C_HANDLE->I2C_Registers->I2C_CR1 & (1 << I2C_CR1_ACK))))
//		I2C_HANDLE->I2C_Registers->I2C_CR1 |= (1 << I2C_CR1_ACK);		//ENABLE THE ACKING
//}
//
//static void I2C_close_data_transmit(I2C_HANDLE_t* I2C_HANDLE)
//{
//	I2C_HANDLE->TxLen = 0;
//	I2C_HANDLE->TxSize = 0;
//	I2C_HANDLE->pTxBufferAddr = NULL;
//	I2C_HANDLE->TxRxState = I2C_READY;
//
//	//DISABLE THE INTERRUPTS
//	I2C_HANDLE->I2C_Registers->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);		//DISABLE THE BUFFER INTERRUPTS
//	I2C_HANDLE->I2C_Registers->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);		//DISABLE THE EVENT INTERRUPTS
//
//	//ENABLE THE ACKING IF IT IS DISABLED AND THE USER WANTS IT ENABLED
//	if((I2C_HANDLE->I2C_Config.ACK_control == ACK_ENABLE) && (!(I2C_HANDLE->I2C_Registers->I2C_CR1 & (1 << I2C_CR1_ACK))))
//		I2C_HANDLE->I2C_Registers->I2C_CR1 |= (1 << I2C_CR1_ACK);		//ENABLE THE ACKING
//}
//
//static void I2C_generate_stop(I2C_REGISTERS_t *I2C_REGISTERS)
//{
//	//GENERATE THE STOP
//	I2C_REGISTERS->I2C_CR1 |= (1 << I2C_CR1_STOP);
//}
//
//int I2C_FLAG_STAT(I2C_REGISTERS_t *I2C_REGISTERS, uint16_t flagname)
//{
//	if(I2C_REGISTERS->I2C_SR1 & flagname)
//	{
//		return FLAG_SET;
//	}
//	return FLAG_RESET;
//}
//
//int I2C_SCAN_ADDR(I2C_REGISTERS_t* I2C_REGISTERS, uint8_t ADDR_MODE)
//{
//	int count = 0;
//
//	if(ADDR_MODE == BIT_7)
//	{
//		count = 127;
//	}
//	else if(ADDR_MODE == BIT_8)
//	{
//		count = 255;
//	}
//	else if(ADDR_MODE == BIT_10)
//	{
//		count = 1023;
//	}
//	else
//	{
//		count = 127;   //SET TO 7BIT IF CHOICE IS NONE OF THESE
//	}
//
//
//	//NOW SEARCH FOR THE ADDRESS
//	int i;
//	int return_addr;
//
//		for(i = 1; i <= count; i++)
//		{
//			//STEP 2: CLEAR EV5 EVENT AND SEND THE ADDRESS
//			//EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
//			I2C_generate_start(I2C_REGISTERS);
//			while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_SB)));			//WAIT TILL THE SB BIT POSITION IS SET
//			I2C_clear_EV5_Write(I2C_REGISTERS, i);								//SEND 'i' AS THE SLAVE_ADDRESS
//
//
//
//			if(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_AF)))	//IF THE AF FLAG IS SET, THEN IT IS A NACK ELSE ACK
//			{
//				printf("!!!!Found 1 Device on %d address (address in decimal)!!!!\n", i);
//				return_addr = i;
//				break;
//
//			}
//
//			//STEP 3: CLEAR THE ADDR FLAG THE EV6
//			//EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
//
//			while(!(I2C_FLAG_STAT(I2C_REGISTERS, I2C_FLAG_ADDR)));			//WAIT TILL THE ADDR BIT POSITION IS SET
//			I2C_clear_EV6(I2C_REGISTERS);
//
//		}
//
//	return return_addr;
//}
//
//
//void I2C_SEND_SLAVE_ADDR(I2C_REGISTERS_t* I2C_REGISTERS, uint16_t SLAVE_ADDRESS, uint8_t READ_or_WRITE)
//{
//	// 1] WRITING THE DR (DATA REGISTER) WITH THE ADDRESS
//	SLAVE_ADDRESS = SLAVE_ADDRESS << 1;   	//THIS WILL DO: ADD = 0b1010101 as 0b10101010 thus creating a extra space at the LSB for Read-Write Bit
//
//	//NOW AS THIS IS SENDING DATA WHICH IS WRITING THE READ-WRITE BIT WILL BE 0
//	//SO WE GET:
//	if(READ_or_WRITE == WRITE)
//	{
//		SLAVE_ADDRESS &= ~(1 << 0); //HERE, WE ARE JUST CLEARING THE 0TH BIT OF THE SLAVE_ADDRESS
//	}
//	else if(READ_or_WRITE == READ)
//	{
//		SLAVE_ADDRESS |= (1 << 0); //HERE, WE ARE JUST SETTING THE 0TH BIT OF THE SLAVE_ADDRESS
//	}
//
//	//NOW PUT THIS ADDRESS WITH THE READ-WRITE (R/W) BIT INTO THE DATA REGISTER
//	I2C_REGISTERS->I2C_DR |= (SLAVE_ADDRESS << 0);
//}
//
//void I2C_START_GENERATER(I2C_REGISTERS_t* I2C_REGISTERS)
//{
//	I2C_REGISTERS->I2C_CR1 |= (1 << I2C_CR1_START);		//GENERATE THE START
//}
//
//#endif /* INC_I2C_DRIVER_H_ */
//
//
