/*
 * Rcc_Driver.h
 *
 *  Created on: Aug 26, 2020
 *      Author: Aditya Ubarhande.
 */

#ifndef RCC_DRIVER_H_
#define RCC_DRIVER_H_

#include<stdint.h>
#include"stm32f407.h"

#define HSI				0
#define HSE				1
#define PLL				2

#define HSI_FREQ		16000000
#define HSE_FREQ		8000000
#define NONE			0x00000

int GET_APB_FREQUENCY();
int GET_APB2_FREQENCY();

int GET_APB_FREQUENCY()
{
	uint32_t clcsrc_frequency = 0;
	uint32_t return_val = 0;

	//THIS IS TO FIND THE SYSTEM CLOCK FREQUENCY (MAX = 16MHz 16000000)

	if(Rcc_Registers->RCC_CFGR == HSI)					//IF CLOCK SOURCE IS HSI
	{
		clcsrc_frequency = HSI_FREQ;
	}
	else if(Rcc_Registers->RCC_CFGR == HSE)				//IF CLOCK SOURCE IS HSE
	{
		clcsrc_frequency = HSE_FREQ;
	}
	else if(Rcc_Registers->RCC_CFGR == PLL)				//IF CLOCK SOURCE IS PLL
	{
		clcsrc_frequency = NONE;
	}

	//THIS IS TO FIND THE AHB PRESCALAR	(MAX = 512, MIN = 1)

	uint16_t HPRE_VAL = ((Rcc_Registers->RCC_CFGR >> 4) & 0xF);		//0xF = 15 = 1111 in binary
	uint32_t AHB_PRSC = 0; //AHB PRESCALAR;

	if(HPRE_VAL == 0)
	{
		AHB_PRSC = 1;
	}
	else if(HPRE_VAL == 8)
	{
		AHB_PRSC = 2;
	}
	else if(HPRE_VAL == 9)
	{
		AHB_PRSC = 4;
	}
	else if(HPRE_VAL == 10)
	{
		AHB_PRSC = 8;
	}
	else if(HPRE_VAL == 11)
	{
		AHB_PRSC = 16;
	}
	else if(HPRE_VAL == 12)
	{
		AHB_PRSC = 64;
	}
	else if(HPRE_VAL == 13)
	{
		AHB_PRSC = 128;
	}
	else if(HPRE_VAL == 14)
	{
		AHB_PRSC = 256;
	}
	else if(HPRE_VAL == 15)
	{
		AHB_PRSC = 512;
	}


	//NOW FIND THE APB 1 PRESCALAR (MAX = 16, MIN = 1)
	uint16_t APB_FRQ = (Rcc_Registers->RCC_CFGR >> 10 & (7 << 0));		//7 = 111 in binary
	uint32_t APB_PRSC = 0; //APB PRESCALAR;

	if(APB_FRQ == 0)
	{
		APB_PRSC = 1;
	}
	else if(APB_FRQ == 4)
	{
		APB_PRSC = 2;
	}
	else if(APB_FRQ == 5)
	{
		APB_PRSC = 4;
	}
	else if(APB_FRQ == 6)
	{
		APB_PRSC = 8;
	}
	else if(APB_FRQ == 7)
	{
		APB_PRSC = 16;
	}

	return_val = (clcsrc_frequency / AHB_PRSC) / APB_PRSC;
	return return_val;

}

int GET_APB2_FREQENCY()
{
	uint32_t clcsrc_frequency = 0;
	uint32_t return_val = 0;

	//THIS IS TO FIND THE SYSTEM CLOCK FREQUENCY (MAX = 16MHz 16000000)

	if(Rcc_Registers->RCC_CFGR == HSI)					//IF CLOCK SOURCE IS HSI
	{
		clcsrc_frequency = HSI_FREQ;
	}
	else if(Rcc_Registers->RCC_CFGR == HSE)				//IF CLOCK SOURCE IS HSE
	{
		clcsrc_frequency = HSE_FREQ;
	}
	else if(Rcc_Registers->RCC_CFGR == PLL)				//IF CLOCK SOURCE IS PLL
	{
		clcsrc_frequency = NONE;
	}

	//THIS IS TO FIND THE AHB PRESCALAR	(MAX = 512, MIN = 1)

	uint16_t HPRE_VAL = ((Rcc_Registers->RCC_CFGR >> 4) & 0xF);		//0xF = 15 = 1111 in binary
	uint32_t AHB_PRSC = 0; //AHB PRESCALAR;

	if(HPRE_VAL == 0)
	{
		AHB_PRSC = 1;
	}
	else if(HPRE_VAL == 8)
	{
		AHB_PRSC = 2;
	}
	else if(HPRE_VAL == 9)
	{
		AHB_PRSC = 4;
	}
	else if(HPRE_VAL == 10)
	{
		AHB_PRSC = 8;
	}
	else if(HPRE_VAL == 11)
	{
		AHB_PRSC = 16;
	}
	else if(HPRE_VAL == 12)
	{
		AHB_PRSC = 64;
	}
	else if(HPRE_VAL == 13)
	{
		AHB_PRSC = 128;
	}
	else if(HPRE_VAL == 14)
	{
		AHB_PRSC = 256;
	}
	else if(HPRE_VAL == 15)
	{
		AHB_PRSC = 512;
	}


	//NOW FIND THE APB 1 PRESCALAR (MAX = 16, MIN = 1)
	uint16_t APB_FRQ = (Rcc_Registers->RCC_CFGR >> 13 & 7);		//7 = 111 in binary
	uint32_t APB_PRSC = 0; //APB PRESCALAR;

	if(APB_FRQ == 0)
	{
		APB_PRSC = 1;
	}
	else if(APB_FRQ == 4)
	{
		APB_PRSC = 2;
	}
	else if(APB_FRQ == 5)
	{
		APB_PRSC = 4;
	}
	else if(APB_FRQ == 6)
	{
		APB_PRSC = 8;
	}
	else if(APB_FRQ == 7)
	{
		APB_PRSC = 16;
	}

	return_val = (clcsrc_frequency / AHB_PRSC) / APB_PRSC;
	return return_val;
}

#endif /* RCC_DRIVER_H_ */
