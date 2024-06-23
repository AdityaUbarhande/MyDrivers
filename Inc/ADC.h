/*
 * ADC_Driver.h
 *
 *  Created on: 11-Jan-2021
 *      Author: Aditya Ubarhande
 */

//todo: Complete the channel select API (too easy)

#ifndef ADC_DRIVER_H_
#define ADC_DRIVER_H_

#include"stm32f407.h"
#include"Bit_Position.h"
#include"Gpio_Driver.h"

#define ADC_IN_TEMP_SENSOR					ADC_IN16

typedef struct
{
	uint8_t ADC_Channel;				//SELECT ONE OF THE CHANNELS
	uint8_t ADC_Channel_Type;			//CAN BE INJECTED OR REGULAR
}ADC_CONFIG_t;

typedef struct
{
	ADC_REGISTERS_t *Adc_Registers;
	ADC_CONFIG_t Adc_Config;
}ADC_HANDLE_t;

void ADC_CLK_ENABLE(ADC_REGISTERS_t *ADC);
void ADC_CLK_DISABLE(ADC_REGISTERS_t *ADC);

void ADC_ENABLE(ADC_REGISTERS_t *ADC);
void ADC_DISABLE(ADC_REGISTERS_t *ADC);
void TEMP_SENSOR_EN();
void TEMP_SENSOR_DI();
double READ_TEMP_DATA();

void SET_SAMPLING_TIME(ADC_REGISTERS_t *ADC, uint8_t channel_name, uint8_t sampling_time);

void ADC_CHANNEL_SELECT(ADC_REGISTERS_t *ADC, uint8_t priority, uint8_t channel_num);
uint16_t ADC_READ_DATA(ADC_REGISTERS_t *ADC);

static uint16_t ADC_data_return(ADC_REGISTERS_t *ADC);
static void ADC_strt_conv(ADC_REGISTERS_t *ADC);
uint8_t ADC_FLAG_STAT(ADC_REGISTERS_t *ADC, uint8_t flag_name);

void ADC_CLK_ENABLE(ADC_REGISTERS_t *ADC)
{
	if(ADC == ADC1)
	{
		Rcc_Registers->RCC_APB2ENR |= (1 << 8);
	}
	else if(ADC == ADC2)
	{
		Rcc_Registers->RCC_APB2ENR |= (1 << 9);
	}
	else if(ADC == ADC3)
	{
		Rcc_Registers->RCC_APB2ENR |= (1 << 10);
	}
}

void ADC_CLK_DISABLE(ADC_REGISTERS_t *ADC)
{
	if(ADC == ADC1)
	{
		Rcc_Registers->RCC_APB2ENR &= ~(1 << 8);
	}
	else if(ADC == ADC2)
	{
		Rcc_Registers->RCC_APB2ENR &= ~(1 << 9);
	}
	else if(ADC == ADC3)
	{
		Rcc_Registers->RCC_APB2ENR &= ~(1 << 10);
	}
}

void ADC_ENABLE(ADC_REGISTERS_t *ADC)
{
	ADC->ADC_CR2 |= (1 << ADC_CR2_ADON); 			//ENABLE THE ADC
}

void ADC_DISABLE(ADC_REGISTERS_t *ADC)
{
	ADC->ADC_CR2 &= ~(1 << ADC_CR2_ADON); 			//DISABLE THE ADC
}


void ADC_CHANNEL_SELECT(ADC_REGISTERS_t *ADC, uint8_t priority, uint8_t channel_num)
{
	switch(priority)
	{
	case 1: ADC->ADC_SQR3 |= (channel_num << 0);		//SET CHANNEL 1 IN SQR1 (refer the reference manual)
			break;
	case 2: ADC->ADC_SQR3 |= (channel_num << 5); 		//SET CHANNEL 2 IN SQR1 (refer the reference manual)
			break;
	case 3: ADC->ADC_SQR3 |= (channel_num << 10); 	//SET CHANNEL 3 IN SQR1 (refer the reference manual)
			break;
	case 4: ADC->ADC_SQR3 |= (channel_num << 15); 	//SET CHANNEL 4 IN SQR1 (refer the reference manual)
			break;
	case 5: ADC->ADC_SQR3 |= (channel_num << 20); 	//SET CHANNEL 5 IN SQR1 (refer the reference manual)
			break;
	case 6: ADC->ADC_SQR3 |= (channel_num << 25); 	//SET CHANNEL 6 IN SQR1 (refer the reference manual)
			break;
	case 7: ADC->ADC_SQR2 |= (channel_num << 0); 		//SET CHANNEL 7 IN SQR2 (refer the reference manual)
			break;
	case 8: ADC->ADC_SQR2 |= (channel_num << 5); 		//SET CHANNEL 8 IN SQR2 (refer the reference manual)
			break;
	case 9: ADC->ADC_SQR2 |= (channel_num << 10); 	//SET CHANNEL 9 IN SQR2 (refer the reference manual)
			break;
	case 10: ADC->ADC_SQR2 |= (channel_num << 15); 	//SET CHANNEL 10 IN SQR2 (refer the reference manual)
			break;
	case 11: ADC->ADC_SQR2 |= (channel_num << 20); 	//SET CHANNEL 11 IN SQR2 (refer the reference manual)
			break;
	case 12: ADC->ADC_SQR2 |= (channel_num << 25); 	//SET CHANNEL 12 IN SQR2 (refer the reference manual)
			break;
	case 13: ADC->ADC_SQR1 |= (channel_num << 0); 	//SET CHANNEL 13 IN SQR3 (refer the reference manual)
			 break;
	case 14: ADC->ADC_SQR1 |= (channel_num << 5); 	//SET CHANNEL 14 IN SQR3 (refer the reference manual)
			 break;
	case 15: ADC->ADC_SQR1 |= (channel_num << 10); 	//SET CHANNEL 15 IN SQR3 (refer the reference manual)
			 break;
	case 16: ADC->ADC_SQR1 |= (channel_num << 15); 	//SET CHANNEL 16 IN SQR3 (refer the reference manual)
			 break;
	default: printf("\n!!CAUTION: PLEASE CHOOSE A VALID ADC CHANNEL!!\n");
			 fflush(stdout);

	}
}

uint16_t ADC_READ_DATA(ADC_REGISTERS_t *ADC)
{
	uint16_t Data;
	ADC_strt_conv(ADC);			//START THE CONVERSION

	while(!(ADC_FLAG_STAT(ADC, ADC_FLAG_STRT)));     			//WAIT TILL THE CONVERSION HAS STARTED
	while(!(ADC_FLAG_STAT(ADC, ADC_FLAG_EOC)));		  			//WAIT TILL THE CONVERSION IS COMPLETED

	Data = ADC_data_return(ADC);			//READ DATA FROM DATA REGISTER
	return Data;
}

static void ADC_strt_conv(ADC_REGISTERS_t *ADC)
{
	ADC->ADC_CR2 |= (1 << ADC_CR2_SWSTART);			//START THE CONVERSION BY SETTING THE SWSTART BIT IN CONTROL REGISTER 2 (CR2)
}

static uint16_t ADC_data_return(ADC_REGISTERS_t *ADC)
{
	uint16_t data;
	data = (uint16_t)ADC->ADC_DR;
	return data;
}

void TEMP_SENSOR_EN()
{
	ADC_COMMON->ADC_CCR &= ~(1 << ADC_CCR_VBATE);			//DISABLE THE VBATE (given in reference manual)
	ADC_COMMON->ADC_CCR |= (1 << ADC_CCR_TSVREFE);			//ENABLE THE TEMP SENSOR.
}

void TEMP_SENSOR_DI()
{
	ADC_COMMON->ADC_CCR &= ~(1 << ADC_CCR_TSVREFE);			//DISABLE THE TEMP SENSOR.
}

void SET_SAMPLING_TIME(ADC_REGISTERS_t *ADC, uint8_t channel_name, uint8_t sampling_time)
{
	if((channel_name >= 0) && (channel_name <= 9))
	{
		//GO TO SMPR2
		ADC->ADC_SMPR2 |= (sampling_time << ((channel_name * 3) % 30));
	}
	else if((channel_name >= 10) && (channel_name <= 18))
	{
		//GO TO SMPR1
		ADC->ADC_SMPR1 |= (sampling_time << ((channel_name * 3) % 30));
	}
}

double READ_TEMP_DATA()
{
	//FORMULA USED: Temperature (in °C) = {(VSENSE – V25) / Avg_Slope} + 25

	//VSENSE: DATA READ IN THE DATA REGISTER
	//Avg_Slope: 2.5 (value given in datasheet)
	//NOTE: THE Avg_Slope IS TAKEN TO BE 0.0025 AS IN DATASHEET IT IS GIVEN 2.5mV which = 0.0025V

	ADC_strt_conv(ADC1);		//ENABLE CONVERSION

	while(!(ADC_FLAG_STAT(ADC1, ADC_FLAG_EOC)));		  			//WAIT TILL THE CONVERSION IS COMPLETED

	uint16_t data = ADC1->ADC_DR;		//READ DATA;
	double voltage, celsius;

	voltage = (double)data/4095*3.3;
	celsius = (double)(((voltage - 0.76) / 0.0025) + 25);

	return celsius;
}

uint8_t ADC_FLAG_STAT(ADC_REGISTERS_t *ADC, uint8_t flag_name)
{
	uint8_t return_val = 0;
	if(ADC->ADC_SR & flag_name)
	{
		return_val = 1;
	}
	return return_val;
}

#endif /* ADC_DRIVER_H_ */
