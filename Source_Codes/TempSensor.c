#include <stdint.h>
#include<stdio.h>
int main(void)
{
	//Variable Definitions
    uint32_t* GPIO_MODER = (uint32_t*)0x40020C00;
    uint32_t* GPIO_ODR = (uint32_t*)0x40020C14;

    uint32_t* RCC_CLK_AHB1 = (uint32_t*)0x40023830;
    uint32_t* RCC_CLK_APB2 = (uint32_t*)0x40023844;

    uint32_t* ADC_CR2 = (uint32_t*)0x40012008;
    uint32_t* ADC_SQR3 = (uint32_t*)0x40012034;
    uint32_t* ADC_CCR = (uint32_t*)0x40012304;
    uint32_t* ADC_DR = (uint32_t*)0x4001204C;
    uint32_t* ADC_STAT = (uint32_t*)0x40012000;


    //Clock enable for GPIOD and ADC
    *RCC_CLK_AHB1 |= (1 << 3);	//Enable GPIOD clock
    *RCC_CLK_APB2 |= (1 << 8);		//Enable ADC clock

    //Gpio mode set (LEDs set to OUTPUT)
    *GPIO_MODER |= (1 << 30);
    *GPIO_MODER |= (1 << 28);
    *GPIO_MODER |= (1 << 26);
    *GPIO_MODER |= (1 << 24);

    //ADC Config

    *ADC_SQR3 |= (16 << 0);		//Select channel 16
    *ADC_CR2 |= (1 << 0);			//Set ADON bit
    *ADC_CCR |= (1 << 23);			//Enable Temperature Sensor

    while(1)
    {
		 *ADC_CR2 |= (1 << 30);					//Start Conversion
		 while(!((*ADC_STAT >> 4) == 1));		//Wait for conversion to start

		   uint16_t Vsense =  (uint16_t)*ADC_DR;			//Read ADC output data
		   double voltage, celsius;

		   voltage = (double)Vsense/4095*3.3;
		   celsius = (double)(((voltage - 0.76) / 0.0025) + 25);


		   printf("\nTemperature is: %.2f", celsius);			//Print the temperature through ITM (for debugging purposes)

		   if(celsius < 57.5)
			{
			   *GPIO_ODR &= ~(1 << 13);
				*GPIO_ODR &= ~(1 << 14);
				*GPIO_ODR |= (1 << 15);
			}
			else if(celsius < 68 && celsius > 58)
			{
				*GPIO_ODR &= ~(1 << 15);
				*GPIO_ODR &= ~(1 << 14);
				*GPIO_ODR |= (1 << 13);
			}
			else if(celsius > 65)
			{
				*GPIO_ODR &= ~(1 << 15);
				*GPIO_ODR &= ~(1 << 13);
				*GPIO_ODR |= (1 << 14);
			}

    }


}

