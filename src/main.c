/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "Utility.h"

void dois_fev_v1(void);
void dois_fev_v2(void);

int main(void)
{
	dois_fev_v2();
}


void dois_fev_v1(void)
{

	Utility_Init(); //habilita as funções especiais

	RCC -> AHB1ENR |= 0b11001;

	GPIOA -> MODER |= 0b01<<12;
	GPIOA -> MODER |= 0b01<<14;
	GPIOE -> MODER |= 0B01;
	GPIOD -> MODER |= 0B01 << 10;

	while(1)
	{
		GPIOA -> ODR &= ~(1<<6);
		GPIOA -> ODR |= 1 << 7;
		GPIOE -> ODR &= ~(1);
		GPIOD -> ODR &= ~(1<<5);
		Delay_ms(100);

		GPIOA -> ODR |= 1 << 6;
		GPIOA -> ODR &= ~(1<<7);
		GPIOE -> ODR &= ~(1);
		Delay_ms(100);
		GPIOA -> ODR |= 1<<6;
		GPIOA -> ODR |= 1 << 7;
		GPIOE -> ODR &= ~(1);
		GPIOD -> ODR &= ~(1<<5);
		Delay_ms(50);
		GPIOE -> ODR |= 1;
		Delay_ms(50);
		GPIOE -> ODR &= ~(1);
		Delay_ms(50);
		GPIOE -> ODR |= 1;
		Delay_ms(50);

		//Delay_ms(1000);
		GPIOA -> ODR |= 1<<6;
		GPIOA -> ODR |= 1 << 7;
		GPIOE -> ODR &= ~(1);
		GPIOD -> ODR &= ~(1<<5);
		Delay_ms(50);
		GPIOD -> ODR |= 1<<5;
		Delay_ms(50);
		GPIOD -> ODR &= ~(1<<5);
		Delay_ms(50);
		GPIOD -> ODR |= 1<<5;
		Delay_ms(50);

	}


}

void dois_fev_v2(void)
{

	Utility_Init(); //habilita as funções especiais

	RCC -> AHB1ENR |= 0b11001;

	GPIOA -> MODER |= 0b01<<12;
	GPIOA -> MODER |= 0b01<<14;
	GPIOE -> MODER |= 0B01;
	GPIOE -> MODER |= 0B01 << 2;

	while(1)
	{

		GPIOE -> ODR |= 1 | 1 << 1;
		Delay_ms(50);
		GPIOE -> ODR &= ~(1 | 1<<1);
		Delay_ms(50);


	}


}

