/*
 * Utility.h
 *
 *  Created on: 25 de abr de 2020
 *  Author: Fagner de Araujo Pereira
 */

#ifndef UTILITY_H_
#define UTILITY_H_

//Declarações de funções
void Utility_Init(void);			//inicialização de funções dessa biblioteca
void Configure_Clock(void);			//configuração do sistema de clock
void TIM2_Setup(void);				//ajusta o Timer2 para geração de base de tempo para delays
void Delay_Start(void);				//inicializa as funções de delay
void Delay_ms(uint32_t delay);		//delay em ms
void Delay_us(uint32_t delay);		//delay em us
void USART1_Init(void);				//configuração e inicialização da USART1
void LCD_Init();                  	//inicialização do LCD com interface de 4 bits
void LCD_Clear();                 	//limpa o LCD
void LCD_SL();                    	//deslocamento à esquerda do LCD
void LCD_SR();                    	//deslocamento à direita do LCD
void LCD_CMD(unsigned char);      	//envia comando para o LCD
void LCD_DATA(unsigned char);     	//envia um dado de 4 bits para o LCD
void LCD_Set_Cursor(unsigned char, unsigned char);  //configura a posição do cursor
void LCD_Write_Char(char);        	//escreve um caracter no LCD na posição do cursor
void LCD_Write_String(char*);     	//escreve uma string no LCD a partir da posição do cursor

//Inicialização de funções dessa biblioteca
void Utility_Init(void)
{
	Configure_Clock();
	Delay_Start();
}

//Configuração do sistema de clock para velocidade máxima
//Cristal externo de 8MHz e HCLK de 168 MHz
void Configure_Clock(void)
{
	//Parâmetros do PLL principal
	#define PLL_M	4
	#define PLL_N	168
	#define PLL_P	2
	#define PLL_Q	7

	//Reseta os registradores do módulo RCC para o estado inicial
	RCC->CIR = 0;				//desabilita todas as interrupções de RCC
	RCC->CR |= RCC_CR_HSION;	//liga o oscilador HSI
	RCC->CFGR = 0;				//reseta o registrador CFGR
	//Desliga HSE, CSS e o PLL e o bypass de HSE
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON |
			   RCC_CR_PLLON | RCC_CR_HSEBYP);
	RCC->PLLCFGR = 0x24003010;	//reseta o registrador PLLCFGR

	//Configura a fonte de clock (HSE), os parâmetros do PLL,
	//prescalers dos barramentos AHB, APB
	RCC->CR |= RCC_CR_HSEON;				//habilita HSE
	while(!((RCC->CR) & RCC_CR_HSERDY));	//espera HSE ficar pronto
    RCC->CFGR |= 0x9400;	//HCLK = SYSCLK/1, PCLK2 = HCLK/2, PCLK1 = HCLK/4

    //Configura a fonte de clock e os parâmetros do PLL principal
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (0x400000)           | (PLL_Q << 24);

    RCC->CR |= RCC_CR_PLLON;			//habilita o PLL
    while(!(RCC->CR & RCC_CR_PLLRDY));	//espera o PLL ficar pronto verificando a flag PLLRDY

    //Seleciona o PLL como fonte de SYSCLK e espera o PLL ser a fonte de SYSCLK
    RCC->CFGR |= 0x2;
    while((RCC->CFGR & 0xC) != 0x8);
}


//O timer 2 é o primeiro timer de uso geral de 32 bits.
//A configuração abaixo usa o timer 2 como base para permitir o uso de delays em us e ms.
void Delay_Start(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;	//liga o clock do Timer2
	TIM2->CR1 &= ~TIM_CR1_DIR;			//contador em modo de contagem crescente
	TIM2->PSC = 83;						//prescaler para pulsos a cada 1uS
	TIM2->EGR = TIM_EGR_UG;				//update event para escrever o valor do prescaler
	TIM2->CR1 |= TIM_CR1_CEN;			//habilita o timer
	//DBGMCU->APB1FZ |= 1;				//permite vizualizar o valor do contador no modo debug
}

//Criação de delay em us
void Delay_us(uint32_t delay)
{
	TIM2->CNT = 0;				//inicializa o contador com 0
	while(TIM2->CNT < delay);	//aguarda o tempo passar
}

//Criação de delay em ms
void Delay_ms(uint32_t delay)
{
	uint32_t max = 1000*delay;
	TIM2->CNT = 0;				//inicializa o contador com 0
	while(TIM2->CNT < max);		//aguarda o tempo passar
}

//Configuração básica da USART1
void USART1_Init(void)
{
	//configuração da USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;				//habilita o clock da USART1
	USART1->BRR = 84;									//ajusta baud rate para 1 Mbps (frequência de clock de 84MHz)
	//O estado default do registrador USART1->CR1 garante:
	//1 stop bit, 8 bits de dados, sem bit de paridade,
	//oversampling de 16 amostras em cada bit
	USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE |		//habilita o trasmissor e o receptor
					USART_CR1_RXNEIE | USART_CR1_UE);	//habilita interrupção de RX e a USART1

	//Habilita a interrupção da USART1 no NVIC
	NVIC_SetPriority(USART1_IRQn, 0);		//seta a prioridade da USART1
	NVIC_EnableIRQ(USART1_IRQn);			//habilita a interrupção da USART1

	//Configuração dos pinos PA9 (TX) e PA10(RX)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			//habilita o clock do GPIOA
	GPIOA->MODER |= (0b10 << 20) | (0b10 << 18) ;	//pinos PA10 e PA9 como função alternativa
	GPIOA->AFR[1] |= (0b0111 << 8) | (0b0111 << 4);	//função alternativa 7 (USART1)
}


//Redefinição da função de envio de dados pela USART1
int __io_putchar(int ch)
{
	USART1->DR = (ch & (uint16_t)0x01FF);	//escreve o dado a ser transmitido
	while (!(USART1->SR & USART_SR_TXE));	//espera pelo fim da transmissão
	return ch;
}

//Redefinição da função de recebimento de dados pela USART1
int __io_getchar(void)
{
   return (uint16_t)(USART1->DR & (uint16_t)0x01FF);	//lê o dado recebido
}

//ISR da USART1. Todas as ISR's estão definidas no arquivo startup_stm32.s
void USART1_IRQHandler(void)
{
	__io_putchar(__io_getchar());	//lê o dado e reenvia pela USART1
}



//Funções para manipulação de LCDs
//Criação de estrutura de configuração dos pinos do LCD
typedef struct
{
	GPIO_TypeDef *LCD_GPIO;
	uint16_t D4_PIN;
	uint16_t D5_PIN;
	uint16_t D6_PIN;
	uint16_t D7_PIN;
	uint16_t EN_PIN;
	uint16_t RS_PIN;
	uint16_t LCD_EN_Delay;
}LCD_CfgType;
//inicialização da estrutura de configuração dos pinos do LCD
const LCD_CfgType LCD_CfgParam =
{
	GPIOC,
	GPIO_Pin_2,
	GPIO_Pin_3,
	GPIO_Pin_4,
	GPIO_Pin_5,
	GPIO_Pin_1,
	GPIO_Pin_0,
	20
};

//Escreve um dado de 4 bits no barramento de dados do LCD
void LCD_DATA(unsigned char Data)
{
	if(Data & 1)
    	LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.D4_PIN;
    else
    	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D4_PIN;

    if(Data & 2)
    	LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.D5_PIN;
    else
    	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D5_PIN;

    if(Data & 4)
    	LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.D6_PIN;
    else
    	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D6_PIN;

    if(Data & 8)
    	LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.D7_PIN;
    else
    	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D7_PIN;
}

//Envia um comando para o LCD
void LCD_CMD(unsigned char a_CMD)
{
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.RS_PIN;	//seleciona o registrador de comando
    LCD_DATA(a_CMD);									//escreve o comando no barramento

    LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.EN_PIN;	//envia o sinal de clock (bit EN)
    Delay_us(LCD_CfgParam.LCD_EN_Delay);
    LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.EN_PIN;
}

//Limpa o LCD
void LCD_Clear()
{
    LCD_CMD(0);
    LCD_CMD(1);
    Delay_ms(2);
}

//Configura a posição do cursor
void LCD_Set_Cursor(unsigned char linha, unsigned char coluna)
{
    unsigned char Temp,Low4,High4;
    if(linha == 1)
    {
    	Temp  = 0x80 + coluna - 1; //0x80 é o endereço da primeira posição da linha 1
    	High4 = Temp >> 4;
    	Low4  = Temp & 0x0F;
    	LCD_CMD(High4);
    	LCD_CMD(Low4);
    }

    if(linha == 2)
    {
    	Temp  = 0xC0 + coluna - 1;	//0xC0 é o endereço da primeira posição da linha 2
    	High4 = Temp >> 4;
    	Low4  = Temp & 0x0F;
    	LCD_CMD(High4);
    	LCD_CMD(Low4);
    }

    if(linha == 3)
    {
    	Temp  = 0x94 + coluna - 1;	//0x94 é o endereço da primeira posição da linha 3
    	High4 = Temp >> 4;
    	Low4  = Temp & 0x0F;
    	LCD_CMD(High4);
    	LCD_CMD(Low4);
    }

    if(linha == 4)
    {
    	Temp  = 0xD4 + coluna - 1;	//0xD4 é o endereço da primeira posição da linha 4
    	High4 = Temp >> 4;
    	Low4  = Temp & 0x0F;
    	LCD_CMD(High4);
    	LCD_CMD(Low4);
    }

    Delay_ms(2);	//aguarda o comando ser executado
}

//Inicializa o LCD
void LCD_Init()
{
	Delay_Start();	//inicializa as funções de Delay
	Delay_ms(50);	//aguarda 50ms para se comunicar com o LCD

    //Configuração dos pinos
	//Habilitação do clock da porta
	if(LCD_CfgParam.LCD_GPIO == GPIOA)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	else if(LCD_CfgParam.LCD_GPIO == GPIOB)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	else if(LCD_CfgParam.LCD_GPIO == GPIOC)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	else if(LCD_CfgParam.LCD_GPIO == GPIOD)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	else if(LCD_CfgParam.LCD_GPIO == GPIOE)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	//Reset dos pinos da porta
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D4_PIN;
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D5_PIN;
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D6_PIN;
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.D7_PIN;
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.RS_PIN;
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.EN_PIN;
	//Configuração de saída dos pinos da porta
	GPIO_InitTypeDef GPIO_InitDef;
	GPIO_InitDef.GPIO_Pin = LCD_CfgParam.D4_PIN | LCD_CfgParam.D5_PIN |
							LCD_CfgParam.D6_PIN | LCD_CfgParam.D7_PIN |
							LCD_CfgParam.RS_PIN | LCD_CfgParam.EN_PIN;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(LCD_CfgParam.LCD_GPIO, &GPIO_InitDef);

	//Processo de inicialização conforme descrito no datasheet
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.RS_PIN;
	LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.EN_PIN;
	LCD_DATA(0x00);
    Delay_ms(2);
    LCD_CMD(0x03);
    Delay_ms(2);
    LCD_CMD(0x03);
    Delay_ms(2);
    LCD_CMD(0x03);
    LCD_CMD(0x02);
    LCD_CMD(0x02);
    LCD_CMD(0x08);
    LCD_CMD(0x00);
    LCD_CMD(0x0C);
    LCD_CMD(0x00);
    LCD_CMD(0x06);
    LCD_CMD(0x00);
    LCD_CMD(0x01);
    Delay_ms(2);
}

//Escreve um caractere no LCD
void LCD_Write_Char(char Data)
{
   char Low4,High4;
   Low4  = Data & 0x0F;
   High4 = (Data & 0xF0) >> 4;

   LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.RS_PIN;

   LCD_DATA(High4);
   LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.EN_PIN;
   Delay_us(LCD_CfgParam.LCD_EN_Delay);
   LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.EN_PIN;

   LCD_DATA(Low4);
   LCD_CfgParam.LCD_GPIO->ODR |= LCD_CfgParam.EN_PIN;
   Delay_us(LCD_CfgParam.LCD_EN_Delay);
   LCD_CfgParam.LCD_GPIO->ODR &= ~LCD_CfgParam.EN_PIN;

   Delay_us(45);
}

//Escreve uma string no LCD
void LCD_Write_String(char *str)
{
    int i;
    for(i=0;str[i]!='\0';i++)
       LCD_Write_Char(str[i]);
}

//Deslocamento do texto à esquerda
void LCD_SL()
{
    LCD_CMD(0x01);
    LCD_CMD(0x08);
    Delay_us(45);
}

//Deslocamento do texto à direita
void LCD_SR()
{
    LCD_CMD(0x01);
    LCD_CMD(0x0C);
    Delay_us(45);
}

#endif /* UTILITY_H_ */
