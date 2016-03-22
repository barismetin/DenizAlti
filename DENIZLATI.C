/*
	Special notes:
		
		GPIOB pin 6 USART1 Tx
		GPIOB pin 7 USART1 Rx
		GPIOC pin 6  Motor1
		GPIOC pin 7  Motor2
		GPIOC pin 8  Motor3
		GPIOC pin 9  Motor4
		TIM3 is used for this program
		USART1 is used for this program

	-----------------------------------------------------------
	Communication setup
		Baud Rate = 9600
		Data bits = 8
		Stop Bits = 1
		Parity= None
		Flow Control = None
		COM PORT must be set in the Visual Studio C# program
	------------------------------------------------------------
	Communication protocol
		serial number		        Task
		-----------------------------------------
			1000000...............All LEDs on
			1100000...............All LEDs off

*/
//********************** Libraries *******************************
#include <misc.h>		
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_usart.h> 

//********************** Defines ***********************************

#define MAX_STRLEN 3 // This is the maximum string length of our string in characters


//********************** Prototype functions ************************

void setup_Periph(void);
void SysTick_Handler(void);
void USART_puts(USART_TypeDef *USARTx, volatile char *str);
//************* global variables for program ******************

int received_int;
char received_string[MAX_STRLEN+1]; // this will hold the recieved string
int PrescalerValue;
int analogNum;
int butonNum;


//********************** Process type functions ************************
//********************** Print to serial ************************

//Pass in any USART; 1,2,3 and so on. the pass in string
void USART_puts(USART_TypeDef *USARTx, volatile char *str)
{
	while(*str)
	{
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx, *str);
		*str++;
	}
}

//********************** STM32F4 peripheral setup ************************
void setup_Periph(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; //Port initialization
	USART_InitTypeDef  USART_InitStructure; //USART initialization
	NVIC_InitTypeDef  NVIC_InitStructure; //Interrupt initialization
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //Timer initialization 
  TIM_OCInitTypeDef  TIM_OCInitStructure; //Pwm initialization
	
	
	//Enable the GPIOB clock for USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	//Enable the GPIOB clock for serial communication
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//Enable the GPIOD clock for LEDs
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	//Enable the GPIOC clock for MOTORs
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	// Enable the TIM4 clock for PWM
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//Setup for GPIOD pins for LEDS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//Setup for GPIOC pin for pwm 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

	//Assign GPIOC pins to TIM3 alternate functions 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);/* TIM3 CH1 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);/* TIM3 CH2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);/* TIM3 CH3 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);/* TIM3 CH4 */
	
	
  // Compute the prescaler value 
  PrescalerValue = (int) ((SystemCoreClock /2) / 200000) - 1;
	
  // Time base configuration 
  TIM_TimeBaseStructure.TIM_Period = 1000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	// PWM1 Mode configuration 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;

  // PWM1 Mode configuration: Channel1  
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
 
  // PWM1 Mode configuration: Channel2 
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  // PWM1 Mode configuration: Channel3  
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  // PWM1 Mode configuration: Channel4 
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM3, ENABLE);

  // TIM3 enable counter  
  TIM_Cmd(TIM3, ENABLE);
    
	
	//Setup for GPIOB pins for serial communication
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Assign GPIOB pins to USART1 alternate functions
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);/* USART1_TX */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);/* USART1_RX */

	//serial communication controls
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

	//USART1_IRQHandler() interrupt and configure
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Finally enable the USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}



int main(void) {
  
  setup_Periph();

  USART_puts(USART1, "Bismillahirrahmanirrahim\r\n"); // just send a message to indicate that it works

  while (1)
	{  
		butonNum = (received_int)/10;
		analogNum =(received_int)%10;
					
			  switch(butonNum) //Check for what number was passed
				{
				 
				case 1:// All LEDs on
						GPIO_SetBits(GPIOD  , GPIO_Pin_13);
						 
				break;
				
				case 2:// All LEDs off
					GPIO_SetBits(GPIOD  , GPIO_Pin_12);
					 
				break;

				case 3:// All LEDs off
					GPIO_SetBits(GPIOD  , GPIO_Pin_14);
				 
				break;
				case 4:// All LEDs off
					 
				break;
				case 5:// All LEDs off
					 
				break;
				case 6:// All LEDs off
					  
				break;
				case 7:// All LEDs off
					 
				break;
				case 8:// All LEDs off
					 
				break;
				case 9:// All LEDs off
				 
				break;
				 case 10:// All LEDs off
					 
				break;
				case 11:// All LEDs off
		 			 
				break;
				case 12:// All LEDs off
				 
				break;
				case 13:// All LEDs off
				 
					TIM_SetCompare1(TIM3,10*analogNum);
				break;
				case 14:// All LEDs off
 
					TIM_SetCompare2(TIM3,10*analogNum);
				break;
		    case 15:// All LEDs off
 
					TIM_SetCompare3(TIM3,10*analogNum);
				break;
						
				//TIM_SetCompare2(TIM3,Pulse2);
				}
	
  }
	
  }

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){
	
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		
		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t
		
		/* check if the received character is not the LF character (used to determine end of string) 
		 * or the if the maximum string length has been been reached 
		 */
		if( (t != '\n') && (cnt < MAX_STRLEN) ){ 
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter 
			cnt = 0;
	 	 received_int=atoi(received_string);
		}
	}
}
