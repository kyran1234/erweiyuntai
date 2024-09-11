#include "stm32f10x.h" // Device header
#include "Delay.h"

void KEY_INIT(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

uint8_t KEY_GETNUM(void){
	uint8_t KeyNum = 0;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0){
		Delay_ms(15);
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0);
		Delay_ms(15);
		KeyNum = 1;
	}
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0){
		Delay_ms(15);
		while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0);
		Delay_ms(15);
		KeyNum = 2;	
	}
	return 	KeyNum;
}
