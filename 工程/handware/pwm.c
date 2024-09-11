#include "stm32f10x.h"                  // Device header

void pwm_init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 20000- 1;//公式里把一加回来了
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//重复计时器，高级才有，不用置0
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//因为这里下面不像把一些暂时用不到的结构体成员赋值，但是在换用高级定时器时可能由于忘记添加成员赋值而出错，所以这里先对所有成员初始化，再修改需要的变量
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = ENABLE;
	TIM_OCInitStructure.TIM_Pulse = 0;//设置CRR寄存器的值
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_Pulse = 0;//设置CRR寄存器的值
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}

void pwm_setcompare2(uint16_t compare){
	TIM_SetCompare2(TIM2, compare);
}

void pwm_setcompare1(uint16_t compare){
	TIM_SetCompare1(TIM2, compare);
}
