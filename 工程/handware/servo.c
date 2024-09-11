#include "stm32f10x.h"                  // Device header
#include "pwm.h"

void servo_init(void){
	pwm_init();
}

void servo_setangle2(float angle){
	pwm_setcompare2(angle /180 * 2000 + 500);
}

void servo_setangle1(float angle){
	pwm_setcompare1(angle /180 * 2000 + 500);
}
