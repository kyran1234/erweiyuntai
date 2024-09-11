#include "stm32f10x.h"  // Device header
#include "OLED.h"
#include "Delay.h"
#include "servo.h"
#include "mpu6050.h"


float angle;
float yaw ,roll, pitch;

int main(void){
	OLED_Init();		//OLED初始化
	servo_init();		//舵机初始化
	mpu6050_init();
	OLED_ShowString(1,1,"ID:");
	uint8_t id = mpu6050_getid();
	OLED_ShowHexNum(1,4,id,2);
	servo_setangle1(60);		//设置舵机的角度为角度变量
	servo_setangle2(90);
	setup();
	
	
	
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "Angle:");
	
	
	while(1){
		mpu6050_getangel(&yaw, &roll, &pitch);
		mpu6050_getan(&pitch,&roll);
	

		
		OLED_ShowString(2,1,"YAW:");
		OLED_ShowString(3,1,"ROLL:");
		OLED_ShowString(4,1,"PICTH:");
		OLED_ShowSignedNum(2,5,yaw,5);
		OLED_ShowSignedNum(3,6,roll,5);
		OLED_ShowSignedNum(4,7,pitch,5);
		servo_setangle1(75 + pitch);
		servo_setangle2(90 - roll);
		
	
		
		
		OLED_ShowNum(1, 7, angle, 3);	//OLED显示角度变量
		
	} 
	
}

