#include "stm32f10x.h"                  // Device header
#include "mpu6050_reg.h"
#include "math.h"
#include "Delay.h"

#define MPU6050_ADDRESS 0xD0

/*计算偏移量*/
float i;                                    //计算偏移量时的循环次数
float ax_offset = 0, ay_offset = 0;         //x,y轴的加速度偏移量
float gx_offset = 0, gy_offset = 0;         //x,y轴的角速度偏移量

/*参数*/
float rad2deg = 57.29578;                   //弧度到角度的换算系数
float roll_v = 0, pitch_v = 0;              //换算到x,y轴上的角速度

/*定义微分时间*/
float now = 0, lasttime = 0, dt = 0;        //定义微分时间

/*三个状态，先验状态，观测状态，最优估计状态*/
float gyro_roll = 0, gyro_pitch = 0;        //陀螺仪积分计算出的角度，先验状态
float acc_roll = 0, acc_pitch = 0;          //加速度计观测出的角度，观测状态
float k_roll = 0, k_pitch = 0;              //卡尔曼滤波后估计出最优角度，最优估计状态

/*误差协方差矩阵P*/
float e_P[2][2] ={{1,0},{0,1}};             //误差协方差矩阵，这里的e_P既是先验估计的P，也是最后更新的P

/*卡尔曼增益K*/
float k_k[2][2] ={{0,0},{0,0}};             //这里的卡尔曼增益矩阵K是一个2X2的方阵







void MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;									
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)	
	{
		Timeout --;										
		if (Timeout == 0)								  
		{
			
			break;										
		}
	}
}

void mpu6050_writereg(uint8_t RegAddress, uint8_t Data){
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);//EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//EV6
	
	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);//EV8
	
	I2C_SendData(I2C2, Data);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);//EV8_2
	
	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t mpu6050_readreg(uint8_t RegAddress){
	uint8_t data;
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);//EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);//EV6
	
	I2C_SendData(I2C2, RegAddress);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);//EV8
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);//EV5
	
	I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Receiver);
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);//EV6
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	MPU6050_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);//EV7
	data = I2C_ReceiveData(I2C2);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	return data;
	
}

void mpu6050_init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//区别:软件是通用开漏，不走外设
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 50000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//只有大于100kHZ的快速状态才能使用，否则1：1
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//之后修改有单独的函数
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//选响应地址（7或10）STM32作从机用的
	I2C_InitStructure.I2C_OwnAddress1 = 0x00 ;//STM32作从机用的,方便被呼叫
	
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);
	
	mpu6050_writereg(MPU6050_PWR_MGMT_1, 0X01);
	mpu6050_writereg(MPU6050_PWR_MGMT_2, 0X00);
	mpu6050_writereg(MPU6050_SMPLRT_DIV, 0X09);
	mpu6050_writereg(MPU6050_CONFIG, 0X06);
	mpu6050_writereg(MPU6050_GYRO_CONFIG, 0X18);
	mpu6050_writereg(MPU6050_ACCEL_CONFIG, 0X00);
}

void mpu6050_getdata(int16_t *accx, int16_t *accy, int16_t *accz, int16_t *gyox, int16_t *gyoy, int16_t *gyoz){
	uint8_t dataH;
	uint8_t dataL;
	dataH = mpu6050_readreg(MPU6050_ACCEL_XOUT_H);
	dataL = mpu6050_readreg(MPU6050_ACCEL_XOUT_L);
	*accx = (dataH << 8) | dataL;
	dataH = mpu6050_readreg(MPU6050_ACCEL_YOUT_H);
	dataL = mpu6050_readreg(MPU6050_ACCEL_YOUT_L);
	*accy = (dataH << 8) | dataL;
	dataH = mpu6050_readreg(MPU6050_ACCEL_ZOUT_H);
	dataL = mpu6050_readreg(MPU6050_ACCEL_ZOUT_L);
	*accz = (dataH << 8) | dataL;
	dataH = mpu6050_readreg(MPU6050_GYRO_XOUT_H);
	dataL = mpu6050_readreg(MPU6050_GYRO_XOUT_L);
	*gyox = (dataH << 8) | dataL;
	dataH = mpu6050_readreg(MPU6050_GYRO_YOUT_H);
	dataL = mpu6050_readreg(MPU6050_GYRO_YOUT_L);
	*gyoy = (dataH << 8) | dataL;
	dataH = mpu6050_readreg(MPU6050_GYRO_ZOUT_H);
	dataL = mpu6050_readreg(MPU6050_GYRO_ZOUT_L);
	*gyoz = (dataH << 8) | dataL;
}

uint8_t mpu6050_getid(void){
	return mpu6050_readreg(MPU6050_WHO_AM_I);
}

void setup(void) {//计算偏移量
  for (i = 1; i <= 2000; i++) {
    int16_t ax0,ay0,az0,gx0,gy0,gz0;
	float ax,ay,az,gx,gy,gz;
	//float k_roll ,k_picth;
	//k_roll = *roll;
	//k_picth = *picth;
	
	mpu6050_getdata(&ax0,&ay0,&az0,&gx0,&gy0,&gz0);
	
	
	ax = ax0 / 16384.0f * 9.8f;
	ay = ay0 / 16384.0f * 9.8f;
	az = az0 / 16384.0f * 9.8f;
	gx = gx0 / 16.4f;
	gy = gy0 / 16.4f;
	gz = gz0 / 16.4f;
    ax_offset = ax_offset + ax;//计算x轴加速度的偏移总量
    ay_offset = ay_offset + ay;//计算y轴加速度的偏移总量
    gx_offset = gx_offset + gx;
    gy_offset = gy_offset + gy;
  }
  ax_offset = ax_offset / 2000; //计算x轴加速度的偏移量
  ay_offset = ay_offset / 2000; //计算y轴加速度的偏移量
  gx_offset = gx_offset / 2000; //计算x轴角速度的偏移量
  gy_offset = gy_offset / 2000; //计算y轴角速度的偏移量
  Delay_us(100);
}

void mpu6050_getan(float *pitch,float *roll) {

    /*获取角速度和加速度 */
	int16_t ax0,ay0,az0,gx0,gy0,gz0;
	float ax,ay,az,gx,gy,gz;
	//float k_roll ,k_picth;
	//k_roll = *roll;
	//k_picth = *picth;
	
	mpu6050_getdata(&ax0,&ay0,&az0,&gx0,&gy0,&gz0);
	
	
	ax = ax0 / 16384.0f * 9.8f;
	ay = ay0 / 16384.0f * 9.8f;
	az = az0 / 16384.0f * 9.8f;
	gx = gx0 / 16.4f;
	gy = gy0 / 16.4f;
	gz = gz0 / 16.4f;

    /*step1:计算先验状态*/
    /*计算x,y轴上的角速度*/
    roll_v = (gx-gx_offset) + ((sin(k_pitch)*sin(k_roll))/cos(k_pitch))*(gy-gy_offset) + ((sin(k_pitch)*cos(k_roll))/cos(k_pitch))*gz;//roll轴的角速度
    pitch_v = cos(k_roll)*(gy-gy_offset) - sin(k_roll)*gz;//pitch轴的角速度
    gyro_roll = k_roll + dt*roll_v;//先验roll角度
	gyro_pitch = k_pitch + dt*pitch_v;//先验pitch角度

    /*step2:计算先验误差协方差矩阵P*/
    e_P[0][0] = e_P[0][0] + 0.0025;//这里的Q矩阵是一个对角阵且对角元均为0.0025
    e_P[0][1] = e_P[0][1] + 0;
    e_P[1][0] = e_P[1][0] + 0;
    e_P[1][1] = e_P[1][1] + 0.0025;

    /*step3:更新卡尔曼增益K*/
    k_k[0][0] = e_P[0][0]/(e_P[0][0]+0.3);
    k_k[0][1] = 0;
    k_k[1][0] = 0;
    k_k[1][1] = e_P[1][1]/(e_P[1][1]+0.3);

    /*step4:计算最优估计状态*/
    /*观测状态*/
    //roll角度
    //acc_roll = atan((ay - ay_offset) / (az))*rad2deg;
    //pitch角度
    //acc_pitch = -1*atan((ax - ax_offset) / sqrt(sq(ay - ay_offset) + sq(a.acceleration.z)))*rad2deg;
	float roll_a = atan2(ay - ay_offset, az) / 3.141593f * 180.0f;
	float pitch_a = -atan2(ax, sqrt(az * az + (ay - ay_offset) * (ay - ay_offset))) / 3.141593f * 180.0f;
    /*最优估计状态*/
    k_roll = gyro_roll + k_k[0][0]*(roll_a - gyro_roll);
    k_pitch = gyro_pitch + k_k[1][1]*(pitch_a - gyro_pitch);

    /*step5:更新协方差矩阵P*/
    e_P[0][0] = (1 - k_k[0][0])*e_P[0][0];
    e_P[0][1] = 0;
    e_P[1][0] = 0;
    e_P[1][1] = (1 - k_k[1][1])*e_P[1][1];
	
	*pitch = k_pitch;
	*roll = k_roll;
}

void mpu6050_getangel(float *yaw, float *roll, float *picth){
	int16_t ax0,ay0,az0,gx0,gy0,gz0;
	float ax,ay,az,gx,gy,gz;
	//float k_roll ,k_picth;
	//k_roll = *roll;
	//k_picth = *picth;
	
	mpu6050_getdata(&ax0,&ay0,&az0,&gx0,&gy0,&gz0);
	
	
	ax = ax0 / 16384.0f * 9.8f;
	ay = ay0 / 16384.0f * 9.8f;
	az = az0 / 16384.0f * 9.8f;
	gx = gx0 / 16.4f;
	gy = gy0 / 16.4f;
	gz = gz0 / 16.4f;
	
	float roll_a = atan2(ay, az) / 3.141593f * 180.0f;
	float picth_a = -atan2(ax, sqrt(az * az + ay * ay)) / 3.141593f * 180.0f;
	
	float yaw_g = *yaw + gz * 0.005;
	float roll_g = *roll + gx * 0.005;
	float picth_g = *picth + gy * 0.005;
	
	//float roll_g = (gx) + ((sin(k_picth)*sin(k_roll))/cos(k_picth))*(gy) + ((sin(k_picth)*cos(k_roll))/cos(k_picth))*gz;
	//float picth_g= cos(k_roll)*(gy) - sin(k_roll)*gz;
	//float yaw_g = ( sin(k_roll) / cos(k_picth) ) * gy + (cos(k_roll) / cos(k_picth)) * gz;
	
	const float aphla = 0.98765;
	
	*yaw  = yaw_g;
	*roll = aphla * roll_g + (1 - aphla) * roll_a;
	*picth = aphla * picth_g + (1 - aphla) * picth_a;
}
