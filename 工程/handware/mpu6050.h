#ifndef __MPU6050_H
#define __MPU6050_H

void mpu6050_writereg(uint8_t RegAddress, uint8_t Data);
uint8_t mpu6050_readreg(uint8_t RegAddress);
void mpu6050_init(void);
void mpu6050_getdata(int16_t *accx, int16_t *accy, int16_t *accz, int16_t *gyox, int16_t *gyoy, int16_t *gyoz);
uint8_t mpu6050_getid(void);
void mpu6050_getangel(float *yaw, float *roll, float *picth);
void mpu6050_getan(float *pitch,float *roll);
void setup(void);

#endif
