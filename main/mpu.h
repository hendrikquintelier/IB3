#ifndef MPU_H
#define MPU_H

#include <stdbool.h>

#define SDA_PIN 42
#define SCL_PIN 21
#define I2C_MASTER_NUM I2C_NUM_0
#define MPU6050_ADDR 0x68

typedef struct
{
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float roll, pitch, yaw;
} mpu6050_data_t;

void i2c_master_init();
bool mpu6050_init();
void calculate_IMU_error();
void mpu6050_read_data(mpu6050_data_t *mpuData, float elapsedTime);
void mpu_main();

#endif // MPU_H
