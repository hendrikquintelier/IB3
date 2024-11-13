#include "mpu.h"
#include <math.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MPU6050";

// Calibration variables
float AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
#define GYRO_THRESHOLD 3 // Threshold to consider as stationary

#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number
#define I2C_MASTER_SCL_IO 21      // I2C SCL pin
#define I2C_MASTER_SDA_IO 42      // I2C SDA pin
#define I2C_MASTER_FREQ_HZ 100000 // I2C frequency

// I2C initialization
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ};
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// MPU6050 initialization
bool mpu6050_init()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // Power management register
    i2c_master_write_byte(cmd, 0x00, true); // Wake up MPU6050
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
    }
    return ret == ESP_OK;
}

// Calculate IMU error
void calculate_IMU_error()
{
    mpu6050_data_t tempData;
    for (int i = 0; i < 200; i++)
    {
        mpu6050_read_data(&tempData, 0);
        AccErrorX += atan(tempData.accY / sqrt(pow(tempData.accX, 2) + pow(tempData.accZ, 2))) * 180 / M_PI;
        AccErrorY += atan(-1 * tempData.accX / sqrt(pow(tempData.accY, 2) + pow(tempData.accZ, 2))) * 180 / M_PI;
        GyroErrorX += tempData.gyroX;
        GyroErrorY += tempData.gyroY;
        GyroErrorZ += tempData.gyroZ;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    AccErrorX /= 200;
    AccErrorY /= 200;
    GyroErrorX /= 200;
    GyroErrorY /= 200;
    GyroErrorZ /= 200;
}

// Read data from MPU6050 and compute roll, pitch, yaw
void mpu6050_read_data(mpu6050_data_t *mpuData, float elapsedTime)
{
    uint8_t data[14];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // Starting register for accelerometer data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Convert raw data
    mpuData->accX = (float)((int16_t)(data[0] << 8 | data[1])) / 16384.0;
    mpuData->accY = (float)((int16_t)(data[2] << 8 | data[3])) / 16384.0;
    mpuData->accZ = (float)((int16_t)(data[4] << 8 | data[5])) / 16384.0;

    mpuData->gyroX = (float)((int16_t)(data[8] << 8 | data[9])) / 131.0 - GyroErrorX;
    mpuData->gyroY = (float)((int16_t)(data[10] << 8 | data[11])) / 131.0 - GyroErrorY;
    mpuData->gyroZ = (float)((int16_t)(data[12] << 8 | data[13])) / 131.0 - GyroErrorZ;

    // Check if the device is stationary by comparing gyro data to the threshold
    bool isStationary = fabs(mpuData->gyroX) < GYRO_THRESHOLD &&
                        fabs(mpuData->gyroY) < GYRO_THRESHOLD &&
                        fabs(mpuData->gyroZ) < GYRO_THRESHOLD;

    // Angle calculations
    float accAngleX = atan(mpuData->accY / sqrt(pow(mpuData->accX, 2) + pow(mpuData->accZ, 2))) * 180 / M_PI;
    float accAngleY = atan(-1 * mpuData->accX / sqrt(pow(mpuData->accY, 2) + pow(mpuData->accZ, 2))) * 180 / M_PI;

    if (isStationary)
    {
    }
    else
    {
        // If moving, apply complementary filter with accelerometer correction
        mpuData->roll = (0.9999 * (mpuData->roll + mpuData->gyroX * elapsedTime)) + (0.0001 * accAngleX);
        mpuData->pitch = (0.9999 * (mpuData->pitch + mpuData->gyroY * elapsedTime)) + (0.0001 * accAngleY);
        mpuData->yaw += mpuData->gyroZ * elapsedTime; // Gyroscope only for yaw
    }
}

// Main function to continuously read and log MPU6050 data
void mpu_main()
{
    i2c_master_init();
    // Initialize MPU6050
    if (!mpu6050_init())
    {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return;
    }

    // Calibrate the IMU
    calculate_IMU_error();

    mpu6050_data_t mpuData = {0}; // Initialize roll, pitch, yaw to 0
    float previousTime = (float)esp_timer_get_time() / 1000000.0;

    while (1)
    {
        float currentTime = (float)esp_timer_get_time() / 1000000.0;
        float elapsedTime = currentTime - previousTime;
        previousTime = currentTime;

        mpu6050_read_data(&mpuData, elapsedTime);

        ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", mpuData.roll, mpuData.pitch, mpuData.yaw);

        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}
