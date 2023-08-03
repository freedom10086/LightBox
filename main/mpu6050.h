//
// Created by yang on 2023/8/1.
//

#ifndef ESP32C6_HELLO_MPU6050_H
#define ESP32C6_HELLO_MPU6050_H

enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
};

enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

esp_err_t mpu_init();
void mpu_prepare();
void mpu_read_accel_raw(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z);
void mpu_read_gyro_raw(int16_t *g_x, int16_t *g_y, int16_t *g_z);
void mpu_read_accel(float *acc_x, float *acc_y, float *acc_z);
void mpu_read_gyro(float *g_x, float *g_y, float *g_z);
float mpu_read_temperature();
void mpu_calibrate(float *gyro_bias_dest, float *accel_bias_dest);
void mpu_self_test(float *destination);
bool mpu_data_int_ready();
void mpu_reset();
void mpu_deinit();

#endif //ESP32C6_HELLO_MPU6050_H
