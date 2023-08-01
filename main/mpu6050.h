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

void mpu_init();

void read_accel(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z);
void read_gyro(int16_t *g_x, int16_t *g_y, int16_t *g_z);
int16_t mpu_read_temperature();

void mpu_calibrate(float *dest1, float *dest2);
void mpu_self_test(float *destination);

void mpu_reset();
void mpu_deinit();

#endif //ESP32C6_HELLO_MPU6050_H
