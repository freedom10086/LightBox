#include "driver/i2c.h"
#include "esp_event.h"
#include "esp_log.h"
#include "math.h"

#include "mpu6050.h"
#include "io_config.h"
#include "my_common.h"

#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       200

#define MPU_ADDR                    0b1101000

#define MPU_REG_ADDR_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define MPU_REG_ADDR_XA_OFFSET_L_TC   0x07
#define MPU_REG_ADDR_YA_OFFSET_H      0x08
#define MPU_REG_ADDR_YA_OFFSET_L_TC   0x09
#define MPU_REG_ADDR_ZA_OFFSET_H      0x0A
#define MPU_REG_ADDR_ZA_OFFSET_L_TC   0x0B

#define MPU_REG_ADDR_SELF_TEST_X        0x0D
#define MPU_REG_ADDR_SELF_TEST_Y        0x0E
#define MPU_REG_ADDR_SELF_TEST_Z        0x0F
#define MPU_REG_ADDR_SELF_TEST_A        0x10

#define MPU_REG_ADDR_XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define MPU_REG_ADDR_XG_OFFS_USRL     0x14
#define MPU_REG_ADDR_YG_OFFS_USRH     0x15
#define MPU_REG_ADDR_YG_OFFS_USRL     0x16
#define MPU_REG_ADDR_ZG_OFFS_USRH     0x17
#define MPU_REG_ADDR_ZG_OFFS_USRL     0x18

#define MPU_REG_ADDR_SMPLRT_DIV         0x19
#define MPU_REG_ADDR_CONFIG             0x1A
// [4:3] 0:± 250 °/s 1:± 500 °/s 2:± 1000 °/s 3:± 2000 °/s
#define MPU_REG_ADDR_GYRO_CONFIG        0x1B
// [4:3] 0:± 2g 1:± 4g 2:± 8g 3:± 16g
#define MPU_REG_ADDR_ACCEL_CONFIG       0x1C
#define MPU_REG_ADDR_FIFO_EN            0x23
#define MPU_REG_ADDR_I2C_MST_CTRL            0x24
#define MPU_REG_ADDR_I2C_MST_STATUS            0x36
#define MPU_REG_ADDR_INT_PIN_CFG            0x37
//[0] data ready en [4] fifo overflow en
#define MPU_REG_ADDR_INT_ENABLE            0x38
//[0] data ready [4] fifo overflow
#define MPU_REG_ADDR_INT_STATUS            0x3A

#define MPU_REG_ADDR_ACCEL_XOUT_H            0x3B
#define MPU_REG_ADDR_ACCEL_XOUT_L            0x3C
#define MPU_REG_ADDR_ACCEL_YOUT_H            0x3D
#define MPU_REG_ADDR_ACCEL_YOUT_L            0x3E
#define MPU_REG_ADDR_ACCEL_ZOUT_H            0x3F
#define MPU_REG_ADDR_ACCEL_ZOUT_L            0x40

#define MPU_REG_ADDR_TEMP_OUT_H            0x41
#define MPU_REG_ADDR_TEMP_OUT_L            0x42

#define MPU_REG_ADDR_GYRO_XOUT_H            0x43
#define MPU_REG_ADDR_GYRO_XOUT_L            0x44
#define MPU_REG_ADDR_GYRO_YOUT_H            0x45
#define MPU_REG_ADDR_GYRO_YOUT_L            0x46
#define MPU_REG_ADDR_GYRO_ZOUT_H            0x47
#define MPU_REG_ADDR_GYRO_ZOUT_L            0x48

#define MPU_REG_ADDR_SIGNAL_PATH_RESET            0x68
#define MPU_REG_ADDR_USER_CTRL            0x6A
#define MPU_REG_ADDR_PWR_MGMT_1            0x6B
#define MPU_REG_ADDR_PWR_MGMT_2            0x6C
#define MPU_REG_ADDR_FIFO_COUNTH            0x72
#define MPU_REG_ADDR_FIFO_COUNTL            0x73
#define MPU_REG_ADDR_FIFO_RW                0x74
#define MPU_REG_ADDR_WHO_AM_I             0x75

static const char *TAG = "MPU-6050";

ESP_EVENT_DEFINE_BASE(MPU_SENSOR_EVENT);

// x y z
uint8_t accel_buf[6];
uint8_t temp_buf[2];
uint8_t gyro_buf[6];

int gyro_scale = GFS_250DPS;
int accel_scale = AFS_2G;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = GPIO_SDA,
            .scl_io_num = GPIO_SCL,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);
}

static esp_err_t i2c_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    uint8_t u8_reg_addr[] = {reg_addr};
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU_ADDR,
                                                 u8_reg_addr, sizeof(u8_reg_addr), data, len,
                                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read i2c failed, for dev:%x addr:%x,  %s", MPU_ADDR, reg_addr, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t i2c_write_data(uint8_t reg_addr, uint8_t data) {
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU_ADDR,
                                     write_buf,
                                     sizeof(write_buf),
                                     I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "write i2c cmd failed, for addr %x,  %s", reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

static uint8_t crc8(const uint8_t *data, uint8_t len) {
    const uint8_t POLYNOMIAL = 0x31;
    uint8_t crc = 0xFF;

    for (int j = len; j; --j) {
        crc ^= *data++;

        for (int i = 8; i; --i) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

static float get_gyro_res() {
    switch (gyro_scale) {
        case GFS_250DPS:
            return 250.0 / 32768.0;
            break;
        case GFS_500DPS:
            return 500.0 / 32768.0;
            break;
        case GFS_1000DPS:
            return 1000.0 / 32768.0;
            break;
        case GFS_2000DPS:
            return 2000.0 / 32768.0;
            break;
    }

    assert(0);
}

static float get_accel_res() {
    switch (accel_scale) {
        case AFS_2G:
            return 2.0 / 32768.0;
            break;
        case AFS_4G:
            return 4.0 / 32768.0;
            break;
        case AFS_8G:
            return 8.0 / 32768.0;
            break;
        case AFS_16G:
            return 16.0 / 32768.0;
            break;
    }
    assert(0);
}

void mpu_read_accel_raw(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z) {
    i2c_read(MPU_REG_ADDR_ACCEL_XOUT_H, accel_buf, 6);
    *acc_x = (int16_t) ((accel_buf[0] << 8) | accel_buf[1]);
    *acc_y = (int16_t) ((accel_buf[2] << 8) | accel_buf[3]);
    *acc_z = (int16_t) ((accel_buf[4] << 8) | accel_buf[5]);
}

void mpu_read_gyro_raw(int16_t *g_x, int16_t *g_y, int16_t *g_z) {
    i2c_read(MPU_REG_ADDR_GYRO_XOUT_H, gyro_buf, 6);
    *g_x = (int16_t) ((gyro_buf[0] << 8) | gyro_buf[1]);
    *g_y = (int16_t) ((gyro_buf[2] << 8) | gyro_buf[3]);
    *g_z = (int16_t) ((gyro_buf[4] << 8) | gyro_buf[5]);
}

void mpu_read_accel(float *acc_x, float *acc_y, float *acc_z) {
    int16_t acc_x_raw, acc_y_raw, acc_z_raw;
    mpu_read_accel_raw(&acc_x_raw, &acc_y_raw, &acc_z_raw);
    float acc_res = get_accel_res();
    *acc_x = (float)acc_x_raw * acc_res;
    *acc_y = (float)acc_y_raw * acc_res;
    *acc_z = (float)acc_z_raw * acc_res;
}

void mpu_read_gyro(float *g_x, float *g_y, float *g_z) {
    int16_t g_x_raw, g_y_raw, g_z_raw;
    mpu_read_gyro_raw(&g_x_raw, &g_y_raw, &g_z_raw);
    float g_res = get_gyro_res();
    *g_x = (float)g_x_raw * g_res;
    *g_y = (float)g_y_raw * g_res;
    *g_z = (float)g_z_raw * g_res;
}

float mpu_read_temperature() {
    i2c_read(MPU_REG_ADDR_TEMP_OUT_H, temp_buf, 2);
    int16_t temp_raw =  ((int16_t) temp_buf[0]) << 8 | temp_buf[1];
    // ESP_LOGI(TAG, "temp[0] %d temp[1] %d temp %d", temp_buf[0], temp_buf[1], temp_raw);
    return ((float) (temp_raw / 340)) + 36.53f;
}

void mpu_calibrate(float *gyro_bias_dest, float *accel_bias_dest) {
    ESP_LOGI(TAG, "mpu start calibrate");

    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    mpu_reset();
    vTaskDelay(pdMS_TO_TICKS(100));

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    i2c_write_data(MPU_REG_ADDR_PWR_MGMT_1, 0x01);
    i2c_write_data(MPU_REG_ADDR_PWR_MGMT_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Configure device for bias calculation
    i2c_write_data(MPU_REG_ADDR_INT_ENABLE, 0x00);   // Disable all interrupts
    i2c_write_data(MPU_REG_ADDR_FIFO_EN, 0x00);      // Disable FIFO
    i2c_write_data(MPU_REG_ADDR_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    i2c_write_data(MPU_REG_ADDR_I2C_MST_CTRL, 0x00); // Disable I2C master
    i2c_write_data(MPU_REG_ADDR_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    i2c_write_data(MPU_REG_ADDR_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    vTaskDelay(pdMS_TO_TICKS(15));

    // Configure MPU6050 gyro and accelerometer for bias calculation
    i2c_write_data(MPU_REG_ADDR_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    i2c_write_data(MPU_REG_ADDR_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    i2c_write_data(MPU_REG_ADDR_GYRO_CONFIG,
                   0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    i2c_write_data(MPU_REG_ADDR_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    i2c_write_data(MPU_REG_ADDR_USER_CTRL, 0x40);   // Enable FIFO
    i2c_write_data(MPU_REG_ADDR_FIFO_EN,
                   0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    // accumulate 80 samples in 80 milliseconds = 960 bytes
    vTaskDelay(pdMS_TO_TICKS(80));

    // At end of sample accumulation, turn off FIFO sensor read
    i2c_write_data(MPU_REG_ADDR_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    i2c_read(MPU_REG_ADDR_FIFO_COUNTH, &data[0], 2); // read FIFO sample count
    fifo_count = ((uint16_t) data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        i2c_read(MPU_REG_ADDR_FIFO_RW, &data[0], 12); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]);
        // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0] += (int32_t) gyro_temp[0];
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0] /= (int32_t) packet_count;
    gyro_bias[1] /= (int32_t) packet_count;
    gyro_bias[2] /= (int32_t) packet_count;

    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
    } else {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) &
              0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    i2c_write_data(MPU_REG_ADDR_XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
    i2c_write_data(MPU_REG_ADDR_XG_OFFS_USRL, data[1]);
    i2c_write_data(MPU_REG_ADDR_YG_OFFS_USRH, data[2]);
    i2c_write_data(MPU_REG_ADDR_YG_OFFS_USRL, data[3]);
    i2c_write_data(MPU_REG_ADDR_ZG_OFFS_USRH, data[4]);
    i2c_write_data(MPU_REG_ADDR_ZG_OFFS_USRL, data[5]);

    // construct gyro bias in deg/s for later manual subtraction
    gyro_bias_dest[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
    gyro_bias_dest[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
    gyro_bias_dest[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    i2c_read(MPU_REG_ADDR_XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];
    i2c_read(MPU_REG_ADDR_YA_OFFSET_H, &data[0], 2);
    accel_bias_reg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];
    i2c_read(MPU_REG_ADDR_ZA_OFFSET_H, &data[0], 2);
    accel_bias_reg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
        if (accel_bias_reg[ii] & mask)
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] /
                          8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] |
              mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] |
              mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2];
    // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    i2c_write_data(MPU_REG_ADDR_XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    i2c_write_data(MPU_REG_ADDR_XA_OFFSET_L_TC, data[1]);
    i2c_write_data(MPU_REG_ADDR_YA_OFFSET_H, data[2]);
    i2c_write_data(MPU_REG_ADDR_YA_OFFSET_L_TC, data[3]);
    i2c_write_data(MPU_REG_ADDR_ZA_OFFSET_H, data[4]);
    i2c_write_data(MPU_REG_ADDR_ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    accel_bias_dest[0] = (float) accel_bias[0] / (float) accelsensitivity;
    accel_bias_dest[1] = (float) accel_bias[1] / (float) accelsensitivity;
    accel_bias_dest[2] = (float) accel_bias[2] / (float) accelsensitivity;

    ESP_LOGI(TAG, "mpu calibrate done. gyro bias: x:%.2f y:%.2f z:%.2f accel bias: x:%.2f y:%.2f z:%.2f",
             gyro_bias_dest[0], gyro_bias_dest[1], gyro_bias_dest[2],
             accel_bias_dest[0], accel_bias_dest[1], accel_bias_dest[2]);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void mpu_self_test(float *destination) {
    ESP_LOGI(TAG, "mpu start self test");

    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    i2c_write_data(MPU_REG_ADDR_ACCEL_CONFIG,
                   0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    i2c_write_data(MPU_REG_ADDR_GYRO_CONFIG,
                   0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    vTaskDelay(pdMS_TO_TICKS(250));  // Delay a while to let the device execute the self-test
    i2c_read(MPU_REG_ADDR_SELF_TEST_X, &rawData[0], 1); // X-axis self-test results
    i2c_read(MPU_REG_ADDR_SELF_TEST_Y, &rawData[1], 1); // Y-axis self-test results
    i2c_read(MPU_REG_ADDR_SELF_TEST_Z, &rawData[2], 1); // Z-axis self-test results
    i2c_read(MPU_REG_ADDR_SELF_TEST_A, &rawData[3], 1); // Mixed-axis self-test results
    // Extract the acceleration test results
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03); // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results
    selfTest[3] = rawData[0] & 0x1F; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1] & 0x1F; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2] & 0x1F; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0 * 0.34) *
                     (pow((0.92 / 0.34), (((float) selfTest[0] - 1.0) / 30.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0 * 0.34) *
                     (pow((0.92 / 0.34), (((float) selfTest[1] - 1.0) / 30.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0 * 0.34) *
                     (pow((0.92 / 0.34), (((float) selfTest[2] - 1.0) / 30.0))); // FT[Za] factory trim calculation
    factoryTrim[3] =
            (25.0 * 131.0) * (pow(1.046, ((float) selfTest[3] - 1.0)));         // FT[Xg] factory trim calculation
    factoryTrim[4] =
            (-25.0 * 131.0) * (pow(1.046, ((float) selfTest[4] - 1.0)));         // FT[Yg] factory trim calculation
    factoryTrim[5] =
            (25.0 * 131.0) * (pow(1.046, ((float) selfTest[5] - 1.0)));         // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    ESP_LOGI(TAG, "self test %d %d %d %d %d %d", selfTest[0], selfTest[1], selfTest[2], selfTest[3], selfTest[4],
             selfTest[5]);
    ESP_LOGI(TAG, "factory trim %.2f %.2f %.2f %.2f %.2f %.2f", factoryTrim[0], factoryTrim[1], factoryTrim[2],
             factoryTrim[3], factoryTrim[4], factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0 * ((float) selfTest[i] - factoryTrim[i]) / factoryTrim[i];
        // Report percent differences
    }

    ESP_LOGI(TAG, "self test deviation %.2f %.2f %.2f %.2f %.2f %.2f",
             destination[0], destination[1], destination[2],
             destination[3], destination[4], destination[5]);
}

esp_err_t mpu_init() {
    esp_err_t iic_err = i2c_master_init();
    if (iic_err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialized failed %d %s", iic_err, esp_err_to_name(iic_err));
        return iic_err;;
    }
    ESP_LOGI(TAG, "I2C initialized successfully");

    // who am i
    uint8_t data;
    i2c_read(MPU_REG_ADDR_WHO_AM_I, &data, 1);
    if (data == MPU_ADDR) {
        ESP_LOGI(TAG, "mpu 6050 check success");
    } else {
        ESP_LOGE(TAG, "not mpu 6050 who am i %x", data);
        return ESP_FAIL;
    }

    mpu_reset();

    ESP_LOGI(TAG, "Configure MPU INT gpio");
    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << GPIO_MPU_INT),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = 0,
            .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));

    return ESP_OK;
}

void mpu_prepare() {
    // wake up device-don't need this here if using calibration function below
    //  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    uint8_t data;
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    i2c_write_data(MPU_REG_ADDR_PWR_MGMT_1, 0x01);

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
    i2c_write_data(MPU_REG_ADDR_CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    i2c_write_data(MPU_REG_ADDR_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    i2c_read(MPU_REG_ADDR_GYRO_CONFIG, &data, 1);
    i2c_write_data(MPU_REG_ADDR_GYRO_CONFIG, data & ~0xE0); // Clear self-test bits [7:5]
    i2c_write_data(MPU_REG_ADDR_GYRO_CONFIG, data & ~0x18); // Clear AFS bits [4:3]
    i2c_write_data(MPU_REG_ADDR_GYRO_CONFIG, data | gyro_scale << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    i2c_read(MPU_REG_ADDR_ACCEL_CONFIG, &data, 1);
    i2c_write_data(MPU_REG_ADDR_ACCEL_CONFIG, data & ~0xE0); // Clear self-test bits [7:5]
    i2c_write_data(MPU_REG_ADDR_ACCEL_CONFIG, data & ~0x18); // Clear AFS bits [4:3]
    i2c_write_data(MPU_REG_ADDR_ACCEL_CONFIG, data | accel_scale << 3); // Set full scale range for the accelerometer

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN
    // INT_LEVEL    INT_OPEN            LATCH_INT_EN    INT_RD_CLEAR
    // 0(高电平)     0(pull push, 1开漏)  1               0(读取的时候清空)
    // FSYNC_INT_LEVEL  FSYNC_INT_EN    I2C_BYPASS_EN   -
    // 0                0               1               0
    i2c_write_data(MPU_REG_ADDR_INT_PIN_CFG, 0b00100010);
    // Enable data ready (bit 0) interrupt
    i2c_write_data(MPU_REG_ADDR_INT_ENABLE, 0x01);
    ESP_LOGI(TAG, "mpu 6050 init success");
}

bool mpu_data_int_ready() {
    uint8_t data;
    i2c_read(MPU_REG_ADDR_INT_STATUS, &data, 1);
    return data & 0x01;
}

void mpu_reset() {
    uint8_t data = 1 << 7;
    i2c_write_data(MPU_REG_ADDR_PWR_MGMT_1, data);
    ESP_LOGI(TAG, "reset mpu");
    // wait reset done
    do {
        vTaskDelay(pdMS_TO_TICKS(10));
        i2c_read(MPU_REG_ADDR_PWR_MGMT_1, &data, 1);
        ESP_LOGI(TAG, "wait reset done");
    } while (data >> 7 & 0x01);

    ESP_LOGI(TAG, "reset mpu done");
}

void mpu_sleep() {
    uint8_t data = 1 << 6;
    i2c_write_data(MPU_REG_ADDR_PWR_MGMT_1, data);
    ESP_LOGI(TAG, "mpu sleep");
}

void mpu_deinit() {
    ESP_LOGI(TAG, "mpu deinit");
}