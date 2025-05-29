#pragma once

#include <cstdint>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cmath>
#include <linux/i2c-dev.h>

// MPU6050 Register Addresses
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_GYRO_XOUT_H   0x43

// Structure to hold accelerometer data.
struct AccelData {
    int16_t x;  // Acceleration on X-axis
    int16_t y;  // Acceleration on Y-axis
    int16_t z;  // Acceleration on Z-axis
};

// Structure to hold gyroscope data.
struct GyroData {
    int16_t x;  // Angular velocity on X-axis
    int16_t y;  // Angular velocity on Y-axis
    int16_t z;  // Angular velocity on Z-axis
};

class MPU6050 {
public:
    /**
     * @brief Constructor.
     * @param device The I2C device file (default is "/dev/i2c-1").
     * @param address The I2C address of the MPU6050 (default is 0x68).
     */
    MPU6050(const std::string &device = "/dev/i2c-1", uint8_t address = 0x68);

    /**
     * @brief Destructor.
     * Closes the I2C file descriptor if it's open.
     */
    ~MPU6050();

    /**
     * @brief Initializes the MPU6050 sensor.
     * Opens the I2C device and wakes up the sensor.
     * @return true if initialization succeeds.
     */
    bool initialize();

    /**
     * @brief Set the accelerometer sensitivity range.
     * @param range The range setting:
     *        0 -> ±2g, 1 -> ±4g, 2 -> ±8g, 3 -> ±16g.
     * @return true if the setting is applied successfully.
     */
    bool setAccelerometerRange(uint8_t range);

        /**
     * @brief Sets the gyroscope sensitivity range.
     * @param range The range value:
     *              0 -> ±250°/s, 1 -> ±500°/s, 2 -> ±1000°/s, 3 -> ±2000°/s.
     * @return true if the operation is successful.
     */
    bool setGyroRange(uint8_t range);

    /**
     * @brief Set the sample rate divider.
     * The output sample rate is: sample_rate = gyro_rate / (1 + divider).
     * @param divider The divider value.
     * @return true if the divider is set successfully.
     */
    bool setSampleRateDivider(uint8_t divider);


    /**
     * @brief Sets the Digital Low Pass Filter (DLPF) configuration.
     * @param config The DLPF configuration value (0-6), where lower numbers mean higher bandwidth. 
     * Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled
     * @return true if the configuration is set successfully.
     */
    bool setDLPF(uint8_t config);

    /**
     * @brief Reads accelerometer data.
     * @param data Reference to an AccelData structure where the data will be stored.
     * @return true if the data is read successfully.
     */
    bool readAccelerometer(AccelData &data);

    /**
     * @brief Reads gyroscope data.
     * @param data Reference to a GyroData structure where the data will be stored.
     * @return true if the data is read successfully.
     */
    bool readGyroscope(GyroData &data);

    /**
     * @brief Converts raw gyro data to rad/s using the current range.
     * @return Conversion factor in rad/s per raw unit.
     */
    double getGyroConversionFactor();

private:
    int fd;                     // File descriptor for the I2C device.
    std::string deviceFile;     // I2C device file (e.g., "/dev/i2c-1").
    uint8_t devAddress;         // I2C address of the MPU6050.
    uint8_t accelRange, gyroRange;
    inline static constexpr double DEG_TO_RAD = M_PI / 180.0;

    /**
     * @brief Writes a single byte to a given register.
     * @param reg Register address.
     * @param value Data byte to write.
     * @return true if the write is successful.
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Reads multiple bytes from registers starting at a given address.
     * @param startReg The starting register address.
     * @param buffer Buffer to store the read data.
     * @param length Number of bytes to read.
     * @return true if the read is successful.
     */
    bool readRegisters(uint8_t startReg, uint8_t *buffer, size_t length);
};