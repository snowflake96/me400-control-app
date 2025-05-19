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
     * @brief Converts raw accel data to m/s^2 using the current range.
     * @return Conversion factor in m/s^s per raw unit.
     */

    /**
     * @brief Reads both accelerometer and gyroscope data in a single I2C transaction.
     * @param accel Reference to AccelData struct to store accelerometer readings.
     * @param gyro Reference to GyroData struct to store gyroscope readings.
     * @return true if the read is successful.
     */
    bool readAccelGyro(AccelData &accel, GyroData &gyro);

    double getAccelConversionFactor();

    /**
     * @brief Converts raw gyro data to deg/s using the current range.
     * @return Conversion factor in deg/s per raw unit.
     */
    double getGyroConversionFactor();

    // DLPF settings
    enum class DLPFConfig : uint8_t {
        BW_260HZ = 0,
        BW_184HZ = 1,
        BW_94HZ  = 2,
        BW_44HZ  = 3,
        BW_21HZ  = 4,
        BW_10HZ  = 5,
        BW_5HZ   = 6
    };

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


// Implementation of the MPU6050 class.

inline MPU6050::MPU6050(const std::string &device, uint8_t address)
    : fd(-1), deviceFile(device), devAddress(address), accelRange(0), gyroRange(0) {}

inline MPU6050::~MPU6050() {
    if (fd >= 0) {
        close(fd);
    }
}

inline bool MPU6050::initialize() {
    // Open the I2C device file.
    fd = open(deviceFile.c_str(), O_RDWR);
    if (fd < 0) {
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, devAddress) < 0) {
        close(fd);
        fd = -1;
        return false;
    }

    // Wake up the MPU6050 (clear the sleep bit in the power management register).
    if (!writeRegister(MPU6050_PWR_MGMT_1, 0)) {
        return false;
    }

    return true;
}

inline bool MPU6050::setAccelerometerRange(uint8_t range) {
    accelRange = range;
    uint8_t currentConfig = 0;
    if (!readRegisters(MPU6050_ACCEL_CONFIG, &currentConfig, 1)) {
        return false;
    }

    currentConfig &= ~0x18;
    currentConfig |= (range << 3);

    return writeRegister(MPU6050_ACCEL_CONFIG, currentConfig);
}

inline bool MPU6050::setGyroRange(uint8_t range) {
    gyroRange = range;
    uint8_t currentConfig = 0;
    if (!readRegisters(MPU6050_GYRO_CONFIG, &currentConfig, 1)) {
        return false;
    }

    currentConfig &= ~0x18;
    currentConfig |= (range << 3);

    return writeRegister(MPU6050_GYRO_CONFIG, currentConfig);
}

inline double MPU6050::getAccelConversionFactor() {
    static constexpr double conv_factor[] = {
        (1.0 / 16384.0) * 9.80665, // ±2g
        (1.0 / 8192.0)  * 9.80665, // ±4g
        (1.0 / 4096.0)  * 9.80665, // ±8g
        (1.0 / 2048.0)  * 9.80665  // ±16g
    };
    return (accelRange <= 3) ? conv_factor[accelRange] : conv_factor[0];
}

// Raw to degrees/s
inline double MPU6050::getGyroConversionFactor() {
    static constexpr double conv_factor[] = {
        (1.0 / 131.0), // ±250°/s
        (1.0 / 65.5), // ±500°/s
        (1.0 / 32.8), // ±1000°/s
        (1.0 / 16.4)  // ±2000°/s
    };
    return (gyroRange <= 3) ? conv_factor[gyroRange] : conv_factor[0];
}

inline bool MPU6050::setSampleRateDivider(uint8_t divider) {
    return writeRegister(MPU6050_SMPLRT_DIV, divider);
}

inline bool MPU6050::setDLPF(uint8_t config) {
    return writeRegister(MPU6050_CONFIG, config & 0x07);
}

inline bool MPU6050::readAccelerometer(AccelData &data) {
    uint8_t buffer[6] = {0};
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, buffer, 6)) {
        return false;
    }

    data.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}

inline bool MPU6050::readGyroscope(GyroData &data) {
    uint8_t buffer[6] = {0};
    if (!readRegisters(MPU6050_GYRO_XOUT_H, buffer, 6)) {
        return false;
    }

    data.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}

inline bool MPU6050::readAccelGyro(AccelData &accel, GyroData &gyro) {
    uint8_t buffer[14] = {0};

    // Start from ACCEL_XOUT_H (0x3B) and read 14 bytes (accel, temp, gyro)
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, buffer, 14)) {
        return false;
    }

    // Parse accelerometer
    accel.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Skip buffer[6–7] (temperature)

    // Parse gyroscope
    gyro.x = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro.y = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyro.z = (int16_t)((buffer[12] << 8) | buffer[13]);

    return true;
}

inline bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    if (write(fd, buffer, 2) != 2) {
        return false;
    }
    return true;
}

inline bool MPU6050::readRegisters(uint8_t startReg, uint8_t *buffer, size_t length) {
    if (write(fd, &startReg, 1) != 1) {
        return false;
    }
    ssize_t bytes_read = read(fd, buffer, length);
    if (bytes_read < 0 || static_cast<size_t>(bytes_read) != length) {
        return false;
    }
    return true;
}