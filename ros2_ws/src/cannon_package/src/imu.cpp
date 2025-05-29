#include "cannon_package/imu.hpp"

// Implementation of the MPU6050 class.

MPU6050::MPU6050(const std::string &device, uint8_t address)
    : fd(-1), deviceFile(device), devAddress(address), accelRange(0), gyroRange(0) {}

MPU6050::~MPU6050() {
    if (fd >= 0) {
        close(fd);
    }
}

bool MPU6050::initialize() {
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

bool MPU6050::setAccelerometerRange(uint8_t range) {
    accelRange = range;
    uint8_t currentConfig = 0;
    if (!readRegisters(MPU6050_ACCEL_CONFIG, &currentConfig, 1)) {
        return false;
    }

    currentConfig &= ~0x18;
    currentConfig |= (range << 3);

    return writeRegister(MPU6050_ACCEL_CONFIG, currentConfig);
}

bool MPU6050::setGyroRange(uint8_t range) {
    gyroRange = range;
    uint8_t currentConfig = 0;
    if (!readRegisters(MPU6050_GYRO_CONFIG, &currentConfig, 1)) {
        return false;
    }

    currentConfig &= ~0x18;
    currentConfig |= (range << 3);

    return writeRegister(MPU6050_GYRO_CONFIG, currentConfig);
}

double MPU6050::getGyroConversionFactor() {
    static constexpr double conv_factor[] = {
        (1.0 / 131.0) * DEG_TO_RAD, // ±250°/s
        (1.0 / 65.5)  * DEG_TO_RAD, // ±500°/s
        (1.0 / 32.8)  * DEG_TO_RAD, // ±1000°/s
        (1.0 / 16.4)  * DEG_TO_RAD  // ±2000°/s
    };
    return (gyroRange <= 3) ? conv_factor[gyroRange] : conv_factor[0];
}

bool MPU6050::setSampleRateDivider(uint8_t divider) {
    return writeRegister(MPU6050_SMPLRT_DIV, divider);
}

bool MPU6050::setDLPF(uint8_t config) {
    return writeRegister(MPU6050_CONFIG, config & 0x07);
}

bool MPU6050::readAccelerometer(AccelData &data) {
    uint8_t buffer[6] = {0};
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, buffer, 6)) {
        return false;
    }

    data.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}

bool MPU6050::readGyroscope(GyroData &data) {
    uint8_t buffer[6] = {0};
    if (!readRegisters(MPU6050_GYRO_XOUT_H, buffer, 6)) {
        return false;
    }

    data.x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}

bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    if (write(fd, buffer, 2) != 2) {
        return false;
    }
    return true;
}

bool MPU6050::readRegisters(uint8_t startReg, uint8_t *buffer, size_t length) {
    if (write(fd, &startReg, 1) != 1) {
        return false;
    }
    ssize_t bytes_read = read(fd, buffer, length);
    if (bytes_read < 0 || static_cast<size_t>(bytes_read) != length) {
        return false;
    }
    return true;
}