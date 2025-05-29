#pragma once
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

/**
 * @brief Header-only driver for the PCA9685 16-channel PWM controller.
 * Supports setting frequency, raw PWM ticks, microsecond pulses,
 * and normalized continuous-servo speeds.
 */
class PCA9685Driver {
public:
    /**
     * @param i2c_dev Path to I2C device (e.g. "/dev/i2c-1").
     * @param addr   7-bit I2C address (default 0x40).
     */
    PCA9685Driver(const char* i2c_dev = "/dev/i2c-1", uint8_t addr = 0x40);
    ~PCA9685Driver();

    /**
     * @brief Set PWM output frequency.
     * @param freq_hz Desired frequency in Hz (e.g. 50.0f).
     */
    void setFrequency(float freq_hz);

    /**
     * @brief Write raw PWM on/off tick values for a channel.
     * @param channel Channel number (0–15)
     * @param on      Tick (0–4095) when signal goes high
     * @param off     Tick (0–4095) when signal goes low
     */
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);

    /**
     * @brief Set pulse width in microseconds for a channel.
     * @param channel  Channel number (0–15)
     * @param pulse_us Pulse width in µs (clamped to 500–2500)
     */
    void setPulseWidth(uint8_t channel, float pulse_us);

    /**
     * @brief Set a hobby-servo to a given angle.
     * @param channel       PCA9685 channel (0–15)
     * @param angle_deg     Desired position in degrees
     * @param min_angle_deg Minimum angle (default 0°)
     * @param max_angle_deg Maximum angle (default 180°)
     * @param min_pulse_us  Pulse width at min_angle (default 1000µs)
     * @param max_pulse_us  Pulse width at max_angle (default 2000µs)
     */
    void setServoPosition(uint8_t channel,
                         float angle_deg,
                         const float min_angle_deg = 0.0f, 
                         const float max_angle_deg = 180.0f,
                         const float min_pulse_us  = 500.0f,
                         const float max_pulse_us  = 2500.0f);

    /**
     * @brief Set continuous-servo speed normalized to [-1.0, 1.0].
     * @param channel   Channel number (0–15)
     * @param speed     -1.0 (full reverse) to +1.0 (full forward)
     *                  0.0 → stop (center pulse)
     * @param cw_min_us Minimum pulse width for clockwise motion (CW)
     * @param cw_max_us Maximum pulse width for clockwise motion (CW)
     * @param ccw_min_us Minimum pulse width for counter-clockwise motion (CCW)
     * @param ccw_max_us Maximum pulse width for counter-clockwise motion (CCW)
     */
    void setServoSpeed(uint8_t channel,
                      float speed,
                      const float cw_min_us = 1000.0f,
                      const float cw_max_us = 1740.0f,
                      const float ccw_min_us = 1880.0f,
                      const float ccw_max_us = 2600.0f);

    /**
     * @brief Set ESC throttle on a channel.
     *
     * Maps throttle ∈ [–1.0…+1.0] into:
     *   –1.0 → (center_us – span_us)
     *    0.0 → center_us
     *   +1.0 → (center_us + span_us)
     *
     * @param channel    PCA9685 channel (0–15)
     * @param throttle   –1.0 = full reverse, 0.0 = stop, +1.0 = full forward
     * @param center_us  Pulse width at zero throttle (default 1740 µs) - NOW HARDCODED FOR SAFETY
     * @param span_us    Max deviation from center (default 500 µs) - NOW HARDCODED FOR SAFETY
     */
    void setESC(uint8_t channel, float throttle);

    /**
     * @brief Disable PWM output on a single channel (force full-off).
     * @param channel  Channel number (0–15)
     */
    void disableChannel(uint8_t channel);

    /**
     * @brief Disable PWM output on all 16 channels (force full-off).
     */
    void disableAllChannels();

private:
    int     fd_;
    uint8_t address_;

    static constexpr uint8_t MODE1         = 0x00;
    static constexpr uint8_t PRESCALE      = 0xFE;
    static constexpr uint8_t LED0_ON_L     = 0x06;
    static constexpr float   OSC_FREQ      = 25e6f;   // 25 MHz
    static constexpr int     RESOLUTION    = 4096;
    static constexpr float   PERIOD_US     = 20000.0f; // 20 ms
    static constexpr float   MIN_US        = 500.0f;
    static constexpr float   MAX_US        = 2500.0f;
    static constexpr uint8_t SLEEP_BIT     = 0x10;
    static constexpr uint8_t RESTART_BIT   = 0x80;

    /**
     * @brief Write a single-byte register.
     */
    void writeReg(uint8_t reg, uint8_t val);

    /**
     * @brief Read a single-byte register.
     */
    uint8_t readReg(uint8_t reg);
};



// Constructor: opens I2C device and sets slave address
inline PCA9685Driver::PCA9685Driver(const char* i2c_dev, uint8_t addr)
  : fd_(-1), address_(addr)
{
  fd_ = ::open(i2c_dev, O_RDWR);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open I2C device");
  }
  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    ::close(fd_);
    throw std::runtime_error("Failed to set I2C slave address");
  }
}

// Destructor: closes I2C file descriptor
inline PCA9685Driver::~PCA9685Driver() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

// Read a single byte from a register
inline uint8_t PCA9685Driver::readReg(uint8_t reg) {
  if (::write(fd_, &reg, 1) != 1) {
    throw std::runtime_error("I2C write for readReg failed");
  }
  uint8_t val;
  if (::read(fd_, &val, 1) != 1) {
    throw std::runtime_error("I2C read for readReg failed");
  }
  return val;
}

// Write a single byte to a register
inline void PCA9685Driver::writeReg(uint8_t reg, uint8_t val) {
  uint8_t buf[2] = { reg, val };
  if (::write(fd_, buf, 2) != 2) {
    throw std::runtime_error("I2C writeReg failed");
  }
}

inline void PCA9685Driver::setFrequency(float freq_hz) {
  uint8_t oldmode = readReg(MODE1);
  // go to sleep (clear RESTART, set SLEEP)
  writeReg(MODE1, (oldmode & ~RESTART_BIT) | SLEEP_BIT);

  // compute and write prescale
  float prescaleval = OSC_FREQ/(RESOLUTION*freq_hz) - 1.0f;
  uint8_t prescale = uint8_t(std::floor(prescaleval + 0.5f));
  writeReg(PRESCALE, prescale);

  // wake up *with* Auto-Increment (bit 5 = 1)
  uint8_t wake = (oldmode & ~SLEEP_BIT) | (1<<5);
  writeReg(MODE1, wake);
  ::usleep(5000);  

  // finally, restart (bit 7) while keeping AI
  writeReg(MODE1, wake | RESTART_BIT);
}

// Set raw PWM on/off ticks for a channel
inline void PCA9685Driver::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
  if (channel > 15) {
    throw std::out_of_range("Channel must be in range 0-15");
  }
  uint8_t buf[5];
  buf[0] = LED0_ON_L + 4 * channel;
  buf[1] = on & 0xFF;
  buf[2] = (on >> 8) & 0x0F;
  buf[3] = off & 0xFF;
  buf[4] = (off >> 8) & 0x0F;
  if (::write(fd_, buf, 5) != 5) {
    throw std::runtime_error("I2C setPWM write failed");
  }
}

// Set pulse width in microseconds (clamped 500-2500)
inline void PCA9685Driver::setPulseWidth(uint8_t channel, float pulse_us) {
  float clamped = std::clamp(pulse_us, MIN_US, MAX_US);
  uint16_t ticks = static_cast<uint16_t>(clamped / PERIOD_US * RESOLUTION);
  setPWM(channel, 0, ticks);
}

// Set a hobby-servo to a given angle
inline void PCA9685Driver::setServoPosition(uint8_t channel,
                                           float angle_deg,
                                           const float min_angle_deg,
                                           const float max_angle_deg,
                                           const float min_pulse_us,
                                           const float max_pulse_us) {
  if(std::isnan(angle_deg)) return;
  const float angle = std::clamp(angle_deg, min_angle_deg, max_angle_deg);
  const float span = max_angle_deg - min_angle_deg;
  const float ratio = (angle - min_angle_deg) / span;
  const float pulse = min_pulse_us + ratio * (max_pulse_us - min_pulse_us);
  setPulseWidth(channel, pulse);
}

// Set continuous-servo speed normalized [-1.0, +1.0]
inline void PCA9685Driver::setServoSpeed(uint8_t channel,
                                        float speed,
                                        const float cw_min_us,
                                        const float cw_max_us,
                                        const float ccw_min_us,
                                        const float ccw_max_us) {
  if (std::isnan(speed)) return;

  float clipped = std::clamp(speed, -1.0f, 1.0f);
  float pulse = 0.0f;

  if (clipped > 0.0f) {
    // CW: from stop (cw_max_us) down to cw_min_us
    pulse = cw_max_us + clipped * (cw_min_us - cw_max_us);
  }
  else if (clipped < 0.0f) {
    // CCW: from stop (ccw_min_us) up to ccw_max_us
    pulse = ccw_min_us + (-clipped) * (ccw_max_us - ccw_min_us);
  } else {
    // speed == 0, stop: use either neutral pulse
    pulse = 0.5f * (cw_max_us + ccw_min_us);  // or assume they are equal
  }

  setPulseWidth(channel, pulse);
}


inline void PCA9685Driver::setESC(uint8_t channel,
                                  float throttle) {
  // Now hardcoded for safety
  static constexpr float SAFETY_LIMIT = 0.5f;
  static constexpr float center_us = 1795.0f;
  static constexpr float span_us   = 500.0f;

  if (std::isnan(throttle)) return;
  // clamp to [-0.1, +SAFETY_LIMIT]
  throttle = std::clamp(throttle, -0.1f, SAFETY_LIMIT);
  // linear map around center
  float pulse_us = center_us - throttle * span_us; // sign reversed to fix direction
  setPulseWidth(channel, pulse_us);
}

inline void PCA9685Driver::disableChannel(uint8_t channel) {
  if (channel > 15) {
    throw std::out_of_range("Channel must be in range 0-15");
  }
  // LEDn_ON = 0
  writeReg(LED0_ON_L     + 4*channel, 0x00);
  writeReg(LED0_ON_L + 1 + 4*channel, 0x00);
  // LEDn_OFF = full-off (bit4 = 1)
  writeReg(LED0_ON_L + 2 + 4*channel, 0x00);
  writeReg(LED0_ON_L + 3 + 4*channel, 0x10);
}

inline void PCA9685Driver::disableAllChannels() {
  for (uint8_t ch = 0; ch < 16; ++ch) {
    disableChannel(ch);
  }
}