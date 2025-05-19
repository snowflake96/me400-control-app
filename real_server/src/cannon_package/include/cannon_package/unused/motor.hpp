/*
Motor class code for setting PWM duty cycle on Raspberry Pi 4B
Created by You, Jisang 2025/3/14
*/

#include <gpiod.h>
#include <lgpio.h>
#include <unistd.h>
#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>

// Frequency = 50Hz
// Pulse width = 1.0ms / 1.5ms / 2.0ms

// pulse -> microseconds
// time -> milliseconds

class Motor {
    public:
        Motor(int pin) : pin_(pin), running_(false) {
            const char *chipname = "gpiochip4";
            unsigned int line_num = pin;

            chip = gpiod_chip_open_by_name(chipname);
            if (!chip) {
                std::cerr << "Failed to open " << chipname << "in CONSTRUCTOR"<< std::endl;
            }

            line = gpiod_chip_get_line(chip, line_num);
            if (!line) {
                std::cerr << "Failed to get line " << line_num << "in CONSTRUCTOR"<< std::endl;
                gpiod_chip_close(chip);
            }

            if (gpiod_line_request_output(line, "my_output", 0) < 0) {
                std::cerr << "Failed to request line as output" << "in CONSTRUCTOR"<< std::endl;
                gpiod_chip_close(chip);
            }
            running_ = false;
            CCW = false;
            CW = false;
        }
    bool isRunning() const {
        return running_;
    }

    void OutputGivenPulse(float pulse, gpiod_line *line) {
        if (pulse < 0 || pulse > 2000) {
            std::cerr << "Pulse width must be between 0 and 2000 microseconds." << std::endl;
            return;
        }
        while (running_) {
            gpiod_line_set_value(line, 1); // HIGH
            usleep(pulse); // 1.0ms / 1.5ms / 2.0ms
            gpiod_line_set_value(line, 0); // LOW
            usleep(20000 - pulse); // 19.0ms / 18.5ms / 18.0ms
        }
    }

    void Run() {
        int pulse = 0;
        while (running_) {
            if (CCW) {
                pulse = 1000;
            }
            else if (CW) {
                pulse = 2000;
            }
            else {
                pulse = 1500;
            }
            if (gpiod_line_request_output(line, "my_output", 0) < 0) {
                std::cerr << "Failed to request line as output" << std::endl;
                gpiod_chip_close(chip);
            }
            if (gpiod_line_set_value(line, 1) < 0) {
                perror("Failed to set line value");
            } // HIGH
            usleep(pulse); // 1.0ms / 1.5ms / 2.0ms
            gpiod_line_set_value(line, 0); // LOW
            usleep(20000 - pulse); // 19.0ms / 18.5ms / 18.0ms
            std::cout << "Pulse: " << pulse << std::endl;

        }
    }

    void start() {
        if (running_) return;
        running_ = true;
        run_thread_ = std::thread(&Motor::Run, this);
    }

    void total_stop() {
        if (!running_) return;
        running_ = false;
        if (run_thread_.joinable()) {
            run_thread_.join();
        }
    }

    void ToCCW() {
        CCW = true;
        CW = false;
    }

    void ToCW() {
        CCW = false;
        CW = true;
    }

    void stop() {
        CCW = false;
        CW = false;
    }

    ~Motor() {
        stop();
        // gpiod_line_release(line);
        // gpiod_chip_close(chip);
        // std::cout << "Motor stopped and resources released." << std::endl;
    }

private:
    // From C++17 static constexpr is implicitly inlined
    int pin_;
    std::atomic<bool> running_;
    gpiod_line *line;
    gpiod_chip *chip;
    std::atomic<bool> CCW;
    std::atomic<bool> CW;
    std::thread run_thread_;
};