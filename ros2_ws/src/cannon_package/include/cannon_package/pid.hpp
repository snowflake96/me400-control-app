/*
This is a PID controller class
Created by: You, Jisang on 2021-06-30
*/
#pragma once
#include <chrono>
#include <algorithm> // std::clamp
#include <limits>

class LowPassFilter {
private:
    double alpha;  // Smoothing factor (between 0 and 1)
    double filtered_value;
    bool initialized;

public:
    // Cutoff frequency in Hz, sampling time in seconds
    LowPassFilter(double cutoff_freq = 2.0, double sampling_time = 0.1) {
        setCutoffFrequency(cutoff_freq, sampling_time);
        filtered_value = 0.0;
        initialized = false;
    }

    // Set a new cutoff frequency and recompute alpha
    void setCutoffFrequency(double cutoff_freq, double sampling_time) {
        double RC = 1.0 / (2.0 * 3.141592653589793 * cutoff_freq);
        alpha = sampling_time / (sampling_time + RC);
    }

    // Apply the filter to a new raw input
    double filter(double input) {
        if (!initialized) {
            filtered_value = input;  // Initialize with the first value
            initialized = true;
        } else {
            filtered_value = alpha * input + (1.0 - alpha) * filtered_value;
        }
        return filtered_value;
    }

    // Get the current filtered value
    double getFilteredValue() const {
        return filtered_value;
    }
};


class PID {
private:
    // Member variables for PID controller
    double Kp_, Ki_, Kd_;
    double prev_error, integral;
    double last_time;
    double integral_limit_;

    // Low-pass filter for derivative term
    LowPassFilter lpf;

public:
    PID() : prev_error(0), integral(0), last_time(0), integral_limit_(std::numeric_limits<double>::max()) {
        setGains(0.0, 0.0, 0.0);
    }

    void setGains(double Kp, double Ki, double Kd){
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }

    void setParams(double cutoff_freq, double sampling_time, double limit) {
        lpf.setCutoffFrequency(cutoff_freq, sampling_time);
        integral_limit_ = limit;
    }

    double getCurrentTime() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double compute(double setpoint, double measured_value) {
        double error = setpoint - measured_value;
        double current_time = getCurrentTime();
        double dt = current_time - last_time;

        if (dt <= 0.0) return 0.0;  // Avoid division by zero

        integral += error * dt;
        integral = std::clamp(integral, -integral_limit_, integral_limit_); // Prevent windup
        
        double derivative = lpf.filter(error - prev_error) / dt; // filtered derivative

        double output = (Kp_ * error) + (Ki_ * integral) + (Kd_ * derivative);

        prev_error = error;
        last_time = current_time;

        return output;
    }
};