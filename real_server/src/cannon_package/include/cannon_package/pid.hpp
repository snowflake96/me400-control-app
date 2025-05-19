/*
This is a PID controller class
Created by: You, Jisang on 2021-06-30
*/
#pragma once
#include <chrono>
#include <algorithm> // std::clamp
#include <limits>
#include <cmath>      // std::isnan, M_PI
#include <cassert>

class LowPassFilter {
private:
    // time constant RC = 1/(2π f_c)
    double RC_;            

    // last output
    double y_prev_;        

    // flag for first sample
    bool   initialized_;    

    // timestamp of the last filter() call
    using Clock      = std::chrono::steady_clock;
    Clock::time_point t_prev_;  

public:
    // f_c = cutoff frequency [Hz]
    LowPassFilter(double cutoff_hz = 10.0)
      : RC_(1.0 / (2.0 * M_PI * cutoff_hz)),
        y_prev_(0.0),
        initialized_(false),
        t_prev_(Clock::now())
    {
        assert(cutoff_hz > 0);
    }

    // Call this on each new sample; dt is computed internally.
    double filter(double x) {
        if (std::isnan(x)) {
            return y_prev_;  // ignore NaN inputs
        }

        // compute elapsed time (seconds) since last call
        auto now      = Clock::now();
        std::chrono::duration<double> diff = now - t_prev_;
        double dt      = diff.count();
        t_prev_        = now;

        // guard
        assert(dt > 0);

        // compute smoothing factor
        double alpha = dt / (RC_ + dt);
        alpha = std::clamp(alpha, 0.0, 1.0);

        // apply filter
        if (!initialized_) {
            y_prev_      = x;
            initialized_ = true;
        } else {
            y_prev_ = alpha * x + (1.0 - alpha) * y_prev_;
        }
        return y_prev_;
    }

    // Reset internal state (and timestamp) if you jump to a new operating point
    void reset(double v = 0.0) {
        y_prev_      = v;
        initialized_ = true;
        t_prev_      = Clock::now();
    }

    // Change cutoff frequency on the fly
    void setCutoffFrequency(double cutoff_hz) {
        assert(cutoff_hz > 0);
        RC_ = 1.0 / (2.0 * M_PI * cutoff_hz);
    }
};

class PID {
private:
    // Member variables for PID controller
    double Kp_, Ki_, Kd_;
    double error, prev_error, integral;
    double last_time;
    double integral_limit_;

    // Low-pass filter for derivative term
    LowPassFilter lpf;

public:
    PID() : prev_error(0), integral(0), last_time(0), integral_limit_(std::numeric_limits<double>::max()) {
        setGains(1.0, 0.0, 0.0);
        setIntegralLimit(1.0);
    }

    void setGains(double Kp, double Ki, double Kd){
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }

    void setLPFCutoff(double cutoff_freq) {
        lpf.setCutoffFrequency(cutoff_freq);
    }

    void setIntegralLimit(double limit){
        integral_limit_ = limit;
    }

    double getCurrentTime() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double getLastError(){
        return error;
    }

    void resetIntegral(){
        integral = 0;
    }

    double compute(double setpoint, double measured_value) {
        error = setpoint - measured_value;
        double current_time = getCurrentTime();
        double dt = current_time - last_time;

        if (dt <= 0.0) return 0.0;  // Avoid division by zero

        integral += error * dt;
        integral = std::clamp(integral, -integral_limit_, integral_limit_); // Prevent windup
        
        // double derivative = lpf.filter(error - prev_error) / dt; // filtered derivative
        // prev_error = error;
        // last_time = current_time;
        // double output = (Kp_ * error) + (Ki_ * integral) + (Kd_ * derivative);

        double output = (Kp_ * error) + (Ki_ * integral);

        return output;
    }
};