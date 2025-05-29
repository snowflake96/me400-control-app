/*
This is a PID controller class
Created by: You, Jisang on 2025 April 5th
*/
#pragma once
#include <chrono>
#include <algorithm> // std::clamp
#include <limits>
#include <cmath>      // std::isnan, M_PI
#include <cassert>
#include <utility>

class LowPassFilter {
private:
    // time constant RC = 1/(2π f_c)
    double RC_;            

    // last output
    double y_prev_;        

    // Cutoff frequency
    double cutoff_freq_;

    // flag for first sample
    bool   initialized_;    

    // timestamp of the last filter() call
    using Clock      = std::chrono::steady_clock;
    Clock::time_point t_prev_;  

public:
    // f_c = cutoff frequency [Hz]
    LowPassFilter(double cutoff_hz = 5.0)
      : RC_(1.0 / (2.0 * M_PI * cutoff_hz)),
        y_prev_(0.0),
        cutoff_freq_(cutoff_hz),
        initialized_(false),
        t_prev_(Clock::now())
    {
        assert(cutoff_hz > 0);
    }

    // Call this on each new sample; dt is computed internally.
    double filter(double x){
        if (std::isnan(x))
            return y_prev_;
    
        if (!initialized_) {
            // first sample – no history, no dt needed
            y_prev_      = x;
            initialized_ = true;
            t_prev_      = Clock::now();
            return y_prev_;
        }
    
        // --- normal path ---
        auto   now  = Clock::now();
        double dt   = std::chrono::duration<double>(now - t_prev_).count();
        t_prev_     = now;
    
        assert(dt > 0);
    
        double alpha = std::clamp(dt / (RC_ + dt), 0.0, 1.0);
        y_prev_      = alpha * x + (1.0 - alpha) * y_prev_;
        return y_prev_;
    }

    void reset() {
        initialized_ = false;
    }

    // Change cutoff frequency on the fly
    void setCutoffFrequency(double cutoff_hz) {
        cutoff_freq_ = std::abs(cutoff_hz);
        RC_ = 1.0 / (2.0 * M_PI * cutoff_freq_);
        reset();
    }

    double getCutoffFrequency() const {
        return cutoff_freq_;
    }
};

// PID class
class PID {
    double Kp_{10.0}, Ki_{0.0}, Kd_{0.0};
    double error_{0.0}, prev_error_{0.0}, integral_{0.0};
    double last_time_;
    double integral_limit_{1.0};
    bool   first_call_{true};
    double psignal_, isignal_;
    double integral_threshold_{std::numeric_limits<double>::max()}; // To start, there is no threshold

    static double nowSeconds() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

public:
    PID() : last_time_(nowSeconds()) {}
    
    void setGains(double kp, double ki, double kd) { Kp_=kp; Ki_=ki; Kd_=kd; }
    void setIntegralLimit(double lim) { integral_limit_=lim; }
    void setIntegralThreshold(double threshold){ integral_threshold_ = std::abs(threshold); }
    double getIntegralLimit() const { return integral_limit_; }
    void reset() { integral_=0.0; first_call_=true; }
    double getLastError() { return error_; }
    double getLastPsignal() const { return psignal_; }
    double getLastIsignal() const { return isignal_; }
    std::pair<double, double> getPIgains(){ return {Kp_, Ki_}; }

    double compute(double setpoint, double measured)
    {
        double current_time = nowSeconds();
        double dt = current_time - last_time_;
        if (dt <= 0.0) return 0.0;          // should never hit, but be safe

        error_ = setpoint - measured;

        // ---- Integral term ----
        if (!first_call_ && std::abs(error_) < integral_threshold_) {
            integral_ += error_ * dt;
            integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        }

        // ---- PID Output ----
        psignal_ = Kp_ * error_;
        isignal_ = Ki_ * integral_;
        double output = psignal_ + isignal_; // use only PI terms

        // ---- Update state ----
        prev_error_ = error_;
        last_time_  = current_time;
        first_call_ = false;

        return output;
    }
};
