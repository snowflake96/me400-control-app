#pragma once

#include <vector>
#include <utility>
#include <algorithm>
#include <stdexcept>

class CubicSpline {
public:
    // Default constructor with predefined (x, y) points to interpolate
    // Format: {pitch angle (deg), offset} in increasing order of pitch angle
    CubicSpline()
        : CubicSpline({
            {49, 0.037},
            {55.5, 0.038},
            {65, 0.054},
        })
    {}

    // Constructor from initializer list of (x, y) pairs
    CubicSpline(std::initializer_list<std::pair<double, double>> points)
        : points_(points.begin(), points.end()), size_(points_.size())
    {
        if (size_ < 2) {
            throw std::invalid_argument("At least two data points are required.");
        }

        x_.resize(size_);
        y_.resize(size_);
        for (size_t i = 0; i < size_; ++i) {
            x_[i] = points_[i].first;
            y_[i] = points_[i].second;
        }

        calculateCoefficients();
    }

    // Evaluate the spline at a given x
    double interpolate(double x) const {
        auto it = std::upper_bound(x_.begin(), x_.end(), x);
        int i = std::distance(x_.begin(), it);
        if (i == 0) i = 1;
        if (i >= static_cast<int>(size_)) i = static_cast<int>(size_ - 1);

        int j = i - 1;
        double h = x - x_[j];
        return a_[j] + b_[j] * h + c_[j] * h * h + d_[j] * h * h * h;
    }

    // Optional operator() overload for cleaner syntax
    double operator()(double x) const {
        return interpolate(x);
    }

private:
    std::vector<std::pair<double, double>> points_;
    std::vector<double> x_, y_;
    std::vector<double> a_, b_, c_, d_;
    size_t size_;

    void calculateCoefficients() {
        a_ = y_;
        std::vector<double> h(size_ - 1);

        for (size_t i = 0; i < size_ - 1; ++i)
            h[i] = x_[i + 1] - x_[i];

        std::vector<double> alpha(size_ - 1);
        for (size_t i = 1; i < size_ - 1; ++i)
            alpha[i] = (3.0 / h[i]) * (a_[i + 1] - a_[i]) - (3.0 / h[i - 1]) * (a_[i] - a_[i - 1]);

        std::vector<double> l(size_), mu(size_), z(size_);
        l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;

        for (size_t i = 1; i < size_ - 1; ++i) {
            l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[size_ - 1] = 1.0;
        z[size_ - 1] = 0.0;
        c_.resize(size_);
        c_[size_ - 1] = 0.0;

        b_.resize(size_ - 1);
        d_.resize(size_ - 1);

        for (int j = static_cast<int>(size_) - 2; j >= 0; --j) {
            c_[j] = z[j] - mu[j] * c_[j + 1];
            b_[j] = (a_[j + 1] - a_[j]) / h[j] - h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
            d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
        }
    }
};
