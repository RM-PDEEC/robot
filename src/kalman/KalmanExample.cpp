#include "Kalman.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

// Source(1) : https://medium.com/@ab.jannatpour/kalman-filter-with-python-code-98641017a2bd
// Source(2) : https://github.com/zziz/kalman-filter/tree/master

void example()
{
    // Process noise covariance
    // Matrix Q = {{0.05, 0.0},
    //             {0.05, 0.0}};
    // // Measurement noise covariance
    // Matrix R = {{0.5, 0},
    //             {0, 0.5}};
    // // Measurement matrix
    // Matrix H = {{1, 0},
    //             {0, 1}};
    // // System model
    // Matrix F = {{1, 0},
    //             {0, 1}};

    Vector x(50), y(50);
    Matrix measurements;
    Matrix predictions;
    Vector x0 = {0, 0}; // Initial state vector representing initial x and y positions

    // KalmanFilter kf(F, H, Q, R, x0);
    KalmanFilter kf;
    // Generate measurements
    for (size_t i = 0; i < x.size(); ++i)
    {
        x[i] = -10 + i * (20.0 / 99.0);
        y[i] = -10 + i * (20.0 / 99.0);
        Vector measurement = {-(x[i] * x[i] + 2 * x[i] - 2) + ((double) rand() / RAND_MAX) * 2.0 - 1.0,
                              -(y[i] * y[i] + 2 * y[i] - 2) + ((double) rand() / RAND_MAX) * 2.0 - 1.0};
        measurements.push_back(measurement);
    }
    int status;
    for (const auto &z : measurements)
    {
        predictions.push_back(kf.Predict());
        status = kf.Update(z);
        if (status < 0)
            printf("status: %d\n", status);
    }

    for (size_t i = 0; i < measurements.size(); ++i)
    {
        printf("Measurement: (%.2f, %.2f) Prediction: (%.2f, %.2f)\n",
               measurements[i][0], measurements[i][1],
               predictions[i][0], predictions[i][1]);
    }
}

#ifndef KALMAN

int main() {
    example();
    return 0;
}

#endif
