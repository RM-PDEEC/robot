#include "Kalman.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

void example() {
    double dt = 1.0 / 60.0;
    Matrix F = {{1, dt},
                {0, 1}};
    Matrix H = {{1, 0}};
    Matrix Q = {{0.05, 0.05},
                {0.05, 0.05}};
    Matrix R = {{0.5}};
    Vector x0 = {0, 0};
    KalmanFilter kf(F, H, Q, R, x0);
    Vector predictions;
    Vector x(100), measurements(100);
    for (size_t i = 0; i < x.size(); ++i)
    {
        x[i] = -10 + i * (20.0 / 99.0);
        measurements[i] = -(x[i] * x[i] + 2 * x[i] - 2) + ((double) rand() / RAND_MAX) * 2.0 - 1.0;
    }

    for (const auto &z : measurements)
    {
        predictions.push_back(kf.Predict()[0]);
        kf.Update({z});
    }

    for (size_t i = 0; i < measurements.size(); ++i)
    {
        printf("Measurement: %f Prediction: %f\n", measurements[i], predictions[i]);
    }
}

int main() {
    example();
    return 0;
}
