#include "Kalman.h"
#include "Variables.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <limits>

// Source(1) : https://medium.com/@ab.jannatpour/kalman-filter-with-python-code-98641017a2bd
// Source(2) : https://github.com/zziz/kalman-filter/tree/master

using namespace std;

void example()
{
    // Process noise covariance
    Matrix Q = {{0.05, 0.0},
                {0.05, 0.0}};
    // Measurement noise covariance
    Matrix R = {{0.5, 0},
                {0, 0.5}};
    // Measurement matrix
    Matrix H = {{1, 0},
                {0, 1}};
    // System model
    Matrix F = {{1, 0.04},
                {0, 1}};

    Vector x(100), y(100);
    Matrix measurements;
    Matrix predictions;
    Vector x0 = {0, 0}; // Initial state vector representing initial x and y positions

    KalmanFilter kf(F, H, Q, R, x0);

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

void real()
{
    Matrix predictions;

    Matrix Q = {{0.01, 0.0},
                {0.2, 0.0}};
    Matrix R = {{0.5, 0.0},
                {0.0, 0.002}};
    Matrix H = {{1, 0},
                {0, 1}};
    Matrix F = {{1, 0.04},
                {0, 1}};
    Vector x0 = real_positions[0];

    KalmanFilter kf(F, H, Q, R, x0);
    int status;
    for (const auto &z : real_positions)
    {
        predictions.push_back(kf.Predict());
        status = kf.Update(z);
        if (status < 0)
            printf("status: %d\n", status);
    }

    for (size_t i = 0; i < real_positions.size(); ++i)
    {
        printf("Measurement: (%.9f, %.9f) Prediction: (%.9f, %.9f)\n",
               real_positions[i][0], real_positions[i][1],
               predictions[i][0], predictions[i][1]);
    }
}

double calculateMSE(const Matrix &measurements, const Matrix &predictions) {
    size_t n = measurements.size();
    double mse = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        mse += pow(measurements[i][0] - predictions[i][0], 2);
        mse += pow(measurements[i][1] - predictions[i][1], 2);
    }

    return mse / n;
}

void autoTune()
{
    Matrix H = {{1, 0},
                {0, 1}};
    Matrix F = {{1, 0.04},
                {0, 1}};
    Vector x0 = real_positions[0];
    Matrix predictions;
    int status;

    Vector q_values = {0.01, 0.05, 0.1, 0.15, 0.2};
    Vector r_values = {0.1, 0.5, 1.0, 1.5, 2.0};
    Matrix Q_best;
    Matrix R_best;

    double best_mse = numeric_limits<double>::max();

    for (double qx : q_values)
    {
        for (double qy : q_values)
        {
            for (double rx : r_values)
            {
                for (double ry : r_values)
                {
                    Matrix Q = {{qx, 0.0},
                                {qy, 0.0}};
                    Matrix R = {{rx, 0},
                                {0, ry}};
                    KalmanFilter kf(F, H, Q, R, x0);

                    Matrix predictions;
                    for (const auto &z : real_positions)
                    {
                        predictions.push_back(kf.Predict());
                        status = kf.Update(z);
                        if (status < 0)
                            printf("status: %d\n", status);
                    }

                    double mse = calculateMSE(real_positions, predictions);

                    if (mse < best_mse)
                    {
                        best_mse = mse;
                        Q_best = Q;
                        R_best = R;
                    }
                }
            }
        }
    }

    KalmanFilter kf(F, H, Q_best, R_best, x0);

    for (const auto &z : real_positions)
    {
        predictions.push_back(kf.Predict());
        status = kf.Update(z);
        if (status < 0)
            printf("status: %d\n", status);
    }

    for (size_t i = 0; i < real_positions.size(); ++i)
    {
        printf("Measurement: (%.9f, %.9f) Prediction: (%.9f, %.9f)\n",
               real_positions[i][0], real_positions[i][1],
               predictions[i][0], predictions[i][1]);
    }

    printf("Best Q:\n");
    MatrixUtils::Print(Q_best);
    printf("Best R:\n");
    MatrixUtils::Print(R_best);
}

#ifndef KALMAN

// int main() {
//     real();
//     return 0;
// }

#endif
