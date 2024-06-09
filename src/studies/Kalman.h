#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "MatrixUtils.h"

using namespace std;

class KalmanFilter {
public:
    KalmanFilter(const Matrix &F, const Matrix &H, 
                 const Matrix &Q, const Matrix &R, 
                 const Vector &x0);

    Vector Predict(const Vector &u = {});
    void Update(const Vector &z);

private:
    /** 
     * F - State transition matrix (Describes the state transition from the previous time step to the current one)
     * H - Observation matrix (Maps the true state space into the observed space)
     * Q - Process noise covariance matrix
     * R - Measurement noise covariance matrix
     * P - Estimation error covariance matrix (P) 
     * B - Control input matrix (optional, can be set to zero if no control input is used)
    */
    Matrix F, H, Q, R, P, B;
    // x - It is the updated estimate of the state
    Vector x;
};

#endif // KALMANFILTER_H