#include "Kalman.h"

KalmanFilter::KalmanFilter()
{
    Q = {{0.05, 0.0},
         {0.0, 0.05}};
    R = {{0.5, 0},
         {0, 0.5}};
    H = {{1, 0},
         {0, 1}};
    F = {{1, 0},
         {0, 1}};
    B = MatrixUtils::Identity(F.size());
    P = MatrixUtils::Identity(F.size());
    x = {0, 0}; 
}

KalmanFilter::KalmanFilter(const Matrix &F, const Matrix &H, 
                           const Matrix &Q, const Matrix &R, 
                           const Vector &x0)
    : F(F), H(H), Q(Q), R(R), x(x0)
{
    B = MatrixUtils::Identity(F.size());
    P = MatrixUtils::Identity(F.size());
}

Vector KalmanFilter::Predict(const Vector &v)
{
    x = MatrixUtils::Mult(F, x);

    if (!v.empty())
    {
        Vector Bv = MatrixUtils::Mult(B, v);
        for (size_t i = 0; i < x.size(); ++i)
        {
            x[i] += Bv[i];
        }
    }

    P = MatrixUtils::Add(MatrixUtils::Mult(F, MatrixUtils::Mult(P, MatrixUtils::Transpose(F))), Q);

    return x;
}

void KalmanFilter::Update(const Vector &z)
{
    Vector y = z;
    Vector Hx = MatrixUtils::Mult(H, x);

    for (size_t i = 0; i < y.size(); ++i)
    {
        y[i] -= Hx[i];
    }

    Matrix S = MatrixUtils::Add(R, MatrixUtils::Mult(H, MatrixUtils::Mult(P, MatrixUtils::Transpose(H))));
    // Kalman Gain
    Matrix K = MatrixUtils::Mult(P, MatrixUtils::Mult(MatrixUtils::Transpose(H), MatrixUtils::Inv(S)));
    Vector K_y = MatrixUtils::Mult(K, y);

    for (size_t i = 0; i < x.size(); ++i)
    {
        x[i] += K_y[i];
    }

    Matrix I = MatrixUtils::Identity(F.size());
    Matrix KH = MatrixUtils::Mult(K, H);

    P = MatrixUtils::Mult(MatrixUtils::Sub(I, KH),
                          MatrixUtils::Mult(P, MatrixUtils::Transpose(MatrixUtils::Sub(I, KH))));
    P = MatrixUtils::Add(P, MatrixUtils::Mult(K, MatrixUtils::Mult(R, MatrixUtils::Transpose(K))));
}
