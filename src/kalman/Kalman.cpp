#include "Kalman.h"

KalmanFilter::KalmanFilter()
{
    Q = {{0.01, 0.0},
         {0.2, 0.0}};
    R = {{0.5, 0.0},
         {0.0, 0.1}};
    H = {{1, 0},
         {0, 1}};
    F = {{1, 0.04},
         {0, 1}};
    B = MatrixUtils::Identity(F.size());
    P = MatrixUtils::Identity(F.size());
    x = {0.0, 0.0}; 
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
        x = MatrixUtils::Add(x, MatrixUtils::Mult(B, v));
    }

    P = MatrixUtils::Add(MatrixUtils::Mult(MatrixUtils::Mult(F, P), MatrixUtils::Transpose(F)), Q);

    return x;
}

int KalmanFilter::Update(const Vector &z)
{
    Vector y = MatrixUtils::Sub(z, MatrixUtils::Mult(H, x));
    Matrix PHt = MatrixUtils::Mult(P, MatrixUtils::Transpose(H));
    Matrix S = MatrixUtils::Add(R, MatrixUtils::Mult(H, PHt));

    Matrix *result = nullptr;
    MatrixUtils::Inv(S, &result);
    if (result == nullptr)
    {
        return -1;
    }
    // Kalman Gain
    Matrix K = MatrixUtils::Mult(PHt, *result);

    x = MatrixUtils::Add(x, MatrixUtils::Mult(K, y));

    Matrix I = MatrixUtils::Identity(F.size());

    Matrix eq1 = MatrixUtils::Sub(I, MatrixUtils::Mult(K, H));
    Matrix eq2 = MatrixUtils::Transpose(eq1);
    Matrix eq3 = MatrixUtils::Mult(MatrixUtils::Mult(K, R),
                                   MatrixUtils::Transpose(K));
    
    Matrix A = MatrixUtils::Mult(eq1, P);
    Matrix B = MatrixUtils::Add(eq2, eq3);

    P = MatrixUtils::Mult(A, B);

    return 0;
}
