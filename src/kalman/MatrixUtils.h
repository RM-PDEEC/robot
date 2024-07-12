#ifndef MATRIXUTILS_H
#define MATRIXUTILS_H

#include <vector>
#include <stdexcept>
#include <iostream>

using namespace std;

#define INVALID_MATRIX      22

typedef vector<double> Vector;
typedef vector<Vector> Matrix;

class MatrixUtils
{
public:
    static Matrix Add(const Matrix &A, const Matrix &B);
    static Matrix Sub(const Matrix &A, const Matrix &B);
    static Matrix Transpose(const Matrix &A);
    static int Inv(const Matrix &A, Matrix &result);
    static void Inv(const Matrix &A, Matrix **result);
    static Matrix Identity(int n);
    static Matrix Mult(const Matrix &A, const Matrix &B);
    static vector<double> Mult(const Matrix &A, const Vector &B);
    static Matrix OuterProduct(const Vector &A, const Vector &B);
    static void Print(const Matrix &A);
    static void Print(const Vector &A);
};

#endif // MATRIXUTILS_H
