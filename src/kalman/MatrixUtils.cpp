#include "MatrixUtils.h"

Matrix MatrixUtils::Add(const Matrix &A, const Matrix &B)
{
    size_t n = A.size(), m = A[0].size();

    Matrix C(n, Vector(m, 0));

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = 0; j < m; ++j)
        {
            C[i][j] = A[i][j] + B[i][j];
        }
    }

    return C;
}

Matrix MatrixUtils::Sub(const Matrix &A, const Matrix &B)
{
    size_t n = A.size(), m = A[0].size();

    Matrix C(n, Vector(m, 0));

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = 0; j < m; ++j)
        {
            C[i][j] = A[i][j] - B[i][j];
        }
    }

    return C;
}

Matrix MatrixUtils::Transpose(const Matrix &A)
{
    size_t n = A.size(), m = A[0].size();

    Matrix C(m, Vector(n, 0));

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = 0; j < m; ++j)
        {
            C[j][i] = A[i][j];
        }
    }

    return C;
}

Matrix MatrixUtils::Identity(int n)
{
    Matrix I(n, Vector(n, 0));

    for (int i = 0; i < n; ++i)
    {
        I[i][i] = 1;
    }

    return I;
}

Matrix MatrixUtils::Mult(const Matrix &A, const Matrix &B)
{
    size_t n = A.size(), m = B[0].size(), p = B.size();

    Matrix C(n, Vector(m, 0));

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = 0; j < m; ++j)
        {
            for (size_t k = 0; k < p; ++k)
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return C;
}

Vector MatrixUtils::Mult(const Matrix &A, const Vector &B)                        
{
    size_t n = A.size(), m = B.size();

    Vector b(n, 0);

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = 0; j < m; ++j)
        {
            b[i] += A[i][j] * B[j];
        }
    }

    return b;
}

Matrix MatrixUtils::OuterProduct(const Vector &a, const Vector &b)
{
    size_t n = a.size(), m = b.size();

    Matrix C(n, Vector(m, 0));

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = 0; j < m; ++j)
        {
            C[i][j] = a[i] * b[j];
        }
    }

    return C;
}

int MatrixUtils::Inv(const Matrix &A, Matrix &result)
{
    int status = 0;

    if (A.size() == 1 && A[0].size() == 1)
    {
        if (A[0][0] == 0)
        {
            status = -INVALID_MATRIX;
            goto exit;
        }

        result = {{1 / A[0][0]}};
    }
    else if (A.size() == 2 && A[0].size() == 2)
    {
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (det == 0)
        {
            status = -INVALID_MATRIX;
            goto exit;
        }

        double inv_det = 1.0 / det;

        result = {{A[1][1] * inv_det, -A[0][1] * inv_det},
                {-A[1][0] * inv_det, A[0][0] * inv_det}};
    }

exit:
    return status;
}

void MatrixUtils::Inv(const Matrix &A, Matrix **result)
{
    if (result == nullptr)
    {
        return;
    }

    if (A.size() == 1 && A[0].size() == 1)
    {
        if (A[0][0] == 0)
        {
            return;
        }

        *result = new Matrix({
            {1 / A[0][0]}
        });
    }
    else if (A.size() == 2 && A[0].size() == 2)
    {
        double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        if (det == 0)
        {
            return;
        }

        double inv_det = 1.0 / det;

        *result = new Matrix({
            {A[1][1] * inv_det, -A[0][1] * inv_det},
            {-A[1][0] * inv_det, A[0][0] * inv_det}
        });
    }
}

void MatrixUtils::Print(const Matrix &A)
{
    for (const auto &row : A)
    {
        for (const auto &elem : row)
        {
            cout << elem << " ";
        }
        cout << std::endl;
    }
}

void MatrixUtils::Print(const Vector &A)
{
    for (const auto &elem : A)
    {
        cout << elem << " ";
    }
    cout << std::endl;
}
