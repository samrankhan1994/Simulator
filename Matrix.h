#pragma once

#include <iostream>
#include <initializer_list>
#include <compare>

class Matrix
{
	size_t rows;
	size_t cols;
	double* data;
	double* allocate(size_t size);
	void rowSwap(size_t r1, size_t r2);
	void rowOp(size_t r1, size_t r2, double factor); //useful for R1 = R1 - factor * R2 

public:
	Matrix();
	Matrix(size_t rows, size_t cols);
	Matrix(size_t rows, size_t cols, std::initializer_list<double> list);
	Matrix(std::initializer_list<std::initializer_list<double>> list);

	Matrix(const Matrix& m);
	Matrix(Matrix&& m) noexcept;
	Matrix& operator=(const Matrix& m);
	Matrix& operator=(Matrix&& m) noexcept;

	double& operator()(size_t i, size_t j);
	double operator()(size_t i, size_t j) const;
	Matrix operator~();

	Matrix& operator+=(const Matrix& m);
	Matrix& operator+=(double scalar);
	Matrix operator+(const Matrix& m) const;
	Matrix operator+(double scalar) const;

	Matrix& operator-=(const Matrix& m);
	Matrix& operator-=(double scalar);
	Matrix operator-(const Matrix& m) const;
	Matrix operator-(double scalar) const;

	Matrix& operator*=(const Matrix& m);
	Matrix& operator*=(double scalar);
	Matrix operator*(const Matrix& m) const;
	Matrix operator*(double scalar) const;

	Matrix& operator/=(const Matrix& m);
	Matrix& operator/=(double scalar);
	Matrix operator/(const Matrix& m) const;
	Matrix operator/(double scalar) const;

	double det() const;
	Matrix inv() const;
	static Matrix eye(size_t dim);
	static Matrix ones(size_t rows, size_t cols);
	static Matrix zeros(size_t rows, size_t cols);
	static Matrix skew(const Matrix& m);

	~Matrix();

	friend std::ostream& operator<<(std::ostream& os, const Matrix& m);
	friend Matrix operator+(double scalar, const Matrix& m);
	friend Matrix operator-(double scalar, const Matrix& m);
	friend Matrix operator*(double scalar, const Matrix& m);
	friend Matrix operator/(double scalar, const Matrix& m);
	friend Matrix abs(const Matrix& m);
};

