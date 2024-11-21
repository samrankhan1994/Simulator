#pragma once

#include <iostream>
#include <initializer_list>

class Matrix
{
	size_t rows;
	size_t cols;
	double* data;
	double* allocate(size_t size);
public:
	Matrix();
	Matrix(size_t rows, size_t cols);
	Matrix(size_t rows, size_t cols, std::initializer_list<double> list);
	Matrix(std::initializer_list<std::initializer_list<double>> list);

	Matrix(const Matrix& m);
	Matrix(Matrix&& m) noexcept;
	Matrix& operator=(const Matrix& m);
	Matrix& operator=(Matrix&& m) noexcept;

	double& operator()(const size_t i, const size_t j);
	Matrix operator~();

	Matrix& operator+=(const Matrix& m);
	Matrix& operator+=(double scalar);
	Matrix operator+(const Matrix& m);
	Matrix operator+(double scalar);

	Matrix& operator-=(const Matrix& m);
	Matrix& operator-=(double scalar);
	Matrix operator-(const Matrix& m);
	Matrix operator-(double scalar);

	Matrix& operator*=(const Matrix& m);
	Matrix& operator*=(double scalar);
	Matrix operator*(const Matrix& m);
	Matrix operator*(double scalar);

	//division operators

	static Matrix eye(size_t dim);
	static Matrix ones(size_t rows, size_t cols);
	static Matrix zeros(size_t rows, size_t cols);
	static Matrix skew(const Matrix& m);


	~Matrix();

	friend std::ostream& operator<<(std::ostream& os, const Matrix& m);
	friend Matrix operator+(double scalar, const Matrix& m);
	friend Matrix operator-(double scalar, const Matrix& m);
	friend Matrix operator*(double scalar, const Matrix& m);
};

