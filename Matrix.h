#pragma once

#include <initializer_list>
#include <stdint.h>

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

	double& operator()(size_t i, size_t j);
	Matrix& operator+=(const Matrix& m);
	Matrix& operator+=(double scalar);
	Matrix operator+(const Matrix& m);
	Matrix operator+(double scalar);
	~Matrix();
};

