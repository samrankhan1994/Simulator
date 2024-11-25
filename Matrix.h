#pragma once

#include <iostream>
#include <initializer_list>

class Matrix
{
	constexpr static inline double TOL = 1.0e-6;
	size_t rows;
	size_t cols;
	double* data;
	double* allocate(size_t size) noexcept;
	void rowSwap(size_t r1, size_t r2) noexcept;
	void rowOp(size_t r1, size_t r2, double factor) noexcept;				//useful for R1 = R1 - factor * R2 
	void rowScaling(size_t r, double factor) noexcept;						//useful for R1 = factor * R1
	std::tuple<bool, Matrix, Matrix, size_t> LUDecompose() const noexcept;	// output(isDecomposed, Lower, Upper, numOfSwaps)
	
public:
	Matrix() noexcept;
	Matrix(size_t rows, size_t cols) noexcept;
	Matrix(size_t rows, size_t cols, std::initializer_list<double> list) noexcept;
	Matrix(std::initializer_list<std::initializer_list<double>> list) noexcept;

	Matrix(const Matrix& m) noexcept;
	Matrix(Matrix&& m) noexcept;
	Matrix& operator=(const Matrix& m) noexcept;
	Matrix& operator=(Matrix&& m) noexcept;

	double& operator()(size_t i, size_t j) noexcept;
	double operator()(size_t i, size_t j) const noexcept;
	Matrix operator~() noexcept;

	Matrix& operator+=(const Matrix& m) noexcept;
	Matrix& operator+=(double scalar) noexcept;
	Matrix operator+(const Matrix& m) const noexcept;
	Matrix operator+(double scalar) const noexcept;

	Matrix& operator-=(const Matrix& m) noexcept;
	Matrix& operator-=(double scalar) noexcept;
	Matrix operator-(const Matrix& m) const noexcept;
	Matrix operator-(double scalar) const noexcept;

	Matrix& operator*=(const Matrix& m) noexcept;
	Matrix& operator*=(double scalar) noexcept;
	Matrix operator*(const Matrix& m) const noexcept;
	Matrix operator*(double scalar) const noexcept;

	Matrix& operator/=(const Matrix& m) noexcept;
	Matrix& operator/=(double scalar) noexcept;
	Matrix operator/(const Matrix& m) const noexcept;
	Matrix operator/(double scalar) const noexcept;

	double det() const noexcept;
	Matrix inv() const noexcept;
	static Matrix eye(size_t dim) noexcept;
	static Matrix ones(size_t rows, size_t cols) noexcept;
	static Matrix zeros(size_t rows, size_t cols) noexcept;
	static Matrix skew(const Matrix& m) noexcept;

	~Matrix() noexcept;

	friend std::ostream& operator<<(std::ostream& os, const Matrix& m);
	friend Matrix operator+(double scalar, const Matrix& m) noexcept;
	friend Matrix operator-(double scalar, const Matrix& m) noexcept;
	friend Matrix operator*(double scalar, const Matrix& m) noexcept;
	friend Matrix operator/(double scalar, const Matrix& m) noexcept;
	friend Matrix abs(const Matrix& m) noexcept;
};

