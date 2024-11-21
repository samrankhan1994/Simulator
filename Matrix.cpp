#include "Matrix.h"

#include <iostream>
#include <exception>
#include <memory>
#include <cassert>


double* Matrix::allocate(size_t size)
{
	double* ptr = nullptr;
	if (size > 0)
	{
		ptr = new(std::nothrow) double[size];
		assert(ptr != nullptr);
	}
	return ptr;
}

Matrix::Matrix(): rows(0), cols(0), data(nullptr) {}

Matrix::Matrix(size_t _rows, size_t _cols): rows(_rows), cols(_cols)
{
	data = allocate(rows * cols);
}

Matrix::Matrix(size_t _rows, size_t _cols, std::initializer_list<double> list): rows(_rows), cols(_cols)
{
	assert(rows * cols == list.size());
	data = allocate(rows * cols);
	int i = 0;
	for (auto& e : list)
	{
		*(data + i) = e;
		i++;
	}
}

Matrix::Matrix(std::initializer_list<std::initializer_list<double>> list)
{
	rows = list.size();
	cols = list.begin()->size();
	data = allocate(rows * cols);
	size_t i = 0;
	for (auto& row : list)
	{
		assert(cols == row.size());
		for (auto& e : row)
		{
			*(data + i) = e;
			i++;
		}
	}
}

Matrix::Matrix(const Matrix& m)
{
	rows = m.rows;
	cols = m.cols;
	data = allocate(rows * cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) = *(m.data + i * cols + j);
		}
	}
}

Matrix::Matrix(Matrix&& m) noexcept
{
	rows = m.rows;
	cols = m.cols;
	data = m.data;
	m.rows = m.cols = 0;
	m.data = nullptr;
}

Matrix& Matrix::operator=(const Matrix& m)
{
	if (data != nullptr) delete[] data;
	rows = m.rows;
	cols = m.cols;
	data = allocate(rows * cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) = *(m.data + i * cols + j);
		}
	}
	return *this;
}

Matrix& Matrix::operator=(Matrix&& m) noexcept
{
	if (data != nullptr) delete[] data;
	rows = m.rows;
	cols = m.cols;
	data = m.data;
	m.rows = m.cols = 0;
	m.data = nullptr;
	return *this;
}

double& Matrix::operator()(size_t i, size_t j)
{
	return *(data + i * cols + rows);
}

Matrix& Matrix::operator+=(const Matrix& m)
{
	assert(rows == m.rows && cols == m.cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) += *(m.data + i * cols + j);
		}
	}
	return *this;
}

Matrix& Matrix::operator+=(double scalar)
{
	if (data == nullptr)
	{
		rows = cols = 1;
		data = allocate(1);
		*data = scalar;
		return *this;
	}
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) += scalar;
		}
	}
	return *this;
}

Matrix Matrix::operator+(const Matrix& m)
{
	assert(rows == m.rows && cols == m.cols);
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * cols + j) = *(data + i * cols + j) + *(m.data + i * cols + j);
		}
	}
	return out;
}

Matrix Matrix::operator+(double scalar)
{
	if (data == nullptr) return Matrix(1, 1, { scalar });
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * cols + j) = *(data + i * cols + j) + scalar;
		}
	}
	return out;
}

Matrix::~Matrix()
{
	if (data != nullptr) delete[] data;
}


