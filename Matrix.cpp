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
		try {
			ptr = new double[size] {0};
		}
		catch (std::bad_alloc ba) {
			std::cerr << "Matrix Memory allocation failed" << std::endl;
			exit(EXIT_FAILURE);
		}
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

double& Matrix::operator()(const size_t i, const size_t j)
{
	return *(data + i * cols + j);
}

Matrix Matrix::operator~()
{
	Matrix out(cols, rows);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + j * out.cols + i) = *(data + i * cols + j);
		}
	}
	return out;
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

Matrix& Matrix::operator-=(const Matrix& m)
{
	assert(rows == m.rows && cols == m.cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) -= *(m.data + i * cols + j);
		}
	}
	return *this;
}

Matrix& Matrix::operator-=(double scalar)
{
	if (data == nullptr)
	{
		rows = cols = 1;
		data = allocate(1);
		*data = -scalar;
		return *this;
	}
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) -= scalar;
		}
	}
	return *this;
}

Matrix Matrix::operator-(const Matrix& m)
{
	assert(rows == m.rows && cols == m.cols);
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * cols + j) = *(data + i * cols + j) - *(m.data + i * cols + j);
		}
	}
	return out;
}

Matrix Matrix::operator-(double scalar)
{
	if (data == nullptr) return Matrix(1, 1, { -scalar });
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * cols + j) = *(data + i * cols + j) - scalar;
		}
	}
	return out;
}

Matrix& Matrix::operator*=(const Matrix& m)
{
	assert(cols == m.rows);
	Matrix out(rows, m.cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			double sum = 0.0;
			for (size_t k = 0; k < cols; k++)
			{
				sum += *(data + i * cols + k) * *(m.data + k * m.cols + j);
			}
			*(out.data + i * out.cols + j) = sum;
		}
	}
	*this = std::move(out);
	return *this;
}

Matrix& Matrix::operator*=(double scalar)
{
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) *= scalar;
		}
	}
	return *this;
}

Matrix Matrix::operator*(const Matrix& m)
{
	assert(cols == m.rows);
	Matrix out(rows, m.cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			double sum = 0.0;
			for (size_t k = 0; k < cols; k++)
			{
				sum += *(data + i * cols + k) * *(m.data + k * m.cols + j);
			}
			*(out.data + i * out.cols + j) = sum;
		}
	}
	return out;
}

Matrix Matrix::operator*(double scalar)
{
	if (data == nullptr) return Matrix();
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * cols + j) = *(data + i * cols + j) * scalar;
		}
	}
	return out;
}

Matrix Matrix::eye(size_t dim)
{
	assert(dim > 0);
	Matrix out(dim, dim);
	for (size_t i = 0; i < dim; i++)
	{
		for (size_t j = 0; j < dim; j++)
		{
			*(out.data + i * out.cols + j) = (i == j) ? 1.0 : 0.0;
		}
	}
	return out;
}

Matrix Matrix::ones(size_t rows, size_t cols)
{
	assert(rows * cols > 0);
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * out.cols + j) = 1.0;
		}
	}
	return out;
}

Matrix Matrix::zeros(size_t rows, size_t cols)
{
	assert(rows * cols > 0);
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * out.cols + j) = 0.0;
		}
	}
	return out;
}

Matrix Matrix::skew(const Matrix& m)
{
	assert(m.rows == 3 && m.cols == 1);
	Matrix out(3, 3);
	*(out.data + 0) = 0.0; *(out.data + 1) = - *(m.data + 2); *(out.data + 2) = *(m.data + 1);
	*(out.data + 3) = *(m.data + 2); *(out.data + 4) = 0.0; *(out.data + 5) = - *(m.data + 0);
	*(out.data + 6) = -*(m.data + 1); *(out.data + 7) = *(m.data + 0); *(out.data + 8) = 0.0;
	return out;
}

Matrix::~Matrix()
{
	if (data != nullptr) delete[] data;
}

std::ostream& operator<<(std::ostream& os, const Matrix& m)
{
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			os << *(m.data + i * m.cols + j) << " ";
		}
		std::cout << "\n";
	}
	return os;
}

Matrix operator+(double scalar, const Matrix& m)
{
	if (m.data == nullptr) return Matrix(1, 1, { scalar });
	Matrix out(m.rows, m.cols);
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			*(out.data + i * out.cols + j) = scalar  + *(m.data + i * m.cols + j);
		}
	}
	return out;
}

Matrix operator-(double scalar, const Matrix& m)
{
	if (m.data == nullptr) return Matrix(1, 1, { scalar });
	Matrix out(m.rows, m.cols);
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			*(out.data + i * out.cols + j) = scalar - *(m.data + i * m.cols + j);
		}
	}
	return out;
}

Matrix operator*(double scalar, const Matrix& m)
{
	if (m.data == nullptr) return Matrix();
	Matrix out(m.rows, m.cols);
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			*(out.data + i * out.cols + j) = scalar * *(m.data + i * m.cols + j);
		}
	}
	return out;
}
