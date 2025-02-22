#include "Matrix.h"

std::ostream& operator<<(std::ostream& os, const Matrix& m)
{
	os << std::showpoint;
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			os << *(m.data + i * m.cols + j) << " ";
		}
		std::cout << "\n";
	}
	os << std::noshowpoint;
	return os;
}

Matrix operator+(double scalar, const Matrix& m) noexcept
{
	if (m.data == nullptr) return Matrix(1, 1, { scalar });
	Matrix out(m.rows, m.cols);
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			*(out.data + i * out.cols + j) = scalar + *(m.data + i * m.cols + j);
		}
	}
	return out;
}

Matrix operator-(double scalar, const Matrix& m) noexcept
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

Matrix operator*(double scalar, const Matrix& m) noexcept
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

Matrix operator/(double scalar, const Matrix& m) noexcept
{
	if (m.data == nullptr) return Matrix();
	return scalar * m.inv();
}

Matrix abs(const Matrix& m) noexcept
{
	Matrix out(m.rows, m.cols);
	for (size_t i = 0; i < m.rows; i++)
	{
		for (size_t j = 0; j < m.cols; j++)
		{
			*(out.data + i * out.cols + j) = abs(*(m.data + i * m.cols + j));
		}
	}
	return out;
}
