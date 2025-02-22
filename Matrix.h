#pragma once

#include <iostream>
#include <initializer_list>
#include <memory>
#include <cassert>

class Matrix
{
	constexpr inline static double TOL = 1.0e-6;
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
	explicit Matrix(size_t rows, size_t cols) noexcept;
	Matrix(std::initializer_list<double> list);
	explicit Matrix(size_t rows, size_t cols, std::initializer_list<double> list) noexcept;
	explicit Matrix(std::initializer_list<std::initializer_list<double>> list) noexcept;

	Matrix(const Matrix& m) noexcept;
	Matrix(Matrix&& m) noexcept;
	Matrix& operator=(const Matrix& m) noexcept;
	Matrix& operator=(Matrix&& m) noexcept;

	double& operator()(size_t i, size_t j) noexcept;
	double operator()(size_t i, size_t j) const noexcept;
	Matrix operator~() const noexcept;

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
	std::pair<size_t, size_t> getDimension() const;
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

std::ostream& operator<<(std::ostream& os, const Matrix& m);
Matrix operator+(double scalar, const Matrix& m) noexcept;
Matrix operator-(double scalar, const Matrix& m) noexcept;
Matrix operator*(double scalar, const Matrix& m) noexcept;
Matrix operator/(double scalar, const Matrix& m) noexcept;
Matrix abs(const Matrix& m) noexcept;


inline double* Matrix::allocate(size_t size) noexcept
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

inline void Matrix::rowSwap(size_t r1, size_t r2) noexcept
{
	for (size_t i = 0; i < cols; i++)
	{
		double temp = *(data + r1 * cols + i);
		*(data + r1 * cols + i) = *(data + r2 * cols + i);
		*(data + r2 * cols + i) = temp;
	}
}

inline void Matrix::rowOp(size_t r1, size_t r2, double factor) noexcept
{
	for (size_t i = 0; i < cols; i++)
	{
		*(data + r1 * cols + i) = *(data + r1 * cols + i) - *(data + r2 * cols + i) * factor;
	}
}

inline void Matrix::rowScaling(size_t r, double factor) noexcept
{
	for (size_t i = 0; i < cols; i++)
	{
		*(data + r * cols + i) = *(data + r * cols + i) * factor;
	}
}

inline std::tuple<bool, Matrix, Matrix, size_t> Matrix::LUDecompose() const noexcept
{
	assert(data != nullptr && rows == cols);
	Matrix L = Matrix::eye(rows), U(*this);
	size_t numSwaps = 0;
	double absValue;
	for (size_t j = 0; j < cols - 1; j++)
	{
		double max = 0;
		size_t maxRowIndex = j;
		for (size_t i = j; i < rows; i++)
		{
			if ((absValue = abs(*(U.data + i * U.cols + j))) > max)
			{
				max = absValue;
				maxRowIndex = i;
			}
		}
		if (max < TOL) return { false, Matrix(), Matrix(), numSwaps };
		if (maxRowIndex != j)
		{
			U.rowSwap(j, maxRowIndex);
			L.rowSwap(j, maxRowIndex);
			numSwaps++;
		}
		for (size_t i = j + 1; i < rows; i++)
		{
			double factor = *(U.data + i * U.cols + j) / *(U.data + j * U.cols + j);
			U.rowOp(i, j, factor);
			L.rowOp(i, j, factor);
		}
	}
	return { true, std::move(L), std::move(U), numSwaps };
}

inline Matrix::Matrix() noexcept : rows(0), cols(0), data(nullptr) {}

inline Matrix::Matrix(size_t _rows, size_t _cols) noexcept : rows(_rows), cols(_cols)
{
	data = allocate(rows * cols);
}

inline Matrix::Matrix(std::initializer_list<double> list)
{
	data = allocate(list.size());
	rows = list.size();
	cols = 1;
	int i = 0;
	for (auto& e : list)
	{
		*(data + i) = e;
		i++;
	}
}

inline Matrix::Matrix(size_t _rows, size_t _cols, std::initializer_list<double> list) noexcept : rows(_rows), cols(_cols)
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

inline Matrix::Matrix(std::initializer_list<std::initializer_list<double>> list) noexcept
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

inline Matrix::Matrix(const Matrix& m) noexcept
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

inline Matrix::Matrix(Matrix&& m) noexcept
{
	rows = m.rows;
	cols = m.cols;
	data = m.data;
	m.rows = m.cols = 0;
	m.data = nullptr;
}

inline Matrix& Matrix::operator=(const Matrix& m) noexcept
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

inline Matrix& Matrix::operator=(Matrix&& m) noexcept
{
	if (data != nullptr) delete[] data;
	rows = m.rows;
	cols = m.cols;
	data = m.data;
	m.rows = m.cols = 0;
	m.data = nullptr;
	return *this;
}

inline double& Matrix::operator()(size_t i, size_t j) noexcept
{
	return *(data + i * cols + j);
}

inline double Matrix::operator()(size_t i, size_t j) const noexcept
{
	return *(data + i * cols + j);
}


inline Matrix Matrix::operator~() const noexcept
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

inline Matrix& Matrix::operator+=(const Matrix& m) noexcept
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

inline Matrix& Matrix::operator+=(double scalar) noexcept
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

inline Matrix Matrix::operator+(const Matrix& m) const noexcept
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

inline Matrix Matrix::operator+(double scalar) const noexcept
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

inline Matrix& Matrix::operator-=(const Matrix& m) noexcept
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

inline Matrix& Matrix::operator-=(double scalar) noexcept
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

inline Matrix Matrix::operator-(const Matrix& m) const noexcept
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

inline Matrix Matrix::operator-(double scalar) const noexcept
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

inline Matrix& Matrix::operator*=(const Matrix& m) noexcept
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

inline Matrix& Matrix::operator*=(double scalar) noexcept
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

inline Matrix Matrix::operator*(const Matrix& m) const noexcept
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

inline Matrix Matrix::operator*(double scalar) const noexcept
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

inline Matrix& Matrix::operator/=(const Matrix& m) noexcept
{
	assert(cols == m.rows && m.cols == m.rows);
	Matrix mInv = m.inv();
	*this = *this * mInv;
	return *this;
}

inline Matrix& Matrix::operator/=(double scalar) noexcept
{
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(data + i * cols + j) /= scalar;
		}
	}
	return *this;
}

inline Matrix Matrix::operator/(const Matrix& m) const noexcept
{
	assert(cols == m.rows && m.cols == m.rows);
	Matrix mInv = m.inv();
	return *this * mInv;
}

inline Matrix Matrix::operator/(double scalar) const noexcept
{
	if (data == nullptr) return Matrix();
	Matrix out(rows, cols);
	for (size_t i = 0; i < rows; i++)
	{
		for (size_t j = 0; j < cols; j++)
		{
			*(out.data + i * cols + j) = *(data + i * cols + j) / scalar;
		}
	}
	return out;
}

inline double Matrix::det() const noexcept
{
	auto [ok, L, U, numSwaps] = LUDecompose();
	if (ok)
	{
		double ans = 1.0;
		for (size_t i = 0; i < rows; i++)
		{
			ans *= *(U.data + i * cols + i);
		}
		return numSwaps % 2 ? -ans : ans;
	}
	else
		return 0.0;
}

inline Matrix Matrix::inv() const noexcept
{
	auto [ok, L, U, numSwaps] = LUDecompose();
	assert(ok);
	assert(abs(*(U.data + rows * cols - 1)) > TOL);

	for (size_t j = cols - 1; j > 0; j--)
	{
		for (int i = (int)j - 1; i >= 0; i--)
		{
			double factor = *(U.data + i * U.cols + j) / *(U.data + j * U.cols + j);
			U.rowOp(i, j, factor);
			L.rowOp(i, j, factor);
		}
	}
	for (size_t i = 0; i < rows; i++)
	{
		double factor = 1 / *(U.data + i * U.cols + i);
		L.rowScaling(i, factor);
	}
	return L;
}

inline std::pair<size_t, size_t> Matrix::getDimension() const
{
	return std::pair<size_t, size_t>(rows, cols);
}

inline Matrix Matrix::eye(size_t dim) noexcept
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

inline Matrix Matrix::ones(size_t rows, size_t cols) noexcept
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

inline Matrix Matrix::zeros(size_t rows, size_t cols) noexcept
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

inline Matrix Matrix::skew(const Matrix& m) noexcept
{
	assert(m.rows == 3 && m.cols == 1);
	Matrix out(3, 3);
	*(out.data + 0) = 0.0; *(out.data + 1) = -*(m.data + 2); *(out.data + 2) = *(m.data + 1);
	*(out.data + 3) = *(m.data + 2); *(out.data + 4) = 0.0; *(out.data + 5) = -*(m.data + 0);
	*(out.data + 6) = -*(m.data + 1); *(out.data + 7) = *(m.data + 0); *(out.data + 8) = 0.0;
	return out;
}

inline Matrix::~Matrix() noexcept
{
	if (data != nullptr) delete[] data;
}