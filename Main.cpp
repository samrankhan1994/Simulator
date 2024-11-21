#include <iostream>

#include "Matrix.h"

int main()
{
	Matrix m1(1, 3, { 1,2,3 });
	Matrix m2(3, 1, { 1,2,3 });
	m1 = Matrix::skew(m2) * m2;
	std::cout << m1;
}