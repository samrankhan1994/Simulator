#include <iostream>

#include "Matrix.h"

int main()
{
	Matrix m1(2, 2, { 1,2, 3, 4 });
	Matrix m2(m1);
	m2 /= m1;
	std::cout << m2 << std::endl;
}