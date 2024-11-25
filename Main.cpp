#include <iostream>

#include "Matrix.h"
#include "Quaternion.h"

int main()
{
	Matrix m{ {5} };
	std::cout << m.inv() << std::endl;
	
}