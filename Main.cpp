#include <iostream>

#include "Matrix.h"
#include "Quaternion.h"

int main()
{
	Quaternion q1{ 1,2,3,4 };
	Quaternion q2(q1);

	std::cout << q1+q1-2*q1 + q1<< std::endl;
	
}