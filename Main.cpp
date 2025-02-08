#include <iostream>

#include "Matrix.h"
#include "Quaternion.h"
#include "Dubins.h"

int main()
{
	Dubins d;
	Matrix c1{ 0,0,0 };
	Matrix c2{ 4, 4, 0.0*atan(1.0)};

	try {
		DubinsPathSegment s = d.connect(c1, c2, 4);
		const Matrix& ll = s.interpolate(s.pathLength);
		std::cout << ll << std::endl;
	}
	catch (DubinsException e)
	{
		std::cout << e.what() << std::endl;
	}
	
	
	
}