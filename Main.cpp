#include <iostream>

#include "Matrix.h"
#include "Quaternion.h"
#include "Dubins.h"
#include "BinaryOccupancyMap.h"

int main()
{
	BinaryOccupancyMap m({ 0, 20 }, { 0, 40 }, 1.0);
	m.setOccupancy(Point(0, 0), true);
	m.setOccupancy(Point(10, 10), true);
	m.inflate(2);
	m.print();
}