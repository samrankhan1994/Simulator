#include <iostream>
#include <fstream>

#include "Matrix.h"
#include "Quaternion.h"
#include "HAStar.h"
#include "Vector3D.h"


int main()
{
	std::ofstream file("path.txt");
	double pi = 4.0 * atan(1.0);
	BinaryOccupancyMap map({ -400, 400 }, { -400, 400 }, 1.0);
	map.setOccupancy(Point(200, 0), true);
	map.setOccupancy(Point(200, 1), true);
	map.setOccupancy(Point(200, 2), true);
	map.setOccupancy(Point(200, 3), true);
	map.setOccupancy(Point(200, 4), true);
	map.setOccupancy(Point(200, 5), true);
	map.setOccupancy(Point(200, 6), true);
	map.setOccupancy(Point(200, 7), true);
	map.setOccupancy(Point(200, 8), true);
	map.setOccupancy(Point(200, 9), true);
	

	StateSpace stateSpace({ {-400, -400.0, -pi},{400, 400, pi} });
	ValidatorOccupancyMap validator(stateSpace, map, 1.0);
	HAStar planner(validator, 8.0);

	auto[found, numNodeExplored, numIterations, exitFlag] = planner.plan({0,0,0}, {30, 0.0, 0.0});
	std::cout << std::boolalpha << found <<", " << numNodeExplored << ", " << numIterations << ", " << exitFlag << std::endl;

	std::tie(found, numNodeExplored, numIterations, exitFlag) = planner.plan({ 30, 0.0, 0.0 }, { 46, 10, 0.0 });
	std::cout << std::boolalpha << found << ", " << numNodeExplored << ", " << numIterations << ", " << exitFlag << std::endl;

	std::tie(found, numNodeExplored, numIterations, exitFlag) = planner.plan({ 46, 10, 0.0 }, { 250, 0.0, 0.0 });
	std::cout << std::boolalpha << found << ", " << numNodeExplored << ", " << numIterations << ", " << exitFlag << std::endl;

	auto& path = planner.getPath();
	for (auto& i : path.getStates())
	{
		auto &[x, y, theta] = i;
		file << x << "\t" << y << "\t" << theta << "\r\n";
	}
	file.close();
	

}