#pragma once
#include <iostream>
#include <cassert>
#include <vector>
#include <queue>

typedef std::pair<size_t, size_t> Index;
typedef std::pair<double, double> Point;

class BinaryOccupancyMap
{
	Index gridSize;
	double resolution;
	Point xLimit;
	Point yLimit;
	bool* map;
public:
	BinaryOccupancyMap();
	BinaryOccupancyMap(const Point& xLimit, const Point& yLimit, const double& resolution);
	BinaryOccupancyMap(const BinaryOccupancyMap& ob);
	BinaryOccupancyMap(BinaryOccupancyMap&& ob) noexcept;
	BinaryOccupancyMap& operator=(const BinaryOccupancyMap& ob);
	BinaryOccupancyMap& operator=(BinaryOccupancyMap&& ob) noexcept;

	//getters
	const Index& getGridSize() const noexcept;
	const double& getResolution() const noexcept;
	const Point& getXLimit() const noexcept;
	const Point& getYLimit() const noexcept;

	
	//other functionality
	void setOccupancy(const Point& xy, bool val) noexcept;
	void setOccupancy(const Index& ij, bool val) noexcept;
	void setOccupancy(const Point& xy, bool val, const double& radius) noexcept;
	Point grid2World(const Index& ij) const noexcept;
	Index world2Grid(const Point& xy) const noexcept;
	bool checkOccupancy(const Point& xy) const noexcept;
	bool checkOccupancy(const Index& ij) const noexcept;
	bool& getOccupancy(const Point& xy) noexcept;
	bool& getOccupancy(const Index& ij) noexcept;
	void print() const noexcept;
	bool valid(const Point& xy) const noexcept;
	bool valid(const Index& ij) const noexcept;
	void inflate(const double& radius) noexcept;
	double distance(const Point& xy1, const Point& xy2) const noexcept;
	double distance(const Index& ij1, const Index& ij2) const noexcept;
	~BinaryOccupancyMap();
};

inline double BinaryOccupancyMap::distance(const Point& xy1, const Point& xy2) const noexcept
{
	return sqrt((xy2.first - xy1.first) * (xy2.first - xy1.first) + (xy2.second - xy1.second) * (xy2.second - xy1.second));
}

inline double BinaryOccupancyMap::distance(const Index& ij1, const Index& ij2) const noexcept
{
	return distance(grid2World(ij1), grid2World(ij2));
}

inline BinaryOccupancyMap::BinaryOccupancyMap(): BinaryOccupancyMap({0, 10}, {0, 10}, 1)
{
}

inline BinaryOccupancyMap::BinaryOccupancyMap(const Point& xLimit, const Point& yLimit, const double& resolution):
	xLimit(xLimit), yLimit(yLimit), resolution(resolution), map(nullptr)
{
	size_t iSize = static_cast<size_t>(std::floor(xLimit.second - xLimit.first) / resolution + 1);
	size_t jSize = static_cast<size_t>(std::floor(yLimit.second - yLimit.first) / resolution + 1);
	try {
		map = new bool[iSize * jSize] {0};
		gridSize = { iSize, jSize };
	}
	catch (std::bad_alloc e)
	{
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
}

inline BinaryOccupancyMap::BinaryOccupancyMap(const BinaryOccupancyMap& ob):
	xLimit(ob.xLimit), yLimit(ob.yLimit), resolution(ob.resolution), gridSize(ob.gridSize), map(nullptr)
{
	try {
		map = new bool[gridSize.first * gridSize.second] {0};
	}
	catch (std::bad_alloc e)
	{
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
	for (size_t i = 0; i < gridSize.first; i++)
	{
		for (size_t j = 0; j < gridSize.second; j++)
		{
			*(map + i * gridSize.second + j) = *(ob.map + i * gridSize.second + j);
		}
	}
}

inline BinaryOccupancyMap::BinaryOccupancyMap(BinaryOccupancyMap&& ob) noexcept:
	xLimit(ob.xLimit), yLimit(ob.yLimit), resolution(ob.resolution), gridSize(ob.gridSize), map(map)
{
	free(ob.map);
}

inline BinaryOccupancyMap& BinaryOccupancyMap::operator=(const BinaryOccupancyMap& ob)
{
	gridSize = ob.gridSize;
	resolution = ob.resolution;
	xLimit = ob.xLimit;
	yLimit = ob.yLimit;

	try {
		map = new bool[gridSize.first * gridSize.second] {0};
	}
	catch (std::bad_alloc e)
	{
		std::cerr << e.what() << std::endl;
		exit(EXIT_FAILURE);
	}
	for (size_t i = 0; i < gridSize.first; i++)
	{
		for (size_t j = 0; j < gridSize.second; j++)
		{
			*(map + i * gridSize.second + j) = *(ob.map + i * gridSize.second + j);
		}
	}
	return *this;
}

inline BinaryOccupancyMap& BinaryOccupancyMap::operator=(BinaryOccupancyMap&& ob) noexcept
{
	gridSize = ob.gridSize;
	resolution = ob.resolution;
	xLimit = ob.xLimit;
	yLimit = ob.yLimit;
	map = ob.map;

	free(ob.map);
	return *this;
}

inline const Index& BinaryOccupancyMap::getGridSize() const noexcept
{
	return gridSize;
}

inline const double& BinaryOccupancyMap::getResolution() const noexcept
{
	return resolution;
}

inline const Point& BinaryOccupancyMap::getXLimit() const noexcept
{
	return xLimit;
}

inline const Point& BinaryOccupancyMap::getYLimit() const noexcept
{
	return yLimit;
}

inline void BinaryOccupancyMap::setOccupancy(const Point& xy , bool val) noexcept
{
	if (!valid(xy)) return;
	auto [i, j] = world2Grid(xy);
	*(map + i * gridSize.second + j) = val;
}

inline void BinaryOccupancyMap::setOccupancy(const Index& ij, bool val) noexcept
{
	if (!valid(ij)) return;
	*(map + ij.first * gridSize.second + ij.second) = val;
}

inline void BinaryOccupancyMap::setOccupancy(const Point& xy, bool val, const double& radius) noexcept
{
	//check validity of input point
	if (!valid(xy)) return;

	BinaryOccupancyMap visited({ xy.first - radius, xy.first + radius }, { xy.second - radius, xy.second + radius }, resolution);
	std::queue<Point> q;

	q.push(xy);
	while (!q.empty())
	{
		auto p = q.front();
		q.pop();

		visited.setOccupancy(p, true);
		setOccupancy(p, val);

		//For each neighbhour
		const Point& left = { p.first - resolution, p.second };
		if (valid(left))
		{
			if (!visited.checkOccupancy(left) && distance(xy, left) <= radius)
				q.push(left);
		}

		const Point& leftBottom = { p.first - resolution, p.second - resolution };
		if (valid(leftBottom))
		{
			if (!visited.checkOccupancy(leftBottom) && distance(xy, leftBottom) <= radius)
				q.push(leftBottom);
		}

		const Point& bottom = { p.first, p.second - resolution };
		if (valid(bottom))
		{
			if (!visited.checkOccupancy(bottom) && distance(xy, bottom) <= radius)
				q.push(bottom);
		}

		const Point& bottomRight = { p.first + resolution, p.second - resolution };
		if (valid(bottomRight))
		{
			if (!visited.checkOccupancy(bottomRight) && distance(xy, bottomRight) <= radius)
				q.push(bottomRight);
		}

		const Point& right = { p.first + resolution, p.second };
		if (valid(right))
		{
			if (!visited.checkOccupancy(right) && distance(xy, right) <= radius)
				q.push(right);
		}

		const Point& rightTop = { p.first + resolution, p.second + resolution };
		if (valid(rightTop))
		{
			if (!visited.checkOccupancy(rightTop) && distance(xy, rightTop) <= radius)
				q.push(rightTop);
		}

		const Point& top = { p.first, p.second + resolution};
		if (valid(top))
		{
			if (!visited.checkOccupancy(top) && distance(xy, top) <= radius)
				q.push(top);
		}

		const Point& topLeft = { p.first - resolution, p.second + resolution };
		if (valid(topLeft))
		{
			if (!visited.checkOccupancy(topLeft) && distance(xy, topLeft) <= radius)
				q.push(topLeft);
		}
	}
}

inline Point BinaryOccupancyMap::grid2World(const Index& ij) const noexcept
{
	double x = xLimit.first + resolution * ij.first;
	double y = yLimit.first + resolution * ij.second;
	return { x, y };
}

inline Index BinaryOccupancyMap::world2Grid(const Point& xy) const noexcept
{
	size_t i = static_cast<size_t>(floor((xy.first - xLimit.first) / resolution) * resolution);
	size_t j = static_cast<size_t>(floor((xy.second - yLimit.first) / resolution) * resolution);
	return { i, j };
}

inline bool BinaryOccupancyMap::checkOccupancy(const Point& xy) const noexcept
{
	if (!valid(xy)) return false;
	auto [i, j] = world2Grid(xy);
	return *(map + i * gridSize.second + j);
}

inline bool BinaryOccupancyMap::checkOccupancy(const Index& ij) const noexcept
{
	if (!valid(ij)) return false;
	return *(map + ij.first * gridSize.second + ij.second);
}

inline bool& BinaryOccupancyMap::getOccupancy(const Point& xy) noexcept
{
	auto [i, j] = world2Grid(xy);
	return *(map + i * gridSize.second + j);
}

inline bool& BinaryOccupancyMap::getOccupancy(const Index& ij) noexcept
{
	return* (map + ij.first * gridSize.second + ij.second);
}

inline void BinaryOccupancyMap::print() const noexcept
{
	for (long long i = static_cast<long long> (gridSize.first - 1); i >= 0; i--)
	{
		for (size_t j = 0; j < gridSize.second; j++)
		{
			std::cout << *(map + i * gridSize.second + j);
		}
		std::cout << std::endl;
	}
}

inline bool BinaryOccupancyMap::valid(const Point& xy) const noexcept
{
	return xy.first >= xLimit.first && xy.first <= xLimit.second && xy.second >= yLimit.first && xy.second <= yLimit.second;
}

inline bool BinaryOccupancyMap::valid(const Index& ij) const noexcept
{
	return ij.first < gridSize.first && ij.second < gridSize.second;
}

inline void BinaryOccupancyMap::inflate(const double& radius) noexcept
{
	BinaryOccupancyMap oldMap(*this);
	for (size_t i = 0; i < gridSize.first; i++)
	{
		for (size_t j = 0; j < gridSize.second; j++)
		{
			const Index& ij = { i, j };
			if(oldMap.checkOccupancy(ij))
				setOccupancy(grid2World(ij), true, radius);
		}
	}
}

inline BinaryOccupancyMap::~BinaryOccupancyMap()
{
	free(map);
}

