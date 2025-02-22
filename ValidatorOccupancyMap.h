#pragma once
#include <iostream>
#include <algorithm>
#include "StateSpace.h"
#include "BinaryOccupancyMap.h"

class ValidatorOccupancyMap
{
	inline static const double pi = 4.0 * atan(1.0);
	BinaryOccupancyMap map;
	double validationDistance;
	StateSpace stateSpace;
public:
	ValidatorOccupancyMap();
	ValidatorOccupancyMap(const StateSpace& stateSpace, const BinaryOccupancyMap& map);
	ValidatorOccupancyMap(const StateSpace& stateSpace, const BinaryOccupancyMap& map, double validationDistance);

	//getters
	const BinaryOccupancyMap& getOccupancyMap() const noexcept;
	double getValidationDistance() const noexcept;
	StateSpace getStateSpace() const noexcept;

	//setters
	void setOccupancyMap(const BinaryOccupancyMap& map) noexcept;
	void setValidationDistance(const double& distance) noexcept;
	void setStateSpace(const StateSpace& space) noexcept;

	//functionality
	bool isStateValid(const State& s) const noexcept;
	bool isMotionValid(const State& s1, const State& s2) const noexcept;
};


inline ValidatorOccupancyMap::ValidatorOccupancyMap():validationDistance(DBL_MAX)
{
}

inline ValidatorOccupancyMap::ValidatorOccupancyMap(const StateSpace& stateSpace, const BinaryOccupancyMap& map) :
	stateSpace(stateSpace), map(map), validationDistance(DBL_MAX)
{
}

inline ValidatorOccupancyMap::ValidatorOccupancyMap(const StateSpace& stateSpace, const BinaryOccupancyMap& map, double validationDistance) :
	stateSpace(stateSpace), map(map), validationDistance(validationDistance)
{
}

inline const BinaryOccupancyMap& ValidatorOccupancyMap::getOccupancyMap() const noexcept
{
	return map;
}

inline double ValidatorOccupancyMap::getValidationDistance() const noexcept
{
	return validationDistance;
}

inline StateSpace ValidatorOccupancyMap::getStateSpace() const noexcept
{
	return stateSpace;
}

inline void ValidatorOccupancyMap::setOccupancyMap(const BinaryOccupancyMap& map) noexcept
{
	this->map = map;
}

inline void ValidatorOccupancyMap::setValidationDistance(const double& distance) noexcept
{
	validationDistance = distance;
}

inline void ValidatorOccupancyMap::setStateSpace(const StateSpace& space) noexcept
{
	stateSpace = space;
}

inline bool ValidatorOccupancyMap::isStateValid(const State& s) const noexcept
{
	auto& [x, y, theta] = s;
	Point xy(x, y);
	return map.valid(xy) && !map.checkOccupancy(xy);
}

inline bool ValidatorOccupancyMap::isMotionValid(const State& s1, const State& s2) const noexcept
{
	if (!isStateValid(s1)) return false;

	//get normalised distance and compute delta ratio
	auto& [x1, y1, theta1] = s1;
	auto& [x2, y2, theta2] = s2;
	double distance = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	double deltaRatio = std::clamp(validationDistance / distance, 0.0, 1.0);

	//check for all incremental state for validation
	for (double ratio = deltaRatio; ratio <= 1; ratio += deltaRatio)
	{
		const State& s = stateSpace.interpolate(s1, s2, ratio);
		if (!isStateValid(s)) return false;
	}
	return true;
}
