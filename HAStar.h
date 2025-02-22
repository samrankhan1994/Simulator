#pragma once
#include "ValidatorOccupancyMap.h"
#include "NavPath.h"
#include "Dubins.h"


class HAStarException : public std::runtime_error
{
public:
	explicit HAStarException(const char* msg) : std::runtime_error(msg) {}
};

typedef std::tuple<double, double, double, State> PQNode;
typedef std::pair<State, Index> Node;

class HAStar
{
	inline static double pi = 4.0 * atan(1.0);

	const ValidatorOccupancyMap& validator;
	double l;
	double R;
	double straightCost;
	double turnCost;
	size_t analyticExpansionInterval;
	State startPose;
	State goalPose;
	size_t numNodesExplored;
	size_t numIterations;
	double validationDistance;
	NavPath path;
	std::vector<std::vector<double>> gCostMatrix;
	void updateGCostMatrix();
	double heuristic(const State& s, const State& goal) const;
	std::vector<State> getNeigbhours(const State& s) const noexcept;
	double toMinusPiToPi(double angle) const noexcept;
	bool isPrimitiveValid(const State& s1, const State& s2, char type) const noexcept;
	bool isExpansionValid(const DubinsPathSegment& pathSeg);

public:
	HAStar(const ValidatorOccupancyMap& validator, const double& minTurningRadius);
	std::tuple<bool, size_t, size_t, size_t> plan(const State& startPose, const State& goalPose); //isPathFound, numNodesExplored, numIterations, exitFlag
	const NavPath& getPath() const noexcept;
};

