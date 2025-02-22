#pragma once
#include "HAStar.h"

void HAStar::updateGCostMatrix()
{
	double delta, newCost;
	Index nbr;
	const BinaryOccupancyMap& map = validator.getOccupancyMap();
	auto [iSize, jSize] = map.getGridSize();
	std::vector<std::vector<bool>> visited(iSize, std::vector<bool>(jSize, false));
	std::vector<std::vector<double>> gCost(iSize, std::vector<double>(jSize, DBL_MAX));
	auto [xGoal, yGoal, thetaGoal] = goalPose;
	auto ijGoal = map.world2Grid({ xGoal, yGoal });
	std::priority_queue<std::pair<double, Index>, std::vector<std::pair<double, Index>>, std::greater<std::pair<double, Index>>> pq;

	gCost[ijGoal.first][ijGoal.second] = 0.0;
	pq.push({0.0, ijGoal });


	while (!pq.empty())
	{
		auto[d, idx] = pq.top();
		pq.pop();
		auto& [i, j] = idx;

		visited[i][j] = true;

		//for all 8 neighbhours
		//top
		nbr = { i + 1, j };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//top left
		nbr = { i + 1, j - 1 };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//left
		nbr = { i, j - 1 };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//bottom left
		nbr = { i - 1, j - 1 };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//bottom
		nbr = { i - 1, j };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//bottom right
		nbr = { i - 1, j + 1 };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//right
		nbr = { i, j + 1 };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}

		//top right
		nbr = { i + 1, j + 1 };
		if (map.valid(nbr) && !map.checkOccupancy(nbr) && !visited[nbr.first][nbr.second])
		{
			delta = map.distance(nbr, idx);
			newCost = d + delta;
			auto& currentCost = gCost[nbr.first][nbr.second];
			if (newCost < gCost[nbr.first][nbr.second])
			{
				currentCost = newCost;
				pq.push({ newCost, nbr });
			}
		}
	}
	gCostMatrix = move(gCost);
}

double HAStar::heuristic(const State& s, const State& goal) const
{
	auto& [x, y, theta] = s;
	auto& [xGoal, yGoal, thetaGoal] = goal;
	auto ij = validator.getOccupancyMap().world2Grid({ x,y });
	auto ijGoal = validator.getOccupancyMap().world2Grid({ xGoal,yGoal });

	double dubinsDistance = 0.0;
	double minHolonomicDistance;

	//try Dubins connection
	try {
		Dubins dubins;
		const DubinsPathSegment& path = dubins.connect({ x,y,theta }, { xGoal, yGoal, thetaGoal }, R);
		dubinsDistance =  path.pathLength;
	}
	catch (...)
	{
		//do nothing
	}
	minHolonomicDistance = gCostMatrix[ij.first][ij.second];

	return std::max(dubinsDistance, minHolonomicDistance);
}

std::vector<State> HAStar::getNeigbhours(const State& s) const noexcept
{
	std::vector<State> result;
	double thetaNew;
	State sNew;
	auto& [x, y, theta] = s;

	thetaNew = toMinusPiToPi(theta - l / R);
	sNew = { x - R * (sin(thetaNew) - sin(theta)), y - R * (-cos(thetaNew) + cos(theta)), thetaNew };
	result.push_back(sNew);

	sNew = { x + l * cos(theta), y + l * sin(theta), theta };
	result.push_back(sNew);

	thetaNew = toMinusPiToPi(theta + l / R);
	sNew = { x + R * (sin(thetaNew) - sin(theta)), y + R * (-cos(thetaNew) + cos(theta)), thetaNew };
	result.push_back(sNew);

	return result;
}

double HAStar::toMinusPiToPi(double angle) const noexcept
{
	while (angle < -pi) angle += 2 * pi;
	while (angle > pi) angle -= 2 * pi;
	return angle;
}

bool HAStar::isPrimitiveValid(const State& s1, const State& s2, char type) const noexcept
{
	assert(type == 'L' || type == 'S' || type == 'R');
	double thetaNew;
	State sNew;
	auto& [x1, y1, theta1] = s1;
	double temp = 0.0;
	while (temp <= l)
	{
		switch (type)
		{
		case 'L':
			thetaNew = toMinusPiToPi(theta1 - temp / R);
			sNew = { x1 - R * (sin(thetaNew) - sin(theta1)), y1 - R * (-cos(thetaNew) + cos(theta1)), thetaNew };
			break;
		case 'S':
			sNew = { x1 + temp * cos(theta1), y1 + temp * sin(theta1), theta1 };
			break;
		case 'R':
			thetaNew = toMinusPiToPi(theta1 + temp / R);
			sNew = { x1 + R * (sin(thetaNew) - sin(theta1)), y1 + R * (-cos(thetaNew) + cos(theta1)), thetaNew };
			break;
		}

		if (!validator.isStateValid(sNew)) return false;
		temp += validationDistance;
	}
	return validator.isStateValid(s2);
}

bool HAStar::isExpansionValid(const DubinsPathSegment& pathSeg)
{
	double temp = 0;
	while (temp <= pathSeg.pathLength)
	{
		const Matrix& m = pathSeg.interpolate(temp);
		if (!validator.isStateValid({ m(0,0), m(1,0), m(2,0) })) return false;
		temp += validationDistance;
	}
	return true;
}


HAStar::HAStar(const ValidatorOccupancyMap& validator, const double& minTurningRadius): 
	validator(validator), R(minTurningRadius)
{
	validationDistance = validator.getOccupancyMap().getResolution();
	l = sqrt(2) * validationDistance;
	assert(R >= 2 * l / pi );
	straightCost = 1.0;
	turnCost = 1.15;
	analyticExpansionInterval = 5;
	numNodesExplored = 0;
	numIterations = 0;
	path = NavPath(validator.getStateSpace());
}

std::tuple<bool, size_t, size_t, size_t> HAStar::plan(const State& startPose, const State& goalPose)
{
	//check validity of start and end pose
	if (!validator.isStateValid(startPose)) throw HAStarException("HAStar Exception: Input start pose is not a valid state");
	if (!validator.isStateValid(goalPose)) throw HAStarException("HAStar Exception: Input goal pose is not a valid state");

	this->startPose = startPose;
	this->goalPose = goalPose;

	if (startPose == goalPose)
	{
		path.append(startPose);
		return { true, 0, 0, 1 };
	}

	updateGCostMatrix();
	const BinaryOccupancyMap& map = validator.getOccupancyMap();
	auto& [iSize, jSize] = map.getGridSize();
	std::vector<std::vector<bool>> visited(iSize, std::vector<bool>(jSize, false));
	std::vector<std::vector<Node>> parent(iSize, std::vector<Node>(jSize, { State(), Index(-1,-1) }));
	std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
	Dubins dubins;
	const Index& ijStart = map.world2Grid({ std::get<0>(startPose), std::get<1>(startPose) });
	char type[3] = { 'L', 'S', 'R' };

	auto& [xGoal, yGoal, thetaGoal] = goalPose;

	numNodesExplored = 0;
	numIterations = 0;

	State s;
	double g = 0.0;
	double h = heuristic(startPose, goalPose);
	if (h == DBL_MAX) return { false, 0, 0, 2 };
	double f = g + h;
	PQNode pqNode = { f, g, h, startPose };
	pq.push(pqNode);
	while (!pq.empty())
	{
		auto& currentPqNode = pq.top();
		std::tie(f, g, h, s) = currentPqNode;
		auto& [x, y, theta] = s;
		pq.pop();
		auto ij = map.world2Grid(Point(x, y));
		visited[ij.first][ij.second] = true;

		if (numIterations % analyticExpansionInterval == 0)
		{
			try {
				const DubinsPathSegment& pathSeg = dubins.connect({ x,y,theta }, { xGoal, yGoal, thetaGoal }, R);
				if (isExpansionValid(pathSeg))
				{
					path.append(goalPose);
					double temp = pathSeg.pathLength;
					while (temp >= 0)
					{
						const Matrix& m = pathSeg.interpolate(temp);
						path.append({ m(0,0), m(1,0), m(2,0) });
						temp -= validationDistance;
					}
					path.append(s);
					auto& [sParent, ijParent] = parent[ij.first][ij.second];
					while (map.valid(ijParent) && ijParent!= ijStart)
					{
						path.append(sParent);
						std::tie(sParent, ijParent) = parent[ijParent.first][ijParent.second];
					}
					path.append(sParent);
					return { true, numNodesExplored, numIterations, 1 };
				}
			}
			catch (...)
			{

			}
		}

		std::vector<State> n = getNeigbhours(s);
		for (size_t i = 0; i < 3; i++)
		{
			auto& [x1, y1, theta1] = n[i];
			if (isPrimitiveValid(s, n[i], type[i]))
			{
				const Index& idx = map.world2Grid(Point(x1, y1));
				if (!visited[idx.first][idx.second])
				{
					double h_n = heuristic(n[i], goalPose);
					double g_n = g + l * ((i == 1) ? straightCost : turnCost);
					double f_n = h_n + g_n;
					pq.push({ f_n, g_n, h_n, n[i] });
					numNodesExplored++;
					parent[idx.first][idx.second] = { s, ij };
				}
			}
		}
		numIterations++;
	}
	return { false, numNodesExplored, numIterations, 3 };
}

const NavPath& HAStar::getPath() const noexcept
{
	return path;
}
