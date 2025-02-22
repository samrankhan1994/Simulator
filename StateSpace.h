#pragma once
#include <iostream>
#include <algorithm>

typedef std::tuple<double, double, double> State;

class StateSpace
{
	inline static double pi = 4.0 * atan(1.0);
	std::pair<State, State> stateBound;
	double weightXY;
	double weightTheta;
public:
	StateSpace();
	StateSpace(const std::pair<State, State>& stateBound);
	StateSpace(const std::pair<State, State>& stateBound, const double& weightXY, const double& weightTheta);

	//getters
	std::pair<State, State> getStateBound() const noexcept;
	double getWeightXY() const noexcept;
	double getWeightTheta() const noexcept;

	//setters
	void setStateBound(const std::pair<State, State>& bounds) noexcept;
	void setWeightXY(const double& weight) noexcept;
	void setWeightTheta(const double& weight) noexcept;

	//functionality
	double distance(const State& s1, const State& s2) const noexcept;
	State enforceStateBound(const State& s) const noexcept;
	State interpolate(const State& s1, const State& s2, double ratio) const noexcept;
};


inline StateSpace::StateSpace() : weightXY(1.0), weightTheta(0.1)
{
	const State& min = { 0.0, 0.0, -pi };
	const State& max = { 10.0, 10.0, pi };
	stateBound = { min, max };
}

inline StateSpace::StateSpace(const std::pair<State, State>& stateBound) : stateBound(stateBound), weightXY(1.0), weightTheta(0.1)
{

}

inline StateSpace::StateSpace(const std::pair<State, State>& stateBound, const double& weightXY, const double& weightTheta) :
	stateBound(stateBound), weightXY(weightXY), weightTheta(weightTheta)
{
}

inline double StateSpace::distance(const State& s1, const State& s2) const noexcept
{
	auto& [x1, y1, theta1] = s1;
	auto [x2, y2, theta2] = s2;
	double dx = x2 - x1;
	double dy = y2 - y1;
	while (theta2 - theta1 < -pi) theta2 += 2 * pi;
	while (theta2 - theta1 > pi) theta2 -= 2 * pi;
	double dTheta = theta2 - theta1;
	return sqrt(weightXY * (dx * dx + dy * dy) + weightTheta * dTheta * dTheta);
}

inline State StateSpace::enforceStateBound(const State& s) const noexcept
{
	auto& [xMin, yMin, thetaMin] = stateBound.first;
	auto& [xMax, yMax, thetaMax] = stateBound.second;
	auto [x, y, theta] = s;
	while (theta < thetaMin) theta += 2 * pi;
	while (theta > thetaMax) theta -= 2 * pi;
	return State(std::clamp(x, xMin, xMax), std::clamp(y, yMin, yMax), theta);
}

inline State StateSpace::interpolate(const State& s1, const State& s2, double ratio) const noexcept
{
	auto& [x1, y1, theta1] = s1;
	auto& [x2, y2, theta2] = s2;
	auto& [xMin, yMin, thetaMin] = stateBound.first;
	auto& [xMax, yMax, thetaMax] = stateBound.second;

	ratio = std::clamp(ratio, 0.0, 1.0);
	double x = (1 - ratio) * x1 + ratio * x2;
	double y = (1 - ratio) * y1 + ratio * y2;

	//Get smallest angle difference
	double theta = theta1;
	while (theta - theta2 < -pi) theta += 2 * pi;
	while (theta - theta2 > pi) theta -= 2 * pi;

	//interpolate theta and convert to within range
	theta = (1 - ratio) * theta + ratio * theta2;
	while (theta < thetaMin) theta += 2 * pi;
	while (theta > thetaMax) theta -= 2 * pi;
	return State(x, y, theta);
}

inline std::pair<State, State> StateSpace::getStateBound() const noexcept
{
	return stateBound;
}

inline double StateSpace::getWeightXY() const noexcept
{
	return weightXY;
}

inline double StateSpace::getWeightTheta() const noexcept
{
	return weightTheta;
}

inline void StateSpace::setStateBound(const std::pair<State, State>& bounds) noexcept
{
	stateBound = bounds;
}

inline void StateSpace::setWeightXY(const double& weight) noexcept
{
	weightXY = weight;
}

inline void StateSpace::setWeightTheta(const double& weight) noexcept
{
	weightTheta = weight;
}

