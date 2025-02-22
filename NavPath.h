#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include "ValidatorOccupancyMap.h"

class NavPathException : public std::runtime_error
{
public:
	explicit NavPathException(const char* msg) : std::runtime_error(msg) {}
};

class NavPath
{
	StateSpace stateSpace;
	std::vector<State> states;
	double pathLength;
	size_t numStates;
public:
	NavPath(const StateSpace& stateSpace = StateSpace());
	
	//getters
	const std::vector<State>& getStates() const noexcept;
	double getPathLength() const noexcept;
	size_t getNumStates() const noexcept;

	//setters
	void setStateSpace(const StateSpace& stateSpace) noexcept;

	//functionality
	void append(const State& s) noexcept;
	void interpolate(size_t newNumStates);
};

inline NavPath::NavPath(const StateSpace& stateSpace): stateSpace(stateSpace), pathLength(0.0), numStates(0)
{
}

inline void NavPath::append(const State& s) noexcept
{
	const State& s1 = stateSpace.enforceStateBound(s);
	if (!states.empty())
	{
		pathLength += stateSpace.distance(states.back(), s1);
	}
	states.push_back(s1);
	numStates++;
}

inline void NavPath::interpolate(size_t newNumStates)
{
	if (newNumStates < numStates) throw NavPathException("NavPath Exception: New number of states should be >= current number of states");
	else if (numStates < 2) throw NavPathException("NavPath Exception: Current number of states should be > 1");
	else if(newNumStates == numStates) {}
	else if (numStates == 2 && states.front() == states.back())
	{
		size_t diff = newNumStates - numStates;
		const State& last = states.back();
		for (size_t i = 0; i < diff; i++)
		{
			states.push_back(last);
			numStates++;
		}
	}
	else
	{
		size_t numSegments = numStates - 1;
		size_t ptsToAdd = newNumStates - numStates;
		size_t sumPointsToAddPerSegment = 0;
		std::vector<size_t> ptsToAddPerSegment(numSegments, 0);
		std::priority_queue<std::pair<double, size_t>> pq;
		std::vector<State> newStates;

		for (size_t i = 1; i < numStates; i++)
		{
			double pLength = stateSpace.distance(states[i], states[i - 1]);
			double temp = floor(pLength * ptsToAdd / pathLength);
			ptsToAddPerSegment[i - 1] = isnan(temp) ? 0 : static_cast<size_t>(temp);
			sumPointsToAddPerSegment += ptsToAddPerSegment[i - 1];
			pq.push({ pLength, i - 1 });
		}
		ptsToAdd = ptsToAdd - sumPointsToAddPerSegment;
		for (size_t i = 0; i < ptsToAdd; i++)
		{
			auto [len, idx] = pq.top(); pq.pop();
			ptsToAddPerSegment[idx]++;
		}
		for (size_t i = 0; i < numSegments; i++)
		{
			if (ptsToAddPerSegment[i] > 0)
			{
				newStates.push_back(states[i]);
				double deltaRatio = 1.0 / (ptsToAddPerSegment[i] + 1.0);
				double ratio = deltaRatio;
				size_t pts = ptsToAddPerSegment[i];
				for (size_t j = 0 ; j < pts; j++, ratio+= deltaRatio)
				{
					newStates.push_back(stateSpace.interpolate(states[i], states[i + 1], ratio));
				}
			}
			else
			{
				newStates.push_back(states[i]);
			}
		}
		newStates.push_back(states[numSegments]);
		states = move(newStates);
		numStates = newNumStates;
	}
}

inline const std::vector<State>& NavPath::getStates() const noexcept
{
	return states;
}

inline double NavPath::getPathLength() const noexcept
{
	return pathLength;
}

inline size_t NavPath::getNumStates() const noexcept
{
	return numStates;
}

inline void NavPath::setStateSpace(const StateSpace& stateSpace) noexcept
{
	this->stateSpace = stateSpace;
}


