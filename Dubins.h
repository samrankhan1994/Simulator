#pragma once
#include <algorithm>
#include <cmath>
#include <cassert>
#include "Matrix.h"

class DubinsException : public std::runtime_error
{
public:
	explicit DubinsException(const char* msg) : std::runtime_error(msg) {}
};

class DubinsPathSegment
{
    inline static double pi = 4.0 * atan(1.0);

	Matrix z1, z2;
	double toMinusPiToPi(double angle) const;
	Matrix leftTransform(const Matrix& initialConfig, double length) const;
	Matrix rightTransform(const Matrix& initialConfig, double length) const;
	Matrix straightTransform(const Matrix& initialConfig, double length) const;
public:
	const double minTurningRadius;
	const Matrix startConfig;
	const Matrix goalConfig;
	const Matrix motionLengths;
	const std::string motionType;
	const double pathLength;

	DubinsPathSegment(const double& minTurningRadius, const Matrix& startConfig, const Matrix& goalConfig,
		const Matrix& motionLengths, const std::string& motionType, const double& pathLength);
	Matrix interpolate(double length) const;
	bool operator<(const DubinsPathSegment& ps) const;
};

class Dubins
{
	inline static double pi = 4.0 * atan(1.0);

	//Utility fn for 2D vectors
	Matrix Rz(double angle) const;
	Matrix unit(const Matrix& v) const;
	Matrix unit(Matrix&& v) const;
	double dotProduct(const Matrix& v1, const Matrix& v2) const;
	double crossProduct(const Matrix& v1, const Matrix& v2) const;
	double angle(const Matrix& v1, const Matrix& v2) const;
	double angle(Matrix&& v1, Matrix&& v2) const;
	double angle(Matrix&& v1, const Matrix& v2) const;
	double angle(const Matrix& v1, Matrix&& v2) const;
	double toZeroToTwoPI(double angle) const;
	Matrix computeCenterOfCircle(const Matrix& vehicleConfig, double minTurningRadius, char tt) const;
	DubinsPathSegment RSR(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const;
	DubinsPathSegment RSL(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const;
	DubinsPathSegment LSR(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const;
	DubinsPathSegment LSL(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const;
public:
	Dubins();
	DubinsPathSegment connect(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const;
};



inline double DubinsPathSegment::toMinusPiToPi(double angle) const
{
    while (angle < -pi) angle += 2 * pi;
    while (angle > pi) angle -= 2 * pi;
    return angle;
}

inline Matrix DubinsPathSegment::leftTransform(const Matrix& initialConfig, double length) const
{
    double theta, theta_prev = initialConfig(2, 0);
    theta = toMinusPiToPi(initialConfig(2, 0) - length / minTurningRadius);
    return Matrix({ initialConfig(0, 0) - minTurningRadius * (sin(theta) - sin(theta_prev)),
                    initialConfig(1, 0) - minTurningRadius * (-cos(theta) + cos(theta_prev)),
                    theta });
}

inline Matrix DubinsPathSegment::rightTransform(const Matrix& initialConfig, double length) const
{
    double theta, theta_prev = initialConfig(2, 0);
    theta = toMinusPiToPi(initialConfig(2, 0) + length / minTurningRadius);
    return Matrix({ initialConfig(0, 0) + minTurningRadius * (sin(theta) - sin(theta_prev)),
                    initialConfig(1, 0) + minTurningRadius * (-cos(theta) + cos(theta_prev)),
                    theta });
}

inline Matrix DubinsPathSegment::straightTransform(const Matrix& initialConfig, double length) const
{
    double theta_prev = initialConfig(2, 0);
    return Matrix({ initialConfig(0, 0) + length * cos(theta_prev),
                    initialConfig(1, 0) + length * sin(theta_prev),
                    theta_prev });
}

inline DubinsPathSegment::DubinsPathSegment(const double& minTurningRadius, const Matrix& startConfig,
    const Matrix& goalConfig, const Matrix& motionLengths, const std::string& motionType, const double& pathLength) :
    minTurningRadius(minTurningRadius), startConfig(startConfig), goalConfig(goalConfig), motionLengths(motionLengths),
    motionType(motionType), pathLength(pathLength)
{
    z1 = motionType[0] == 'L' ? leftTransform(startConfig, motionLengths(0, 0)) : rightTransform(startConfig, motionLengths(0, 0));
    z2 = straightTransform(z1, motionLengths(1, 0));
}

inline Matrix DubinsPathSegment::interpolate(double length) const
{
    length = std::clamp(length, 0.0, pathLength);
    double l1 = motionLengths(0, 0);
    double l2 = l1 + motionLengths(1, 0);
    if (length <= l1)
        return motionType[0] == 'L' ? leftTransform(startConfig, length) : rightTransform(startConfig, length);
    else if (length <= l2)
        return straightTransform(z1, length - l1);
    else
        return motionType[2] == 'L' ? leftTransform(z2, length - l2) : rightTransform(z2, length - l2);
}

inline bool DubinsPathSegment::operator<(const DubinsPathSegment& ps) const
{
    return pathLength < ps.pathLength;
}

inline Matrix Dubins::Rz(double angle) const
{
    return Matrix({ {cos(angle), -sin(angle)},{sin(angle), cos(angle)} });
}

inline Matrix Dubins::unit(const Matrix& v) const
{
    double magnitude = sqrt(dotProduct(v, v));
    return v / magnitude;
}

inline Matrix Dubins::unit(Matrix&& v) const
{
    double magnitude = sqrt(dotProduct(v, v));
    v /= magnitude;
    return std::move(v);
}

inline double Dubins::dotProduct(const Matrix& v1, const Matrix& v2) const
{
    return v1(0, 0) * v2(0, 0) + v1(1, 0) * v2(1, 0);
}

inline double Dubins::crossProduct(const Matrix& v1, const Matrix& v2) const
{
    return v1(0, 0) * v2(1, 0) - v1(0, 1) * v2(0, 0);
}

inline double Dubins::angle(const Matrix& v1, const Matrix& v2) const
{
    const Matrix& unitV1 = unit(v1);
    const Matrix& unitV2 = unit(v2);
    double angle = acos(dotProduct(unitV1, unitV2));
    return crossProduct(unitV1, unitV2) < 0.0 ? 2 * pi - angle : angle;
}

inline double Dubins::angle(Matrix&& v1, Matrix&& v2) const
{
    const Matrix& unitV1 = unit(std::move(v1));
    const Matrix& unitV2 = unit(std::move(v2));
    double angle = acos(dotProduct(unitV1, unitV2));
    return crossProduct(unitV1, unitV2) < 0.0 ? 2 * pi - angle : angle;
}

inline double Dubins::angle(Matrix&& v1, const Matrix& v2) const
{
    const Matrix& unitV1 = unit(std::move(v1));
    const Matrix& unitV2 = unit(v2);
    double angle = acos(dotProduct(unitV1, unitV2));
    return crossProduct(unitV1, unitV2) < 0.0 ? 2 * pi - angle : angle;
}

inline double Dubins::angle(const Matrix& v1, Matrix&& v2) const
{
    const Matrix& unitV1 = unit(v1);
    const Matrix& unitV2 = unit(std::move(v2));
    double angle = acos(dotProduct(unitV1, unitV2));
    return crossProduct(unitV1, unitV2) < 0.0 ? 2 * pi - angle : angle;
}

inline double Dubins::toZeroToTwoPI(double angle) const
{
    while (angle < 0) angle += 2 * pi;
    while (angle > 2 * pi) angle -= 2 * pi;
    return angle;
}

inline Matrix Dubins::computeCenterOfCircle(const Matrix& vehicleConfig, double minTurningRadius, char tt) const
{
    double theta = (tt == 'L') ? vehicleConfig(2, 0) - pi / 2 : vehicleConfig(2, 0) + pi / 2;
    return Matrix({ vehicleConfig(0, 0) + minTurningRadius * cos(theta),
        vehicleConfig(1, 0) + minTurningRadius * sin(theta) });
}

inline DubinsPathSegment Dubins::RSR(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const
{
    const Matrix& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'R');
    const Matrix& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'R');
    const Matrix& CstoCe = Ce - Cs;
    double l2 = sqrt(dotProduct(CstoCe, CstoCe));
    double thetaStart = startConfig(2, 0);
    double thetaGoal = goalConfig(2, 0);
    double v = angle({ 1,0,0 }, CstoCe);
    double l1 = minTurningRadius * fmod(2 * pi + toZeroToTwoPI(v - pi / 2) - toZeroToTwoPI(thetaStart - pi / 2), 2 * pi);
    double l3 = minTurningRadius * fmod(2 * pi + toZeroToTwoPI(thetaGoal - pi / 2) - toZeroToTwoPI(v - pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "RSR", l1 + l2 + l3);
}

inline DubinsPathSegment Dubins::LSR(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const
{
    const Matrix& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'L');
    const Matrix& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'R');
    const Matrix& CstoCe = Ce - Cs;
    double l = sqrt(dotProduct(CstoCe, CstoCe));
    double thetaStart = startConfig(2, 0);
    double thetaGoal = goalConfig(2, 0);
    double v = angle({ 1,0,0 }, CstoCe);
    double v2 = acos(2 * minTurningRadius / l);
    double l1 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(v2 + v) + toZeroToTwoPI(thetaStart + pi / 2), 2 * pi);
    double l2 = sqrt(l * l - 4 * minTurningRadius * minTurningRadius);
    double l3 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(v2 + v + pi) + toZeroToTwoPI(thetaGoal - pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "LSR", l1 + l2 + l3);
}

inline DubinsPathSegment Dubins::RSL(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const
{
    const Matrix& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'R');
    const Matrix& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'L');
    const Matrix& CstoCe = Ce - Cs;
    double l = sqrt(dotProduct(CstoCe, CstoCe));
    double thetaStart = toZeroToTwoPI(startConfig(2, 0));
    double thetaGoal = toZeroToTwoPI(goalConfig(2, 0));
    double v = angle({ 1,0,0 }, CstoCe);
    double v2 = toZeroToTwoPI(v - pi / 2 + asin(2 * minTurningRadius / l));
    double l1 = minTurningRadius * fmod(2 * pi + v2 - toZeroToTwoPI(thetaStart - pi / 2), 2 * pi);
    double l2 = sqrt(l * l - 4 * minTurningRadius * minTurningRadius);
    double l3 = minTurningRadius * fmod(2 * pi + toZeroToTwoPI(v2 + pi) - toZeroToTwoPI(thetaGoal + pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "RSL", l1 + l2 + l3);
}

inline DubinsPathSegment Dubins::LSL(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const
{
    const Matrix& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'L');
    const Matrix& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'L');
    const Matrix& CstoCe = Ce - Cs;
    double l2 = sqrt(dotProduct(CstoCe, CstoCe));
    double thetaStart = startConfig(2, 0);
    double thetaGoal = goalConfig(2, 0);
    double v = angle({ 1,0,0 }, CstoCe);
    double l1 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(v + pi / 2) + toZeroToTwoPI(thetaStart + pi / 2), 2 * pi);
    double l3 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(thetaGoal + pi / 2) + toZeroToTwoPI(v + pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "LSL", l1 + l2 + l3);
}

inline Dubins::Dubins()
{
}

inline DubinsPathSegment Dubins::connect(const Matrix& startConfig, const Matrix& goalConfig, double minTurningRadius) const
{
    //Input Check
    assert(minTurningRadius > 0);
    auto [row1, col1] = startConfig.getDimension();
    assert(row1 == 3 && col1 == 1);
    auto [row2, col2] = goalConfig.getDimension();
    assert(row2 == 3 && col2 == 1);

    //Check for long path case
    double thetaStart = startConfig(2, 0);
    double thetaGoal = goalConfig(2, 0);
    const Matrix& v = { goalConfig(0,0) - startConfig(0,0), goalConfig(1,0) - startConfig(1,0) };
    double theta = angle({ 1, 0, 0 }, v);
    double alpha = theta - thetaStart;
    double beta = theta - thetaGoal;
    double d = sqrt(dotProduct(v, v));
    double distanceCheck = minTurningRadius * (sqrt(4.0 - pow(abs(cos(alpha)) + abs(cos(beta)), 2.0)) + abs(sin(alpha)) + abs(sin(beta)));
    if (d <= distanceCheck) 
        throw DubinsException("Dubins Exception: Long path check failed for start and goal configuration");

    //find shortest path and return
    const DubinsPathSegment& ps1 = RSR(startConfig, goalConfig, minTurningRadius);
    const DubinsPathSegment& ps2 = RSL(startConfig, goalConfig, minTurningRadius);
    const DubinsPathSegment& ps3 = LSR(startConfig, goalConfig, minTurningRadius);
    const DubinsPathSegment& ps4 = LSL(startConfig, goalConfig, minTurningRadius);
    return std::min({ ps1, ps2, ps3, ps4 });
}
