#pragma once
#include <algorithm>
#include <cmath>
#include <cassert>
#include "Vector3D.h"

class DubinsException : public std::runtime_error
{
public:
	explicit DubinsException(const char* msg) : std::runtime_error(msg) {}
};

class DubinsPathSegment
{
    inline static double pi = 4.0 * atan(1.0);

	double toMinusPiToPi(double angle) const;
	Vector3D leftTransform(const Vector3D& initialConfig, const double& length) const;
	Vector3D rightTransform(const Vector3D& initialConfig, const double& length) const;
	Vector3D straightTransform(const Vector3D& initialConfig, const double& length) const;
public:
	const double minTurningRadius;
	const Vector3D startConfig;
	const Vector3D goalConfig;
	const Vector3D motionLengths;
	const std::string motionType;
	const double pathLength;
    Vector3D z1, z2;

	DubinsPathSegment(const double& minTurningRadius, const Vector3D& startConfig, const Vector3D& goalConfig,
		const Vector3D& motionLengths, const std::string& motionType, const double& pathLength);
	Vector3D interpolate(double length) const;
	bool operator<(const DubinsPathSegment& ps) const;
};

class Dubins
{
	inline static double pi = 4.0 * atan(1.0);

	//Utility fn for 2D vectors
	double angle(const Vector3D& v1, const Vector3D& v2) const;
	double toZeroToTwoPI(double angle) const;
	Vector3D computeCenterOfCircle(const Vector3D& vehicleConfig, double minTurningRadius, char tt) const;
	DubinsPathSegment RSR(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const;
	DubinsPathSegment RSL(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const;
	DubinsPathSegment LSR(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const;
	DubinsPathSegment LSL(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const;
public:
	Dubins();
	DubinsPathSegment connect(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const;
    bool isLongPath(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const noexcept;
};



inline double DubinsPathSegment::toMinusPiToPi(double angle) const
{
    while (angle < -pi) angle += 2 * pi;
    while (angle > pi) angle -= 2 * pi;
    return angle;
}

inline Vector3D DubinsPathSegment::leftTransform(const Vector3D& initialConfig, const double& length) const
{
    double theta, theta_prev = initialConfig(2);
    theta = toMinusPiToPi(theta_prev - length / minTurningRadius);
    return Vector3D({ initialConfig(0) - minTurningRadius * (sin(theta) - sin(theta_prev)),
                    initialConfig(1) - minTurningRadius * (-cos(theta) + cos(theta_prev)),
                    theta });
}

inline Vector3D DubinsPathSegment::rightTransform(const Vector3D& initialConfig, const double& length) const
{
    double theta, theta_prev = initialConfig(2);
    theta = toMinusPiToPi(theta_prev + length / minTurningRadius);
    return Vector3D({ initialConfig(0) + minTurningRadius * (sin(theta) - sin(theta_prev)),
                    initialConfig(1) + minTurningRadius * (-cos(theta) + cos(theta_prev)),
                    theta });
}

inline Vector3D DubinsPathSegment::straightTransform(const Vector3D& initialConfig, const double& length) const
{
    double theta_prev = initialConfig(2);
    return Vector3D({ initialConfig(0) + length * cos(theta_prev),
                    initialConfig(1) + length * sin(theta_prev),
                    theta_prev });
}

inline DubinsPathSegment::DubinsPathSegment(const double& minTurningRadius, const Vector3D& startConfig,
    const Vector3D& goalConfig, const Vector3D& motionLengths, const std::string& motionType, const double& pathLength) :
    minTurningRadius(minTurningRadius), startConfig(startConfig), goalConfig(goalConfig), motionLengths(motionLengths),
    motionType(motionType), pathLength(pathLength)
{
    z1 = motionType[0] == 'L' ? leftTransform(startConfig, motionLengths(0)) : rightTransform(startConfig, motionLengths(0));
    z2 = straightTransform(z1, motionLengths(1));
}

inline Vector3D DubinsPathSegment::interpolate(double length) const
{
    length = std::clamp(length, 0.0, pathLength);
    double l1 = motionLengths(0);
    double l2 = l1 + motionLengths(1);
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



inline double Dubins::angle(const Vector3D& v1, const Vector3D& v2) const
{
    const double& len1 = v1.length();
    const double& len2 = v2.length();
    double l1l2 = len1 * len2;
    if (l1l2 == 0.0) throw DubinsException("Dubins Expception: Cannot find angle with zero vector");
    double angle = acos(v1.dot(v2) / l1l2);
    return v1.cross(v2)[2] < 0.0 ? 2 * pi - angle : angle;
}

inline double Dubins::toZeroToTwoPI(double angle) const
{
    while (angle < 0) angle += 2 * pi;
    while (angle > 2 * pi) angle -= 2 * pi;
    return angle;
}

inline Vector3D Dubins::computeCenterOfCircle(const Vector3D& vehicleConfig, double minTurningRadius, char tt) const
{
    double theta = (tt == 'L') ? vehicleConfig(2) - pi / 2 : vehicleConfig(2) + pi / 2;
    return Vector3D(vehicleConfig(0) + minTurningRadius * cos(theta),
        vehicleConfig(1) + minTurningRadius * sin(theta), 0.0);
}

inline DubinsPathSegment Dubins::RSR(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const
{
    const Vector3D& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'R');
    const Vector3D& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'R');
    const Vector3D& CstoCe = Ce - Cs;
    double l2 = CstoCe.length();
    double thetaStart = startConfig(2);
    double thetaGoal = goalConfig(2);
    double v = angle({ 1,0,0 }, CstoCe);
    double l1 = minTurningRadius * fmod(2 * pi + toZeroToTwoPI(v - pi / 2) - toZeroToTwoPI(thetaStart - pi / 2), 2 * pi);
    double l3 = minTurningRadius * fmod(2 * pi + toZeroToTwoPI(thetaGoal - pi / 2) - toZeroToTwoPI(v - pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "RSR", l1 + l2 + l3);
}

inline DubinsPathSegment Dubins::LSR(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const
{
    const Vector3D& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'L');
    const Vector3D& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'R');
    const Vector3D& CstoCe = Ce - Cs;
    double l = CstoCe.length();
    double thetaStart = startConfig(2);
    double thetaGoal = goalConfig(2);
    double v = angle({ 1,0,0 }, CstoCe);
    double v2 = acos(2 * minTurningRadius / l);
    double l1 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(v2 + v) + toZeroToTwoPI(thetaStart + pi / 2), 2 * pi);
    double l2 = sqrt(l * l - 4 * minTurningRadius * minTurningRadius);
    double l3 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(v2 + v + pi) + toZeroToTwoPI(thetaGoal - pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "LSR", l1 + l2 + l3);
}

inline DubinsPathSegment Dubins::RSL(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const
{
    const Vector3D& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'R');
    const Vector3D& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'L');
    const Vector3D& CstoCe = Ce - Cs;
    double l = CstoCe.length();
    double thetaStart = toZeroToTwoPI(startConfig(2));
    double thetaGoal = toZeroToTwoPI(goalConfig(2));
    double v = angle({ 1,0,0 }, CstoCe);
    double v2 = toZeroToTwoPI(v - pi / 2 + asin(2 * minTurningRadius / l));
    double l1 = minTurningRadius * fmod(2 * pi + v2 - toZeroToTwoPI(thetaStart - pi / 2), 2 * pi);
    double l2 = sqrt(l * l - 4 * minTurningRadius * minTurningRadius);
    double l3 = minTurningRadius * fmod(2 * pi + toZeroToTwoPI(v2 + pi) - toZeroToTwoPI(thetaGoal + pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "RSL", l1 + l2 + l3);
}

inline DubinsPathSegment Dubins::LSL(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const
{
    const Vector3D& Cs = computeCenterOfCircle(startConfig, minTurningRadius, 'L');
    const Vector3D& Ce = computeCenterOfCircle(goalConfig, minTurningRadius, 'L');
    const Vector3D& CstoCe = Ce - Cs;
    double l2 = CstoCe.length();
    double thetaStart = startConfig(2);
    double thetaGoal = goalConfig(2);
    double v = angle({ 1,0,0 }, CstoCe);
    double l1 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(v + pi / 2) + toZeroToTwoPI(thetaStart + pi / 2), 2 * pi);
    double l3 = minTurningRadius * fmod(2 * pi - toZeroToTwoPI(thetaGoal + pi / 2) + toZeroToTwoPI(v + pi / 2), 2 * pi);
    return DubinsPathSegment(minTurningRadius, startConfig, goalConfig, { l1, l2, l3 }, "LSL", l1 + l2 + l3);
}

inline Dubins::Dubins()
{
}

inline DubinsPathSegment Dubins::connect(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const
{
    //Input Check
    assert(minTurningRadius > 0);

    //Check for long path case
    if (!isLongPath(startConfig, goalConfig, minTurningRadius))
        throw DubinsException("Dubins Exception: Long path check failed for start and goal configuration");

    //find shortest path and return
    const DubinsPathSegment& ps1 = RSR(startConfig, goalConfig, minTurningRadius);
    const DubinsPathSegment& ps2 = RSL(startConfig, goalConfig, minTurningRadius);
    const DubinsPathSegment& ps3 = LSR(startConfig, goalConfig, minTurningRadius);
    const DubinsPathSegment& ps4 = LSL(startConfig, goalConfig, minTurningRadius);
    return std::min({ ps1, ps2, ps3, ps4 });
}

inline bool Dubins::isLongPath(const Vector3D& startConfig, const Vector3D& goalConfig, double minTurningRadius) const noexcept
{
    const Vector3D& v = goalConfig - startConfig;
    const double& theta = angle({ 1, 0, 0 }, v);
    double alpha = theta - startConfig(2);
    double beta = theta - goalConfig(2);
    double distanceCheck = minTurningRadius * (sqrt(4.0 - pow(abs(cos(alpha)) + abs(cos(beta)), 2.0)) + abs(sin(alpha)) + abs(sin(beta)));
    return v.length() > distanceCheck;
}
