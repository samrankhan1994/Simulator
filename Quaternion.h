#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>

class QuaternionException : public std::runtime_error
{
public:
	explicit QuaternionException(const char* msg) : std::runtime_error(msg) {}
};

class Quaternion
{
	double q[4];
	double dot(const Quaternion& q) const noexcept;
public:
	constexpr Quaternion();
	constexpr Quaternion(const double& q0, const double& q1, const double& q2, const double& q3);
	constexpr Quaternion(double (&arr)[4]);

	// Defaults
	constexpr Quaternion(const Quaternion& other) = default;
	constexpr Quaternion(Quaternion&& other) noexcept = default;
	constexpr Quaternion& operator=(const Quaternion& other) = default;
	constexpr Quaternion& operator=(Quaternion&& other) = default;

	constexpr Quaternion& operator+=(const Quaternion& other) noexcept;
	constexpr Quaternion& operator+=(const double& scalar) noexcept;
	constexpr Quaternion operator+(const Quaternion& other) const noexcept;
	constexpr Quaternion operator+(const double& scalar) const noexcept;

	constexpr Quaternion& operator-=(const Quaternion& other) noexcept;
	constexpr Quaternion& operator-=(const double& scalar) noexcept;
	constexpr Quaternion operator-(const Quaternion& other) const noexcept;
	constexpr Quaternion operator-(const double& scalar) const noexcept;

	constexpr Quaternion& operator*=(const Quaternion& other) noexcept;
	constexpr Quaternion& operator*=(const double& scalar) noexcept;
	constexpr Quaternion operator*(const Quaternion& other) const noexcept;
	constexpr Quaternion operator*(const double& scalar) const noexcept;

	constexpr Quaternion& operator/=(const Quaternion& other);
	constexpr Quaternion& operator/=(const double& scalar);
	constexpr Quaternion operator/(const Quaternion& other) const;
	constexpr Quaternion operator/(const double& scalar) const;

	bool operator==(const Quaternion& other) const noexcept;

	double mag() const noexcept;
	Quaternion normalize() const;
	Quaternion& normalize();
	constexpr Quaternion conj() const noexcept;
	Quaternion inv() const;

	static Quaternion lerp(const Quaternion& start, const Quaternion& end, double u) noexcept;
	static Quaternion slerp(const Quaternion& start, const Quaternion& end, double u) noexcept;

	constexpr const double(&getData()const noexcept)[4];

	~Quaternion() noexcept = default;

	friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
	friend Quaternion operator+(const double& scalar, const Quaternion& q) noexcept;
	friend Quaternion operator-(const double& scalar, const Quaternion& q) noexcept;
	friend Quaternion operator*(const double& scalar, const Quaternion& q) noexcept;
	friend Quaternion operator/(const double& scalar, const Quaternion& q);
};

inline constexpr Quaternion::Quaternion() : q{0.0, 0.0,  0.0, 0.0} {}

inline constexpr Quaternion::Quaternion(const double& q0, const double& q1, const double& q2, const double& q3) : q{q0, q1, q2, q3} {}

inline constexpr Quaternion::Quaternion(double(&arr)[4])
{
	q[0] = arr[0];
	q[1] = arr[1];
	q[2] = arr[2];
	q[3] = arr[3];
}

inline constexpr Quaternion& Quaternion::operator+=(const Quaternion& other) noexcept
{
	q[0] += other.q[0];
	q[1] += other.q[1];
	q[2] += other.q[2];
	q[3] += other.q[3];
	return *this;
}

inline constexpr Quaternion& Quaternion::operator+=(const double& scalar) noexcept
{
	q[0] += scalar;
	q[1] += scalar;
	q[2] += scalar;
	q[3] += scalar;
	return *this;
}

inline constexpr Quaternion Quaternion::operator+(const Quaternion& other) const noexcept
{
	return Quaternion(q[0] + other.q[0], q[1] + other.q[1], q[2] + other.q[2], q[3] + other.q[3]);
}

inline constexpr Quaternion Quaternion::operator+(const double& scalar) const noexcept
{
	return Quaternion(q[0] + scalar, q[1] + scalar, q[2] + scalar, q[3] + scalar);
}

inline constexpr Quaternion& Quaternion::operator-=(const Quaternion& other) noexcept
{
	q[0] -= other.q[0];
	q[1] -= other.q[1];
	q[2] -= other.q[2];
	q[3] -= other.q[3];
	return *this;
}

inline constexpr Quaternion& Quaternion::operator-=(const double& scalar) noexcept
{
	q[0] -= scalar;
	q[1] -= scalar;
	q[2] -= scalar;
	q[3] -= scalar;
	return *this;
}

inline constexpr Quaternion Quaternion::operator-(const Quaternion& other) const noexcept
{
	return Quaternion(q[0] - other.q[0], q[1] - other.q[1], q[2] - other.q[2], q[3] - other.q[3]);
}

inline constexpr Quaternion Quaternion::operator-(const double& scalar) const noexcept
{
	return Quaternion(q[0] - scalar, q[1] - scalar, q[2] - scalar, q[3] - scalar);
}

inline constexpr Quaternion& Quaternion::operator*=(const Quaternion& other) noexcept
{
	*this = Quaternion(
		q[0] * other.q[0] - q[1] * other.q[1] - q[2] * other.q[2] - q[3] * other.q[3], // q[0]
		q[0] * other.q[1] + q[1] * other.q[0] + q[2] * other.q[3] - q[3] * other.q[2], // q[1]
		q[0] * other.q[2] - q[1] * other.q[3] + q[2] * other.q[0] + q[3] * other.q[1], // q[2]
		q[0] * other.q[3] + q[1] * other.q[2] - q[2] * other.q[1] + q[3] * other.q[0]  // q[3]
	);
	return *this;
}

inline constexpr Quaternion& Quaternion::operator*=(const double& scalar) noexcept
{
	q[0] *= scalar;
	q[1] *= scalar;
	q[2] *= scalar;
	q[3] *= scalar;
	return *this;
}

inline constexpr Quaternion Quaternion::operator*(const Quaternion& other) const noexcept
{
	return Quaternion(
		q[0] * other.q[0] - q[1] * other.q[1] - q[2] * other.q[2] - q[3] * other.q[3], // q[0]
		q[0] * other.q[1] + q[1] * other.q[0] + q[2] * other.q[3] - q[3] * other.q[2], // q[1]
		q[0] * other.q[2] - q[1] * other.q[3] + q[2] * other.q[0] + q[3] * other.q[1], // q[2]
		q[0] * other.q[3] + q[1] * other.q[2] - q[2] * other.q[1] + q[3] * other.q[0]  // q[3]
	);
}

inline constexpr Quaternion Quaternion::operator*(const double& scalar) const noexcept
{
	return Quaternion(q[0] * scalar, q[1] * scalar, q[2] * scalar, q[3] * scalar);
}

inline constexpr Quaternion& Quaternion::operator/=(const Quaternion& other)
{
	*this = *this * other.inv();
	return *this;
}

inline constexpr Quaternion& Quaternion::operator/=(const double& scalar)
{
	if (scalar == 0.0) throw QuaternionException("Quaternion Exception: divide by zero");
	q[0] /= scalar;
	q[1] /= scalar;
	q[2] /= scalar;
	q[3] /= scalar;
	return *this;
}

inline constexpr Quaternion Quaternion::operator/(const Quaternion& other) const
{
	return *this * other.inv();
}

inline constexpr Quaternion Quaternion::operator/(const double& scalar) const
{
	if (scalar == 0.0) throw QuaternionException("Quaternion Exception: divide by scalar zero");
	return Quaternion(q[0] / scalar, q[1] / scalar, q[2] / scalar, q[3] / scalar);
}

inline bool Quaternion::operator==(const Quaternion& other) const noexcept
{
	return memcmp(q, other.q, 4 * sizeof(double)) == 0;
}

inline double Quaternion::mag() const noexcept
{
	return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

inline Quaternion Quaternion::normalize() const
{
	double m = mag();
	if (m == 0.0) throw QuaternionException("Quaternion Exception: Cannot normalize zero quaternion");
	return Quaternion(q[0] / m, q[1] / m, q[2] / m, q[3] / m);
}

inline Quaternion& Quaternion::normalize()
{
	double m = mag();
	if (m == 0.0) throw QuaternionException("Quaternion Exception: Cannot normalize zero quaternion");
	*this /= m;
	return *this;
}

inline constexpr Quaternion Quaternion::conj() const noexcept
{
	return Quaternion(q[0], -q[1], -q[2], -q[3]);
}

inline Quaternion Quaternion::inv() const
{
	double m = mag();
	if (m == 0.0) throw QuaternionException("Quaternion Exception: Cannot invert zero quaternion");
	return conj() / (m * m);
}

inline double Quaternion::dot(const Quaternion& other) const noexcept
{
	return q[0] * other.q[0] + q[1] * other.q[1] + q[2] * other.q[2] + q[3] * other.q[3];
}

inline Quaternion Quaternion::lerp(const Quaternion& start, const Quaternion& end, double u)  noexcept
{
	u = std::clamp(u, 0.0, 1.0);
	return (1 - u) * start + u * end;
}

inline Quaternion Quaternion::slerp(const Quaternion& start, const Quaternion& end, double u) noexcept
{
	u = std::clamp(u, 0.0, 1.0);
	double dotProduct = start.dot(end);
	dotProduct = std::clamp(dotProduct, 0.0, 1.0);
	double omega = std::acos(dotProduct);
	if (omega < 1e-3) return start;
	double sinOmega = std::sin(omega);
	return (sin((1 - u) * omega) / sinOmega) * start + (sin(u * omega) / sinOmega) * end;
}

inline constexpr const double(&Quaternion::getData() const noexcept)[4]
{
	return q;
}

inline std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
	os << std::showpoint << "( " << q.q[0] << ", " << q.q[1] << ", " << q.q[2] << ", " << q.q[3] << " )" << std::noshowpoint;
	return os;
}

inline Quaternion operator+(const double& scalar, const Quaternion& q) noexcept
{
	return Quaternion(scalar + q.q[0], scalar + q.q[1], scalar + q.q[2], scalar + q.q[3]);
}

inline Quaternion operator-(const double& scalar, const Quaternion& q) noexcept
{
	return Quaternion(scalar - q.q[0], scalar - q.q[1], scalar - q.q[2], scalar - q.q[3]);
}

inline Quaternion operator*(const double& scalar, const Quaternion& q) noexcept
{
	return Quaternion(scalar * q.q[0], scalar * q.q[1], scalar * q.q[2], scalar * q.q[3]);
}

inline Quaternion operator/(const double& scalar, const Quaternion& q)
{
	return scalar * q.inv();
}

