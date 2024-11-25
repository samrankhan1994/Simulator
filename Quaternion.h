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
	constexpr inline static double TOL = 1.0e-6;
	double w, x, y, z;
public:
	Quaternion();
	explicit Quaternion(const double& _w, const double& _x, const double& _y, const double& _z);
	explicit Quaternion(const double& psi, const double& theta, const double& phi);

	Quaternion(const Quaternion& q) = default;
	Quaternion(Quaternion&& q) noexcept = default;

	Quaternion& operator=(const Quaternion& q) = default;
	Quaternion& operator=(Quaternion&& q) = default;

	Quaternion& operator=(std::initializer_list<double> list);

	Quaternion& operator+=(const Quaternion& q) noexcept;
	Quaternion& operator+=(const double& scalar) noexcept;
	Quaternion operator+(const Quaternion& q) const noexcept;
	Quaternion operator+(const double& scalar) const noexcept;

	Quaternion& operator-=(const Quaternion& q) noexcept;
	Quaternion& operator-=(const double& scalar) noexcept;
	Quaternion operator-(const Quaternion& q) const noexcept;
	Quaternion operator-(const double& scalar) const noexcept;

	Quaternion& operator*=(const Quaternion& q) noexcept;
	Quaternion& operator*=(const double& scalar) noexcept;
	Quaternion operator*(const Quaternion& q) const noexcept;
	Quaternion operator*(const double& scalar) const noexcept;

	Quaternion& operator/=(const Quaternion& q);
	Quaternion& operator/=(const double& scalar);
	Quaternion operator/(const Quaternion& q) const;
	Quaternion operator/(const double& scalar) const;

	double mag() const noexcept;
	Quaternion normalize() const;
	Quaternion conj() const noexcept;
	Quaternion inv() const;
	double dot(const Quaternion& q) const noexcept;

	Quaternion derivative(const double& p, const double& q, const double& r) const noexcept;
	static Quaternion lerp(const Quaternion& start, const Quaternion& end, double u) noexcept;
	static Quaternion slerp(const Quaternion& start, const Quaternion& end, double u) noexcept;
	static Quaternion fromAxisAngle(const double& x, const double& y, const double& z, const double& angle) noexcept;
	~Quaternion() noexcept = default;

	friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
	friend Quaternion operator+(const double& scalar, const Quaternion& q) noexcept;
	friend Quaternion operator-(const double& scalar, const Quaternion& q) noexcept;
	friend Quaternion operator*(const double& scalar, const Quaternion& q) noexcept;
	friend Quaternion operator/(const double& scalar, const Quaternion& q);
};

inline Quaternion::Quaternion() : w(1), x(0), y(0), z(0) {}

inline Quaternion::Quaternion(const double& _w, const double& _x, const double& _y, const double& _z) : w(_w), x(_x), y(_y), z(_z) {}

inline Quaternion::Quaternion(const double& psi, const double& theta, const double& phi)
{
	double halfYaw = psi * 0.5;
	double halfPitch = theta * 0.5;
	double halfRoll = phi * 0.5;

	// Calculate the components of the quaternion
	double cosHalfYaw = cos(halfYaw);
	double sinHalfYaw = sin(halfYaw);
	double cosHalfPitch = cos(halfPitch);
	double sinHalfPitch = sin(halfPitch);
	double cosHalfRoll = cos(halfRoll);
	double sinHalfRoll = sin(halfRoll);

	// Set the quaternion components based on the Euler angles
	w = cosHalfYaw * cosHalfPitch * cosHalfRoll + sinHalfYaw * sinHalfPitch * sinHalfRoll;
	x = sinHalfYaw * cosHalfPitch * cosHalfRoll - cosHalfYaw * sinHalfPitch * sinHalfRoll;
	y = cosHalfYaw * sinHalfPitch * cosHalfRoll + sinHalfYaw * cosHalfPitch * sinHalfRoll;
	z = cosHalfYaw * cosHalfPitch * sinHalfRoll - sinHalfYaw * sinHalfPitch * cosHalfRoll;
}

inline Quaternion& Quaternion::operator=(std::initializer_list<double> list)
{
	if (list.size() != 4) throw QuaternionException("Size of initializer list is not 4");
	auto itr = list.begin();
	w = *itr++;
	x = *itr++;
	y = *itr++;
	z = *itr++;
	return *this;
}

inline Quaternion& Quaternion::operator+=(const Quaternion& q) noexcept
{
	w += q.w;
	x += q.x;
	y += q.y;
	z += q.z;
	return *this;
}

inline Quaternion& Quaternion::operator+=(const double& scalar) noexcept
{
	w += scalar;
	x += scalar;
	y += scalar;
	z += scalar;
	return *this;
}

inline Quaternion Quaternion::operator+(const Quaternion& q) const noexcept
{
	return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

inline Quaternion Quaternion::operator+(const double& scalar) const noexcept
{
	return Quaternion(w + scalar, x + scalar, y + scalar, z + scalar);
}

inline Quaternion& Quaternion::operator-=(const Quaternion& q) noexcept
{
	w -= q.w;
	x -= q.x;
	y -= q.y;
	z -= q.z;
	return *this;
}

inline Quaternion& Quaternion::operator-=(const double& scalar) noexcept
{
	w -= scalar;
	x -= scalar;
	y -= scalar;
	z -= scalar;
	return *this;
}

inline Quaternion Quaternion::operator-(const Quaternion& q) const noexcept
{
	return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
}

inline Quaternion Quaternion::operator-(const double& scalar) const noexcept
{
	return Quaternion(w - scalar, x - scalar, y - scalar, z - scalar);
}

inline Quaternion& Quaternion::operator*=(const Quaternion& q) noexcept
{
	*this = Quaternion(
		w * q.w - x * q.x - y * q.y - z * q.z, // w
		w * q.x + x * q.w + y * q.z - z * q.y, // x
		w * q.y - x * q.z + y * q.w + z * q.x, // y
		w * q.z + x * q.y - y * q.x + z * q.w  // z
	);
	return *this;
}

inline Quaternion& Quaternion::operator*=(const double& scalar) noexcept
{
	w = w * scalar;
	x = x * scalar;
	y = y * scalar;
	z = z * scalar;
	return *this;
}

Quaternion Quaternion::operator*(const Quaternion& q) const noexcept
{
	return Quaternion(
		w * q.w - x * q.x - y * q.y - z * q.z, // w
		w * q.x + x * q.w + y * q.z - z * q.y, // x
		w * q.y - x * q.z + y * q.w + z * q.x, // y
		w * q.z + x * q.y - y * q.x + z * q.w  // z
	);
}

inline Quaternion Quaternion::operator*(const double& scalar) const noexcept
{
	return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

inline Quaternion& Quaternion::operator/=(const Quaternion& q)
{
	*this = *this * q.inv();
	return *this;
}

inline Quaternion& Quaternion::operator/=(const double& scalar)
{
	if (abs(scalar) < TOL) throw QuaternionException("Quaternion divide by zero");
	w = w / scalar;
	x = x / scalar;
	y = y / scalar;
	z = z / scalar;
	return *this;
}

inline Quaternion Quaternion::operator/(const Quaternion& q) const
{
	return *this * q.inv();
}

inline Quaternion Quaternion::operator/(const double& scalar) const
{
	if (abs(scalar) < TOL) throw QuaternionException("Quaternion divide by zero");
	return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
}

inline double Quaternion::mag() const noexcept
{
	return std::sqrt(w * w + x * x + y * y + z * z);
}

inline Quaternion Quaternion::normalize() const
{
	double m = mag();
	if (m < TOL) throw QuaternionException("Quaternion magnitude is zero");
	return Quaternion(w / m, x / m, y / m, z / m);
}

inline Quaternion Quaternion::conj() const noexcept
{
	return Quaternion(w, -x, -y, -z);
}

inline Quaternion Quaternion::inv() const
{
	double m = mag();
	if(m < TOL) throw QuaternionException("Quaternion magnitude is zero");
	return conj() / (m * m);
}

inline double Quaternion::dot(const Quaternion& q) const noexcept
{
	return w * q.w + x * q.x + y * q.y + z * q.z;
}

inline Quaternion Quaternion::derivative(const double& p, const double& q, const double& r) const noexcept
{
	return Quaternion(
		-0.5 * (x * p + y * q + z * r),
		0.5 * (w * p + y * r - z * q),
		0.5 * (w * q - x * r + z * p),
		0.5 * (w * r + x * q - y * p)
	);
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
	if (omega < TOL) return start;
	double sinOmega = std::sin(omega);
	return (sin((1 - u) * omega) / sinOmega) * start + (sin(u * omega) / sinOmega) * end;
}

inline Quaternion Quaternion::fromAxisAngle(const double& x, const double& y, const double& z, const double& angle) noexcept
{
	double s = std::sin(angle / 2);
	return Quaternion(std::cos(angle/2), x * s, y * s, z * s);
}

inline std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
	os << std::showpoint << "( " << q.w << " + " << q.x << "i + " << q.y << "j + " << q.z << "k )" << std::noshowpoint;
	return os;
}

inline Quaternion operator+(const double& scalar, const Quaternion& q) noexcept
{
	return Quaternion(scalar + q.w, scalar + q.x, scalar + q.y, scalar + q.z);
}

inline Quaternion operator-(const double& scalar, const Quaternion& q) noexcept
{
	return Quaternion(scalar - q.w, scalar - q.x, scalar - q.y, scalar - q.z);
}

inline Quaternion operator*(const double& scalar, const Quaternion& q) noexcept
{
	return Quaternion(scalar * q.w, scalar * q.x, scalar * q.y, scalar * q.z);
}

inline Quaternion operator/(const double& scalar, const Quaternion& q)
{
	return scalar * q.inv();
}

