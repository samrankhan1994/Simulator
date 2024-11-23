#pragma once

#include <iostream>
#include <cmath>

class Quaternion
{
	double w, x, y, z;
public:
	Quaternion();
	Quaternion(double _w, double _x, double _y, double _z);

	Quaternion(const Quaternion& q) = default;
	Quaternion(Quaternion&& q) noexcept = default;

	Quaternion& operator=(const Quaternion& q) = default;
	Quaternion& operator=(Quaternion&& q) = default;

	Quaternion& operator+=(const Quaternion& q);
	Quaternion& operator+=(double scalar);
	Quaternion operator+(const Quaternion& q) const;
	Quaternion operator+(double scalar) const;

	Quaternion& operator-=(const Quaternion& q);
	Quaternion& operator-=(double scalar);
	Quaternion operator-(const Quaternion& q) const;
	Quaternion operator-(double scalar) const;

	Quaternion& operator*=(const Quaternion& q);
	Quaternion& operator*=(double scalar);
	Quaternion operator*(const Quaternion& q);
	Quaternion operator*(double scalar);

	Quaternion& operator/=(double scalar);
	Quaternion operator/(double scalar);

	~Quaternion() = default;

	friend std::ostream& operator<<(std::ostream& os, const Quaternion& q);
	friend Quaternion operator+(double scalar, const Quaternion& q);
	friend Quaternion operator-(double scalar, const Quaternion& q);
	friend Quaternion operator*(double scalar, const Quaternion& q);
};

inline Quaternion::Quaternion() : w(0), x(0), y(0), z(0) {}

inline Quaternion::Quaternion(double _w, double _x, double _y, double _z) : w(_w), x(_x), y(_y), z(_z) {}

inline Quaternion& Quaternion::operator+=(const Quaternion& q)
{
	w += q.w;
	x += q.x;
	y += q.y;
	z += q.z;
	return *this;
}

inline Quaternion& Quaternion::operator+=(double scalar)
{
	w += scalar;
	x += scalar;
	y += scalar;
	z += scalar;
	return *this;
}

inline Quaternion Quaternion::operator+(const Quaternion& q) const
{
	return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

inline Quaternion Quaternion::operator+(double scalar) const
{
	return Quaternion(w + scalar, x + scalar, y + scalar, z + scalar);
}

inline Quaternion& Quaternion::operator-=(const Quaternion& q)
{
	w -= q.w;
	x -= q.x;
	y -= q.y;
	z -= q.z;
	return *this;
}

inline Quaternion& Quaternion::operator-=(double scalar)
{
	w -= scalar;
	x -= scalar;
	y -= scalar;
	z -= scalar;
	return *this;
}

inline Quaternion Quaternion::operator-(const Quaternion& q) const
{
	return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z);
}

inline Quaternion Quaternion::operator-(double scalar) const
{
	return Quaternion(w - scalar, x - scalar, y - scalar, z - scalar);
}

inline Quaternion& Quaternion::operator*=(const Quaternion& q)
{
	*this = Quaternion(
		w * q.w - x * q.x - y * q.y - z * q.z, // w
		w * q.x + x * q.w + y * q.z - z * q.y, // x
		w * q.y - x * q.z + y * q.w + z * q.x, // y
		w * q.z + x * q.y - y * q.x + z * q.w  // z
	);
	return *this;
}

inline Quaternion& Quaternion::operator*=(double scalar)
{
	w = w * scalar;
	x = x * scalar;
	y = y * scalar;
	z = z * scalar;
	return *this;
}

Quaternion Quaternion::operator*(const Quaternion& q)
{
	return Quaternion(
		w * q.w - x * q.x - y * q.y - z * q.z, // w
		w * q.x + x * q.w + y * q.z - z * q.y, // x
		w * q.y - x * q.z + y * q.w + z * q.x, // y
		w * q.z + x * q.y - y * q.x + z * q.w  // z
	);
}

inline Quaternion Quaternion::operator*(double scalar)
{
	return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

inline Quaternion& Quaternion::operator/=(double scalar)
{
	w = w / scalar;
	x = x / scalar;
	y = y / scalar;
	z = z / scalar;
	return *this;
}

inline Quaternion Quaternion::operator/(double scalar)
{
	return Quaternion(w / scalar, x / scalar, y / scalar, z / scalar);
}

inline std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
	os << std::showpoint << "( " << q.w << " + " << q.x << "i + " << q.y << "j + " << q.z << "k )" << std::noshowpoint;
	return os;
}

inline Quaternion operator+(double scalar, const Quaternion& q)
{
	return Quaternion(scalar + q.w, scalar + q.x, scalar + q.y, scalar + q.z);
}

inline Quaternion operator-(double scalar, const Quaternion& q)
{
	return Quaternion(scalar - q.w, scalar - q.x, scalar - q.y, scalar - q.z);
}

inline Quaternion operator*(double scalar, const Quaternion& q)
{
	return Quaternion(scalar * q.w, scalar * q.x, scalar * q.y, scalar * q.z);
}

