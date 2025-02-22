#pragma once
#include <iostream>
#include <cmath>

class Vector3DException : public std::runtime_error
{
public:
    explicit Vector3DException(const char* msg) : std::runtime_error(msg) {}
};

class Vector3D {
private:
    double data[3];  // Store x, y, z components

public:
    // Constructors
    constexpr Vector3D();
    constexpr Vector3D(double x, double y, double z);

    // Defaults
    constexpr Vector3D(const Vector3D& other) = default;
    constexpr Vector3D(Vector3D&& other) noexcept = default;
    constexpr Vector3D& operator=(const Vector3D& other) = default;
    constexpr Vector3D& operator=(Vector3D&& other) noexcept = default;
    
    // Accessors
    constexpr double& operator[](int index) noexcept;
    constexpr const double& operator[](int index) const noexcept;
    constexpr double& operator()(int index) noexcept;
    constexpr const double& operator()(int index) const noexcept;

    // Vector operations
    constexpr Vector3D& operator+=(const Vector3D& other) noexcept;
    constexpr Vector3D operator+(const Vector3D& other) const noexcept;

    constexpr Vector3D& operator+=(const double& scalar) noexcept;
    constexpr Vector3D operator+(const double& scalar) const noexcept;

    constexpr Vector3D& operator-=(const Vector3D& other) noexcept;
    constexpr Vector3D operator-(const Vector3D& other) const noexcept;

    constexpr Vector3D& operator-=(const double& scalar) noexcept;
    constexpr Vector3D operator-(const double& scalar) const noexcept;
    
    constexpr Vector3D& operator*=(const double& scalar) noexcept;
    constexpr Vector3D operator*(const double& scalar) const noexcept;

    bool operator==(const Vector3D& other) const noexcept;

    // Dot product
    constexpr double dot(const Vector3D& other) const noexcept;

    // Cross product
    constexpr Vector3D cross(const Vector3D& other) const noexcept;

    // Length of the vector
    double length() const noexcept;

    // Normalize the vector
    void normalize();
    Vector3D normalize() const;

    constexpr const double (&getData()const noexcept)[3];

    ~Vector3D() = default;

    constexpr friend Vector3D operator+(const double& scalar, const Vector3D& other)
    {
        return Vector3D(other.data[0] + scalar, other.data[1] + scalar, other.data[2] + scalar);
    }

    constexpr friend Vector3D operator-(const double& scalar, const Vector3D& other)
    {
        return Vector3D(scalar - other.data[0], scalar - other.data[1], scalar - other.data[2]);
    }

    constexpr friend Vector3D operator*(const double& scalar, const Vector3D& other)
    {
        return Vector3D(scalar * other.data[0], scalar * other.data[1], scalar * other.data[2]);
    }

    // Output stream operator
    friend std::ostream& operator<<(std::ostream& os, const Vector3D& vec) {
        os << "(" << vec[0] << ", " << vec[1] << ", " << vec[2] << ")";
        return os;
    }
};


inline constexpr Vector3D::Vector3D() : data{ 0.0, 0.0, 0.0 } {}

inline constexpr Vector3D::Vector3D(double x, double y, double z) : data{ x, y, z } {}

inline constexpr double& Vector3D::operator[](int index) noexcept
{
    return data[index];
}

inline constexpr const double& Vector3D::operator[](int index) const noexcept
{
    return data[index];
}

inline constexpr double& Vector3D::operator()(int index) noexcept
{
    return data[index];
}

inline constexpr const double& Vector3D::operator()(int index) const noexcept
{
    return data[index];
}

inline constexpr Vector3D& Vector3D::operator+=(const Vector3D& other) noexcept
{
    data[0] += other[0];
    data[1] += other[1];
    data[2] += other[2];
    return *this;
}

inline constexpr Vector3D Vector3D::operator+(const Vector3D& other) const noexcept
{
    return Vector3D(data[0] + other[0], data[1] + other[1], data[2] + other[2]);
}

inline constexpr Vector3D& Vector3D::operator+=(const double& scalar) noexcept
{
    data[0] += scalar;
    data[1] += scalar;
    data[2] += scalar;
    return *this;
}

inline constexpr Vector3D Vector3D::operator+(const double& scalar) const noexcept
{
    return Vector3D(data[0] + scalar, data[1] + scalar, data[2] + scalar);
}

inline constexpr Vector3D& Vector3D::operator-=(const Vector3D& other) noexcept
{
    data[0] -= other[0];
    data[1] -= other[1];
    data[2] -= other[2];
    return *this;
}

inline constexpr Vector3D Vector3D::operator-(const Vector3D& other) const noexcept
{
    return Vector3D(data[0] - other[0], data[1] - other[1], data[2] - other[2]);
}

inline constexpr Vector3D Vector3D::operator-(const double& scalar) const noexcept
{
    return Vector3D(data[0] - scalar, data[1] - scalar, data[2] - scalar);
}

inline constexpr Vector3D& Vector3D::operator-=(const double& scalar) noexcept
{
    data[0] -= scalar;
    data[1] -= scalar;
    data[2] -= scalar;
    return *this;
}

inline constexpr Vector3D& Vector3D::operator*=(const double& scalar) noexcept
{
    data[0] *= scalar;
    data[1] *= scalar;
    data[2] *= scalar;
    return *this;
}

inline constexpr Vector3D Vector3D::operator*(const double& scalar) const noexcept
{
    return Vector3D(data[0] * scalar, data[1] * scalar, data[2] * scalar);
}

inline bool Vector3D::operator==(const Vector3D& other) const noexcept
{
    return memcmp(data, other.data, sizeof(double)) == 0;
}

inline constexpr double Vector3D::dot(const Vector3D& other) const noexcept
{
    return data[0] * other[0] + data[1] * other[1] + data[2] * other[2];
}

inline constexpr Vector3D Vector3D::cross(const Vector3D& other) const noexcept
{
    return Vector3D(data[1] * other[2] - data[2] * other[1],
        data[2] * other[0] - data[0] * other[2],
        data[0] * other[1] - data[1] * other[0]);
}

inline double Vector3D::length() const noexcept
{
    return std::sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]);
}

inline void Vector3D::normalize()
{
    double len = length();
    if (len == 0)
        throw Vector3DException("Vector3D Exception: Cannot Normalize a zero vector");
    else
    {
        data[0] /= len;
        data[1] /= len;
        data[2] /= len;
    }
}

inline Vector3D Vector3D::normalize() const
{
    double len = length();
    if (len == 0)
        throw Vector3DException("Vector3D Exception: Cannot Normalize a zero vector");
    return Vector3D(data[0] / len, data[1] / len, data[2] / len);
}

inline constexpr const double(&Vector3D::getData() const noexcept)[3]
{
    return data;
}

