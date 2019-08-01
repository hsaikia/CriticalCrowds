/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#include "geometry.h"

Vector2::Vector2()
{
	x = 0;
	y = 0;
}

Vector2::Vector2(const double x_, const double y_)
{
	x = x_;
	y = y_;
}

void Vector2::set(const double x_, const double y_)
{
	x = x_;
	y = y_;
}

Vector2 Vector2::operator+(const Vector2 & vec) const
{
	return Vector2(x + vec.x, y + vec.y);
}

Vector2 Vector2::operator-(const Vector2 & vec) const
{
	return Vector2(x - vec.x, y - vec.y);
}

Vector2 Vector2::operator-() const
{
	return Vector2(-x, -y);
}

Vector2 Vector2::operator*(const double & f) const
{
	return Vector2(x * f, y * f);
}

Vector2 Vector2::operator/(const double & f) const
{
	return Vector2(x / f, y / f);
}

double Vector2::dot(const Vector2 & v1, const Vector2 & v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

Vector2 Vector2::normalized() const
{
	auto l = len();
	if (l > 0.0) {
		return Vector2(x / l, y / l);
	}
	return Vector2(x, y);
}

void Vector2::normalize()
{
	auto l = len();
	if (l > 0.0) {
		x /= l;
		y /= l;
	}
}

double Vector2::len() const
{
	return sqrt(lenSquared());
}

double Vector2::lenSquared() const
{
	return x * x + y * y;
}

double Vector2::angle(const Vector2 & vec) const
{
	auto u = normalized();
	auto v = vec.normalized();

	auto w = u - v;

	if (w.len() < eps) {
		return 0;
	}

	int sign = crossProdSigned(*this, vec) < 0 ? -1 : 1;

	// acos(x) = Principal arc cosine of x, in the interval[0, pi] radians
	return sign * acos(dot(u, v));
}

double Vector2::crossProdSigned(const Vector2 & v1, const Vector2 & v2)
{
	return v1.x * v2.y - v1.y * v2.x;
}

Vector2 Vector2::projAlong(const Vector2 & vec) const
{
	if (vec.len() < eps) {
		return Vector2();
	}

	auto D = dot(*this, vec);

	if (D < 0) {
		return Vector2();
	}

	return vec * dot(*this, vec) / vec.lenSquared();
}

