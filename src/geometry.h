/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#pragma once
#include <cmath>

const double PII = 3.14159265359;
const double eps = 0.0001;

class Vector2 {
public:
	double x;
	double y;
	// counter-clockwise rotation
	// equivalent to turning left
	void rotate(const double angle) {
		auto x_temp = x * cos(angle) - y * sin(angle);
		auto y_temp = x * sin(angle) + y * cos(angle);
		x = x_temp;
		y = y_temp;
	}

	Vector2();
	Vector2(const double x_, const double y_);
	void set(const double x_, const double y_);

	Vector2 operator+(const Vector2& vec) const;
	Vector2 operator-(const Vector2& vec) const;
	Vector2 operator-() const;
	Vector2 operator*(const double& f) const;
	Vector2 operator/(const double& f) const;
	
	Vector2 normalized() const;
	void normalize();
	double len() const;
	double lenSquared() const;

	/// signed angle between this vector and vec
	double angle(const Vector2& vec) const;

	/// magnitude of cross product in +Z
	static double crossProdSigned(const Vector2& v1, const Vector2& v2);

	static double dot(const Vector2& v1, const Vector2& v2);

	/// outputs the projected vector of this vector along the given vector
	Vector2 projAlong(const Vector2& vec) const;
};


