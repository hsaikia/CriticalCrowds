/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/

#pragma once

#include "geometry.h"
#include <GL/glut.h>
#include <algorithm>

class Wall {
public:

	Vector2 p1_;
	Vector2 p2_;

	Wall();

	Wall(const Vector2& p1, const Vector2& p2);

	/// Returns the shortest distance from the wall
	double dist(const Vector2& p) const;

	/// Returns the closest point on the wall
	Vector2 getClosestPoint(const Vector2& p) const;

	double orientationToWall(const Vector2& pos, const Vector2& v) const;

	/// OpenGL draw
	void drawWall(const double scale, const double r, const double g, const double b) const;

	/// minimal distance between a agent and this wall and time it takes for the agent to reach this minimal distance
	static void minDistAndTtmd(const Vector2 & p1, const Vector2 & p2, const Vector2& pos, const Vector2& vel, double& minDist, double& ttmd);
};

class Target : public Wall {
public:
	Target();
	void init(const Vector2& p1, const Vector2& p2);
};