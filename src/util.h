/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#pragma once
#include <cstdlib>
#include <random>
#include "geometry.h"
#include <GL/glut.h>

class Util{
public:
/// generates a random number in the closed interval (a, b)
static double sample_uniform(double a, double b);

/// generates a random unit vector
static Vector2 gen_random_unit();

/// cosine of angle between two vectors
static double orientation(const Vector2& v1, const Vector2& v2);

/// rgbhsv in [0,1]
static void  HSVToRGB(double& r, double& g, double& b, const double h, const double s, const double v);

/// draw a filled circle
static void drawFillCircle(const Vector2 o, const double rad, const double scale, const double r, const double g, const double b);

/// clamp a value
static void clamp(double& val, const double& Min, const double& Max);

/// get closest point to a wall
static Vector2 getClosestPointOnWall(const Vector2 & p1, const Vector2 & p2, const Vector2& p);

/// get closest distance from a wall
static double distWall(const Vector2 & p1, const Vector2 & p2, const Vector2& p);

/// get closest distance that two agents may approach, also the time to reach this min distance
static void minDistAndTTmdAgent(const Vector2 & x1, const Vector2 & v1, const Vector2 & x2, const Vector2 & v2, double& minDist, double& ttc);

/// get closest distance that an agent may approach to a wall, also the time to reach this min distance
static void minDistAndTtmdWall(const Vector2 & p1, const Vector2 & p2, const Vector2& pos, const Vector2& vel, double& minDist, double& ttmd);

/// Perform a component wise OR operation
static void BoolArrayOR(const std::vector<bool>& a1, const std::vector<bool>& a2, std::vector<bool>& a3);

/// Returns the number of true entries
static int numTrue(const std::vector<bool> a);

/// Returns the true entries
static std::vector<size_t> getTrue(const std::vector<bool> a);

};