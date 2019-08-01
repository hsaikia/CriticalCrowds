/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/

#include "wall.h"
#include "util.h"
#include "walker.h"

Wall::Wall()
{
}

Wall::Wall(const Vector2 & p1, const Vector2 & p2)
{
	p1_ = p1;
	p2_ = p2;
}

double Wall::dist(const Vector2 & p) const
{
	return Util::distWall(p1_, p2_, p);
}

Vector2 Wall::getClosestPoint(const Vector2 & p) const
{
	return Util::getClosestPointOnWall(p1_, p2_, p);
}

double Wall::orientationToWall(const Vector2 & p, const Vector2 & v) const
{
	auto p_c = getClosestPoint(p);
	auto pcpn = (p_c - p).normalized();
	auto vn = v.normalized();
	return std::max(Vector2::dot(pcpn, vn), 0.0);
}

void Wall::drawWall(const double scale, const double r, const double g, const double b) const
{
	glBegin(GL_LINES);
	glColor3d(r, g, b);
	glVertex2d(p1_.x / scale, p1_.y / scale);
	glVertex2d(p2_.x / scale, p2_.y / scale);
	glEnd();
}

void Wall::minDistAndTtmd(const Vector2 & p1, const Vector2 & p2, const Vector2 & pos, const Vector2 & vel, double & minDist, double & ttmd)
{
	Util::minDistAndTtmdWall(p1, p2, pos, vel, minDist, ttmd);
	
	// if current distance is less than the allowed distance
	if (Util::distWall(p1, p2, pos) < Walker::minWallDist) {
		ttmd = 0;
	}

	//// EXPERIMENTAL!! NOT TESTED THOROUGHLY!
	//// put a cap on max distance
	//if (Util::distWall(p1, p2, pos) > Walker::detectionDist) {
	//	ttmd = std::numeric_limits<double>::max();
	//	minDist = std::numeric_limits<double>::max();
	//}
	
}

Target::Target()
{
}

void Target::init(const Vector2 & p1, const Vector2 & p2)
{
	p1_ = p1;
	p2_ = p2;
}
