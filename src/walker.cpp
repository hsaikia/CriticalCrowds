/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#include "walker.h"

const double Walker::accRelaxation = 1;
const double Walker::agentRadius = 0.25;
const double Walker::desiredSpeed = 0.2;
const double Walker::minDist = 3 * agentRadius;
const double Walker::minWallDist = agentRadius;
const double Walker::collDist = 2 * agentRadius;
const double Walker::detectionDist = 50 * agentRadius;
const double Walker::lambda = 0.9;

Walker::Walker()
{
	reachedTarget_ = false;
	hue_ = 0;
}

Walker::Walker(const size_t id, const Vector2 & pos, const std::shared_ptr<Target>& tar)
{
	init(id, pos, tar);
	reachedTarget_ = false;
	hue_ = 0;
}

WalkerLight Walker::getWalkerLight() const
{
	WalkerLight w;
	w.id_ = id_;
	w.pos_ = pos_;
	w.tar_ = tar_;
	w.v_ = v_;
	return w;
}

void Walker::init(const size_t id, const Vector2 & pos, const std::shared_ptr<Target>& tar)
{
	id_ = id;
	pos_ = pos;
	tar_ = tar;
	v_ = getVp();
}

void Walker::setHue(double hue)
{
	hue_ = hue;
}

bool Walker::reachedTarget()
{
	if (reachedTarget_) {
		return true;
	}

	if (getVecToTar().len() < agentRadius) {
		reachedTarget_ = true;
	}
	return reachedTarget_;
}

void Walker::move_a(Vector2 a)
{
	move_v(v_ + a);
}

void Walker::move_v(Vector2 velocity)
{
	if (reachedTarget()) {
		return;
	}

	if (velocity.len() < eps) {
		v_.normalize();
		v_ = v_ * eps;
	}
	else {
		v_ = velocity;
	}

	if (v_.len() > desiredSpeed) {
		v_.normalize();
		v_ = v_ * desiredSpeed;
	}

	pos_ = pos_ + v_;
}

void Walker::move_auto()
{
	if (reachedTarget()) {
		return;
	}

	move_v(v_);
	v_auto();
}

/// v_next = v_old * lambda + v_pref * (1 - lambda)
void Walker::v_auto()
{
	v_ = v_ * lambda + getVp() * (1 - lambda);
}

Vector2 Walker::v_auto(const Vector2 & x, const Vector2 & v) const
{
	return v * lambda + (tar_->getClosestPoint(x) - x).normalized() * Walker::desiredSpeed * (1 - lambda);
}

void Walker::turn_v(Vector2& v, double amount)
{
	amount = std::max(-0.99, amount);
	amount = std::min(0.99, amount);

	v.rotate(0.5 * PII * amount);
}

void Walker::acc_v(Vector2& v, double amount, bool onlyBraking)
{
	amount = std::max(-0.99, amount);

	if (onlyBraking) {
		amount = std::min(0.0, amount);
	}
	else {
		amount = std::min(0.99, amount);
	}

	v = v + v * sin(0.5 * PII * amount);
	
	if (v.len() < eps) {
		v.normalize();
		v = v * eps;
	}

	if (v.len() > desiredSpeed) {
		v.normalize();
		v = v * desiredSpeed;
	}
}

Vector2 Walker::getRight() const
{
	auto f = getForward();
	return Vector2(f.y, -f.x);
}

Vector2 Walker::getForward() const
{
	return v_.normalized();
}

Vector2 Walker::getVecToTar() const
{
	return tar_->getClosestPoint(pos_) - pos_;;
}

Vector2 Walker::getVp() const
{
	auto x = getVecToTar();
	return x.normalized() * std::min(desiredSpeed, x.len());
}

Vector2 Walker::getAp() const
{
	//return (getVp() - v_) / acc_relaxation;
	return (getVp() - v_);
}

Vector2 Walker::sampleAcceleration(const double r, const double s) const
{
	return v_.normalized() * r + getRight() *s;
}

void Walker::drawAgent(const double scale) const
{
	auto fp = getForward();
	auto v1 = pos_ + fp * 0.2;

	fp.rotate(PII / 2);
	auto v2 = pos_ + fp * 0.1;
	auto v3 = pos_ - fp * 0.1;

	double r, g, b;
	Util::HSVToRGB(r, g, b, hue_, 1, 1);

	glBegin(GL_POLYGON);
	glColor3d(r, g, b);
	//divide by scale to fit on screen
	glVertex2d(v1.x / scale, v1.y / scale);
	glVertex2d(v2.x / scale, v2.y / scale);
	glVertex2d(v3.x / scale, v3.y / scale);
	glEnd();

	//draw radius
	int numPoints = 20;
	double ang = 2 * PII / numPoints;
	auto f = getForward() * agentRadius;
	glBegin(GL_LINE_STRIP);
	glColor3d(r, g, b);
	for (int i = 0; i <= numPoints; i++) {
		auto v = pos_ + f;
		glVertex2d(v.x / scale, v.y / scale);
		f.rotate(ang);
	}
	glEnd();
}

void Walker::drawTarget(const double scale) const
{
	double r, g, b;
	Util::HSVToRGB(r, g, b, hue_, 1, 0.5);
	tar_->drawWall(scale, r, g, b);

	auto tcp = tar_->getClosestPoint(pos_);

	Vector2 up = Vector2(0, 0.1);
	up.rotate(PII / 4);

	auto v1 = tcp + up;
	up.rotate(PII / 2);
	auto v2 = tcp + up;
	up.rotate(PII / 2);
	auto v3 = tcp + up;
	up.rotate(PII / 2);
	auto v4 = tcp + up;


	glBegin(GL_POLYGON);
	glColor3d(r, g, b);
	//divide by scale to fit on screen
	glVertex2d(v1.x / scale, v1.y / scale);
	glVertex2d(v2.x / scale, v2.y / scale);
	glVertex2d(v3.x / scale, v3.y / scale);
	glVertex2d(v4.x / scale, v4.y / scale);
	glEnd();
}

bool Walker::sortByID(const WalkerLight & a, const WalkerLight & b)
{
	return a.id_ < b.id_;
}

double Walker::dist(const WalkerLight & a, const WalkerLight & b)
{
	return (a.pos_ - b.pos_).len();
}

void Walker::minDistAndTtmd(const Vector2 & x1, const Vector2 & v1, const Vector2 & x2, const Vector2 & v2, double & minDist, double & ttmd)
{
	Util::minDistAndTTmdAgent(x1, v1, x2, v2, minDist, ttmd);

	// If current distance is less than allowed distance
	if ((x1 - x2).len() < Walker::minDist) {
		ttmd = 0;
	}

	//// EXPERIMENTAL!! NOT TESTED THOROUGHLY!
	//// put a cap on max distance
	//if ((x1 - x2).len() > Walker::detectionDist) {
	//	ttmd = std::numeric_limits<double>::max();
	//	minDist = std::numeric_limits<double>::max();
	//}

}

bool Walker::sortByX(const WalkerLight& a, const WalkerLight& b)
{
	return a.pos_.x < b.pos_.x;
}

bool Walker::sortByY(const WalkerLight& a, const WalkerLight& b)
{
	return a.pos_.y < b.pos_.y;
}

WalkerTrails::WalkerTrails()
{
	hue_ = 0;
}

void WalkerTrails::setHue(double hue)
{
	hue_ = hue;
}

void WalkerTrails::addPointToTrail(const Vector2 & point)
{
	trails.push_back(point);
}

void WalkerTrails::drawTrails(const double scale) const
{
	double r, g, b;
	Util::HSVToRGB(r, g, b, hue_, 1, 0.5);
	glBegin(GL_LINE_STRIP);
	glColor3d(r, g, b);
	for (auto i = 0; i < trails.size(); i++) {
		glVertex2d(trails[i].x / scale, trails[i].y / scale);
	}
	glEnd();
}
