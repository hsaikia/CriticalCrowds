/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#include "util.h"

double Util::sample_uniform(double a, double b)
{
	return a + (b - a) * rand() / double(RAND_MAX);
}

Vector2 Util::gen_random_unit()
{
	Vector2 ret;
	ret.x = sample_uniform(-1, 1);
	ret.y = sample_uniform(-1, 1);
	ret.normalize();
	return ret;
}

double Util::orientation(const Vector2 & v1, const Vector2 & v2)
{
	if (v1.len() < eps || v2.len() < eps) {
		return 0;
	}

	return Vector2::dot(v1, v2) / (v1.len() * v2.len());
}

void Util::HSVToRGB(double & r, double & g, double & b, const double h, const double s, const double v)
{
	double      hh, p, q, t, ff;
	long        i;

	hh = h * 360;
	hh /= 60.0;
	i = (long)hh;
	if (i == 6) i = 0;
	ff = hh - i;
	p = v * (1.0 - s);
	q = v * (1.0 - (s * ff));
	t = v * (1.0 - (s * (1.0 - ff)));

	switch (i) {
	case 0:
		r = v; g = t; b = p; break;
	case 1:
		r = q; g = v; b = p; break;
	case 2:
		r = p; g = v; b = t; break;
	case 3:
		r = p; g = q; b = v; break;
	case 4:
		r = t; g = p; b = v; break;
	case 5:
	default:
		r = v; g = p; b = q; break;
	}
}

void Util::drawFillCircle(const Vector2 o, const double rad, const double scale, const double r, const double g, const double b)
{
	glBegin(GL_TRIANGLE_FAN);
	int N = 20;
	double ang = 2 * PII / N;

	Vector2 R = Vector2(rad, 0);
	glColor3d(r, g, b);
	glVertex2d(o.x / scale, o.y / scale);
	for (auto i = 0; i <= N; i++) {
		auto v = o + R;
		glVertex2d(v.x / scale, v.y / scale);
		R.rotate(ang);
	}
	glEnd();
}

void Util::clamp(double & val, const double & Min, const double & Max)
{
	val = std::max(Min, val);
	val = std::min(Max, val);
}

Vector2 Util::getClosestPointOnWall(const Vector2 & p1, const Vector2 & p2, const Vector2 & p)
{
	auto p12 = p2 - p1;
	auto p10 = p - p1;

	auto p12len = p12.lenSquared();
	p12len = p12len > eps ? p12len : eps;


	auto t = Vector2::dot(p12, p10) / p12len;

	Util::clamp(t, 0, 1);

	// Closest point
	return p1 + p12 * t;
}

double Util::distWall(const Vector2 & p1, const Vector2 & p2, const Vector2 & p)
{
	auto p_c = getClosestPointOnWall(p1, p2, p);
	return (p_c - p).len();
}

void Util::minDistAndTTmdAgent(const Vector2 & x1, const Vector2 & v1, const Vector2 & x2, const Vector2 & v2, double & minDist, double & ttmd)
{
	auto x = x1 - x2;
	auto v = v1 - v2;

	// both agents have zero relative velocity -> they will maintain the same distance forever
	// minimum distance is maintained right now -> ttc = 0
	if (v.len() < eps) {
		minDist = x.len();
		ttmd = 0;
		return;
	}

	if (x.len() < eps) {
		minDist = x.len();
		ttmd = 0;
		return;
	}

	auto cosine_theta = Vector2::dot(x, v) / (x.len() * v.len());

	// Very important!
	// Sometimes overflow may occur
	// ensure that cosine^2 is always in [0, 1] before calculating sine
	auto cosine_theta2 = pow(cosine_theta, 2);
	Util::clamp(cosine_theta2, 0, 1);

	auto sin_theta = sqrt(1 - cosine_theta2);

	ttmd = -x.len() * cosine_theta / v.len();
	minDist = x.len() * sin_theta;
}

void Util::minDistAndTtmdWall(const Vector2 & p1, const Vector2 & p2, const Vector2 & pos, const Vector2 & vel, double & minDist, double & ttmd)
{
	auto a1 = -vel.lenSquared();
	auto b1 = Vector2::dot(vel, p2 - p1);
	auto c1 = Vector2::dot(vel, p1 - pos);

	auto a2 = -Vector2::dot(vel, p2 - p1);
	auto b2 = (p2 - p1).lenSquared();
	auto c2 = Vector2::dot(p1 - pos, p2 - p1);

	auto deno = a1 * b2 - a2 * b1;

	// no collision
	if (deno < eps) {
		minDist = distWall(p1, p2, pos);
		ttmd = 0;
		return;
	}

	// calculate where in the p1p2 line does the closest point lie
	auto s = (a2 * c1 - a1 * c2) / deno;

	// beyond p1, calculate min dist from p1
	if (s < 0) {
		minDistAndTTmdAgent(pos, vel, p1, Vector2(0, 0), minDist, ttmd);
		return;
	}

	// beyond p2, calculate min dist from p2
	if (s > 1) {
		minDistAndTTmdAgent(pos, vel, p2, Vector2(0, 0), minDist, ttmd);
		return;
	}

	ttmd = (b1 * c2 - b2 * c1) / deno;
	minDist = (p1 + (p2 - p1) * s - (pos + vel * ttmd)).len(); // should be zero
}

void Util::BoolArrayOR(const std::vector<bool>& a1, const std::vector<bool>& a2, std::vector<bool>& a3)
{
	if (a1.size() != a2.size()) {
		printf("Error! Bool Array OR was passed incorrect input arrays!\n");
		return;
	}
	a3.resize(a1.size());

	for (auto i = 0; i < a3.size(); i++) {
		a3[i] = a1[i] || a2[i];
	}
}

int Util::numTrue(const std::vector<bool> a)
{
	int ret = 0;
	for (const auto& e : a) {
		if (e) {
			ret++;
		}
	}
	return ret;
}

std::vector<size_t> Util::getTrue(const std::vector<bool> a)
{
	std::vector<size_t> ret;
	for (size_t i = 0; i < a.size(); i++) {
		if (a[i]) {
			ret.push_back(i);
		}
	}
	return ret;
}

