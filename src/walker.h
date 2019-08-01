/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#pragma once

#include "geometry.h"
#include "util.h"
#include <vector>
#include "wall.h"
#include <memory>

class WalkerLight {
public:
	/// id for agent
	size_t id_;

	/// position of agent
	Vector2 pos_;

	/// Target
	std::shared_ptr<Target> tar_;

	/// veclocity of agent
	Vector2 v_;

	WalkerLight() {};
};

class Walker : public WalkerLight {
public:
	/// Colour of walker
	double hue_;
	/// This is the number of moves it takes for the velocity to align to the targetDirection
	static const double accRelaxation;

	/// radius of comfort and no collision
	static const double agentRadius;

	/// Normal speed of an agent, speed never exceeds this speed
	static const double desiredSpeed;

	/// Minimum distance needed between two agents
	static const double minDist;

	/// Collision distance
	static const double collDist;

	static const double detectionDist;

	/// Wall Dist
	static const double minWallDist;

	/// Velocity relaxation factor [0, 1]
	/// v_next = v_old * lambda + v_pref * (1 - lambda)
	static const double lambda;

	/// Default Constructor and Destructor
	Walker();
	
	/// Constructor
	Walker(const size_t id, const Vector2& pos, const std::shared_ptr<Target>& tar);

	bool reachedTarget_;

	/// Get a lighter version of Walker
	WalkerLight getWalkerLight() const;

	/// Initialize
	void init(const size_t id, const Vector2& pos, const std::shared_ptr<Target>& tar);

	/// Set colour
	void setHue(double hue);

	/// has the agent reached the target?
	bool reachedTarget();

	/// move agent using given acceleration
	void move_a(Vector2 acceleration);

	/// move agent using given velocity
	void move_v(Vector2 velocity);

	/// move, and then relax velocity towards preferred velocity
	void move_auto();

	/// relax automatically towards preferred velocity
	void v_auto();

	/// given a specific x and v
	Vector2 v_auto(const Vector2& x, const Vector2& v) const;

	/// +ve left, -ve right
	/// 1 -> pi / 2, -1 -> -pi / 2
	/// acceptable values [-0.99, 0.99]
	static void turn_v(Vector2& v, double amount);

	/// [0,1] -> no brake
	/// -1 -> full brake
	/// acceptable values [-0.99, 0.99]
	static void acc_v(Vector2& v, double amount, bool onlyBraking);

	/// Right vector
	Vector2 getRight() const;

	/// Forward vector
	Vector2 getForward() const;

	/// Vector to Target
	Vector2 getVecToTar() const;

	/// Preferred velocity relative to agent pos
	Vector2 getVp() const;

	/// Preferred acceleration relative to agent pos
	Vector2 getAp() const;

	/// Generates a unit vector rF + sR, F = forward, R = right
	Vector2 sampleAcceleration(const double r, const double s) const;

	///OpenGL Draw functions
	void drawAgent(const double scale) const;
	void drawTarget(const double scale) const;
	
	/// Sorts agents according to their X position
	static bool sortByX(const WalkerLight& a, const WalkerLight& b);
	
	/// Sorts agents according to their Y position
	static bool sortByY(const WalkerLight& a, const WalkerLight& b);

	/// Sorts agents according to their ID
	static bool sortByID(const WalkerLight& a, const WalkerLight& b);

	/// Distance between two walkers
	static double dist(const WalkerLight& a, const WalkerLight& b);

	static void minDistAndTtmd(const Vector2 & x1, const Vector2 & v1, const Vector2 & x2, const Vector2 & v2, double& minDist, double& ttmd);

};

class WalkerTrails {
private:
	std::vector<Vector2> trails;
	double hue_;
public:
	WalkerTrails();
	/// Set colour
	void setHue(double hue);
	/// add point to trail 
	void addPointToTrail(const Vector2& point);
	/// Draw Trails
	void drawTrails(const double scale) const;

};