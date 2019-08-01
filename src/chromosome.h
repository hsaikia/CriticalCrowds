/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/

#pragma once

#include "walker.h"

class VelocityParameters {
public:
	double vp[2]; // [-1, 0] and [-1, 1]
	
	VelocityParameters() {
		vp[0] = 0.0; // fb
		vp[1] = 0.0; // rl
	}

	void apply(Vector2& vel) const;

	double energyCost() const;

	void initRandom();

	void initBrake();
};

class Actions {
public:

	static const std::string WalkerActions;

	char wa;
	double strength;

	Actions(){
		wa = WalkerActions[0];
		strength = 0.0;
	}

	void apply(Vector2& vel) const;

};

class ChromosomeDE {

private:
	static const double differential_weight; // [0, 2]
public:

	std::vector<VelocityParameters> val;

	ChromosomeDE();

	ChromosomeDE(size_t num_agents);

	void init(size_t num_agents, const std::vector<bool>& ids);

	bool checkSize(const size_t numAgents) const;

	void applyI(Walker& a) const;

	static void crossOverDE(size_t component, VelocityParameters& vi, const VelocityParameters & va0,
		const VelocityParameters & va1, const VelocityParameters & va2, const VelocityParameters & va3,
		const VelocityParameters & va4);
};

//class ChromosomeGA {
//public:
//	std::vector<Actions> val;
//
//	ChromosomeGA();
//
//	ChromosomeGA(size_t num_agents);
//
//	void init(size_t num_agents, const std::vector<bool>& ids);
//
//	double energyCost() const;
//
//	bool checkSize(const size_t numAgents) const;
//
//	void applyI(Walker& a) const;
//
//	static void crossOverGA(Actions& ai, const Actions& a0, const Actions& a1, const Actions& a2, const Actions& a3, const Actions& a4);
//
//};