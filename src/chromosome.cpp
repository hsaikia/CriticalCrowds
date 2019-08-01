/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/

#include "chromosome.h"


const std::string Actions::WalkerActions = { 'F' /* first item is the default */, 'B', 'R', 'L' };

const double ChromosomeDE::differential_weight = 0.5;

void VelocityParameters::apply(Vector2& vel) const
{
	//auto af = vp[0];
	//auto rl = vp[1];
	//Util::clamp(af, -0.99, 0);
	//Util::clamp(rl, -0.99, 0.99);

	//auto left = vel.normalized();
	//left.rotate(PII / 2);

	//vel = vel + vel.normalized() * af + left * rl;

	//if (vel.len() < eps) {
	//	vel.normalize();
	//	vel = vel * eps;
	//}

	//if (vel.len() > Walker::desiredSpeed) {
	//	vel.normalize();
	//	vel = vel * Walker::desiredSpeed;
	//}

	Walker::turn_v(vel, vp[1]);
	Walker::acc_v(vel, vp[0], true); // only braking
}

double VelocityParameters::energyCost() const
{
	double ret = 0.0;
	ret += abs(vp[0]);
	ret += 8 * abs(vp[1]); // more energy to turn than to brake
	return ret;
}

void VelocityParameters::initRandom()
{
	// deceleration can be large 
	// large turns however are bad
	//acc
	vp[0] = Util::sample_uniform(-1, 0);

	//turn
	vp[1] = Util::sample_uniform(-1, 1);
}

void VelocityParameters::initBrake()
{
	vp[0] = -0.99;
	vp[1] = 0.0;
}

ChromosomeDE::ChromosomeDE()
{
}

ChromosomeDE::ChromosomeDE(size_t num_agents)
{
	val.resize(num_agents, VelocityParameters());
}

void ChromosomeDE::init(size_t num_agents, const std::vector<bool>& ids)
{
	val.resize(num_agents, VelocityParameters());
	for (auto i = 0; i < ids.size(); i++) {

		// this agent is not connected rigidly to other agents
		// free to move with current velocity
		if (!ids[i]) {
			continue;
		}

		// deceleration can be large 
		// large turns however are bad
		//acc
		val[i].vp[0] = Util::sample_uniform(-1, 0);

		//turn
		val[i].vp[1] = Util::sample_uniform(-1, 1);

	}
}

bool ChromosomeDE::checkSize(size_t numAgents) const
{
	return val.size() == numAgents;
}

void ChromosomeDE::applyI(Walker & a) const
{
	val[a.id_].apply(a.v_);
}

void ChromosomeDE::crossOverDE(size_t comp, VelocityParameters& vi, const VelocityParameters & va0,
	const VelocityParameters & va1, const VelocityParameters & va2, const VelocityParameters & va3,
	const VelocityParameters & va4)
{
	/*vi.fb = va0.fb + differential_weight * (va1.fb - va2.fb + va3.fb - va4.fb);
	vi.rl = va0.rl + differential_weight * (va1.rl - va2.rl + va3.rl - va4.rl);

	Util::clamp(vi.fb, -1, 0);
	Util::clamp(vi.rl, -1, 1);*/

	vi.vp[comp] = va0.vp[comp] + differential_weight * (va1.vp[comp] - va2.vp[comp] + va3.vp[comp] - va4.vp[comp]);

	if (comp == 0) {
		Util::clamp(vi.vp[comp], -1, 0);
	}
	else {
		Util::clamp(vi.vp[comp], -1, 1);
	}
}

void Actions::apply(Vector2& vel) const
{
	if (wa == 'F') {
		// do nothing
	}
	else if (wa == 'B') {
		Walker::acc_v(vel, -strength, true);
	}
	else if (wa == 'L') {
		Walker::turn_v(vel, strength);
	}
	else if (wa == 'R') {
		Walker::turn_v(vel,-strength);
	}
}

//ChromosomeGA::ChromosomeGA()
//{
//}
//
//ChromosomeGA::ChromosomeGA(size_t num_agents)
//{
//	val.resize(num_agents);
//}
//
//void ChromosomeGA::init(size_t num_agents, const std::vector<bool>& ids)
//{
//	val.resize(num_agents);
//	for (auto i = 0; i < ids.size(); i++) {
//
//		// this agent is not connected rigidly to other agents
//		// free to move with current velocity
//		if (!ids[i]) {
//			continue;
//		}
//
//		// choose randomly between B, L and R
//		val[i].wa = Actions::WalkerActions[1 + rand() % 3] ;
//
//		//turn
//		val[i].strength = Util::sample_uniform(0.0, 1.0);
//
//	}
//}
//
//double ChromosomeGA::energyCost() const
//{
//	double ret = 0.0;
//	for (auto i = 0; i < val.size(); i++) {
//		if (val[i].wa == 'B') {
//			ret += abs(val[i].strength);
//		}
//		else {
//			ret += 8 * abs(val[i].strength);
//		}
//	}
//	return ret;
//}
//
//bool ChromosomeGA::checkSize(size_t numAgents) const
//{
//	return val.size() == numAgents;
//}
//
//void ChromosomeGA::applyI(Walker & a) const
//{
//	val[a.id_].apply(a);
//}
//
//void ChromosomeGA::crossOverGA(Actions & ai, const Actions & a0, const Actions & a1, const Actions & a2, const Actions & a3, const Actions & a4)
//{
//	auto type = rand() % 6;
//	char action;
//	switch (type) {
//	case 0: action = ai.wa;
//		break;
//	case 1: action = a0.wa;
//		break;
//	case 2: action = a1.wa;
//		break;
//	case 3: action = a2.wa;
//		break;
//	case 4: action = a3.wa;
//		break;
//	case 5: action = a4.wa;
//		break;
//	}
//
//	double val = 0.0;
//	int occurences = 0;
//
//	if (ai.wa == action) {
//		val += ai.strength;
//		occurences++;
//	}
//	if (a0.wa == action) {
//		val += a0.strength;
//		occurences++;
//	}
//	if (a1.wa == action) {
//		val += a1.strength;
//		occurences++;
//	}
//	if (a2.wa == action) {
//		val += a2.strength;
//		occurences++;
//	}
//	if (a3.wa == action) {
//		val += a3.strength;
//		occurences++;
//	}
//	if (a4.wa == action) {
//		val += a4.strength;
//		occurences++;
//	}
//	ai.wa = action;
//	ai.strength = val / occurences;
//}
