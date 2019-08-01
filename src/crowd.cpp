/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#include "crowd.h"
#include <climits>
#include <map>
const double Crowd::crossover_prob_ = 0.8;

/// A function which takes in all agents and walls return the agent(s) who have the shortest
/// time to collide with another agent or a wall
void Crowd::resolve(const SimulationCache & cache, const std::vector<bool>& processsedAgents, std::vector<size_t>& toResolveNow, bool multi)
{
	toResolveNow.clear();
	std::vector<int> belongsTo;
	std::vector<CriticalPair> cps;
	cache.findCriticalClusters(processsedAgents, belongsTo, cps);

	//for (int i = 0; i < belongsTo.size(); i++) {
	//	printf("%d -> %d \n", i, belongsTo[i]);
	//}
	
	if (cps.size() == 0) {
		return;
	}

	if (multi) {
		for (auto& cp : cps) {
			toResolveNow.push_back(cp.i);
			if (cp.j < cache.numAgents_) {
				toResolveNow.push_back(cp.j);
			}
		}
	}
	else {
		// find the lowest ttmd out of all clusters

		CriticalPair best;
		best.ttmd = 1e8;
		for (auto& cp : cps) {
			if (best.ttmd > cp.ttmd) {
				best = cp;
			}
		}
		toResolveNow.push_back(best.i);
		if (best.j < cache.numAgents_) {
			toResolveNow.push_back(best.j);
		}
	}

	//for (auto& tbr : toResolveNow) {
	//	if (processsedAgents[tbr]) {
	//		printf("Error! Processed agent added again!\n");
	//	}
	//	printf("[Resolve] returning %d\n", tbr);
	//}

}

/// Given a list of processed agents, a list of agents to optimize (usually one or two) and a chromosome (to act on the processed agents)
/// determine the simulation score
double Crowd::applyAndScore(
	const SimulationCache & cache, 
	const std::vector<bool>& processsedAgents, 
	const std::vector<size_t>& toResolveAgents, 
	const std::vector<VelocityParameters>& toApply,
	const Crowd& c)
{
	double retConfig = 0.0, retEnergy = 0.0;

	const double decayFactor = 1.0;

	/*if (toResolveAgents.size() == 1) {
		retConfig += cache.testAndScoreConfig(toResolveAgents[0], toApply[0], processsedAgents, decayFactor, c);
	}
	else if (toResolveAgents.size() == 2) {
		retConfig += cache.testAndScoreCouple(toResolveAgents[0], toResolveAgents[1], toApply[0], toApply[1], decayFactor, c);
		retConfig += cache.testAndScoreConfig(toResolveAgents[0], toApply[0], processsedAgents, decayFactor, c);
		retConfig += cache.testAndScoreConfig(toResolveAgents[1], toApply[1], processsedAgents, decayFactor, c);
	}
	else {
		printf("Error! More than 2 agents need to be resolved!\n");
	}*/

	retConfig = cache.testAndScoreConfig(toResolveAgents, toApply, decayFactor, c);

	for (auto& v : toApply) {
		retEnergy += v.energyCost();
	}

	const double gamma = 2.0;
	return retConfig / (cache.numAgents_ + cache.numWalls_) - gamma * retEnergy;
}

void Crowd::applyChromosomeSelective(std::vector<Walker>& agents, const ChromosomeDE & ch)
{
	for (auto& a : agents) {
		ch.applyI(a);
	}
}

void Crowd::mergeIntoChromosome(ChromosomeDE & ch, const std::vector<VelocityParameters>& toMerge, const std::vector<size_t>& toResolveAgents)
{
	if (toResolveAgents.size() != toMerge.size()) {
		printf("Error! Merging to Chromosome called with incorrect dimensions 1\n");
		return;
	}

	int j = 0;
	for (auto& id : toResolveAgents) {
		ch.val[id] = toMerge[j];
		j++;
	}
}

/// Solve function
/// Given a list of processed agents, a list of agents to optimize (usually one or two) and a chromosome (to act on the processed agents)
/// Optimizes the actions for only the couple of agents and updates the chromosome
void Crowd::solveLazy(
	SimulationCache& cache,
	std::vector<bool>& processsedAgents, 
	std::vector<size_t>& toResolveAgents, 
	ChromosomeDE & bestSoFar,
	const Crowd& c)
{
	// All agents are processed, return and make a move
	if (Util::numTrue(processsedAgents) == cache.numAgents_) {
		return;
	}

	// See if any agent needs resolving
	resolve(cache, processsedAgents, toResolveAgents, multiAgentResolution);

	// No agent needs resolving, set all as processed

	if (toResolveAgents.size() == 0) {
		for (auto i = 0; i < processsedAgents.size(); i++) {
			processsedAgents[i] = true;
		}
		return;
	}

	printf("Number of Agents to Resolve %d\n", toResolveAgents.size());

	double bestScore = std::numeric_limits<double>::lowest();
	size_t bestIdx = 0;

	std::vector<double> scoreMap(pop_size_);

	std::vector<std::vector<VelocityParameters> > population(pop_size_, std::vector<VelocityParameters>(toResolveAgents.size()));

	for (auto i = 0; i < population.size(); i++) {

		if (i != 0) {
			for (auto j = 0; j < toResolveAgents.size(); j++) {
				population[i][j].initRandom();
			}
		}
		scoreMap[i] = applyAndScore(cache, processsedAgents, toResolveAgents, population[i], c);
		bestScore = std::max(bestScore, scoreMap[i]);
	}

	// create a perturbation vector for random sampling
	std::vector<size_t> pop_idx(pop_size_);
	for (auto i = 0; i < pop_size_; i++) {
		pop_idx[i] = i;
	}

	int iter = int(iterations_);
	while (iter--) {

		for (auto i = 0; i < pop_size_; i++) {
			size_t a[5];


			for (int rn = 0; rn < 5; rn++) {
				unsigned long long rpos;
				while (true) {
					rpos = rand() % (pop_size_ - rn);
					a[rn] = pop_idx[rpos];

					if (a[rn] != i) {
						break;
					}
				}
				std::swap(pop_idx[rpos], pop_idx[pop_size_ - 1 - rn]);
			}

			std::vector<VelocityParameters> replacement(toResolveAgents.size());

			for (auto j = 0; j < population[i].size(); j++) {

				for (auto comp = 0; comp < 2; comp++) {

					double r = Util::sample_uniform(0, 1);
					if (r < Crowd::crossover_prob_) {
						//cross over
						ChromosomeDE::crossOverDE(comp, replacement[j], population[a[0]][j],
							population[a[1]][j], population[a[2]][j],
							population[a[3]][j], population[a[4]][j]);
					}
					else {
						replacement[j].vp[comp] = population[i][j].vp[comp];
					}
				}
			}

			auto score_rep = applyAndScore(cache, processsedAgents, toResolveAgents, replacement, c);

			if (score_rep > scoreMap[i]) {
				population[i] = replacement;
				scoreMap[i] = score_rep;
			}

			if (bestScore < scoreMap[i]) {
				bestIdx = i;
				bestScore = scoreMap[i];
			}

		}
	}
	
	mergeIntoChromosome(bestSoFar, population[bestIdx], toResolveAgents);
	addToProcessed(processsedAgents, toResolveAgents);

	// add to cache!
	for (int i = 0; i < toResolveAgents.size(); i++) {
		cache.alterVelocity0(toResolveAgents[i], population[bestIdx][i], c);
	}

	solveLazy(cache, processsedAgents, toResolveAgents, bestSoFar, c);

}

void Crowd::solveLazyPSO(
	SimulationCache& cache, 
	std::vector<bool>& processsedAgents, 
	std::vector<size_t>& toResolveAgents, 
	ChromosomeDE & bestSoFar,
	const Crowd& c)
{
	// All agents are processed, return and make a move
	if (Util::numTrue(processsedAgents) == cache.numAgents_) {
		return;
	}

	// See if any agent needs resolving
	resolve(cache, processsedAgents, toResolveAgents, multiAgentResolution);

	// No agent needs resolving, set all as processed

	if (toResolveAgents.size() == 0) {
		for (auto i = 0; i < processsedAgents.size(); i++) {
			processsedAgents[i] = true;
		}
		return;
	}

	//printf("Processed agents (before) %d [", Util::numTrue(processsedAgents));
	//for (int i = 0; i < processsedAgents.size(); i++) {
	//	if (processsedAgents[i]) {
	//		printf(" %d ", i);
	//	}
	//}
	//printf("]\n");

	printf("Number of Agents to Resolve %d\n", toResolveAgents.size());

	//for (int i = 0; i < toResolveAgents.size(); i++) {
	//	printf("Agent %d to Resolve\n", toResolveAgents[i]);
	//}

	std::vector<double> scoreMap(pop_size_);
	
	Swarm swarm(pop_size_, toResolveAgents.size());

	// initialize scores
	for (auto i = 0; i < pop_size_; i++) {
		scoreMap[i] = applyAndScore(cache, processsedAgents, toResolveAgents, swarm.particles_[i].pos_, c);
		swarm.setScore(i, scoreMap[i]);
	}

	// repeat until convergence
	int iter = 0;
	while (true) {

		if (swarm.stoppingCriteriaReached() || iter == iterations_) {
			break;
		}

		iter++;
		swarm.updateSwarm();

#pragma omp parallel for
		for (auto i = 0; i < pop_size_; i++) {
			scoreMap[i] = applyAndScore(cache, processsedAgents, toResolveAgents, swarm.particles_[i].pos_, c);
			swarm.setScore(i, scoreMap[i]);
		}
	}

	printf("Best Particle duration %d\n", swarm.bestDuration_);
	mergeIntoChromosome(bestSoFar, swarm.swarmBest_, toResolveAgents);
	addToProcessed(processsedAgents, toResolveAgents);

	//printf("Processed agents (after) %d [", Util::numTrue(processsedAgents));
	//for (int i = 0; i < processsedAgents.size(); i++) {
	//	if (processsedAgents[i]) {
	//		printf(" %d ", i);
	//	}
	//}
	//printf("]\n");
	// add to cache!
	for (int i = 0; i < toResolveAgents.size(); i++) {
		cache.alterVelocity0(toResolveAgents[i], swarm.swarmBest_[i], c);
	}

	solveLazyPSO(cache, processsedAgents, toResolveAgents, bestSoFar, c);
}

void Crowd::applyChromosome(std::vector<Walker>& agents, const ChromosomeDE & ch)
{
	if (!ch.checkSize(agents.size())) {
		printf("Error! Size of Chromosome and Agents do not match!\n");
		return;
	}

	for (auto i = 0; i < agents.size(); i++) {
		ch.applyI(agents[i]);
	}
}

void Crowd::moveAgentsAuto(std::vector<Walker>& agents)
{
	for (auto& a : agents) {
		a.move_auto();
	}
}

void Crowd::addToProcessed(std::vector<bool>& processed, const std::vector<size_t>& resolved)
{
	for (auto& r : resolved) {
		processed[r] = true;
	}
}

void Crowd::init(const size_t n, std::string outputFileAgents)
{
	srand(unsigned int(time(NULL)));
	agents_.resize(n);
	trails_.resize(n);

	// initialize time and output file
	timeStep_ = 0;
	outputFileAgents_.open(outputFileAgents);
	
	outputFileAgents_ << "TIME ,ID ,POS_X ,POS_Y, TAR_X, TAR_Y, AGENT_RADIUS, COLOR_R, COLOR_G, COLOR_B \n";
	

	startTime_ = std::clock();
}

void Crowd::setHue(size_t i, double hue)
{
	agents_[i].setHue(hue);
	trails_[i].setHue(hue);
}

void Crowd::addWall(const Wall & w)
{
	walls_.push_back(w);
}

void Crowd::writeWallsToFile(std::string outputFile)
{
	std::ofstream outputFileWalls;
	outputFileWalls.open(outputFile);
	outputFileWalls << "ID, P1_X, P1_Y, P2_X, P2_Y\n";

	for (int i = 0; i < walls_.size(); i++) {
		outputFileWalls << i << ","
			<< walls_[i].p1_.x << ","
			<< walls_[i].p1_.y << ","
			<< walls_[i].p2_.x << ","
			<< walls_[i].p2_.y << "\n";
	}
	outputFileWalls.close();
}

void Crowd::moveAgentsGUI(bool& done)
{
	SimulationCache sc;
	sc.init(tau_, *this);

	//sc.findCriticalClusters();

	auto numAgents = agents_.size();

	// Add the current point to the output and trails
	for (auto id = 0; id < agents_.size(); id++) {
		double r, g, b;
		Util::HSVToRGB(r, g, b, agents_[id].hue_, 1, 1);

		auto targetPoint = agents_[id].tar_->getClosestPoint(agents_[id].pos_);

		outputFileAgents_ << timeStep_ << "," << id 
			<< "," << agents_[id].pos_.x << "," << agents_[id].pos_.y 
			<< "," << targetPoint.x << "," << targetPoint.y 
			<< "," << Walker::agentRadius 
			<< "," << r << "," << g << "," << b << "\n";
		trails_[id].addPointToTrail(agents_[id].pos_);
	}

	// increment the time
	timeStep_++;
	printf("[Time = %d]\n", timeStep_);

	//checked if all agents have reached their targets
	int numReached = 0;

	for (auto id = 0; id < numAgents; id++) {
		if (agents_[id].reachedTarget()) {
			sc.reached[id] = true;
			agents_[id].v_ = agents_[id].v_.normalized() * eps;
			numReached++;
		}
	}

	if (numReached == numAgents) {
		totalRuntime_ = (std::clock() - startTime_) / double(CLOCKS_PER_SEC);
		printf("Done. Total Time %.2lf\n", totalRuntime_);
		outputFileAgents_.close();
		done = true;
		return;
	}

	ChromosomeDE best(agents_.size());
	std::vector<bool> processed(agents_.size(), false);
	std::vector<size_t> toResolve;

	solveLazyPSO(sc, processed, toResolve, best, *this);
	applyChromosome(agents_, best);
	moveAgentsAuto(agents_);
	done = false;
}

void Crowd::displayAgentsGUI(const double scale) const
{
	auto numAgents = agents_.size();

	for (auto id = 0; id < numAgents; id++) {
		agents_[id].drawTarget(scale);
		trails_[id].drawTrails(scale);
		agents_[id].drawAgent(scale);
	}

	// Draw Walls
	for (auto& w : walls_) {
		w.drawWall(scale, 0.8, 0.8, 0.8);
	}
}

Particle::Particle()
{
	myBestScore_ = std::numeric_limits<double>::lowest();
}

void Particle::init(const size_t feature_size, int idx)
{
	// randomly assign positions to the particles
	pos_.resize(feature_size);
	vel_.resize(feature_size);

	if (idx == 0) {
		// do nothing, already zeroes
	}

	else if (idx == 1) {
		for (auto& v : pos_) {
			v.initBrake();
		}
	}

	else {
		for (auto& v : pos_) {
			v.initRandom();
		}
	}
	myBestPos_ = pos_;
}

double Particle::dist(const Particle & p1, const Particle & p2)
{
	double ret = 0.0;
	for (int i = 0; i < p1.pos_.size(); i++) {
		for (int j = 0; j < 2; j++) {
			ret += pow((p1.pos_[i].vp[j] - p2.pos_[i].vp[j]), 2);
		}
	}
	return sqrt(ret);
}

const double Swarm::inertia_ = 0.5;
const double Swarm::c1_ = 1.0;
const double Swarm::c2_ = 1.0;

Swarm::Swarm()
{
	swarmBestScore_ = std::numeric_limits<double>::lowest();
	bestDuration_ = 1;
}

Swarm::Swarm(const size_t numParticles, const size_t featureSize)
{
	swarmBestScore_ = std::numeric_limits<double>::lowest();
	particles_.resize(numParticles);
	int i = 0;
	for (auto& p : particles_) {
		p.init(featureSize, i);
		i++;
	}
}

void Swarm::setScore(size_t particleID, double score)
{
	if (particles_[particleID].myBestScore_ < score) { // new best score for particle
		particles_[particleID].myBestScore_ = score;
		particles_[particleID].myBestPos_ = particles_[particleID].pos_;

		if (swarmBestScore_ < score) { // found new best score for swarm
			bestDuration_ = 1;
			swarmBestScore_ = score;
			swarmBest_ = particles_[particleID].pos_;
		}
	}
}

// Assuming that best scores have been set externally
void Swarm::updateSwarm()
{
	bestDuration_++;
	for (auto& p : particles_) {
		auto r1 = Util::sample_uniform(0, 1);
		auto r2 = Util::sample_uniform(0, 1);

		for (int i = 0; i < p.pos_.size(); i++) {
			for (int j = 0; j < 2; j++) {
				p.vel_[i].vp[j] = inertia_ * p.vel_[i].vp[j] +
					r1 * c1_ * (p.myBestPos_[i].vp[j] - p.pos_[i].vp[j]) +
					r2 * c2_ * (swarmBest_[i].vp[j] - p.pos_[i].vp[j]);

				p.pos_[i].vp[j] += p.vel_[i].vp[j];

				if (j == 0) {
					Util::clamp(p.pos_[i].vp[j], -1, 0);
				}
				else {
					Util::clamp(p.pos_[i].vp[j], -1, 1);
				}	
			}
		}
	}
}

// returns true when the maximum distance between two particles is less than a threshold
bool Swarm::stoppingCriteriaReached()
{
	if (bestDuration_ > 30) {
		return true;
	}

	return false;

	//double maxDist = std::numeric_limits<double>::lowest();
	//for (int i = 0; i < particles_.size(); i++) {
	//	for (int j = i + 1; j < particles_.size(); j++) {
	//		maxDist = std::max(maxDist, Particle::dist(particles_[i], particles_[j]));
	//	}
	//}

	//if (maxDist < 0.01) {
	//	return true;
	//}
	//return false;
}

SimulationCache::SimulationCache()
{
}

void SimulationCache::init(size_t timesteps, const Crowd & c)
{
	timesteps_ = timesteps;
	numAgents_ = c.agents_.size();
	numWalls_ = c.walls_.size();

	// setting a larger array for convenience
	// walls will obviously always have a false entry
	reached.resize(numAgents_ + numWalls_, false);

	positions.resize(timesteps, std::vector<Vector2>(numAgents_));
	velocities.resize(timesteps, std::vector<Vector2>(numAgents_));
	data.resize(timesteps, std::vector< std::vector<Criticality> >(numAgents_, std::vector<Criticality>(numAgents_ + numWalls_)));

	// populate the current values
	for(int i = 0; i < numAgents_; i++){
		positions[0][i] = c.agents_[i].pos_;
		velocities[0][i] = c.agents_[i].v_;
	}

	for (int t = 0; t < timesteps; t++) {
		for (int i = 0; i < numAgents_; i++) {
			// first the agents
			for (int j = i + 1; j < numAgents_; j++) {
				double md, ttmd;
				Walker::minDistAndTtmd(positions[t][i], velocities[t][i], positions[t][j], velocities[t][j], md, ttmd);
				data[t][i][j] = Criticality(md, ttmd);
				data[t][j][i] = Criticality(md, ttmd);
			}
			// then the walls
			for (int j = numAgents_; j < numAgents_ + numWalls_; j++) {
				double md, ttmd;
				auto& w = c.walls_[j - numAgents_];
				Wall::minDistAndTtmd(w.p1_, w.p2_, positions[t][i], velocities[t][i], md, ttmd);
				data[t][i][j] = Criticality(md, ttmd);
			}
		}

		// move the agents using relaxation to preferred velocity
		if (t < timesteps - 1) {
			for (int i = 0; i < c.agents_.size(); i++) {
				positions[t + 1][i] = positions[t][i] + velocities[t][i];
				velocities[t + 1][i] = c.agents_[i].v_auto(positions[t + 1][i], velocities[t][i]);
			}
		}
	}
}

void SimulationCache::alterVelocity0(size_t agent_id, const VelocityParameters& vp, const Crowd & c)
{
	vp.apply(velocities[0][agent_id]);

	for (int t = 0; t < timesteps_; t++) {
		for (int id = 0; id < numAgents_; id++) {
			if (id == agent_id) {
				continue;
			}
			double md, ttmd;
			Walker::minDistAndTtmd(positions[t][agent_id], velocities[t][agent_id], positions[t][id], velocities[t][id], md, ttmd);
			data[t][agent_id][id] = Criticality(md, ttmd);
			data[t][id][agent_id] = Criticality(md, ttmd);
		}
		for (int id = numAgents_; id < numAgents_ + numWalls_; id++) {
			double md, ttmd;
			auto& w = c.walls_[id - numAgents_];
			Wall::minDistAndTtmd(w.p1_, w.p2_, positions[t][agent_id], velocities[t][agent_id], md, ttmd);
			data[t][agent_id][id] = Criticality(md, ttmd);
		}

		// move the agents using relaxation to preferred velocity
		if (t < timesteps_ - 1) {
			positions[t + 1][agent_id] = positions[t][agent_id] + velocities[t][agent_id];
			velocities[t + 1][agent_id] = c.agents_[agent_id].v_auto(positions[t + 1][agent_id], velocities[t][agent_id]);
		}
	}

}

double SimulationCache::testAndScoreConfig(
	const size_t agent_id, 
	const VelocityParameters& vp,
	const std::vector<bool>& processsedAgents, 
	const double decayFactor, 
	const Crowd & c) const
{
	double ret = 0.0;

	std::vector<Vector2> positionsTmp(timesteps_);
	std::vector<Vector2> velocitiesTmp(timesteps_);

	positionsTmp[0] = positions[0][agent_id];
	velocitiesTmp[0] = velocities[0][agent_id];
	vp.apply(velocitiesTmp[0]);

	double factor = 1.0;

	for (int t = 0; t < timesteps_; t++) {
		for (int id = 0; id < numAgents_; id++) {

			if (id == agent_id) {
				continue;
			}

			//if (!processsedAgents[id]) {
			//	continue;
			//}

			double md, ttmd;
			Walker::minDistAndTtmd(positionsTmp[t], velocitiesTmp[t], positions[t][id], velocities[t][id], md, ttmd);
		
			if (md >= Walker::minDist || ttmd < 0) {
				ret += factor;
				continue;
			}

			auto deno = (md + eps) * (1 + ttmd);
			ret += -(100.0 * factor) / deno;
		}
		for (int id = numAgents_; id < numAgents_ + numWalls_; id++) {
			double md, ttmd;
			auto& w = c.walls_[id - numAgents_];
			Wall::minDistAndTtmd(w.p1_, w.p2_, positionsTmp[t], velocitiesTmp[t], md, ttmd);
			
			if (md >= Walker::minWallDist || ttmd < 0) {
				ret += factor;
				continue;
			}

			auto deno = (md + eps) * (1 + ttmd);
			ret += -(100.0 * factor) / deno;
		}

		// move the agents using relaxation to preferred velocity
		if (t < timesteps_ - 1) {
			positionsTmp[t + 1] = positionsTmp[t] + velocitiesTmp[t];
			velocitiesTmp[t + 1] = c.agents_[agent_id].v_auto(positionsTmp[t + 1], velocitiesTmp[t]);
		}

		factor *= decayFactor;
	}

	return ret;
}

#include <set>
double SimulationCache::testAndScoreConfig(const std::vector<size_t>& ids, const std::vector<VelocityParameters>& vp, const double decayFactor, const Crowd & c) const
{
	double ret = 0.0;

	auto numR = ids.size();
	if (vp.size() != numR) {
		printf("Error. Size of ids and vp do not match! \n");
		return 0.0;
	}

	std::vector<std::vector<Vector2> > positionsTmp(timesteps_, std::vector<Vector2>(numR));
	std::vector<std::vector<Vector2> > velocitiesTmp(timesteps_, std::vector<Vector2>(numR));

	for (auto i = 0; i < numR; i++) {
		const auto& id = ids[i];
		positionsTmp[0][i] = positions[0][id];
		velocitiesTmp[0][i] = velocities[0][id];
		vp[i].apply(velocitiesTmp[0][i]);
	}

	std::set<size_t> idsSet(ids.begin(), ids.end());

	double factor = 1.0;

	for (int t = 0; t < timesteps_; t++) {

		// First calculate interactions between all test agent pairs
		for (auto i = 0; i < numR; i++) {
			for (auto j = i + 1; j < numR; j++) {
				double md, ttmd;
				Walker::minDistAndTtmd(positionsTmp[t][i], velocitiesTmp[t][i], positionsTmp[t][j], velocitiesTmp[t][j], md, ttmd);

				if (md >= Walker::minDist || ttmd < 0) {
					ret += factor;
					continue;
				}
				auto deno = (md + eps) * (1 + ttmd);
				ret += -(100.0 * factor) / deno;
			}
		}

		// Now calculate interactions for all test_agent-other_agent pairs
		for (auto i = 0; i < numR; i++) {
			for (auto id_other = 0; id_other < numAgents_ + numWalls_; id_other++) {
				
				if (idsSet.find(id_other) != idsSet.end()) { // other_id is in the set of ids being tested
					continue; // done already
				}
				
				double md, ttmd;
				if (id_other < numAgents_) {
					Walker::minDistAndTtmd(positionsTmp[t][i], velocitiesTmp[t][i], positions[t][id_other], velocities[t][id_other], md, ttmd);
					if (md >= Walker::minDist || ttmd < 0) {
						ret += factor;
						continue;
					}
				}
				else {
					auto& w = c.walls_[id_other - numAgents_];
					Wall::minDistAndTtmd(w.p1_, w.p2_, positionsTmp[t][i], velocitiesTmp[t][i], md, ttmd);

					if (md >= Walker::minWallDist || ttmd < 0) {
						ret += factor;
						continue;
					}
				}
				auto deno = (md + eps) * (1 + ttmd);
				ret += -(100.0 * factor) / deno;
			}
		}

		// move the agents using relaxation to preferred velocity for the next timestep
		if (t < timesteps_ - 1) {
			for(auto i = 0; i < numR; i++){
				const auto& id = ids[i];
				positionsTmp[t + 1][i] = positionsTmp[t][i] + velocitiesTmp[t][i];
				velocitiesTmp[t + 1][i] = c.agents_[id].v_auto(positionsTmp[t + 1][i], velocitiesTmp[t][i]);
			}
		}

		factor *= decayFactor;
	}

	return ret;
}

double SimulationCache::testAndScoreCouple(
	const size_t agent_id1,
	const size_t agent_id2,
	const VelocityParameters& vp1,
	const VelocityParameters& vp2,
	const double decayFactor,
	const Crowd & c) const
{
	double ret = 0.0;

	std::vector<std::vector<Vector2> > positionsTmp(timesteps_, std::vector<Vector2>(2));
	std::vector<std::vector<Vector2> > velocitiesTmp(timesteps_, std::vector<Vector2>(2));

	positionsTmp[0][0] = positions[0][agent_id1];
	velocitiesTmp[0][0] = velocities[0][agent_id1];
	vp1.apply(velocitiesTmp[0][0]);

	positionsTmp[0][1] = positions[0][agent_id2];
	velocitiesTmp[0][1] = velocities[0][agent_id2];
	vp2.apply(velocitiesTmp[0][1]);

	double factor = 1.0;

	for (int t = 0; t < timesteps_; t++) {

		double md, ttmd;
		Walker::minDistAndTtmd(positionsTmp[t][0], velocitiesTmp[t][0], positionsTmp[t][1], velocitiesTmp[t][1], md, ttmd);

		if (md > Walker::minDist || ttmd < 0) {
			ret += factor;
		}
		else {
			auto deno = (md + eps) * (1 + ttmd);
			ret += -(100.0 * factor) / deno;
		}

		// move the agents using relaxation to preferred velocity
		if (t < timesteps_ - 1) {
			
				positionsTmp[t + 1][0] = positionsTmp[t][0] + velocitiesTmp[t][0];
				velocitiesTmp[t + 1][0] = c.agents_[agent_id1].v_auto(positionsTmp[t + 1][0], velocitiesTmp[t][0]);
			
				positionsTmp[t + 1][1] = positionsTmp[t][1] + velocitiesTmp[t][1];
				velocitiesTmp[t + 1][1] = c.agents_[agent_id2].v_auto(positionsTmp[t + 1][1], velocitiesTmp[t][1]);

		}

		factor *= decayFactor;
	}

	return ret;
}

#include <queue>
void SimulationCache::findCriticalClusters(const std::vector<bool>& processsedAgents, std::vector<int>& belongsTo, std::vector<CriticalPair>& cps) const
{
	std::vector< std::vector<bool> > criticalArray;

	criticalArray.resize(numAgents_, std::vector<bool>(numAgents_ + numWalls_, false));


	for (int i = 0; i < numAgents_; i++) {
		if (reached[i]) {
			continue;
		}
		for (int j = i + 1; j < numAgents_ + numWalls_; j++) {

			if (reached[j]) {
				continue;
			}

			double minDistToCompare = j < numAgents_ ? Walker::minDist : Walker::minWallDist;

			if (data[0][i][j].minDist >= minDistToCompare || data[0][i][j].ttmd < 0) {
				continue;
			}
			criticalArray[i][j] = true;
			if (j < numAgents_) {
				criticalArray[j][i] = true;
			}
		}
	}

	belongsTo.clear();
	belongsTo.resize(numAgents_, -1);

	std::vector<bool> visited = processsedAgents;
	cps.clear();

	int num_cluster = -1;

	// continue until all nodes are visited
	while (Util::numTrue(visited) < numAgents_) {
		// find a cluster
		num_cluster++;
		CriticalPair cp;
		bool found = false;

		int node;

		for (int i = 0; i < numAgents_; i++) {
			if (visited[i]) {
				continue;
			}

			if (found) {
				break;
			}

			for (int j = i + 1; j < numAgents_ + numWalls_; j++) {
				if (j < numAgents_ && visited[j]) {
					continue;
				}

				if (criticalArray[i][j]) {
					found = true;
					node = i;

					cp.i = i;
					cp.j = j;
					cp.ttmd = data[0][i][j].ttmd;

					break;
				}
			}
		}

		if (!found) { // no critical edges found
			break;
		}

		// node is found, now push into a queue

		std::queue<int> connected;
		connected.push(node);

		while (!connected.empty()) {
			auto N = connected.front();
			connected.pop();
			if (visited[N]) {
				continue;
			}
			belongsTo[N] = num_cluster;
			visited[N] = true;

			for (int p = 0; p < numAgents_; p++) {
				if (criticalArray[N][p] && !visited[p]) {

					if (data[0][N][p].ttmd < cp.ttmd) {
						cp.i = N;
						cp.j = p;
						cp.ttmd = data[0][N][p].ttmd;
					}

					connected.push(p);
				}
			}
		}

		//if (processsedAgents[cp.i]) {
		//	printf("Error! Processed agent added to resolved!\n");
		//}

		cps.push_back(cp);
	}
}
