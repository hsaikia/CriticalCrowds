/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#pragma once

#include "walker.h"
#include "util.h"
#include "wall.h"
#include "chromosome.h"
#include <fstream>
#include <ctime>
#include <string>

const bool multiAgentResolution = false;

class Particle {
public:
	std::vector<VelocityParameters> pos_;
	std::vector<VelocityParameters> vel_;
	std::vector<VelocityParameters> myBestPos_;
	double myBestScore_;
	Particle();
	void init(const size_t feature_size, const int idx);
	static double dist(const Particle& p1, const Particle& p2);
};

class Swarm {
private:
	static const double inertia_;
	static const double c1_;
	static const double c2_;
public:
	std::vector<Particle> particles_;
	std::vector<VelocityParameters> swarmBest_;
	double swarmBestScore_;
	int bestDuration_;
	Swarm();
	Swarm(const size_t numParticles, const size_t featureSize);
	void setScore(size_t particleID, double score);
	void updateSwarm();
	bool stoppingCriteriaReached();
};

class SimulationCache;

class Crowd {
private:

	/// CrossOver Probability
	static const double crossover_prob_; // [0, 1]

	/// Look ahead steps in time
	static const size_t tau_ = 10;

	/// Iterations 
	static const size_t iterations_ = 400;

	/// Size of population
	static const size_t pop_size_ = 300;

	//static void resolveFromCluster(std::vector<CriticalPair>& cpairs, const std::vector<bool>& processsedAgents, std::vector<size_t>& toResolveNow, const size_t& numAgents, bool multi);

	/// A function which takes in all agents and walls return the agent(s) who have the shortest
	/// time to collide with another agent or a wall
	static void resolve(
		const SimulationCache& cache,
		const std::vector<bool>& processsedAgents,
		std::vector<size_t>& toResolveNow,
		bool multi);

	/// Given a list of processed agents, a list of agents to optimize (usually one or two) and a chromosome (to act on the processed agents)
	/// determine the objective function score
	static double applyAndScore(
		const SimulationCache& cache, 
		const std::vector<bool>& processsedAgents,
		const std::vector<size_t>& toResolveAgents,
		const std::vector<VelocityParameters>& toApply,
		const Crowd& c);

	/// Apply the chromosome on the agent velocities on ONLY the given agents
	static void applyChromosomeSelective(std::vector<Walker>& agents, const ChromosomeDE& ch);

	/// Merge the velocity parameters corresponding to the resolved agent into the chromosome
	static void mergeIntoChromosome(ChromosomeDE& ch, const std::vector<VelocityParameters>& toMerge, const std::vector<size_t>& toResolveAgents);

	/// Solve function
	/// Given a list of processed agents, a list of agents to optimize (usually one or two) and a chromosome (to act on the processed agents)
	/// Optimizes the actions for only the couple of agents and updates the chromosome
	static void solveLazy(
		SimulationCache& cache,
		std::vector<bool>& processsedAgents,
		std::vector<size_t>& toResolveAgents,
		ChromosomeDE& bestSoFar,
		const Crowd& c);

	/// Solve function
	/// Given a list of processed agents, a list of agents to optimize (usually one or two) and a chromosome (to act on the processed agents)
	/// Optimizes the actions for only the couple of agents and updates the chromosome
	static void solveLazyPSO(
		SimulationCache& cache, 
		std::vector<bool>& processsedAgents,
		std::vector<size_t>& toResolveAgents,
		ChromosomeDE& bestSoFar,
		const Crowd& c);


	/// Apply the chromosome on the agent velocities
	static void applyChromosome(std::vector<Walker>& agents, const ChromosomeDE& ch);

	static void moveAgentsAuto(std::vector<Walker>& agents);

	static void addToProcessed(std::vector<bool>& processed, const std::vector<size_t>& resolved);

	std::ofstream outputFileAgents_;
	
	int timeStep_;
	
	std::clock_t startTime_;
	double totalRuntime_;

public:
	/// List of agents
	std::vector<Walker> agents_;
	/// List of walls
	std::vector<Wall> walls_;
	/// List of Trails
	std::vector<WalkerTrails> trails_;
	
	/// Initialize
	void init(const size_t n, std::string outputFileAgents);
	/// Set Hue
	void setHue(size_t i, double hue);
	/// Add a wall
	void addWall(const Wall& w);
	/// Export wall information as csv
	void writeWallsToFile(std::string outputFileWalls);
	/// move agents
	void moveAgentsGUI(bool& done);
	/// display agents
	void displayAgentsGUI(const double scale) const;
};

struct CriticalPair {
	int i;
	int j;
	double ttmd;
};

struct Criticality {
	double minDist;
	double ttmd;

	Criticality() {
		minDist = std::numeric_limits<double>::max();
		ttmd = std::numeric_limits<double>::max();
	}

	Criticality(double d, double t) {
		minDist = d; ttmd = t;
	}
};

//struct CriticalEdge {
//	int i;
//	int j;
//};

class SimulationCache {
public:
	
	SimulationCache();
	
	void init(const size_t timesteps, const Crowd& c);
	
	void alterVelocity0(const size_t agent_id, const VelocityParameters& vp, const Crowd & c);
	
	double testAndScoreConfig(
		const size_t agent_id,
		const VelocityParameters& vp,
		const std::vector<bool>& processsedAgents,
		const double decayFactor,
		const Crowd & c) const;

	double testAndScoreConfig(
		const std::vector<size_t>& ids,
		const std::vector<VelocityParameters>& vp,
		const double decayFactor,
		const Crowd & c) const;

	double testAndScoreCouple(
		const size_t agent_id1,
		const size_t agent_id2,
		const VelocityParameters& vp1,
		const VelocityParameters& vp2,
		const double decayFactor,
		const Crowd & c) const;

	void findCriticalClusters(const std::vector<bool>& processsedAgents, std::vector<int>& belongsTo, std::vector<CriticalPair>& cps) const;
	
	// spatio temporal cache memory
	std::vector<bool> reached;
	std::vector< std::vector<Vector2> > positions;
	std::vector< std::vector<Vector2> > velocities;
	std::vector< std::vector< std::vector<Criticality> > > data;

	size_t numAgents_;
	size_t numWalls_;
	size_t timesteps_;

};

