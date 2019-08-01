/**
********************************
Crowd Simulation Library

author : Himangshu Saikia
email : saikia@kth.se

*********************************
**/


#include <cstdio>
#include "util.h"
#include "crowd.h"
#include "wall.h"
#include <GL/glut.h>
#include <sstream>
#include <algorithm>


double scale = 10.0;
size_t numAgents;
bool debugSearchSpace = true;
int timeToWaitBetweenMoves = 100; // in ms

Crowd c;

void writeInitConfigToFile(const std::vector<Walker>& agents, const std::vector<Wall>& walls, const std::string& filename) {
	std::ofstream ofile;
	ofile.open(filename);

	ofile << "\n------ Agents -------\n";

	for (auto& a : agents) {
		ofile << "<Agent p_x = \"" << a.pos_.x << "\" p_y = \"" << a.pos_.y << "\" />" << "\n";
	}

	ofile << "\n------ Targets -------\n";

	for (auto& a : agents) {
		ofile << "<Target Point1 p_x = \"" << a.tar_->p1_.x << "\" p_y = \"" << a.tar_->p1_.y << "\" />" << "\n";
		ofile << "<Target Point2 p_x = \"" << a.tar_->p2_.x << "\" p_y = \"" << a.tar_->p2_.y << "\" />" << "\n";
	}

	ofile << "\n------ Walls -------\n";

	for (auto& w : walls) {
		ofile << "<Vertex  p_x = \"" << w.p1_.x << "\" p_y = \"" << w.p1_.y << "\" />" << "\n";
		ofile << "<Vertex  p_x = \"" << w.p2_.x << "\" p_y = \"" << w.p2_.y << "\" />" << "\n\n";
	}

	ofile.close();
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT);

	// Display function
	c.displayAgentsGUI(scale);

	glFlush();
}

void moveAgents(int arg) {
	// Move function
	bool done;
	c.moveAgentsGUI(done);
	glutPostRedisplay();
	if (!done) {
		glutTimerFunc(timeToWaitBetweenMoves, moveAgents, -1);
	}
}

void setupVariables(size_t n, std::string outfile) {
	c.init(n, outfile);
}

void setupStraightSimple() {
	numAgents = 2;
	setupVariables(numAgents, "simple.csv");

	auto tar0 = std::make_shared<Target>();
	tar0->init(Vector2(-5, 0.1), Vector2(-5, -0.1));

	auto tar1 = std::make_shared<Target>();
	tar1->init(Vector2(5, 0.1), Vector2(5, -0.1));

	c.agents_[0].init(0, Vector2(5, 0), tar0);
	c.setHue(0, 0.0); // red

	c.agents_[1].init(1, Vector2(-5, 0), tar1);
	c.setHue(1, 0.666); // blue

	c.addWall(Wall(Vector2(5, 5), Vector2(-5, 5)));
	c.addWall(Wall(Vector2(5, -5), Vector2(-5, -5)));

	c.writeWallsToFile("simple_walls.csv");
}

void setupOrthogonalSimple() {
	numAgents = 2;
	setupVariables(numAgents, "orthogonal_simple.csv");

	auto tar0 = std::make_shared<Target>();
	tar0->init(Vector2(5, 0.1), Vector2(5, -0.1));

	auto tar1 = std::make_shared<Target>();
	tar1->init(Vector2(0.1, 5), Vector2(-0.1, 5));

	c.agents_[0].init(0, Vector2(-5, 0), tar0);
	c.setHue(0, 0.0); // red
	c.agents_[1].init(1, Vector2(0, -5), tar1);
	c.setHue(1, 0.666); // blue
}

void setupBottleNeck(size_t n) {
	scale = 15.0;
	numAgents = n;
	std::stringstream ss;
	ss << "bottleneck_" << n << "agents.csv";
	setupVariables(n, ss.str());

	// setup the bottleneck
	c.addWall(Wall(Vector2(-6, 1), Vector2(0, 5)));
	c.addWall(Wall(Vector2(-6, -1), Vector2(0, -5)));

	ss.str("");
	ss << "bottleneck_walls.csv";
	c.writeWallsToFile(ss.str());

	double sep = Walker::minDist;

	auto tar = std::make_shared<Target>();
	tar->init(Vector2(-10, -2), Vector2(-10, 2));

	int px = 5;
	int py = -5;
	int cx = 0;

	int nrow = 11;

	for (auto i = 0U; i < n; i++) {
		c.agents_[i].init(i, Vector2(px * sep, py * sep), tar);
		cx = (cx + 1) % nrow;
		py = cx - 5;

		if (cx == 0) {
			px = px + 1;
		}

		c.setHue(i, double(i) / n);
	}

	ss.str("");
	ss << "bottleneck_" << n << "_initial_positions.txt";
	writeInitConfigToFile(c.agents_, c.walls_, ss.str());
}

//n is assumed to be even
void setupHallway(size_t n) {
	scale = 15.0;
	numAgents = n;
	std::stringstream ss;
	ss << "hallway" << n << ".csv";
	setupVariables(n, ss.str());
	
	double sep = Walker::minDist;
	
	auto tar1 = std::make_shared<Target>();
	tar1->init(Vector2(-5 * sep, 10), Vector2(5 * sep, 10));

	auto tar2 = std::make_shared<Target>();
	tar2->init(Vector2(-5 * sep, -10), Vector2(5 * sep, -10));

	int px = -5;
	int py = -10;
	int cx = 0;

	int nrow = 11;

	for (auto i = 0U; i < n/2; i++) {
		c.agents_[i].init(i, Vector2(px * sep, py * sep), tar1);
		cx = (cx + 1) % nrow;
		px = cx - 5;

		if (cx == 0) {
			py = py - 1;
		}
		
		c.setHue(i, double(i) / n);
	}

	px = -5;
	py = 10;
	cx = 0;

	for (auto i = n / 2; i < n; i++) {
		
		c.agents_[i].init(i, Vector2(px * sep, py * sep), tar2);

		cx = (cx + 1) % nrow;
		px = cx - 5;

		if (cx == 0) {
			py = py + 1;
		}


		c.setHue(i, double(i) / n);
	}

	ss.str("");
	ss << "hallway_" << n << "_initial_positions.txt";
	writeInitConfigToFile(c.agents_, c.walls_, ss.str());
}

void setupOrthogonalLarge(size_t n) {
	numAgents = n;
	std::stringstream ss;
	ss << "orthogonal_large_" << n << "agents.csv";
	setupVariables(n, ss.str());

	double A = 10.0;
	double B = 16 * Walker::agentRadius;
	double C = 8 * Walker::agentRadius; // row spacing is 2C

	int numAgentsPerRow = 3;
	double spacingBetweenAgents = (2 * C) / (numAgentsPerRow + 1);

	// define walls

	c.addWall(Wall(Vector2(-A,  C), Vector2(-B,  C)));
	c.addWall(Wall(Vector2(-A, -C), Vector2(-B, -C)));
	c.addWall(Wall(Vector2( A, -C), Vector2( B, -C)));
	c.addWall(Wall(Vector2( A,  C), Vector2( B,  C)));
	c.addWall(Wall(Vector2(-C,  A), Vector2(-C,  B)));
	c.addWall(Wall(Vector2( C,  A), Vector2( C,  B)));
	c.addWall(Wall(Vector2(-C, -A), Vector2(-C, -B)));
	c.addWall(Wall(Vector2( C, -A), Vector2( C, -B)));
	c.addWall(Wall(Vector2( C,  B), Vector2( B,  C)));
	c.addWall(Wall(Vector2(-C,  B), Vector2(-B,  C)));
	c.addWall(Wall(Vector2(-C, -B), Vector2(-B, -C)));
	c.addWall(Wall(Vector2( C, -B), Vector2( B, -C)));

	ss.str("");
	ss << "orthogonal_large_walls.csv";
	c.writeWallsToFile(ss.str());


	// define targets
	auto tar1 = std::make_shared<Target>();
	tar1->init(Vector2(A, -C), Vector2(A, C));

	auto tar2 = std::make_shared<Target>();
	tar2->init(Vector2(-C, A), Vector2(C, A));

	int px = - A / 2;
	int py = - C;
	Vector2 nextp;

	for (int i = 0; i < n / 2; i++) {

		int cx = i / numAgentsPerRow;
		int cy = i % numAgentsPerRow;

		nextp.x = px - cx * Walker::minDist;
		nextp.y = py + (cy + 1) * spacingBetweenAgents;

		c.agents_[i].init(i, nextp, tar1);
		c.setHue(i, double(i) / n);
	}

	px = - C;
	py = - A / 2;
	
	for (int i = n / 2; i < n; i++) {

		int cy = (i - n / 2) / numAgentsPerRow;
		int cx = (i - n / 2) % numAgentsPerRow;

		nextp.x = px + (cx + 1) * spacingBetweenAgents;
		nextp.y = py - cy * Walker::minDist;

		c.agents_[i].init(i, nextp, tar2);
		c.setHue(i, double(i) / n);
	}

	ss.str("");
	ss << "orthogonal_large_" << n << "_initial_positions.txt";
	writeInitConfigToFile(c.agents_, c.walls_, ss.str());
}

void setupCircle(size_t n) {
	numAgents = n;
	std::stringstream ss;
	ss << "circle_" << n << "agents.csv";
	setupVariables(n, ss.str());
	double setupRadius = std::max(10.0, ((Walker::minDist + 2 * Walker::agentRadius) * n) / (2 * PII));

	scale = double(1.5 * setupRadius);

	Vector2 up = Vector2(0, setupRadius);
	auto ang = 2 * PII / n;
	for (auto i = 0U; i < n; i++) {
		printf("\nLOC %d : (%.2lf, %.2lf)", i, up.x, up.y);
		auto tar = std::make_shared<Target>();
		tar->init(-up, -up);
		c.agents_[i].init(i, up, tar);
		c.setHue(i, double(i) / n);
		up.rotate(2 * PII / n);
	}

	ss.str("");
	ss << "circle_" << n << "_initial_positions.txt";
	writeInitConfigToFile(c.agents_, c.walls_, ss.str());
}

int main(int argc, char** argv) {
	//setup environment
	//setupStraightSimple();
	//setupOrthogonalSimple();
	//setupStraightLarge(4);
	//setupHallway(50);
	//setupBottleNeck(100);
	setupCircle(20);
	//setupOrthogonalLarge(10);

	glutInit(&argc, argv);
	glutInitWindowSize(1000, 1000);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Social Walker MCTS");

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	//glEnable(GL_LIGHTING);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);

	glutDisplayFunc(display);
	glutTimerFunc(0, moveAgents, -1);
	glutMainLoop();

	//getchar();
	return 0;
}