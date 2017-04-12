#pragma once

#define GRID_HEIGHT 50
//#define DRAW_PRESSURE
#define DRAW_PARTICLES
#define DRAW_STATE_GRID
#define USE_MARKER_RESAMPLING

typedef MACGrid2d<float, float, glm::vec3, GRID_HEIGHT, GRID_HEIGHT> MACGrid2dF;
typedef std::function<float (int, int)> get_Q_t;

#define JACOBI_SOLVE_ITERATIONS 150

enum DrawMode { GRIDSTATE, PRESSURE, VELOCITY };

class Fluid
{
private:
	std::vector<glm::vec2> particles;
	MACGrid2dF * macGrid[2];
	inline MACGrid2dF & grid(int i) { return *macGrid[i]; }
	glm::vec2 topLeft, size;
	int currentGrid = 0;
	float density = 1;
	float cellSize = .5f;
	glm::vec2 bodyForces = glm::vec2(0, 9.81);
	std::vector<glm::vec4> forces;
	int swap()
	{
		currentGrid = 1 - currentGrid;
		return currentGrid;
	}
	glm::vec2 transformGrid2Screen(glm::vec2 v) { return v * (size.x / GRID_HEIGHT); }
public:
	bool enableSink = false, enableTaps = false;
	bool enableDrawParticles = true; 
	DrawMode drawMode = VELOCITY;
	std::vector<glm::vec2> sinks;
	std::vector<glm::vec2> taps;
	get_Q_t getY = [&](int x, int y) -> float { return grid(currentGrid).getVelY(x, y); };
	get_Q_t getX = [&](int x, int y) -> float { return grid(currentGrid).getVelX(x, y); };
	get_Q_t getPressure = [&](int x, int y) -> float { return grid(currentGrid).getPressure(x, y); };
	get_Q_t getMetaX = [&](int x, int y) -> float { return grid(currentGrid).getMeta(x,y).x; };
	get_Q_t getMetaY = [&](int x, int y) -> float { return grid(currentGrid).getMeta(x, y).y; };
	get_Q_t getMetaZ = [&](int x, int y) -> float { return grid(currentGrid).getMeta(x, y).z; };
	float interpolate(float x, float y, get_Q_t getQuantity);
	float interpolate(glm::vec2 gridPos, get_Q_t getQuantity);
	void advect(float timeStep);
	void applyBodyForce(float timeStep, glm::vec2 F);
	void applyForceAt(int x, int y, float timeStep, glm::vec2 F);
	void syncGrid();
	void updatePressure();
	void subtractPressureGradient(float timeStep);
	void updateParticleCounts();
	void stateUpdates();
	void markerParticleResampling();
	void simulateStep(float timeStep);
	void Draw(Tmpl8::Surface* surface);
	void handleTaps();
	void advectParticles(float timeStep);

	void setMeta(int x, int y, glm::vec3 meta) { macGrid[0]->getMeta(x, y) = meta; macGrid[1]->getMeta(x, y) = meta; }
	Fluid(glm::vec2 topLeft, glm::vec2 size)
		: topLeft(topLeft), size(size)
	{
		macGrid[0] = new MACGrid2dF(0, 0, 0);
		macGrid[1] = new MACGrid2dF(0, 0, 0);
		for (int x = 2; x < GRID_HEIGHT - 2; x++)
			for (int y = 2; y < GRID_HEIGHT / 2; y++)
			{
				particles.push_back(glm::vec2(x + 0.5f, y + 0.5f));
			}
	}
};