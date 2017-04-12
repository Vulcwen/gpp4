#include "template.h"

Tmpl8::Color particleColor{ 255, 50, 100, 160 };

inline int roundDown(float v)
{
	if (v >= 0)
		return int(v);
	else
		return int(v) - 1;
}

float Fluid::interpolate(float x, float y, get_Q_t getQuantity)
{
	
	const int xi = roundDown(x), yi = roundDown(y);
	/*
	a - b
	|	  |
	c - d
	*/
	const float a = getQuantity(xi, yi);
	const float b = getQuantity(xi + 1, yi);
	const float c = getQuantity(xi, yi + 1);
	const float d = getQuantity(xi + 1, yi + 1);
	const float dx = x - xi, dy = y - yi;
	const float T = dx * b + (1 - dx) * a;
	const float B = dx * d + (1 - dx) * c;
	return dy * B + (1 - dy) * T;
}

float Fluid::interpolate(vec2 gridPos, get_Q_t getQuantity)
{
	return interpolate(gridPos.x, gridPos.y, getQuantity);
}

bool nancheck(MACGrid2dF & m)
{
	for (int x = 0; x <= m.width() - 1; x++)
		for (int y = 0; y <= m.height() - 1; y++)
		{
			if (std::isnan(m.getVelX(x, y)) || std::isnan(m.getVelY(x, y)) || std::isnan(m.getPressure(x, y)))
				return true;
		}
	return false;
}


void Fluid::advect(float timeStep)
{
	for (int x = 0; x < MACGrid2dF::width(); x++)
		for (int y = 0; y < MACGrid2dF::height(); y++)
		{

			//names are <axis>Vel_<perspective>
			float xVel_x = grid(currentGrid).getVelX(x, y);
			float yVel_y = grid(currentGrid).getVelY(x, y);
			float xVel_y = interpolate(x + 0.5, y - 0.5, getX);
			float yVel_x = interpolate(x - 0.5, y + 0.5, getY);
			float xVel_p = interpolate(x - 0.5, y, getX);
			float yVel_p = interpolate(x, y - 0.5, getY);

			//forward euler for now
			vec2 prev_x = vec2(x, y) - vec2(xVel_x, yVel_x) * timeStep;
			vec2 prev_y = vec2(x, y) - vec2(xVel_y, yVel_y) * timeStep;
			vec2 prev_p = vec2(x, y) - vec2(xVel_p, yVel_p) * timeStep;
			float new_x = interpolate(prev_x, getX);
			float new_y = interpolate(prev_y, getY);
			vec3 new_meta = vec3(interpolate(prev_p, getMetaX), interpolate(prev_p, getMetaY), interpolate(prev_p, getMetaZ));

			grid(1 - currentGrid).getVelX(x, y) = new_x;
			grid(1 - currentGrid).getVelY(x, y) = new_y;
			grid(1 - currentGrid).getMeta(x, y) = new_meta;

		}
	swap();
}


void Fluid::applyBodyForce(float timeStep, vec2 F)
{
	for (int i = 0; i < forces.size(); i++)
	{
		grid(currentGrid).getVelX(forces[i].x, forces[i].y) += (1 / density) * forces[i].z * timeStep;
		grid(currentGrid).getVelY(forces[i].x, forces[i].y) += (1 / density) * forces[i].w * timeStep;
	}
	forces.clear();
	for (int x = 1; x < grid(currentGrid).width() - 1; x++)
		for (int y = 1; y < grid(currentGrid).height() - 1; y++)
		{
			grid(currentGrid).getVelX(x, y) += (1/density) * F.x * timeStep;
			grid(currentGrid).getVelY(x, y) += (1/density) * F.y * timeStep;
		}
}

void Fluid::applyForceAt(int x, int y, float timeStep, glm::vec2 F)
{
	forces.push_back(glm::vec4(x, y, F.x, F.y));
}

//sync both grids, copies the state of the current grid to the inactive grid
void Fluid::syncGrid()
{
	for (int x = 0; x < MACGrid2dF::width(); x++)
		for (int y = 0; y < MACGrid2dF::height(); y++)
		{
			CellState state = grid(currentGrid).getState(x, y);
			grid(1 - currentGrid).getState(x, y) = state;
			float p = grid(currentGrid).getPressure(x, y);
			grid(1 - currentGrid).getPressure(x, y) = p;
			float xv = grid(currentGrid).getVelX(x, y);
			grid(1 - currentGrid).getVelX(x, y) = xv;
			float yv = grid(currentGrid).getVelY(x, y);
			grid(1 - currentGrid).getVelY(x, y) = yv;
			vec3 meta = grid(currentGrid).getMeta(x, y);
			grid(1 - currentGrid).getMeta(x, y) = meta;
		}
}

void Fluid::updatePressure()
{

	syncGrid();

	for (int i = 0; i < JACOBI_SOLVE_ITERATIONS; i++)
	{
		for (int y = 0; y < MACGrid2dF::height(); y++)
			for (int x = 0; x < MACGrid2dF::width(); x++)
			{
				float P = grid(currentGrid).getPressure(x, y);
				float L = grid(currentGrid).getPressure(x - 1, y);
				float R = grid(currentGrid).getPressure(x + 1, y);
				float T = grid(currentGrid).getPressure(x, y - 1);
				float D = grid(currentGrid).getPressure(x, y + 1);
				float laplaceP = (L + R + T + D - 4 * P) / (cellSize*cellSize);

				float vXr = grid(currentGrid).getVelX(x + 1, y);
				float vXl = grid(currentGrid).getVelX(x, y);
				float vYd = grid(currentGrid).getVelY(x, y + 1);
				float vYt = grid(currentGrid).getVelY(x, y);
				float deltaVelX = (vXr - vXl) / 2 * cellSize;
				float deltaVelY = (vYd - vYt) / 2 * cellSize;
				float velDivergence = deltaVelX + deltaVelY;

				float b = laplaceP + velDivergence;

				float jacobiSolveIteration = (L + R + T + D - (cellSize*cellSize) * b) / 4;
				grid(1 - currentGrid).getPressure(x, y) = jacobiSolveIteration;
			}
		swap();
	}

	syncGrid();
}

void Fluid::subtractPressureGradient(float timeStep)
{
	for (int x = 0; x <= grid(currentGrid).width(); x++)
		for (int y = 0; y <= grid(currentGrid).height(); y++)
		{

			//equation (22, 23)
			//x-vel
			if (grid(currentGrid).getState(x, y) == CellState::FLUID || grid(currentGrid).getState(x - 1, y) == CellState::FLUID)
			{
				grid(currentGrid).getVelX(x, y) -= timeStep * (1 / density) *  ((grid(currentGrid).getPressure(x, y) - grid(currentGrid).getPressure(x - 1, y)) / cellSize);
			}

			//y-vel
			if (grid(currentGrid).getState(x, y) == CellState::FLUID || grid(currentGrid).getState(x, y - 1) == CellState::FLUID)
			{
				grid(currentGrid).getVelY(x, y) -= timeStep * (1 / density) *  ((grid(currentGrid).getPressure(x, y) - grid(currentGrid).getPressure(x, y - 1)) / cellSize);
			}

		}
}

void Fluid::updateParticleCounts()
{
	for (int x = 0; x < MACGrid2dF::width(); x++)
		for (int y = 0; y < MACGrid2dF::height(); y++)
			grid(currentGrid).getParticleCount(x, y) = 0;

	for (int i = 0; i < particles.size(); i++)
	{
		grid(currentGrid).getParticleCount(particles[i].x, particles[i].y)++;
	}
}

void Fluid::stateUpdates()
{
	for (int x = 0; x < MACGrid2dF::width(); x++)
		for (int y = 0; y < MACGrid2dF::height(); y++)
		{
			if (grid(currentGrid).getState(x, y) == SINK)
			{
				grid(currentGrid).getVelY(x, y) = 40;
				grid(currentGrid).getPressure(x, y) = -10;
			}

			else if (grid(currentGrid).getState(x, y) == AIR)
			{
				grid(currentGrid).getPressure(x, y) = 0;
				if (grid(currentGrid).getState(x - 1, y) == AIR
					|| grid(currentGrid).getState(x - 1, y) == OBJECT)
					grid(currentGrid).getVelX(x, y) = 0;
				if (grid(currentGrid).getState(x, y - 1) == AIR
					|| grid(currentGrid).getState(x, y - 1) == OBJECT)
					grid(currentGrid).getVelY(x, y) = 0;
			}
			else if (grid(currentGrid).getState(x, y) == OBJECT)
			{
				grid(currentGrid).getPressure(x, y) = 0;
				grid(currentGrid).getVelX(x, y) = 0;
				grid(currentGrid).getVelY(x, y) = 0;
				grid(currentGrid).getVelX(x + 1, y) = 0;
				grid(currentGrid).getVelY(x, y + 1) = 0;
			}
		}
}

void Fluid::markerParticleResampling()
{
	for (int x = 0; x < MACGrid2dF::width(); x++)
		for (int y = 0; y < MACGrid2dF::height(); y++)
		{
			if (grid(currentGrid).getState(x, y) == FLUID)
			{
				int count = grid(currentGrid).getParticleCount(x, y);
				if (count >= 1 && Rand(1.f) < 0.01f)
				{
					particles.push_back(vec2(x, y));
				}	
			}
		}
	for (int i = 0; i < particles.size(); i++)
	{
		int x = particles[i].x, y = particles[i].y;
		int count = grid(currentGrid).getParticleCount(x, y);
		if (count >= 2 && Rand(1.f) < 0.01f)
		{
			//remove the particle in dense areas with a certain probability depending on density
			particles[i] = particles[particles.size() - 1];
			particles.pop_back();
		}
	}
}

vec2 dX(.5f, 0);
vec2 dY(0, .5f);

void Fluid::simulateStep(float timeStep)
{
	//reset states
	for (int x = 0; x < MACGrid2dF::width(); x++)
		for (int y = 0; y < MACGrid2dF::height(); y++)
		{
			grid(currentGrid).getState(x, y) = AIR;
		}

	if (enableSink)
		for (int i = 0; i < sinks.size(); i++)
			grid(currentGrid).getState(sinks[i].x, sinks[i].y) = SINK;

	advectParticles(timeStep);
	
	for (int x = 0; x < MACGrid2dF::width(); x++)
	{
		grid(currentGrid).getState(x, 0) = OBJECT;
		grid(currentGrid).getState(x, GRID_HEIGHT - 1) = OBJECT;
	}
	for (int y = 0; y < MACGrid2dF::width(); y++)
	{
		grid(currentGrid).getState(0, y) = OBJECT;
		grid(currentGrid).getState(GRID_HEIGHT- 1, y) = OBJECT;
	}

	if(enableSink)
		for (int i = 0; i < sinks.size(); i++)
			grid(currentGrid).getState(sinks[i].x, sinks[i].y) = SINK;

	stateUpdates();
	syncGrid();

	advect(timeStep); 
	applyBodyForce(timeStep, bodyForces);
	//diffusion, deals with viscosity, not implemented

	syncGrid();
	updatePressure();
	subtractPressureGradient(timeStep);

	updateParticleCounts();
#ifdef USE_MARKER_RESAMPLING
	markerParticleResampling();
#endif
	
	stateUpdates();
	if(enableTaps)
		handleTaps();
}

unsigned char toByte(float v) { return (unsigned char)int(glm::clamp(v, 0.f, 255.f)); }

void Fluid::Draw(Surface * surface)
{
	float pixelSize = GRID_HEIGHT / size.x;

	if (drawMode == PRESSURE)
	{
		for (int x = 0; x < size.x; x++)
			for (int y = 0; y < size.y; y++)
			{
				float p = interpolate(x * pixelSize, y * pixelSize, getPressure);
				Color col{ 255, toByte(max(0.f, -p * 1.5f)), 1 , toByte(max(0.f,p * 1.5f)) };
				surface->Plot(x, y, col.value);

			}
	}
	else if (drawMode == VELOCITY)
	{
		for (int x = 0; x < size.x; x++)
			for (int y = 0; y < size.y; y++)
			{
				float velX = interpolate(x * pixelSize, y * pixelSize, getX);
				float velY = interpolate(x * pixelSize, y * pixelSize, getY);
				vec3 blue(0, 0, 255), yellow(255, 255, 0), green(0, 255, 0), red(255, 0, 0);
				vec3 color = 0.1f * (((velX > 0) ? yellow * velX : blue * (-velX)) +
					((velY > 0) ? red * velY : green * (-velY)));
				Color col{ 255, toByte(clamp(color.r, 0.f, 255.f)), 
					toByte(clamp(color.g, 0.f, 255.f)), toByte(clamp(color.b, 0.f, 255.f)) };
				surface->Plot(x, y, col.value);

			}
	}
	else if (drawMode == GRIDSTATE)
	{

		for (int x = 0; x < MACGrid2dF::width(); x++)
			for (int y = 0; y < MACGrid2dF::height(); y++)
			{
				Pixel statecol = 0xFF000022;
				vec2 tl = transformGrid2Screen(vec2(x, y));
				vec2 br = transformGrid2Screen(vec2(x + 1, y + 1));
				if (grid(currentGrid).getState(x, y) == OBJECT) statecol = 0xFFFF0000;
				if (grid(currentGrid).getState(x, y) == SINK)  statecol = 0xFF00FF00;
				if (grid(currentGrid).getState(x, y) == FLUID) statecol = 0xFF00AAAA;
				surface->Bar(tl.x, tl.y, br.x, br.y, statecol);

			}
	}
	if (enableDrawParticles)
	{
		for (int i = 0; i < particles.size(); i++)
			surface->Circle(transformGrid2Screen(particles[i]), 3, 255, particleColor);
	}

}

void Fluid::handleTaps()
{
	for (int i = 0; i < taps.size(); i++)
	{
		grid(currentGrid).getVelY(taps[i].x, taps[i].y) = 40;
		grid(currentGrid).getVelX(taps[i].x, taps[i].y) = 0;
		grid(currentGrid).getVelY(taps[i].x, taps[i].y + 1) = 40;
		grid(currentGrid).getVelX(taps[i].x + 1, taps[i].y) = 0;
		if (grid(currentGrid).getParticleCount(taps[i].x, taps[i].y) == 0)
			particles.push_back(taps[i]);
	}
}

void Fluid::advectParticles(float timeStep)
{
	for (int i = 0; i < particles.size(); i++)
	{
		//advect
		float x = interpolate(particles[i] + dX, getX);
		float y = interpolate(particles[i] + dY, getY);
		particles[i] += vec2(x, y) * timeStep;

		//update states
		if (grid(currentGrid).getState(particles[i].x, particles[i].y) == SINK)
		{
			//if particle ended in a sink, remove particle
			particles[i] = particles[particles.size() - 1];
			particles.pop_back();
		}
		else
		{
			grid(currentGrid).getState(particles[i].x, particles[i].y) = FLUID;
			grid(currentGrid).getState(particles[i].x + 1, particles[i].y) = FLUID;
			grid(currentGrid).getState(particles[i].x - 1, particles[i].y) = FLUID;
			grid(currentGrid).getState(particles[i].x, particles[i].y + 1) = FLUID;
			grid(currentGrid).getState(particles[i].x, particles[i].y - 1) = FLUID;
		}
	}
}
