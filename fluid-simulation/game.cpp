#include "template.h"

// -----------------------------------------------------------
// Initialize the game
// -----------------------------------------------------------
void Game::Init()
{
	//boxes.push_back(Box{ vec2(-40,30), vec2(20,30), 40});
	f2 = new Fluid(glm::vec2(0, 0), glm::vec2(SCRWIDTH, SCRHEIGHT));
	//fluid->spawn(vec2(0, 0), vec2(20, 10), 1/50.f);
	f2->sinks.push_back(vec2(3, GRID_HEIGHT - 2));
	f2->sinks.push_back(vec2(4, GRID_HEIGHT - 2));
	f2->sinks.push_back(vec2(5, GRID_HEIGHT - 2));
	f2->sinks.push_back(vec2(3, GRID_HEIGHT - 1));
	f2->sinks.push_back(vec2(4, GRID_HEIGHT - 1));
	f2->sinks.push_back(vec2(5, GRID_HEIGHT - 1));

	f2->taps.push_back(vec2(GRID_HEIGHT / 2 - 1, 1));
	f2->taps.push_back(vec2(GRID_HEIGHT / 2 + 0, 1));
	f2->taps.push_back(vec2(GRID_HEIGHT / 2 + 1, 1));
	f2->taps.push_back(vec2(GRID_HEIGHT / 2 - 1, 2));
	f2->taps.push_back(vec2(GRID_HEIGHT / 2 + 0, 2));
	f2->taps.push_back(vec2(GRID_HEIGHT / 2 + 1, 2));
}


// -----------------------------------------------------------
// Input handling
// -----------------------------------------------------------

bool down1, down2, down3, down4, down5;
bool pressed1, pressed2, pressed3, pressed4, pressed5;
void Game::KeyDown(int a_Key)
{
	if (a_Key == SDL_SCANCODE_1)
	{
		if (!down1) pressed1 = true;
		down1 = true;
	}
	if (a_Key == SDL_SCANCODE_2)
	{
		if (!down2) pressed2 = true;
		down2 = true;
	}
	if (a_Key == SDL_SCANCODE_3)
	{
		if (!down3) pressed3 = true;
		down3 = true;
	}
	if (a_Key == SDL_SCANCODE_4)
	{
		if (!down4) pressed4 = true;
		down4 = true;
	}
	if (a_Key == SDL_SCANCODE_5)
	{
		if (!down5) pressed5 = true;
		down5 = true;
	}
	
}
void Game::KeyUp(int a_Key)
{
	if (a_Key == SDL_SCANCODE_1)
		down1 = false;
	if (a_Key == SDL_SCANCODE_2)
		down2 = false;
	if (a_Key == SDL_SCANCODE_3)
		down3 = false;
	if (a_Key == SDL_SCANCODE_4)
		down4 = false;
	if (a_Key == SDL_SCANCODE_5)
		down5 = false;
}

int cooldown = 0;
void Game::HandleInput( float timeStep )
{
	cooldown--;

	if(mouseDown)
		f2->applyForceAt(mouseX / (600 / GRID_HEIGHT), mouseY / (600 / GRID_HEIGHT), timeStep, glm::vec2(700, -400));

	if (pressed1)
		f2->enableSink = !f2->enableSink;

	if (pressed2)
		f2->enableTaps = !f2->enableTaps;

	if (pressed3)
		f2->enableDrawParticles = !f2->enableDrawParticles;

	if (pressed4)
	{
		(*((int*)&(f2->drawMode)))++;
		if (f2->drawMode == DrawMode::VELOCITY + 1)
			f2->drawMode = DrawMode::GRIDSTATE;
	}
	if (pressed5)
	{
		screen->SaveImage("screeny.png");
	}
	pressed1 = false, pressed2 = false;
	pressed3 = false, pressed4 = false;
	pressed5 = false;
}

// -----------------------------------------------------------
// Physics simulation
// -----------------------------------------------------------



void Game::Simulate(float timeStep)
{
	f2->simulateStep(timeStep);
}

// -----------------------------------------------------------
// Rendering of the scene.
// -----------------------------------------------------------
void Game::Render( float timeStep )
{
	screen->Clear( 0xFFFFFF );
	f2->Draw(screen);
}

// --------------------------------
//below this is inital experimental stuff
//this is not used in the execution of this program
// --------------------------------



void Game::integrateVelocity(float timeStep)
{
	fluid->integrateVelocity(timeStep);
}

void Game::integratePosition(float timeStep)
{
	for (int i = 0; i < boxes.size(); i++)
		boxes[i].integratePosition(timeStep);
	fluid->integratePosition(timeStep);
}

void Game::resolveConstraints(float timeStep)
{
	for (int i = 0; i < 15; i++)
		fluid->resolveConstraints(timeStep);
}

void Game::projectVelocities(float timeStep)
{
	fluid->projectVelocities(timeStep);
	for (int i = 0; i < boxes.size(); i++)
	{
		boxes[i].velocity = (boxes[i].com - boxes[i].prev_com) / timeStep;
		boxes[i].prev_com = boxes[i].com;
	}
}

void Box::Draw(Surface* surface)
{
	vec2 topleft = transform(com) - size / 2.f;
	int tlX = topleft.x; int tlY = topleft.y;
	vec2 bottomright = transform(com) + size / 2.f;
	int brX = bottomright.x; int brY = bottomright.y;
	surface->Bar(tlX, tlY, brX, brY, color.value);
}



void Tmpl8::Fluid_old::Draw(Surface * surface)
{
	for (int i = 0; i < particle_count; i++)
	{
		vec2 pos = transform(positions[i]);
		surface->Circle(pos, 8.f, color.a, color);
	}

}

void Tmpl8::Fluid_old::integratePosition(float timeStep)
{
	for (int i = 0; i < particle_count; i++)
		positions[i] += velocities[i] * timeStep;
}

void Tmpl8::Fluid_old::integrateVelocity(float timeStep)
{
	for (int i = 0; i < particle_count; i++)
		velocities[i] += vec2(0, -9.81) * invDensity * timeStep;
}

bool Tmpl8::Fluid_old::isConstraintsValid()
{
	return false;
}

void Fluid_old::projectVelocities(float timeStep)
{
	for (int i = 0; i < particle_count; i++)
	{
		velocities[i] = (positions[i] - prev_positions[i]) / timeStep;
		prev_positions[i] = positions[i];
	}
}

void Fluid_old::resolveConstraints(float timeStep)
{
	for (int i = 0; i < particle_count; i++)
	{
		if (positions[i].y < -100)
			positions[i].y = -100;
		if (positions[i].y > 100)
			positions[i].y = 100;
		if (positions[i].x < -100)
			positions[i].x = -100;
		if (positions[i].x > 100)
			positions[i].x = 100;
		for (int j = 0; j < particle_count; j++)
		{
			if (i == j)
				continue;
			vec2 diff = positions[i] - positions[j];
			float constraintValue = sqrtf(abs(dot(diff, diff))) - particle_size;
			if (constraintValue < 0)
			{
				positions[i] += diff * constraintValue * 0.5f;
				positions[j] -= diff * constraintValue * 0.5f;
			}
		}
	}
	
}

// --------------------------------
//above this is inital experimental stuff
//this is not used in the execution of this program
// --------------------------------