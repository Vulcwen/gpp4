#pragma once

//settings
#define SCRWIDTH	 600
#define SCRHEIGHT	 600



//old settings
#define SCALE 1.f
#define FLUID_INCOMPR_MAXITER 100

//utility defines

#define WORLD_X_BOUND (SCRWIDTH / (2 * SCALE)) //half world width
#define WORLD_Y_BOUND (SCRHEIGHT / (2 * SCALE)) //half world height

namespace Tmpl8 {
	using namespace glm;
	using namespace std;

	inline vec2 transform(vec2 worldSpace)
	{
		return vec2(worldSpace.x * SCALE + SCRWIDTH / 2, -worldSpace.y * SCALE + SCRHEIGHT / 2);
	}
	inline vec2 invTransform(vec2 screenSpace)
	{
		return vec2((screenSpace.x - SCRWIDTH / 2) / SCALE, -(screenSpace.y - SCRHEIGHT / 2) / SCALE);
	}
	class Surface;

	class Fluid_old
	{
	public:
		int particle_count;
		void Draw(Surface* surface);
		void integratePosition(float timeStep);
		void integrateVelocity(float timeStep);
		//return whether all fluid constraints are met
		bool isConstraintsValid();
		//resolve fluid constraints like incompressiblity (single iteration).
		void resolveConstraints(float timeStep);
		void projectVelocities(float timeStep);

		inline bool spawn(vec2 pos, vec2 vel, float timeStep)
		{
			if (particle_count < positions.size())
			{
				int idx = particle_count++;
				positions[idx] = pos;
				velocities[idx] = vel;
				prev_positions[idx] = pos - vel * timeStep;
				return true;
			}
			return false;
		}

		Color color;
		const float invDensity = 1.f;
		const float particle_size = 6.f;
		std::vector<vec2> prev_positions;
		std::vector<vec2> positions;
		std::vector<vec2> velocities;
		Fluid_old(Color color, int particle_capacity)
			: color(color)
		{
			particle_count = 0;
			prev_positions = std::vector<vec2>(particle_capacity);
			positions = std::vector<vec2>(particle_capacity);
			velocities = std::vector<vec2>(particle_capacity);
			memset(prev_positions.data(), 0, sizeof(vec2));
			memset(positions.data(), 0, sizeof(vec2));
			memset(velocities.data(), 0, sizeof(vec2));
		}
	};
	class Box
	{
	public:
		void Draw(Surface* surface);
		Color color;
		vec2 com;
		vec2 prev_com;
		vec2 size;
		vec2 velocity;
		float invMass;
		inline void integratePosition(float timeStep) { com += velocity * timeStep; }
		Box(vec2 com, vec2 size, float mass, vec2 velocity = vec2(0, 0), Color color = Color(0, 0, 0, 0)) :
			com(com), size(size), color(color), velocity(velocity), prev_com(com)
		{
			if (mass != 0)
				invMass = 1 / mass;
			else
				invMass = 0;
		}
	};

	class Game
	{
	public:
		int mouseX = 0, mouseY = 0;
		bool mouseDown = false;
		void SetTarget(Surface* _Surface) { screen = _Surface; }
		void Init();
		void Shutdown() { /* implement if you want code to be executed upon app exit */ };
		void HandleInput(float dt);
		void Simulate(float timeStep);
		void Render(float dt);
		void MouseUp(int _Button) { mouseDown = false; }
		void MouseDown(int _Button) { mouseDown = true; }
		void MouseMove(int _X, int _Y) { mouseX = _X; mouseY = _Y; }
		void KeyUp(int a_Key);
		void KeyDown(int a_Key);

		void integratePosition(float timeStep);
		void integrateVelocity(float timeStep);
		void resolveConstraints(float timeStep);
		void projectVelocities(float timeStep);
	private:
		Surface* screen;
		Fluid_old* fluid;
		Fluid * f2;
		std::vector<Box> boxes;
	};



}; // namespace Tmpl8