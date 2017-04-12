#pragma once

enum CellState { FLUID, AIR, OBJECT, SINK };

template <class Tpr, class Tvel, class Tmeta, int pr_w, int pr_h>
class MACGrid2d
{
	int particleCount[pr_h][pr_w];
	Tmeta meta[pr_h][pr_w];
	CellState state[pr_h][pr_w];
	Tpr pressure[pr_h][pr_w];
	Tvel velX[pr_h+1][pr_w+1];
	Tvel velY[pr_h+1][pr_w+1];
public:
	MACGrid2d() {}
	MACGrid2d(Tpr prDefault, Tvel velDefaultX, Tvel velDefaultY)
	{
		for (int y = 0; y < pr_h; y++)
			for(int x = 0; x < pr_w; x++)
			{
				state[y][x] = OBJECT;
				pressure[y][x] = prDefault;
				velX[y][x] = velDefaultX;
				velY[y][x] = velDefaultY;
			}
	}
	Tvel zeroVel = 0.0f;
	const static int width() { return pr_w; }
	const static int height() { return pr_h; }
	CellState stateDummy = OBJECT;
	inline CellState & getState(int x, int y) 
	{
		stateDummy = OBJECT;
		if (y < 0 || x < 0 || x >= pr_w || y >= pr_w)
			return stateDummy;
		return state[y][x];
	}
	inline int & getParticleCount(int x, int y)
	{
		return particleCount[y][x];
	}
	inline Tmeta & getMeta(int x, int y) 
	{ 
		x = clamp(x, 0, pr_w - 1);
		y = clamp(y, 0, pr_h - 1);
		return meta[y][x];
	}
	inline Tpr & getPressure(int x, int y) {

		x = clamp(x, 0, pr_w - 1);
		y = clamp(y, 0, pr_h - 1);

		return pressure[y][x];
	}

	
	inline Tvel & getVelX(int x, int y) 
	{
		if (x >= 0 && x <= pr_w && y >= 0 && y <= pr_h)
			return velX[y][x];
		else
			return zeroVel;
	}

	
	inline Tvel & getVelY(int x, int y) 
	{ 
		if (x >= 0 && x <= pr_w && y >= 0 && y <= pr_h)
			return velY[y][x];
		else
			return zeroVel;
	}

	/*
	p   |   p   |   p
	        ^
	-----------------------
	p   |>  X   |>  p
	        ^
	-----------------------
	p   |   p   |   p
	*/
	//Pressures with 0, pr_w -1 or pr_h - 1 as coordinate may not have velocities on all sides, if you do ask boundary values, indices are clamped
};

