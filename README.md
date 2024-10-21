# Fluid simulation (INFOMGP)

Written by Tessa Veldhuis and Klaas Hidde van den Berg

This program uses a template created by Jacco Bikker, which provides a simple image buffer and input handling functionality.
We have modified part of the template code to support saving images and circles.

The fluid simulation makes use of a Eulerian model with marker particles, described in Fluid.cpp/h and Grid.cpp/h.
We also have some older experimental code with particles in Game.h/cpp, you can disregard that part, it's not executed.

## Controls
Use 1 to toggle the sink on the bottom left (drains marker particles from the system)
Use 2 to toggle the tap at the top (adds marker particles to the system)
Use 3 to toggle particle visibility
Use 4 to switch between view modes (State grid, Pressure, Velocity)

The velocity plot works on CIELab axis (blue vs yellow for x-axis velocity, red vs green on y axis).

Using the mouse you can add a small force to the position you're plotting.

## Code-based settings
In game.h you can adjust the resolution of the simulation with SCRWIDTH and SCRHEIGHT. Note: we assume square windows, functionality is undefined when SCRWIDTH =/= SCRHEIGHT.

In fluid.h you can adjust the grid dimensions (set to 50 in this build), and potentially disable a concept we call Marker Particle Resampling. Marker Particle Resampling serves to counter the effect of marker particles clustering together, thus reducing the overall volume of the simulation. It randomly deletes and adds particles inside grid cells that have at least 1 particle in them. This process will never delete the last particle from a cell.

## Build process
First clone the repository.

As mentioned, we use a template by Jacco Bikker, this can be obtained at:
http://www.cs.uu.nl/docs/vakken/magr/2016-2017/files/tmpl85.00a.zip

All dependencies are included in that package. Extract this to the fluid-simulation directory in your local copy, reject any overwrite prompts you might get.
The solution can then be compiled with Visual Studio 2015 or higher.

## Functionality conditions

Note, we can confirm it to work on Windows 7 64-bit, it should work on newer 64-bit versions of Windows as well.
