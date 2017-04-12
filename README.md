# Fluid simulation (INFOMGP)

Written by Twan Veldhuis and Klaas Hidde van den Berg

This program uses a template created by Jacco Bikker, which provides a simple image buffer and input handling functionality.
We have modified part of the template code to support saving images and circles.

The fluid simulation makes use of a Eulerian model with marker particles, described in Fluid.cpp/h and Grid.cpp/h.
We also have some older experimental code with particles in Game.h/cpp, you can disregard that part, it's not executed.

## Controls
Use 1 to toggle the sink on the bottom right (drains marker particles from the system)
Use 2 to toggle the tap at the top (adds marker particles to the system)
Use 3 to toggle particle visibility
Use 4 to switch between view modes (State grid, Pressure, Velocity)

The velocity plot works on CIELab axis (blue vs yellow for x-axis velocity, red vs green on y axis).
