# mobile-robotics
Webots projects for the Mobile Robotics course of the master's degree.

All projects are developed using the Webots simulator using Khepera IV robot with Python.

## Project 1
The first project consists of a simple robot that moves in a 2D environment. The goal is to avoid obstacles using the robot's sensors.

## Project 2
The second project consists of a robot that moves in a 2D environment and has to reach a goal. The robot starts in wall following mode, making a map of the environment. When the map is complete, the robot calculates the shortest path to the goal and moves to it.

### Map
The map consists of a matrix where each cell represents a position in the environment. 
- The robot can move in 4 directions: up, down, left and right. The robot can't move diagonally. 
- The robot can't move to a cell that has an obstacle. 
- The robot can move to a cell that has the value 0. 
- Walls are represented by the value 1. 
- The base of the robot is represented by the value 2. 
- The goal is represented by the value 3.
