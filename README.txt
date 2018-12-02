Material Point Method for Snow Simulation
======================

**based on the paper by Alexey Stomakhin, Craig Schroeder, Lawrence Chai, Joseph Teran,  and Andrew Selle**

Overview
------------
The goal of this project is to create an MPM solver that can be used in the simulation of materials such as snow. The project was written in C++ and in Qt. Each frame was outputted to an Obj file consisting of just point locations that I then loaded into Houdini for visualization. My hope is to alter this to use PartIO for quicker writing and to gain the ability of visualizing other attributes like deformation gradient through color.   

Poisson Disk Sampling
---------
The first thing I implemented was Poisson Disc sampling for an even random distribution of points within an arbitrary mesh. An overview of the algorithm is as follows:

* Given the bounding box of your mesh, select a random point that you know to be within its bounds
* Add this point, x<sub>i</sub>, to the list of active samples
* while the size of the active samples list is > 0, randomly generate K more samples within the radius R to 2R around x<sub>i</sub>
	*for each of the K samples, if the sample is less than R away from any other point, it is invalid
	* if the point is valid, add it to the list of active samples
	* if the current active point being used to generate K more samples doesn't yield any valid samples, remove it from the active list and add it to the final list of points

This will generate a nice sampling within the bounding box of the object, but getting these points to represent a mesh shape involves one more check. For each point in the final list of samples, check if the point is within the mesh by casting a ray in an arbitrary direction and seeing how many times this ray intersects. An odd number of intersections means the point is within the mesh. An even number means it is outside the mesh. This is very slow without any acceleration structures because it would involve checking the ray against every triangle in the mesh. To speed this up a bit I implemented a KD-Tree that loads the triangles of a mesh into a tree structure and makes intersection testing much less painful.


Basics of MPM
--------------
The main idea behind MPM is that it utilizes both a Eulerian and a Lagrangian approach in calculating material properties like velocity, mass, position, and stress. 