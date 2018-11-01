#pragma once
#include "particles.h"
#include "grid.h"
#include "eigen-git-mirror/Eigen/Core"

class Simulation
{

public:
    Simulation(Particles *p);
    ~Simulation();

    // the set of particles over which to run the sim
    Particles* particles;

    // background grid for the particles
    Grid* grid;

    int numParticles;

    int frameNumber = 0;

    bool isRunning = false;

    int Dimension = 2; // dimensions of the grid. start with xy and move to xyz after working

    void fillKernelWeights();
    // helper for finding the base node of a particle's kernel
    glm::vec3 getBaseNode(glm::vec3 particle);
    // P2G takes particles from lagrangian to eulerian grid space
    void P2G();

    // compute forces

    // G2P
    void G2P();

    // resets the grid to be of correct size for this timestep and sets all values to 0
    void initializeGrid();

    // calculations for one timestep of the sim
    void updateParticlePositions(float dt);

    // saves current particle positions in an obj file
    void saveToObj(QString output_filepath);

    // updates the particles positions in accordance with the MPM paper
    // takes in the file path where the obj files for each step will be stored
    void RunSimulation(QString output_filepath);

};
