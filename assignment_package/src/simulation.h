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
    int numOutputFrames = 300; // default number of frames to simulate

    bool isRunning = false;

    void fillKernelWeights();
    // helper for finding the base node of a particle's kernel
    glm::vec3 getBaseNode(glm::vec3 particle);
    // P2G takes particles from lagrangian to eulerian grid space
    void P2G();

    // compute P, or hyperelasticity model that fills particle stresses
    void computeStress();
    // compute forces
    void computeForces();
    void updateGradient(float dt);

    // G2P
    void G2P();

    // resets the grid to be of correct size for this timestep and sets all values to 0
    void initializeGrid(float cellsize);

    // calculations for one timestep of the sim
    void updateParticlePositions(float dt);

    // saves current particle positions in an obj file
    void saveToObj(QString output_filepath);

    // updates the particles positions in accordance with the MPM paper
    // takes in the file path where the obj files for each step will be stored
    void RunSimulation(QString output_filepath);

    // helpers
    float getWeight(int particleId, glm::vec3 node);
    Eigen::Vector3d getWeightGradient(KernelWeights *kernelWeight, glm::vec3 particle, glm::vec3 node);
    // returns list of neighboring node positions for iteration
    std::vector<glm::vec3> getNeighbors(glm::vec3 particle);

};
