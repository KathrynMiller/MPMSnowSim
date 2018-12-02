#pragma once
#include "particles.h"
#include "grid.h"
#include "eigen-git-mirror/Eigen/Core"

class Simulation
{

public:
    Simulation(int numSeconds, int frameRate);
    ~Simulation();
    void setParticles(Particles* p);

    // the set of particles over which to run the sim
    Particles* particles;
    // background grid for the particles
    Grid* grid;

    int numParticles;
    int stepsPerFrame;
    float dt = 1e-3;
    int frameNumber = 0;
    int numOutputFrames; // default number of frames to simulate
    // used to modify dt
    float maxParticleVelocity = -INFINITY;
    bool isRunning = false;

    // constants that determine the consistency of the snow and can be changed in the gui
    float youngsMod = 1.4 * pow(10.0, 5.0);
    float poisson = 0.2;
    float hardeningCoeff = 10.0;
    float thetaC = 2.5 * pow(10.0, -2.0);
    float thetaS = 7.5 * pow(10.0, -3.0);
    glm::vec3 gridMinOffset = glm::vec3(-5, -5, -5);
    glm::vec3 gridMaxOffset = glm::vec3(5, 5, 5);
    bool isSnow = true;



    void fillKernelWeights();
    // helper for finding the base node of a particle's kernel
    glm::vec3 getBaseNode(glm::vec3 particle);
    // P2G takes particles from lagrangian to eulerian grid space
    void P2G();

    // compute P, or hyperelasticity model that fills particle stresses
    void computeStress();
    // integrates plastic component of deformation gradient
    void computeSnowStress();
    // compute forces
    void computeForces();
    void updateGradient();

    // G2P
    void G2P();

    // resets the grid to be of correct size for this timestep and sets all values to 0
    void initializeGrid(float cellsize);

    // calculations for one timestep of the sim
    void updateParticlePositions();

    // saves current particle positions in an obj file
    void saveToObj(QString output_filepath);
    void saveToGeo(QString output_filepath);

    // updates the particles positions in accordance with the MPM paper
    // takes in the file path where the obj files for each step will be stored
    void RunSimulation(QString output_filepath, GLWidget277* mygl);

    // helpers
    float getWeight(KernelWeights *kernelWeight, glm::vec3 particle, glm::vec3 node);
    Eigen::Vector3d getWeightGradient(KernelWeights *kernelWeight, glm::vec3 particle, glm::vec3 node);
    // returns list of neighboring node positions for iteration
    std::vector<glm::vec3> getNeighbors(glm::vec3 particle);
};
