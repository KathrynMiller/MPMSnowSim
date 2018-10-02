#pragma once
#include "particles.h"
#include "eigen-git-mirror/Eigen/Core"

class Simulation
{
private:
    // the set of particles over which to run the sim
    Particles* particles;
    int numParticles;

public:
    Simulation(Particles *p);
    ~Simulation();

    // updates the particles positions in accordance with the MPM paper
    void RunSimulation();

    // getter functions
    Particles* getParticles() const;

};
