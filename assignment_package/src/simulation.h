#pragma once
#include "particles.h"
#include "eigen-git-mirror/Eigen/Core"

class Simulation
{

public:
    Simulation(Particles *p);
    ~Simulation();

    // the set of particles over which to run the sim
    Particles* particles;
    int numParticles;


    // updates the particles positions in accordance with the MPM paper
    void RunSimulation();

};
