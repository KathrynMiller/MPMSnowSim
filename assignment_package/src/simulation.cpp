#include "simulation.h"

Simulation::Simulation(Particles* p): particles(p)
{}

Simulation::~Simulation() {
    delete particles;
}

void Simulation::RunSimulation() {
//    for(int i = 0; i < particles->numParticles; i++) {
//        particles->positions(i, 1) -= .1;
//    }
}
