#include "simulation.h"

Simulation::Simulation(Particles* p): particles(p)
{}

Simulation::~Simulation() {
    delete particles;
}

void Simulation::RunSimulation() {

}

Particles* Simulation::getParticles() const {
    return particles;
}
