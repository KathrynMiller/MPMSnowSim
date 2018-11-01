#include "simulation.h"
#include <algorithm>
#include "la.h"
#include "math.h"
#include "QFileDialog";

Simulation::Simulation(Particles* p): particles(p), grid(nullptr)
{
    numParticles = p->numParticles;
}

Simulation::~Simulation() {
    delete particles;
    delete grid;
}

// returns a vec3 of row i of m
static glm::vec3 toVec3(Eigen::MatrixXd m, int r) {
    return glm::vec3(m(r, 0), m(r, 1), m(r, 2));
}

void Simulation::initializeGrid() {

    float cellsize = .1;
    // initialize origin and dimensions of the grid
    glm::vec3 minCorner = glm::vec3(INFINITY);
    glm::vec3 maxCorner = glm::vec3(-INFINITY);
    for(int i = 0; i < particles->positions.rows(); i++) {
        glm::vec3 p =  toVec3(particles->positions, i);
        minCorner[0] = std::min(p[0], minCorner[0]);
        minCorner[1] = std::min(p[1], minCorner[1]);
        minCorner[2] = std::min(p[2], minCorner[2]);

        maxCorner[0] = std::max(p[0], maxCorner[0]);
        maxCorner[1] = std::max(p[1], maxCorner[1]);
        maxCorner[2] = std::max(p[2], maxCorner[2]);
    }
    // offset by 2 grid cells in each dimension (worldspace dimensions)
    minCorner = minCorner - glm::vec3(cellsize * 2.0);
    maxCorner = maxCorner + glm::vec3(cellsize  * 2.0);
    // set grid origin and dimensions
    glm::vec3 origin = minCorner;
    glm::vec3 dim = (maxCorner - minCorner) / cellsize; // in cellsize units

    grid = new Grid(origin, dim, cellsize);

}

// the piecewise function defined in "A Material Point Method for Snow Simulation"
static float funcN(float x) {
        float absX = fabs(x);
        if (absX < 0.5f) {
            return 0.75f - pow(absX, 2.f);
        }
        else if (absX < 1.5f) {
            return 0.5f * pow((1.5f - absX), 2.f);
        }
        else {
            return 0.f;
        }
    //    if(magX < 0.0) {
    //        return 0;
    //    } else if (magX < 1.0) {
    //        return 0.5 * pow(magX, 3.0) - pow(magX, 2.0) + (2.0 / 3.0);
    //    } else if (magX < 2.0) {
    //        return (-1.0 / 6.0) * pow(magX, 3.0) + pow(magX, 2.0) - 2.0 * magX + (4.0 / 3.0);
    //    }
    //    return 0.0;
}

// the piecewise function defined in "A Material Point Method for Snow Simulation"
static float gradN(float x) {
    float absX = fabs(x);
    if (absX < 0.5f) {
        return -2.0f * x;
    }
    else if (absX < 1.5f) {
        return x - 1.5f * (x < 0 ? -1 : 1);
    }
    else {
        return 0.f;
    }
    //    if(magX < 0.0) {
    //        return 0;
    //    } else if (magX < 1.0) {
    //        return 1.5 * pow(magX, 2.0) - 2.0 * magX;
    //    } else if (magX < 2.0) {
    //        return (-0.5) * pow(magX, 2.0) + 2.0 * magX - 2.0;
    //    }
    //    return 0.0;
}

// returns the local base node of particle p in grid coordinates
glm::vec3 Simulation::getBaseNode(glm::vec3 particle) {

    // position on grid in terms of cellsize
    glm::vec3 gridOffset = (particle - grid->origin) / grid->cellsize;
    // floor the particle to get its node's lower corner, then subtract half a cell in all directions and floor to get base
    glm::vec3 baseNode = glm::vec3(floor(gridOffset[0]), floor(gridOffset[1]), floor(gridOffset[2]));
    //baseNode -= glm::vec3(0.5 * grid->cellsize);
    baseNode -= glm::vec3(0.5);
    baseNode = glm::vec3(floor(baseNode[0]), floor(baseNode[1]), floor(baseNode[2]));

    return baseNode;
}

void Simulation::fillKernelWeights() {
    // for each particle
    for(int p = 0; p < particles->positions.rows(); p++) {

        //        glm::vec3 particle = toVec3(particles->positions, p); // world space pos
        //        glm::vec3 gridOffset = (particle - grid->origin) / grid->cellsize; // pos on grid in cellsize-units
        //        glm::vec3 baseNode = getBaseNode(particle); // in grid coordinates
        //        glm::vec3 baseNodeOffset = gridOffset - baseNode;
        glm::vec3 particle = toVec3(particles->positions, p); // world space pos
        glm::vec3 gridOffset = (particle - grid->origin) / grid->cellsize; // pos on grid in cellsize-units
        glm::vec3 baseNodeOffset = gridOffset - glm::vec3(floor(gridOffset[0] - 0.5), floor(gridOffset[1] - 0.5),
                floor(gridOffset[2] - 0.5));
        // kernel for particle p
        KernelWeights* kernelWeight = particles->kernelWeights[p];

        for(int i = 0; i < grid->Dimension; i++) { // rows of N and N'
            for(int j = 0; j < 3; j++) { // columns of N and N'

                float x = baseNodeOffset[i] - j;//1.0 / grid->cellsize * (particle[i] - currNode[i] * grid->cellsize);
                kernelWeight->N(i, j) = funcN(x);
                kernelWeight->N_deriv(i, j) = gradN(x);
            }
        }
    }
}


void Simulation::P2G() {
    // initialize grid values to 0
    grid->clear();
    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        glm::vec3 baseNode = getBaseNode(particle);

        // for each particle, calculate the nodes it affects and add its contribution to that grid node's weight sum for velocity and momentum
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                for(int k = 0; k < 3; k++) {
                    glm::vec3 currNode = baseNode + glm::vec3(i, j, k);
                    KernelWeights* kernelWeight = particles->kernelWeights[p];
                    // weight for this node based on particle kernel
                    float weight = kernelWeight->N(0, i) * kernelWeight->N(1, j) * kernelWeight->N(2, k);

                    // extract particle mass and velocity from eigen matrix
                    float pMass = particles->masses(p);
                    glm::vec3 pVelocity = glm::vec3(particles->velocities(p, 0), particles->velocities(p, 1), particles->velocities(p, 2));

                    // grid attributes += weight * particle attributes
                    float gridMass = grid->getMass(currNode) + (weight * particles->masses(p));
                    glm::vec3 gridMomentum = grid->getMomentum(currNode) + (weight * pMass * pVelocity);

                    glm::vec3 gridVelocity = glm::vec3(0.0, 0.0, 0.0);
                    if(gridMass > 0.0) {
                        gridVelocity = gridMomentum / gridMass;
                    }

                    // transfer values to grid
                    grid->setMass(currNode, gridMass);
                    grid->setVelocity(currNode, gridVelocity);
                    grid->setMomentum(currNode, gridMomentum);
                }
            }
        }
    }
}

void Simulation::G2P() {
    // clear particle velocities;
    particles->velocities = Eigen::MatrixXd::Zero(numParticles, 3);

    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        glm::vec3 baseNode = getBaseNode(particle);

        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                for(int k = 0; k < 3; k++) {
                    glm::vec3 currNode = baseNode + glm::vec3(i, j, k);

                    KernelWeights* kernelWeight = particles->kernelWeights[p];

                    // weight for this node based on particle kernel
                    float weight = kernelWeight->N(0, i) * kernelWeight->N(1, j) * kernelWeight->N(2, k);
                    glm::vec3 gridVelocity = grid->getVelocity(currNode);

                    glm::vec3 newVel = glm::vec3(particles->velocities(p, 0) + weight * gridVelocity[0],
                            particles->velocities(p, 1) + weight * gridVelocity[1],
                            particles->velocities(p, 2) + weight * gridVelocity[2]);
                    particles->velocities(p, 0) = newVel[0];
                    particles->velocities(p, 1) = newVel[1];
                    particles->velocities(p, 2) = newVel[2];
                }
            }
        }

    }
}

void Simulation::updateParticlePositions(float dt) {
    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 newVel = glm::vec3(particles->positions(p, 0) + particles->velocities(p, 0) * dt,
                                     particles->positions(p, 1) + particles->velocities(p, 1) * dt,
                                     particles->positions(p, 2) + particles->velocities(p, 2) * dt);
        particles->positions(p, 0) = newVel[0];
        particles->positions(p, 1) = newVel[1];
        particles->positions(p, 2) = newVel[2];


        // need to clamp positions to box
        glm::vec3 origMin = grid->origin + glm::vec3(2.0 * grid->cellsize);
        if(particles->positions(p, 0) < origMin[0]) {
            particles->positions(p, 0) = origMin[0];
        }
        if(particles->positions(p, 1) < origMin[1]) {
            particles->positions(p, 1) = origMin[1];
        }
        if(particles->positions(p, 2) < origMin[2]) {
            particles->positions(p, 2) = origMin[2];
        }

        glm::vec3 origMax = grid->origin + (grid->dim * grid->cellsize) - glm::vec3(2.0 * grid->cellsize);
        if(particles->positions(p, 0) > origMax[0]) {
            particles->positions(p, 0) = origMax[0];
        }
        if(particles->positions(p, 1) > origMax[1]) {
            particles->positions(p, 1) = origMax[1];
        }
        if(particles->positions(p, 2) > origMax[2]) {
            particles->positions(p, 2) = origMax[2];
        }


    }
}

void Simulation::RunSimulation(QString output_filepath) {

    float dt = .1;
    // make new grid
    initializeGrid();

    fillKernelWeights();
    int i = 100;
    while(i > 0) {
        // transfer attributes to the grid
        P2G();
        // compute forces
        // transfer attributes back to particles
        G2P();
        //update particle positions
        updateParticlePositions(dt);
        //        particles->destroy();
        //        particles->create();
        saveToObj(output_filepath);

        frameNumber++;
        i--;
    }
}

void Simulation::saveToObj(QString output_filepath) {
    if(output_filepath.length() == 0)
    {
        return;
    }
    QFile file(output_filepath + "/frame_" + QString::number(frameNumber) + ".obj");
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);

        for(int i = 0; i < particles->positions.rows(); i++) {
            stream << "v " << QString::number(particles->positions(i, 0)) << " "
                   << QString::number(particles->positions(i, 1)) << " "
                   << QString::number(particles->positions(i, 2)) << endl;
        }

    }
}
