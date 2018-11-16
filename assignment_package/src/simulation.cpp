#include "simulation.h"
#include <algorithm>
#include "la.h"
#include "math.h"
#include "cmath"
#include "QFileDialog";
#include "eigen-git-mirror/Eigen/Dense"
#include "eigen-git-mirror/Eigen/SVD"
#include "eigen-git-mirror/Eigen/Core"

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

void Simulation::initializeGrid(float cellsize) {

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

    //    // testing one kernel size
    //    minCorner = glm::vec3(-3.5 * cellsize);
    //    maxCorner = glm::vec3(3.5 * cellsize);
    // set grid origin and dimensions
    glm::vec3 origin = minCorner;
    glm::vec3 dim = (maxCorner - minCorner) / cellsize; // in cellsize units
    dim[0] = ceil(dim[0]);
    dim[1] = ceil(dim[1]);
    dim[2] = ceil(dim[2]);
    grid = new Grid(origin, dim, cellsize);

}

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

    //    if (absX < 1.0f) {
    //        return 0.5f * pow(absX, 3.0f) - pow(x, 2.0f) + (2.0f / 3.0f);
    //    }
    //    else if (absX < 2.0f) {
    //        return (-1.0f / 6.0f) * pow(absX, 3.0f) + pow(x, 2.0f) -2.0f * absX + (4.0f / 3.0f);
    //    }
    //    else {
    //        return 0.f;
    //    }
}

static float gradN(float x) {
    float absX = fabs(x);
    if (absX < 0.5f) {
        return -2.0f * x;
    }
    else if (absX < 1.5f) {
        //return x - 1.5f * (x < 0 ? -1 : 1);
        return (-x * (1.5f - absX)) / absX;
    }
    return 0.f;
    //    if (absX < 1.0f) {
    //        return 0.5f * x * ((3.0f * absX) - 4.0f);
    //    }
    //    else if (absX < 2.0f) {
    //        return (-x * pow(absX - 2.0f, 2.0f)) / (2.0f * absX);
    //    }
    //    else {
    //        return 0.f;
    //    }

}

// returns the local base node of particle p in grid coordinates
glm::vec3 Simulation::getBaseNode(glm::vec3 particle) {

    // position on grid in terms of cellsize
    glm::vec3 gridOffset = (particle - grid->origin) / grid->cellsize;
    // floor the particle to get its node's lower corner, then subtract half a cell in all directions and floor to get base
    glm::vec3 baseNode = glm::vec3(floor(gridOffset[0]), floor(gridOffset[1]), floor(gridOffset[2]));
    baseNode -= glm::vec3(0.5);
    baseNode = glm::vec3(floor(baseNode[0]), floor(baseNode[1]), floor(baseNode[2]));

    return baseNode;
}

Eigen::Vector3d Simulation::getWeightGradient(KernelWeights* kernelWeight, glm::vec3 particle, glm::vec3 node) {
    glm::vec3 baseNode = getBaseNode(particle);
    glm::vec3 offset = node - baseNode;
    Eigen::Vector3d weightGrad;
    weightGrad << (kernelWeight->N_deriv(0, int(offset[0])) / grid->cellsize) * kernelWeight->N(0, int(offset[1])) * kernelWeight->N(0, int(offset[2])),
            kernelWeight->N(1, int(offset[0])) * (kernelWeight->N_deriv(1, int(offset[1]))/ grid->cellsize) * kernelWeight->N(1, int(offset[2])),
            kernelWeight->N(2, int(offset[0])) * kernelWeight->N(2, int(offset[1])) * (kernelWeight->N_deriv(2, int(offset[2]))/ grid->cellsize);
    return weightGrad;
}

float Simulation::getWeight(int particleId, glm::vec3 node) {
    glm::vec3 particle = toVec3(particles->positions, particleId);
    glm::vec3 baseNode = getBaseNode(particle);
    glm::vec3 offset = node - baseNode;
    KernelWeights* kernelWeight = particles->kernelWeights[particleId];

    float weight = kernelWeight->N(0, int(offset[0])) * kernelWeight->N(1, int(offset[1])) * kernelWeight->N(2, int(offset[2]));
    return weight;
}

std::vector<glm::vec3> Simulation::getNeighbors(glm::vec3 particle) {
    glm::vec3 baseNode = getBaseNode(particle);
    std::vector<glm::vec3> neighbors = std::vector<glm::vec3>();
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            for(int k = 0; k < 4; k++) {
                glm::vec3 currNode = baseNode + glm::vec3(i, j, k);
                neighbors.push_back(currNode);
            }
        }
    }
    return neighbors;
}

void Simulation::fillKernelWeights() {
    // for each particle
    for(int p = 0; p < particles->positions.rows(); p++) {
        glm::vec3 particle = toVec3(particles->positions, p); // world space pos
        glm::vec3 gridOffset = (particle - grid->origin) / grid->cellsize; // pos on grid in cellsize-units
        glm::vec3 baseNodeOffset = gridOffset - glm::vec3(floor(gridOffset[0] - 0.5), floor(gridOffset[1] - 0.5),
                floor(gridOffset[2] - 0.5));
        //        glm::vec3 baseNode = getBaseNode(particle);
        //        glm::vec3 baseNodeOffset = gridOffset - baseNode;
        // kernel for particle p
        KernelWeights* kernelWeight = particles->kernelWeights[p];
        kernelWeight->N = Eigen::MatrixXd::Zero(3, 4);
        kernelWeight->N_deriv = Eigen::MatrixXd::Zero(3, 4);

        for(int i = 0; i < grid->Dimension; i++) { // rows of N and N'
            for(int j = 0; j < 4; j++) { // columns of N and N'

                float x = baseNodeOffset[i] - j;
                assert(x <= 2);
                kernelWeight->N(i, j) = funcN(x);
                kernelWeight->N_deriv(i, j) = gradN(x);
            }
        }

        // sum to test for valid weights
        double wSum = 0.0;
        Eigen::Vector3d gradWSum = Eigen::Vector3d::Zero();
        std::vector<glm::vec3> neighbors = getNeighbors(particle);
        for(glm::vec3 n: neighbors) {
            wSum += getWeight(p, n);
            gradWSum += getWeightGradient(kernelWeight, particle, n);
        }
        // test that weights are valid
        assert(abs(wSum - 1.0) < 1e-4);
        assert(abs(gradWSum(0, 0)) < 1e-4);
        assert(abs(gradWSum(1, 0)) < 1e-4);
        assert(abs(gradWSum(2, 0)) < 1e-4);
    }
}


void Simulation::P2G() {
    // initialize grid values to 0
    grid->clear();
    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        std::vector<glm::vec3> neighborNodes = getNeighbors(particle);

        // for each particle, calculate the nodes it affects and add its contribution to that grid node's weight sum for velocity and momentum
        for(int i = 0; i < neighborNodes.size(); i++) {
            glm::vec3 currNode = neighborNodes[i];
            // weight for this node based on particle kernel
            float weight = getWeight(p, currNode);

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

static double minorDet(int i, int j, const Eigen::Matrix3d &m) {
    return m(i == 0 ? 1 : 0, j == 0 ? 1 : 0) * m(i == 2 ? i - 1 : 2, j == 2 ? j - 1 : 2)
            - m(i == 0 ? 1 : 0, j == 2 ? j - 1 : 2) * m(i == 2 ? i - 1 : 2, j == 0 ? 1 : 0);
}

static Eigen::Matrix3d ComputeJFInvTranspose(const Eigen::Matrix3d &F) {
    Eigen::Matrix3d result;
    result << minorDet(0, 0, F), -minorDet(0, 1, F),  minorDet(0, 2, F),
            -minorDet(1, 0, F),  minorDet(1, 1, F), -minorDet(1, 2, F),
            minorDet(2, 0, F), -minorDet(2, 1, F),  minorDet(2, 2, F);
    return result;
}
void Simulation::computeStress() {
    for(int i = 0; i < numParticles; i++) {
        // compute the svd of F
        Eigen::Matrix3d F = (particles->deformations[i]->F);
        double J = F.determinant();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Vector3d Sigma = svd.singularValues(); // 3x1 vector representing diagonal values of sigma
        Eigen::Matrix3d V = svd.matrixV();
        /*
        Eigen::RowVector3d temp;
        for (unsigned int i = 1; i < Sigma.size(); ++i) {
            for (unsigned int j = i; j > 0 && Sigma(j - 1, 0) < Sigma(j, 0); j--) {
                std::swap(Sigma(j, 0), Sigma(j - 1, 0));

                temp = U.row(j);
                U.row(j) = U.row(j - 1);
                U.row(j - 1) = temp;

                temp = V.row(j);
                V.row(j) = V.row(j - 1);
                V.row(j - 1) = temp;
            }
        }
        if (U.determinant() < 0) {
            U.col(2) *= -1;
            Sigma(2, 0) *= -1;
        }
        if (V.determinant() < 0) {
            V.col(2) *= -1;
            Sigma(2, 0) *= -1;
        }
*/
        Eigen::Matrix3d R = U * V.transpose();
        Eigen::Matrix3d jFInvTranspose = ComputeJFInvTranspose(F);
        // use it to calculate the stress and put this in the stress vector
        // TODO incorporate plastic part
        Eigen::Matrix3d newStress = 2.0 * particles->mus(i, 0) * (F - R) + particles->lambdas(i, 0) * (J - 1) * jFInvTranspose;
        particles->deformations[i]->stress = newStress;
    }
}

void Simulation::computeForces() {

    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        glm::vec3 baseNode = getBaseNode(particle);
        glm::vec3 currNode;
        // for each particle, calculate the nodes it affects and use its deformation to add to its grid node force
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                for(int k = 0; k < 3; k++) {
                    currNode = baseNode + glm::vec3(i, j, k);
                    KernelWeights* kernelWeight = particles->kernelWeights[p];
                    // weight for this node based on particle kernel
                    Eigen::Vector3d weightGrad(3);
                    // divide by cellsize?
                    weightGrad << (kernelWeight->N_deriv(0, 0) / grid->cellsize) * kernelWeight->N(0, 1) * kernelWeight->N(0, 2),
                            kernelWeight->N(1, 0) * (kernelWeight->N_deriv(1, 1)/ grid->cellsize) * kernelWeight->N(1, 2),
                            kernelWeight->N(2, 0) * kernelWeight->N(2, 1) * (kernelWeight->N_deriv(2, 2)/ grid->cellsize);

                    glm::vec3 force =  grid->getForces(currNode);
                    Eigen::Vector3d f;
                    f<< force[0], force[1], force[2];
                    float volume = particles->volumes(p, 0);
                    Eigen::Matrix3d s = particles->deformations[p]->stress;
                    Eigen::Matrix3d fT = particles->deformations[p]->F.transpose();
                    Eigen::Vector3d sumTerm = volume * s * fT * weightGrad;
                    f(0) = f(0) - sumTerm(0);
                    f(1) = f(1) - sumTerm(1);
                    f(2) = f(2) - sumTerm(2);
                    grid->setForces(currNode, glm::vec3(f(0), f(1), f(2)));
                }
            }
        } // i, j, k

    } // end for each particle
}

void Simulation::G2P() {
    // clear particle velocities;
     particles->velocities = Eigen::MatrixXd::Zero(numParticles, 3);

    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        std::vector<glm::vec3> neighborNodes = getNeighbors(particle);

        for(int i = 0; i < neighborNodes.size(); i++) {
            glm::vec3 currNode = neighborNodes[i];

            // weight for this node based on particle kernel
            float weight = getWeight(p, currNode);
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

void Simulation::updateGradient(float dt) {

    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        glm::vec3 baseNode = getBaseNode(particle);
        Eigen::MatrixXd gradV(3, 3);

        // for each particle, calculate the nodes it affects and add its contribution to that grid node's weight sum for velocity and momentum
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < 3; j++) {
                for(int k = 0; k < 3; k++) {
                    glm::vec3 currNode = baseNode + glm::vec3(i, j, k);
                    KernelWeights* kernelWeight = particles->kernelWeights[p];
                    // weight for this node based on particle kernel
                    Eigen::MatrixXd weightGrad(1, 3); // row vec
                    // divide by cellsize?
                    float h = grid->cellsize;
                    weightGrad << (kernelWeight->N_deriv(0, 0) / h) * kernelWeight->N(0, 1) * kernelWeight->N(0, 2),
                            kernelWeight->N(1, 0) *(kernelWeight->N_deriv(1, 1)/ h) * kernelWeight->N(1, 2),
                            kernelWeight->N(2, 0) * kernelWeight->N(2, 1) * (kernelWeight->N_deriv(2, 2)/ h);

                    glm::vec3 gridV = grid->getVelocity(currNode);
                    Eigen::MatrixXd v(3, 1); // column vec
                    v << gridV[0], gridV[1], gridV[2];

                    gradV = gradV + (v * weightGrad);
                }
            }
        }
        Eigen::MatrixXd res = (Eigen::Matrix3d::Identity(3, 3) + (dt * gradV));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (res(i, j) < 1e-7) res(i, j) = 0;
            }
        }
        Eigen::MatrixXd newF = res * particles->deformations[p]->F;
        particles->deformations[p]->F = newF;
    }
}

void Simulation::RunSimulation(QString output_filepath) {

    float dt = pow(10.0, -6);
    dt = .1;
    float cellSize = .1;
    // make new grid
    initializeGrid(cellSize);

    int i = numOutputFrames;
    while(i > 0) {
        fillKernelWeights();
        // transfer attributes to the grid
        P2G();

        // compute forces on grid
        // computeForces();
        // apply forces to grid
        //grid->applyForces(dt);
        // transfer attributes back to particles
        G2P();

        //update particle positions
        updateParticlePositions(dt);
        // update F
        // updateGradient(dt);

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
