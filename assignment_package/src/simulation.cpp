#include "simulation.h"
#include <algorithm>
#include "la.h"
#include "math.h"
#include "cmath"
#include "QFileDialog";
#include "eigen-git-mirror/Eigen/Dense"
#include "eigen-git-mirror/Eigen/SVD"
#include "eigen-git-mirror/Eigen/Core"
#include <Partio.h>

Simulation::Simulation(Particles* p, int numSeconds, int frameRate): particles(p), grid(nullptr)
{
    stepsPerFrame = ceil(1.0 / (dt / (1.0 / frameRate)));
    //numOutputFrames = numSeconds;
    numOutputFrames = frameRate * numSeconds;
    numParticles = p->numParticles;
    std::cout << "NUMBER OF FINAL FRAMES: " + numOutputFrames;
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

    // make bigger to see more movement
    minCorner -= glm::vec3(cellsize * 7.0);
    maxCorner += glm::vec3(cellsize * 7.0);
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

    //        if (absX < 1.0f) {
    //            return 0.5f * pow(absX, 3.0f) - pow(x, 2.0f) + (2.0f / 3.0f);
    //        }
    //        else if (absX < 2.0f) {
    //            return (-1.0f / 6.0f) * pow(absX, 3.0f) + pow(x, 2.0f) -2.0f * absX + (4.0f / 3.0f);
    //        }
    //        else {
    //            return 0.f;
    //        }
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
    //        if (absX < 1.0f) {
    //            return 0.5f * x * ((3.0f * absX) - 4.0f);
    //        }
    //        else if (absX < 2.0f) {
    //            return (-x * pow(absX - 2.0f, 2.0f)) / (2.0f * absX);
    //        }
    //        else {
    //            return 0.f;
    //        }

}

// returns the local base node of particle p in grid coordinates
glm::vec3 Simulation::getBaseNode(glm::vec3 particle) {
    glm::vec3 baseNode = (particle - grid->origin - glm::vec3(grid->cellsize * 0.5)) / grid->cellsize;
    baseNode = glm::vec3(floor(baseNode[0]), floor(baseNode[1]), floor(baseNode[2]));
    return baseNode;
}

Eigen::Vector3d Simulation::getWeightGradient(KernelWeights* kernelWeight, glm::vec3 particle, glm::vec3 node) {
    glm::vec3 baseNode = getBaseNode(particle);
    glm::vec3 offset = node - baseNode;
    Eigen::Vector3d weightGrad;
    weightGrad << (kernelWeight->N_deriv(0, int(offset[0]))) * kernelWeight->N(1, int(offset[1])) * kernelWeight->N(2, int(offset[2])),
            kernelWeight->N(0, int(offset[0])) * (kernelWeight->N_deriv(1, int(offset[1]))) * kernelWeight->N(2, int(offset[2])),
            kernelWeight->N(0, int(offset[0])) * kernelWeight->N(1, int(offset[1])) * (kernelWeight->N_deriv(2, int(offset[2])));
    return weightGrad / grid->cellsize;
}

float Simulation::getWeight(KernelWeights *kernelWeight, glm::vec3 particle, glm::vec3 node) {
    glm::vec3 baseNode = getBaseNode(particle);
    glm::vec3 offset = node - baseNode;

    float weight = kernelWeight->N(0, int(offset[0])) * kernelWeight->N(1, int(offset[1])) * kernelWeight->N(2, int(offset[2]));
    return weight;
}

std::vector<glm::vec3> Simulation::getNeighbors(glm::vec3 particle) {
    glm::vec3 baseNode = getBaseNode(particle);
    std::vector<glm::vec3> neighbors = std::vector<glm::vec3>();
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            for(int k = 0; k < 3; k++) {
                glm::vec3 currNode = baseNode + glm::vec3(i, j, k);
                neighbors.push_back(currNode);
            }
        }
    }
    return neighbors;
}
/*
static Eigen::Vector3f calcWeights(float index_space, int& baseNode)
{
    baseNode = std::floor(index_space-0.5);

    Eigen::Vector3f output;

    float d0 = index_space - (float)baseNode; // 0.5<d0<1.5
    float z = 1.5f - d0;
    output[0] = 0.5 * z * z;

    float d1 = d0 - 1;          // -0.5<d1<0.5
    output[1] = 0.75 - d1 * d1;

    float d2 = 1 - d1;          // 0.5<d2<1.5
    float zz = 1.5f - d2;
    output[2] = 0.5 * zz * zz;

    return output;
}
static Eigen::Vector3f calcGradWeights(float index_space, int baseNode)
{
    Eigen::Vector3f graInt;

    float d0 = index_space - (float)baseNode;
    float z = 1.5f - d0;

    float d1 = d0 - 1;

    float d2 = 1 - d1;
    float zz = 1.5f - d2;

    graInt[0] = -z;
    graInt[1] = -2*d1;
    graInt[2] = zz;

    return graInt;
}
static void QuadraticInterpolation(Eigen::Vector3f particlePos, Eigen::Vector3i& baseNode, Eigen::Matrix3d& wp, Eigen::Matrix3d& dwp)
{

    Eigen::Vector3f interX = calcWeights(particlePos[0], baseNode[0]);
    Eigen::Vector3f interY = calcWeights(particlePos[1], baseNode[1]);
    Eigen::Vector3f interZ = calcWeights(particlePos[2], baseNode[2]);

    // calculate weight matrix
    wp(0,0)= interX[0]; wp(0,1) = interX[1]; wp(0,2)=interX[2];
    wp(1,0)= interY[0]; wp(1,1) = interY[1]; wp(1,2)=interY[2];
    wp(2,0)= interZ[0]; wp(2,1) = interZ[1]; wp(2,2)=interZ[2];

    Eigen::Vector3f graIntX = calcGradWeights(particlePos[0], baseNode[0]);
    Eigen::Vector3f graIntY = calcGradWeights(particlePos[1], baseNode[1]);
    Eigen::Vector3f graIntZ = calcGradWeights(particlePos[2], baseNode[2]);

    // calculate gradient weight matrix
    dwp(0,0)= graIntX[0]; dwp(0,1) = graIntX[1]; dwp(0,2)=graIntX[2];
    dwp(1,0)= graIntY[0]; dwp(1,1) = graIntY[1]; dwp(1,2)=graIntY[2];
    dwp(2,0)= graIntZ[0]; dwp(2,1) = graIntZ[1]; dwp(2,2)=graIntZ[2];
}
*/

void Simulation::fillKernelWeights() {

    // for each particle
    for(int p = 0; p < particles->positions.rows(); p++) {
        glm::vec3 particle = toVec3(particles->positions, p); // world space pos
        glm::vec3 gridOffset = (particle - grid->origin) / grid->cellsize; // pos on grid in cellsize-units
        glm::vec3 baseNodeOffset = gridOffset - glm::vec3(floor(gridOffset[0] - 0.5), floor(gridOffset[1] - 0.5),
                floor(gridOffset[2] - 0.5));
        // kernel for particle p
        KernelWeights* kernelWeight = particles->kernelWeights[p];
        kernelWeight->N = Eigen::Matrix3d::Zero(3, 3);
        kernelWeight->N_deriv = Eigen::Matrix3d::Zero(3, 3);

        for(int i = 0; i < 3; i++) { // rows of N and N'
            for(int j = 0; j < 3; j++) { // columns of N and N'

                float x = baseNodeOffset[i] - j;
                // assert(x <= 2);
                kernelWeight->N(i, j) = funcN(x);
                kernelWeight->N_deriv(i, j) = gradN(x);
            }
        }

        // sum to test for valid weights
        double wSum = 0.0;
        Eigen::Vector3d gradWSum = Eigen::Vector3d::Zero();
        std::vector<glm::vec3> neighbors = getNeighbors(particle);
        for(glm::vec3 n: neighbors) {
            wSum += getWeight(particles->kernelWeights[p], particle, n);
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

    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        std::vector<glm::vec3> neighborNodes = getNeighbors(particle);

        // for each particle, calculate the nodes it affects
        // and add its contribution to that grid node's weight sum for velocity and momentum
        for(int i = 0; i < neighborNodes.size(); i++) {
            glm::vec3 currNode = neighborNodes[i];
            float weight = getWeight(particles->kernelWeights[p], particle, currNode);

            // extract particle mass and velocity from eigen matrix
            float pMass = particles->masses(p);
            glm::vec3 pVelocity = glm::vec3(particles->velocities(p, 0), particles->velocities(p, 1), particles->velocities(p, 2));

            // grid attributes += weight * particle attributes
            float gridMass = grid->getMass(currNode) + (weight * pMass);
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

        Eigen::Matrix3f tempSigma = Eigen::Matrix3f::Zero();
        tempSigma(0,0) = Sigma(0);
        tempSigma(1,1) = Sigma(1);
        tempSigma(2,2) = Sigma(2);

        //sorting
        if(U.determinant() < 0)
        {
            U(0,2) *= -1;
            U(1,2) *= -1;
            U(2,2) *= -1;
            tempSigma(2,2) *= -1;
        }

        if(V.determinant() < 0)
        {
            V(0,2) *= -1;
            V(1,2) *= -1;
            V(2,2) *= -1;
            tempSigma(2,2) *= -1;
        }

        if(tempSigma(0,0) < tempSigma(1,1))
        {
            float tempRecord = tempSigma(0,0);
            tempSigma(0,0) = tempSigma(1,1);
            tempSigma(1,1) = tempRecord;
        }

        /*
        Eigen::RowVector3d temp = Eigen::RowVector3d::Zero(1, 3);

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
        // use it to calculate the stress and put this in the particle's stress matrix
        // TODO incorporate plastic part
        Eigen::Matrix3d newStress = (2.0 * particles->mus(i, 0) * (F - R)) + (particles->lambdas(i, 0) * (J - 1) * J * jFInvTranspose);
        particles->deformations[i]->stress = newStress;

        //        if(newStress != Eigen::Matrix3d::Zero(3, 3)) {
        //            std::cout << "FRAME: " << frameNumber << "\n";
        //            std::cout << "STRESS: " << newStress << "\n";
        //        }
    }

}

void Simulation::computeForces() {
    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        std::vector<glm::vec3> neighborNodes = getNeighbors(particle);

        float volume = particles->volumes(p, 0);
        Eigen::Matrix3d s = particles->deformations[p]->stress;
        Eigen::Matrix3d fT = particles->deformations[p]->F.transpose();
        Eigen::Matrix3d m = volume * s * fT;
        // for each particle, calculate the nodes it affects and use its deformation to add to its grid node force
        for(int i = 0; i < neighborNodes.size(); i++) {

            glm::vec3 currNode = neighborNodes[i];
            Eigen::Vector3d weightGrad = getWeightGradient(particles->kernelWeights[p], particle, currNode);

            glm::vec3 gridForce =  grid->getForces(currNode);

            Eigen::Vector3d sumTerm = m * weightGrad;
            gridForce[0] -= sumTerm(0);
            gridForce[1] -= sumTerm(1);
            gridForce[2] -= sumTerm(2);
            grid->setForces(currNode, gridForce);
            // std::cout << "GRID FORCE: (" << gridForce[0] << ", " << gridForce[1] << ", " << gridForce[2]  << ") \n \n";
        } // end for neighbor nodes

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
            float weight = getWeight(particles->kernelWeights[p], particle, currNode);

            glm::vec3 gridVelocity = grid->getVelocity(currNode);
            glm::vec3 newVel = glm::vec3(particles->velocities(p, 0) + weight * gridVelocity[0],
                    particles->velocities(p, 1) + weight * gridVelocity[1],
                    particles->velocities(p, 2) + weight * gridVelocity[2]);
            particles->velocities(p, 0) = newVel[0];
            particles->velocities(p, 1) = newVel[1];
            particles->velocities(p, 2) = newVel[2];

            // std::max not working?
            //            if(maxParticleVelocity < newVel.length()) {
            //                maxParticleVelocity = newVel.length();
            //            }
        }

    }
}

void Simulation::updateParticlePositions(float dt) {
    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 newPos = glm::vec3(particles->positions(p, 0) + particles->velocities(p, 0) * dt,
                                     particles->positions(p, 1) + particles->velocities(p, 1) * dt,
                                     particles->positions(p, 2) + particles->velocities(p, 2) * dt);
        particles->positions(p, 0) = newPos[0];
        particles->positions(p, 1) = newPos[1];
        particles->positions(p, 2) = newPos[2];

        /*
        // need to clamp positions to box
        glm::vec3 origMin = grid->origin + glm::vec3(2.0 * grid->cellsize);
        glm::vec3 origMax = grid->origin + (grid->dim * grid->cellsize) - glm::vec3(2.0 * grid->cellsize);
        for(int i = 0; i < 3; i++) {
            if(particles->positions(p, i) < origMin[i]) {
                particles->positions(p, i) = origMin[i];
            }
            if(particles->positions(p, i) > origMax[i]) {
                particles->positions(p, i) = origMax[i];
            }
        }
*/
    }
}

void Simulation::updateGradient(float dt) {

    for(int p = 0; p < particles->numParticles; p++) {

        glm::vec3 particle = toVec3(particles->positions, p);
        std::vector<glm::vec3> neighborNodes = getNeighbors(particle);
        Eigen::Matrix3d gradVp = Eigen::Matrix3d::Zero(3, 3);

        for(int i = 0; i < neighborNodes.size(); i++) {
            glm::vec3 currNode = neighborNodes[i];
            Eigen::Vector3d dwip = (getWeightGradient(particles->kernelWeights[p], particle, currNode));
            glm::vec3 gridVel = grid->getVelocity(currNode);

            Eigen::Matrix3d test;

            test(0,0) = gridVel[0] * dwip(0);
            test(0,1) = gridVel[0] * dwip(1);
            test(0,2) = gridVel[0] * dwip(2);
            test(1,0) = gridVel[1] * dwip(0);
            test(1,1) = gridVel[1] * dwip(1);
            test(1,2) = gridVel[1] * dwip(2);
            test(2,0) = gridVel[2] * dwip(0);
            test(2,1) = gridVel[2] * dwip(1);
            test(2,2) = gridVel[2] * dwip(2);

            gradVp += test;
        }
        Eigen::Matrix3d newF = dt * gradVp * particles->deformations[p]->F;
        particles->deformations[p]->F = particles->deformations[p]->F + newF;
    }
}

void Simulation::RunSimulation(QString output_filepath, GLWidget277* mygl) {

    // dt <= cmax (.2 - .4) * h / vmax

    float cellSize = .12;
    initializeGrid(cellSize);

    int i = numOutputFrames;

    while(i > 0) {
        // iterate step steps per frame
        int step = stepsPerFrame;
        while(step > 0) {
            grid->clear();

            fillKernelWeights();
            // transfer attributes to the grid
            P2G();
            // compute Piola Kirchoff stress per particle
            computeStress();
            // compute forces on grid using stress
            computeForces();
            // apply forces to grid velocity
            grid->applyForces(dt);
            grid->handleCollisions();
            // transfer attributes back to particles
            G2P();
            // update F
            updateGradient(dt);
            //update particle positions
            updateParticlePositions(dt);


            step--;
        }
        saveToBgeo(output_filepath);
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


void Simulation::saveToBgeo(QString output_filepath) {
    if(output_filepath.length() == 0)
    {
        return;
    }
    QFile file(output_filepath + "/frame_" + QString::number(frameNumber) + ".geo");
    if (file.open(QIODevice::ReadWrite)) {
        //QDataStream stream(&file);
        QTextStream stream(&file);

        // header
        stream << "[\"fileversion\",\"16.5.323\",\"hasindex\",false,\"pointcount\"," << QString::number(numParticles) <<
                  ",\"vertexcount\",0,\"primitivecount\",0,\"info\",{\"software\":\"Houdini 16.5.323\"," <<
                  "\"date\":\"2018-11-20 13:33:05\",\"hostname\":\"hnt-ve509-0576.apn.wlan.upenn.edu\"," <<
                  "\"artist\":\"kathrynmiller\",\"bounds\":[-0.488582999,0.495624006,-0.492269009,0.492332995,-0.475816011,0.424921989]," <<
                  "\"attribute_summary\":\"     1 point attributes:\\tP\\n\"},\"topology\",[\"pointref\",[\"indices\",[]]]," <<
                  "\"attributes\",[\"pointattributes\",[[[\"scope\",\"public\",\"type\",\"numeric\",\"name\",\"P\"," <<
                  "\"options\",{\"type\":{\"type\":\"string\",\"value\":\"point\"}}],[\"size\",3,\"storage\",\"fpreal32\"," <<
                  "\"defaults\",[\"size\",1,\"storage\",\"fpreal64\",\"values\",[0]],\"values\",[\"size\",3," <<
                  "\"storage\",\"fpreal32\",\"tuples\",[";

        for(int i = 0; i < particles->positions.rows(); i++) {
            stream << "[" << QString::number(particles->positions(i, 0)) << ","
                   << QString::number(particles->positions(i, 1)) << ","
                   << QString::number(particles->positions(i, 2)) << "]";
            if(i != particles->positions.rows()  - 1) {
                stream << ",";
            }
        }

        stream << "]]]]]],\"primitives\",[]]";

    }
}



