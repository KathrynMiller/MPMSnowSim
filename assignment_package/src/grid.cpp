#include "grid.h"

Grid::Grid(glm::vec3 o, glm::vec3 d, float cellsize) : origin(o), dim(d), cellsize(cellsize)
{
    float matLength = d[0] * d[1] * d[2];
    gridPositions = Eigen::MatrixXd::Zero(matLength, 3);
    gridVelocities = Eigen::MatrixXd::Zero(matLength, 3);
    gridMomentum = Eigen::MatrixXd::Zero(matLength, 3);
    gridMasses = Eigen::MatrixXd::Zero(matLength, 1);
}

// returns a vec3 of row i of m
static glm::vec3 toVec3(Eigen::MatrixXd m, int r) {
    return glm::vec3(m(r, 0), m(r, 1), m(r, 2));
}

int Grid::flat(int x, int y, int z) const {
    return y * dim[0] + x + z * dim[0] * dim[1];
}

glm::vec3 Grid::getPosition(glm::vec3 pos) const {
    return toVec3(gridPositions, flat(pos[0], pos[1], pos[2]));
}

float Grid::getMass(glm::vec3 pos) const{
    return gridMasses(flat(pos[0], pos[1], pos[2]));
}

glm::vec3 Grid::getMomentum(glm::vec3 pos) const{
    return toVec3(gridMomentum, flat(pos[0], pos[1], pos[2]));
}

glm::vec3 Grid::getVelocity(glm::vec3 pos) const {
    return toVec3(gridVelocities, flat(pos[0], pos[1], pos[2]));
}

void Grid::setPosition(glm::vec3 pos, glm::vec3 val) {
   gridPositions(flat(pos[0], pos[1], pos[2]), 0) = val[0];
   gridPositions(flat(pos[0], pos[1], pos[2]), 1) = val[1];
   gridPositions(flat(pos[0], pos[1], pos[2]), 2) = val[2];
}

void Grid::setMass(glm::vec3 pos, float val) {
    gridMasses(flat(pos[0], pos[1], pos[2]), 0) = val;
}

void Grid::setMomentum(glm::vec3 pos, glm::vec3 val) {
    gridMomentum(flat(pos[0], pos[1], pos[2]), 0) = val[0];
    gridMomentum(flat(pos[0], pos[1], pos[2]), 1) = val[1];
    gridMomentum(flat(pos[0], pos[1], pos[2]), 2) = val[2];
}

void Grid::setVelocity(glm::vec3 pos, glm::vec3 val) {
    gridVelocities(flat(pos[0], pos[1], pos[2]), 0) = val[0];
    gridVelocities(flat(pos[0], pos[1], pos[2]), 1) = val[1];
    gridVelocities(flat(pos[0], pos[1], pos[2]), 2) = val[2];
}

void Grid::clear() {
    int numParticles = gridVelocities.rows();
    gridVelocities = Eigen::MatrixXd::Zero(numParticles, 3);
    gridMasses = Eigen::MatrixXd::Zero(numParticles, 1);
    gridVelocities = Eigen::MatrixXd::Zero(numParticles, 3);
    gridMomentum = Eigen::MatrixXd::Zero(numParticles, 3);
    //kernelWeights.clear();
}
