#include "grid.h"

Grid::Grid(glm::vec3 o, glm::vec3 d, float cellsize) : origin(o), dim(d), cellsize(cellsize)
{
    float matLength = d[0] * d[1] * d[2];
    gridPositions = Eigen::MatrixXd::Zero(matLength, 3);
    gridForces = Eigen::MatrixXd::Zero(matLength, 3);
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

glm::vec3 Grid::getForces(glm::vec3 pos) const {
    return toVec3(gridForces, flat(pos[0], pos[1], pos[2]));
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
    int row = flat(pos[0], pos[1], pos[2]);
    gridVelocities(row, 0) = val[0];
    gridVelocities(row, 1) = val[1];
    gridVelocities(row, 2) = val[2];
}

void Grid::setForces(glm::vec3 pos, glm::vec3 val) {
    gridForces(flat(pos[0], pos[1], pos[2]), 0) = val[0];
    gridForces(flat(pos[0], pos[1], pos[2]), 1) = val[1];
    gridForces(flat(pos[0], pos[1], pos[2]), 2) = val[2];
}

void Grid::clear() {
    int matLength = dim[0] * dim[1] * dim[2];
    gridVelocities = Eigen::MatrixXd::Zero(matLength, 3);
    gridPositions = Eigen::MatrixXd::Zero(matLength, 3);
    gridMasses = Eigen::MatrixXd::Zero(matLength, 1);
    gridMomentum = Eigen::MatrixXd::Zero(matLength, 3);
    gridForces = Eigen::MatrixXd::Zero(matLength, 3);
}

void Grid::handleGridCollisions(Collider* collider) {
    bool sticky = true;
    float friction = 0.0f; // mu
    // for all grid node positions
    for(int i = 0; i < dim[0]; i++) {
        for(int j = 0; j < dim[1]; j++) {
            for(int k = 0; k < dim[2]; k++) {
                int index = flat(i, j, k);
                glm::vec3 worldPos = (glm::vec3(i, j, k) * cellsize) + origin;
                glm::vec3 gridVel = toVec3(gridVelocities, index);

                if(collider->isInside(worldPos)) {

                    if(sticky) {
                        gridVelocities(index, 0) = 0.0;
                        gridVelocities(index, 1) = 0.0;
                        gridVelocities(index, 2) = 0.0;
                    } else {
                        glm::vec3 n = collider->getNormal(worldPos);
                        float dot = glm::dot(n, toVec3(gridVelocities, index));
                        if (dot < 0) {
                            gridVelocities(index, 0) = gridVel[0] - dot * n[0];
                            gridVelocities(index, 1) = gridVel[1] - dot * n[1];
                            gridVelocities(index, 2) = gridVel[2] - dot * n[2];
                            if (friction != 0){
                                if (-dot * friction < gridVel.length()) {
                                    glm::vec3 newVel = gridVel + dot * friction * glm::normalize(gridVel);
                                    gridVelocities(index, 0) = newVel[0];
                                    gridVelocities(index, 1) = newVel[1];
                                    gridVelocities(index, 2) = newVel[2];
                                }
                                else {
                                    gridVelocities(index, 0) = 0.0;
                                    gridVelocities(index, 1) = 0.0;
                                    gridVelocities(index, 2) = 0.0;
                                }

                            }
                        }
                    }
                }
            }
        }
    }
}

void Grid::handleBorderCollisions() {

    int thickness = 4;
    int index = 0;
    int X = dim[0];
    int Y = dim[1];
    int Z = dim[2];
    // boundary velocity for x-direction wall
    for (int i = 0; i < thickness; i++){
        for (int j = 0; j < Y; j++){
            for (int k = 0; k < Z; k++){
                index = flat(i, j, k);
                if (gridVelocities(index, 0) < 0){
                    gridVelocities(index, 0) = 0;
                }
            }
        }
    }
    for (int i = X-thickness; i < X; i++){
        for (int j = 0; j < Y; j++){
            for (int k = 0; k < Z; k++){
                index = flat(i, j, k);
                if (gridVelocities(index, 0) > 0){
                    gridVelocities(index, 0) = 0;
                }
            }
        }
    }
    // boundary velocity for y-direction wall
    for (int j = 0; j<thickness; j++){
        for (int i = 0; i < X; i++){
            for (int k = 0; k < Z; k++){
                index = flat(i, j, k);
                if (gridVelocities(index, 1) < 0){
                    gridVelocities(index, 1) = 0;
                }
            }
        }
    }
    for (int j = Y-thickness; j<Y; j++){
        for (int i = 0; i < X; i++){
            for (int k = 0; k < Z; k++){
                index = flat(i, j, k);
                if (gridVelocities(index, 1) > 0){
                    gridVelocities(index, 1) = 0;
                }
            }
        }
    }
    // boundary velocity for y-direction wall
    for (int k = 0; k<thickness; k++){
        for (int i = 0; i < X; i++){
            for (int j = 0; j < Y; j++){
                index = flat(i, j, k);
                if (gridVelocities(index, 1) < 0){
                    gridVelocities(index, 2) = 0;
                }
            }
        }
    }
    for (int k = Z-thickness; k<Z; k++){
        for (int i = 0; i < X; i++){
            for (int j = 0; j < Y; j++){
                index = flat(i, j, k);
                if (gridVelocities(index, 1) > 0){
                    gridVelocities(index, 1) = 0;
                }
            }
        }
    }
}

void Grid::applyForces(float dt) {

    float gravity = 3;

    // use new forces to update velocity
    for(int i = 0; i < gridVelocities.rows(); i++) {
        if(gridMasses(i, 0) > 0) {

            gridForces(i, 1) = gridForces(i, 1) - gravity * gridMasses(i, 0);

            gridVelocities(i, 0) = gridVelocities(i, 0) + (dt * gridForces(i, 0)) / gridMasses(i, 0);
            gridVelocities(i, 1) = gridVelocities(i, 1) + (dt * gridForces(i, 1)) / gridMasses(i, 0);
            gridVelocities(i, 2) = gridVelocities(i, 2) + (dt * gridForces(i, 2)) / gridMasses(i, 0);
        }
    }
}
