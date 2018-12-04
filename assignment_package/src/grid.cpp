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
                glm::vec3 worldPos = (glm::vec3(i, j, k) * cellsize) + origin;
                if(collider->isInside(worldPos)) {
                    int index = flat(i, j, k);
                    if(sticky) {
                        gridVelocities(index, 0) = 0.0;
                        gridVelocities(index, 1) = 0.0;
                        gridVelocities(index, 2) = 0.0;
                    } else {
//                        glm::vec3 n = collider->getNormal(worldPos);
//                        float dot = glm::dot(n, toVec3(gridVelocities, index));
//                        if (dot < 0){
//                            gridVelocities(index) = gridVelocities(index) - dot * n;
//                            if (friction != 0){
//                                if (-dot * friction < gridAttr[index].velG.norm())
//                                    gridAttr[index].velG += dot * friction * gridAttr[index].velG.normalized();
//                                else
//                                    gridAttr[index].velG = Vector3f::Zero();
//                            }
//                        }
                        glm::vec3 n = collider->getNormal(worldPos);
                        glm::vec3 v = toVec3(gridVelocities, index);
                        glm::vec3 vCo = glm::vec3(); // velocity of collider, static for now
                        glm::vec3 vRel = v - vCo;

                        float vN = glm::dot(vRel, n);
                        if(vN >= 0) { // separating, no collision
                            return;
                        }

                        glm::vec3 vRelPrime;
                        glm::vec3 vPrime;
                        glm::vec3 vT = vRel - (n * vN);
                        if(vT.length() <= (-friction * vN)) {
                            vRelPrime = glm::vec3();
                        } else{
                            vRelPrime = vT + (friction * (vN * (glm::normalize(vT)))); // friction 0 for now so doesn't matter
                        }

                        vPrime = vRelPrime + vCo;

                        gridVelocities(index, 0) = vPrime[0];
                        gridVelocities(index, 1) = vPrime[1];
                        gridVelocities(index, 2) = vPrime[2];
                    }
                }
            }
        }
    }
}

void Grid::handleBorderCollisions() {
    /*
    float coFric = 0.0f; // mu
    // n is straight up for plane
    glm::vec3 n = glm::vec3(0.0, 1.0, 0.0);
    //TODO make collision object class to be passed in for arbitrary implicit surface collision
    for(int i = 0; i < gridVelocities.rows(); i++) {
        glm::vec3 v = toVec3(gridVelocities, i);
        glm::vec3 vCo = glm::vec3(); // velocity of collider, static for now
        glm::vec3 vRel = v - vCo;

        float vN = glm::dot(vRel, n);
        if(vN >= 0) { // separating, no collision
            return;
        }

        glm::vec3 vRelPrime;
        glm::vec3 vPrime;
        glm::vec3 vT = vRel - (n * vN);
        if(vT.length() <= (-coFric * vN)) {
            vRelPrime = glm::vec3();
        } else{
            vRelPrime = vT + (coFric * (vN * (glm::normalize(vT)))); // friction 0 for now so doesn't matter
        }

        vPrime = vRelPrime + vCo;

        gridVelocities(i, 0) = vPrime[0];
        gridVelocities(i, 1) = vPrime[1];
        gridVelocities(i, 2) = vPrime[2];
        */

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
    // boudanry velocity for y-direction wall
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
    // boudanry velocity for y-direction wall
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
