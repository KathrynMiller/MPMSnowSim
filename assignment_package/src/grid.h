#pragma once
#include "la.h"
#include "eigen-git-mirror/Eigen/Core"


class Grid
{
public:
    Grid(glm::vec3 o, glm::vec3 d, float cellsize);
    // cellsize
    float cellsize = .2;
    // dimension of grid in 3d space
    glm::vec3 dim;
    // lower left corner of the grid
    glm::vec3 origin;


    int Dimension = 3;


    // getters and setters for attributes in the "3D" grid
    glm::vec3 getPosition(glm::vec3 pos) const;
    float getMass(glm::vec3 pos) const;
    glm::vec3 getMomentum(glm::vec3 pos) const;
    glm::vec3 getVelocity(glm::vec3 pos) const;
   // KernelWeights* getKernelWeight(glm::vec3 pos);

    void setPosition(glm::vec3 pos, glm::vec3 val);
    void setMass(glm::vec3 pos, float val);
    void setMomentum(glm::vec3 pos, glm::vec3 val);
    void setVelocity(glm::vec3 pos, glm::vec3 val);

    void clear();

private:
    // flattens 3d coordinates to become grid index
    int flat(int x, int y, int z) const;

    Eigen::MatrixXd gridPositions;
    Eigen::MatrixXd gridMasses;
    Eigen::MatrixXd gridVelocities;
    Eigen::MatrixXd gridMomentum;
//    std::vector<KernelWeights*> kernelWeights;
};



