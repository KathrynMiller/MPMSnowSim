#pragma once
#include "objloader.h"
#include "sampler.h"


class Sample;

class PoissonSampler
{
public:
    PoissonSampler();

    // follows poisson algorithm for sampling points
    void SampleMesh(QString &meshFileName);
    // returns individual point within objects bounding box
    glm::vec3 sampleNewPoint();
    // returns a sample within radius r of center
    glm::vec3 sampleInSphere(glm::vec3 center);
    // helper for determining whether or not point is within the mesh
    bool isWithinObj(glm::vec3 point);
    // helper for determining whether or not point is within the objects bounding box
    bool isWithinBounds(glm::vec3 point);
    // takes a point in 3d and flattens it to the base grid
    glm::vec3 posOnGrid(glm::vec3 pos);
    // set up dimension variables after obj load
    void initializeVars();


    // radius for poisson sampling
    float radius = 0.1;
    // number of samples to sample around a point (density)
    float K = 25;
   // bool threeDim;
    glm::vec3 voxelDim; // number of voxels across each dimension for the grid
    float voxelSize; //actual size of the voxels [cubes so all 3 same val]


    // holds a kdtree of the mesh and has intersection functions for testing if point is within a mesh
    ObjLoader* objLoader;
    // generates random numbers for sampling
    Sampler sampler;
    // samples that are still being used in sampling and may eventually be added to validSamples
    std::vector<Sample*> activeValidSamples;
    // list of points that are finalized and will be added to the set of particles
    std::vector<Sample*> validSamples;
};

class Sample {
public:
    Sample(glm::vec3 p, glm::vec3 gP) : pos(p), gridPos(gP) {}
    Sample(Sample* s) : gridPos(s->gridPos), pos(s->pos) {}
    glm::vec3 pos;
    glm::vec3 gridPos;
};
