#include "poissonsampler.h"

PoissonSampler::PoissonSampler() : objLoader(new ObjLoader()), sampler(Sampler(5, 5))
{}

void PoissonSampler::SampleMesh(QString& meshFileName) {
    // loads obj and builds kdtree
    objLoader->LoadOBJ(meshFileName);

    int numSamples = 1000;
    for(int i = 0; i < numSamples; i++) {
        float x = sampler.Get1D() *
                (objLoader->root->maxCorner[0] - objLoader->root->minCorner[0]) -
                (objLoader->root->maxCorner[0] - objLoader->root->minCorner[0]) / 2.0;
        float y = sampler.Get1D() *
                (objLoader->root->maxCorner[1] - objLoader->root->minCorner[1]) -
                (objLoader->root->maxCorner[1] - objLoader->root->minCorner[1]) / 2.0;
        float z = sampler.Get1D() *
                (objLoader->root->maxCorner[2] - objLoader->root->minCorner[2]) -
                (objLoader->root->maxCorner[2] - objLoader->root->minCorner[2]) / 2.0;

//        x = 0;
//        y = .1;
//        z = 0;

        glm::vec3 point = glm::vec3(x, y, z);
        int* numIntersections;
        *numIntersections = 0;

        bool intersects1 = objLoader->Intersect(Ray(point, glm::vec3(1, 0, 0)), numIntersections);

        if(intersects1) {
          //  if(*numIntersections % 2 != 0)
            {
                validSamples.push_back(new Sample(point, glm::vec3()));
            }

        }
        activeValidSamples.push_back(new Sample(point, glm::vec3()));
        int l = 0;
    }
    int j = 1;
}

Sample::Sample(glm::vec3 p, glm::vec3 gP) : pos(p), gridPos(gP) {}


