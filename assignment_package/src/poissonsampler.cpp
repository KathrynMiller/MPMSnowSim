#include "poissonsampler.h"

PoissonSampler::PoissonSampler() : objLoader(new ObjLoader()), sampler(Sampler(5, 5))
{}

bool PoissonSampler::isWithinObj(glm::vec3 point) {
    int numIntersections1 = 0;
    int numIntersections2 = 0;
    int numIntersections3 = 0;


    bool intersects1 = objLoader->Intersect(Ray(point, glm::vec3(1, 0, 0)), &numIntersections1);
    bool intersects2 = objLoader->Intersect(Ray(point, glm::vec3(0, 1, 0)), &numIntersections2);
    bool intersects3 = objLoader->Intersect(Ray(point, glm::vec3(0, 0, 1)), &numIntersections3);

    if(intersects1 || intersects2 || intersects3) {
        if(numIntersections1 % 2 != 0 || numIntersections2 % 2 != 0  || numIntersections3 % 2 != 0)
        {
            return true;
        }
    }
    return false;
}

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

//                x = 0;
//                y = .1;
//                z = 0;

        glm::vec3 point = glm::vec3(x, y, z);


        if(isWithinObj(point)) {
            validSamples.push_back(new Sample(point, glm::vec3()));
        }
    }
    int j = 1;
}

Sample::Sample(glm::vec3 p, glm::vec3 gP) : pos(p), gridPos(gP) {}


