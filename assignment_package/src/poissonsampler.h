#pragma once
#include "objloader.h"
#include "sampler.h"


class Sample;

class PoissonSampler
{
public:
    PoissonSampler();

    void SampleMesh(QString &meshFileName);

    float rad = 0.3;
   // QStringRef meshFileName;
    ObjLoader* objLoader;
    Sampler sampler;
    std::vector<Sample*> activeValidSamples;
    std::vector<Sample*> validSamples;
};

class Sample {
public:
    Sample(glm::vec3 p, glm::vec3 gP);

    glm::vec3 pos;
    glm::vec3 gridPos;
};
