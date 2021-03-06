#include "poissonsampler.h"
#include "warpfunctions.h"
#include "la.h"
#include "iostream"

PoissonSampler::PoissonSampler() : objLoader(new ObjLoader()), sampler(Sampler(5, 5))
{}

void PoissonSampler::initializeVars() {

    voxelSize = radius / sqrt(3.0);

    glm::vec3 maxP = objLoader->maxCorner;
    glm::vec3 minP = objLoader->minCorner;

    this->voxelDim = glm::vec3(glm::ceil((maxP[0] - minP[0])/voxelSize),
            glm::ceil((maxP[1] - minP[1])/voxelSize),
            glm::ceil((maxP[2] - minP[2])/voxelSize) );
}

bool PoissonSampler::isWithinObj(glm::vec3 point) {
    int numIntersections1 = 0;
    int numIntersections2 = 0;
    int numIntersections3 = 0;

    glm::vec3 axis1 = glm::vec3(0, 1, 0);
    glm::vec3 axis2 = glm::vec3(1, 0, 0);
    glm::vec3 axis3 = glm::cross(axis1, axis2);
    bool intersects1 = objLoader->Intersect(Ray(point, axis1), &numIntersections1);
    bool intersects2 = objLoader->Intersect(Ray(point, axis2), &numIntersections2);
    bool intersects3 = objLoader->Intersect(Ray(point, axis3), &numIntersections3);
    bool odd1 = numIntersections1 % 2 != 0;
    bool odd2 = numIntersections2 % 2 != 0;
    bool odd3 = numIntersections3 % 2 != 0;

    if(intersects1 || intersects2 || intersects3) {
        if(odd1 && odd2 && odd3)
        {
            return true;
        }
    }
    return false;

}

bool PoissonSampler::isWithinBounds(glm::vec3 point) {
    if(point[0] >= objLoader->minCorner[0] && point[0] <= objLoader->maxCorner[0]) {
        if(point[1] >= objLoader->minCorner[1] && point[1] <= objLoader->maxCorner[1]) {
            if(point[2] >= objLoader->minCorner[2] && point[2] <= objLoader->maxCorner[2]) {
                return true;
            }
        }
    }
    return false;
}

glm::vec3 PoissonSampler::sampleNewPoint() {
    float x = sampler.Get1D() *
            (objLoader->root->maxCorner[0] - objLoader->root->minCorner[0]) -
            (objLoader->root->maxCorner[0] - objLoader->root->minCorner[0]) / 2.0;
    float y = sampler.Get1D() *
            (objLoader->root->maxCorner[1] - objLoader->root->minCorner[1]) -
            (objLoader->root->maxCorner[1] - objLoader->root->minCorner[1]) / 2.0;
    float z = sampler.Get1D() *
            (objLoader->root->maxCorner[2] - objLoader->root->minCorner[2]) -
            (objLoader->root->maxCorner[2] - objLoader->root->minCorner[2]) / 2.0;

    return glm::vec3(x, y, z);
}

glm::vec3 PoissonSampler::sampleInSphere(glm::vec3 center) {

    float val = 4.0f;
    float x = (sampler.Get2D().x * val * radius) - val/2 * radius + center[0];
    float y = (sampler.Get2D().x * val * radius) - val/2 * radius + center[1];
    float z = (sampler.Get2D().x * val * radius) - val/2 * radius + center[2];
    return glm::vec3(x, y, z);

}

glm::vec3 PoissonSampler::posOnGrid(glm::vec3 pos)
{
    glm::vec3 min = objLoader->minCorner;

    // change to voxel?
    int x = (int)(glm::clamp(((pos[0] - min[0])/voxelSize), 0.0f, voxelDim[0] - 1.0f));
    int y = (int)(glm::clamp(((pos[1] - min[1])/voxelSize), 0.0f, voxelDim[1] - 1.0f));
    int z = (int)(glm::clamp(((pos[2] - min[2])/voxelSize), 0.0f, voxelDim[2] - 1.0f));

    return glm::vec3(x, y, z);
}

void PoissonSampler::SampleMesh(QString& meshFileName) {
    // loads obj and builds kdtree
    objLoader->LoadOBJ(meshFileName);
    initializeVars();

    std::vector<Sample*> computedSampled = std::vector<Sample*>();

    std::vector<std::vector<std::vector<Sample*>>> backgroundGrid = std::vector<std::vector<std::vector<Sample*>>>(voxelDim[0],
            std::vector<std::vector<Sample*>>(voxelDim[1],
            std::vector<Sample*>(voxelDim[2], nullptr)));

    glm::vec3 firstPos = objLoader->faces.first()->points[1];
    glm::vec3 firstGridPos = posOnGrid(firstPos);
    Sample* start = new Sample(firstPos, firstGridPos);
    activeValidSamples.push_back(start);
    //validSamples.push_back(start);
    backgroundGrid[firstGridPos[0]][firstGridPos[1]][firstGridPos[2]] = start;

    while(activeValidSamples.size() > 0) {

        Sample* x_i = activeValidSamples[(int)(sampler.Get2D().x * activeValidSamples.size())];
        bool addedK = false;
        for (int i = 0; i < K; i++) {
            glm::vec3 pos = sampleInSphere(x_i->pos);

            //note must make sure ^^ provides valid position that will be within the current grid area so sampling must check that first
            glm::vec3 gLoc = posOnGrid(pos);

            // check surrounding voxels to see if point is valid
            glm::vec3 checkingMin = glm::vec3(fmax(x_i->gridPos[0] - 1.0, 0.0),
                    fmax(x_i->gridPos[1] - 1.0, 0.0), fmax(x_i->gridPos[2] - 1.0, 0.0));
            glm::vec3 checkingMax = glm::vec3(fmin(x_i->gridPos[0] + 1.0, voxelDim[0] - 1.0),
                    fmin(x_i->gridPos[1] + 1.0, voxelDim[1] - 1.0), fmin(x_i->gridPos[2] + 1.0, voxelDim[2] - 1.0));

            // check if sampled point in sphere is valid
            // i.e. it is not the same as our current random point and it is in the correct radius range
            bool valid = true;
            for (int j = checkingMin[0]; j <= checkingMax[0]; j++) {
                for (int k = checkingMin[1]; k <= checkingMax[1]; k++) {
                    for(int l = checkingMin[2]; l <= checkingMax[2]; l++) {

                        if (backgroundGrid[j][k][l] != nullptr) {

                            if (j == gLoc[0] && k == gLoc[1] && l == gLoc[2]) {
                                valid = false; // not valid bc same location as k particle;
                            }
                            glm::vec3 currGridCheck = backgroundGrid[j][k][l]->pos;
                            // check if within range of R - 2R from this grid nodes particle
                            float dist = glm::distance(pos, currGridCheck);
                            valid &= (dist >= radius && dist <= (2.0 * radius));
                        }
                    }
                }
            }

            // if the point is valid on the grid and is within the bounding box of the obj
            if (valid/* && isWithinBounds(pos)*/) {
                // valid then create and add to grid
                Sample* kPoint = new Sample(pos, gLoc);

                activeValidSamples.push_back(kPoint);
                backgroundGrid[gLoc[0]][gLoc[1]][gLoc[2]] = kPoint;

                addedK = true;
            }

        } // end adding K points loop

        if (!addedK) {

            //none of x_i's newly created samples were added so x_i no longer a active sample - add to list of final samples
            computedSampled.push_back(new Sample(x_i));
            activeValidSamples.erase(std::remove(activeValidSamples.begin(), activeValidSamples.end(), x_i),
                                     activeValidSamples.end());

        }

    } //end: while(activeValidSamples.size() > 0)

    for (Sample* s : computedSampled) {
        if (isWithinObj(s->pos)) {// change back to is within obj
            validSamples.push_back(s);
        }
    }

}


