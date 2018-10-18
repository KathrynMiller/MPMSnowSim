#pragma once

#include <scene/transform.h>
#include <raytracing/ray.h>

//A Cube is assumed to have a radius of 1 and a center of <0,0,0>.
//These attributes can be altered by applying a transformation matrix to the Cube.
class Cube
{
public:
    Cube();

    Transform transform;

    bool Intersect(const Ray &ray) const;

    void ComputeTBN(const glm::vec3& P, glm::vec3* nor, glm::vec3* tan, glm::vec3* bit) const;

};
