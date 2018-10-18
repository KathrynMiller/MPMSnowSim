#pragma once
#include <raytracing/ray.h>
#include <QList>

class Material;
class Scene;
class BSDF;

class Intersection
{
public:
    Intersection();

    // Instantiate a Ray that originates from this Intersection and
    // travels in direction d.
    Ray SpawnRay(const glm::vec3 &d) const;

    glm::vec3 point;          // The place at which the intersection occurred
    glm::vec3 normalGeometric; // The surface normal at the point of intersection, NO alterations like normal mapping applied
    float t;                  // The parameterization for the ray (in world space) that generated this intersection.
                              // t is equal to the distance from the point of intersection to the ray's origin if the ray's direction is normalized.

    glm::vec3 tangent, bitangent; // World-space vectors that form an orthonormal basis with the surface normal.
};
