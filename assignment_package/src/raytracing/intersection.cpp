#include <raytracing/intersection.h>
#include "la.h"

Intersection::Intersection():
    point(glm::vec3(0)),
    normalGeometric(glm::vec3(0)),
    t(-1),
    tangent(0.f), bitangent(0.f)
{}

Ray Intersection::SpawnRay(const glm::vec3 &d) const
{
    glm::vec3 originOffset = normalGeometric * .001f;
    //Vector3f originOffset = normalGeometric * RayEpsilon;
    // Make sure to flip the direction of the offset so it's in
    // the same general direction as the ray direction
    originOffset = (glm::dot(d, normalGeometric) > 0) ? originOffset : -originOffset;
    glm::vec3 o(this->point + originOffset);
    return Ray(o, d);
}
