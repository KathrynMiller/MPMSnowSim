#pragma once
#include "la.h"

class SphereCollider
{
public:
    SphereCollider(const glm::vec3 &center, const float &radius);

    bool isInside(glm::vec3 point);

    glm::vec3 getCenter() const;
    glm::vec3 getNormal(glm::vec3 point) const;
    float getRadius() const;

private:
    glm::vec3 center;
    float radius;
};
