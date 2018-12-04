#pragma once
#include "la.h"
#include "collider.h"

class SphereCollider : public Collider
{
public:
    SphereCollider(const glm::vec3 &center, const float &radius, glm::vec3 t, glm::vec3 r, glm::vec3 s);

    bool isInside(glm::vec3 point) const override;

    glm::vec3 getCenter() const;
    glm::vec3 getNormal(glm::vec3 point) const override;
    float getRadius() const;

private:
    glm::vec3 center;
    float radius;
};
