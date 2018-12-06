#pragma once
#include "collider.h"

class BoxCollider : public Collider
{
public:
    BoxCollider(GLWidget277* context, glm::vec3 t, glm::vec3 r, glm::vec3 s);
    bool isInside(glm::vec3 point) const override;
    glm::vec3 getNormal(glm::vec3 point) const override;

    void create() override;

    glm::mat4 worldTransform;
    glm::mat4 inverse_worldTransform;
    glm::mat3 inverse_transpose_worldTransform;
};
