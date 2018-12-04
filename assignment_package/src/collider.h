#pragma once

class Collider
{
protected:
    glm::vec3 translation;
    glm::vec3 rotation;
    glm::vec3 scale;
public:
    Collider(glm::vec3 translation, glm::vec3 rotation, glm::vec3 scale);
    // takes in a point in world space and returns if it is inside the object
    virtual bool isInside(glm::vec3 point) const = 0;
    // returns the normal at the point of intersection between the point and the collider
    virtual glm::vec3 getNormal(glm::vec3 point) const = 0;
};