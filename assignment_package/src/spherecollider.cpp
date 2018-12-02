#include "spherecollider.h"

SphereCollider::SphereCollider(const glm::vec3 &center, const float &radius): center(center), radius(radius)
{}

bool SphereCollider::isInside(glm::vec3 point) {
    if((point - center).length() - radius <= 0.0) {
        return true;
    }
    return false;
}

glm::vec3 SphereCollider::getNormal(glm::vec3 point) const {
    return glm::normalize(point - center);
}

glm::vec3 SphereCollider::getCenter() const {
    return center;
}

float SphereCollider::getRadius() const {
    return radius;
}
