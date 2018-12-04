#include "collider.h"

Collider::Collider(GLWidget277 *context, glm::vec3 translation, glm::vec3 rotation, glm::vec3 scale) : translation(translation),
    rotation(rotation), scale(scale), Drawable(context)
{}
