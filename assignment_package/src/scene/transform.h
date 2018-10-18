#pragma once
#include "la.h"


class Transform
{
public:

    Transform();
    Transform(const glm::vec3 &t, const glm::vec3 &r, const glm::vec3 &s);

    void SetMatrices();

    const glm::mat4 &T() const;
    const glm::mat3 T3() const;
    const glm::mat4 &invT() const;
    const glm::mat3 &invTransT() const;

    const glm::vec3 &position() const {return translation;}
    const glm::vec3 &getScale() const {return scale;}
    const glm::mat4 getTransMat() const {
        glm::mat4 getTransMat = glm::mat4(glm::vec4(1, 0, 0, translation[0]),
                                       glm::vec4(0, 1, 0, translation[1]),
                                       glm::vec4(0, 0, 1, translation[2]),
                                       glm::vec4(0, 0, 0, 1));
    }
    const glm::mat4 getScaleMat() const {
        glm::mat4 scaleMat = glm::mat4(glm::vec4(scale[0], 0, 0, 0),
                                       glm::vec4(0, scale[1], 0, 0),
                                       glm::vec4(0, 0, scale[2], 0),
                                       glm::vec4(0, 0, 0, 1));
    }
    const glm::mat4 getRotationMat() const {
        glm::mat4 xMat = glm::rotate(glm::mat4(), rotation[0], glm::vec3(1, 0, 0));
        glm::mat4 yMat = glm::rotate(glm::mat4(), rotation[1], glm::vec3(0, 1, 0));
        glm::mat4 zMat = glm::rotate(glm::mat4(), rotation[2], glm::vec3(0, 0, 1));
        return xMat * yMat * zMat;
    }

    glm::vec3 translation;
    glm::vec3 rotation;
    glm::vec3 scale;

private:


    glm::mat4 worldTransform;
    glm::mat4 inverse_worldTransform;
    glm::mat3 inverse_transpose_worldTransform;
};
