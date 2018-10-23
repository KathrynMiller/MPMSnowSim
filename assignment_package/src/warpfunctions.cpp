#define _USE_MATH_DEFINES
#include "warpfunctions.h"
#include <math.h>


glm::vec3 WarpFunctions::squareToDiskUniform(const glm::vec2 &sample)
{
    float r = glm::sqrt(sample[0]);
    float theta = 2.0 * 3.1415 * sample[1];
    return glm::vec3(r * glm::cos(theta), r * glm::sin(theta), 0.0);
}

glm::vec3 WarpFunctions::squareToDiskConcentric(const glm::vec2 &sample)
{
    float phi, r, u, v;
    float a = 2.0 * sample[0] - 1.0;
    float b = 2.0 * sample[1] - 1.0;

    if(a > -b) {
        if(a > b) {
            r = a;
            phi = (M_PI / 4.0) * (b / a);
        } else {
            r = b;
            phi = (M_PI / 4.0) * (2.0 - (a / b));
        }
    } else {
        if(a < b) {
            r = -a;
            phi = (M_PI / 4.0) * (4.0 + (b / a));
        } else {
            r = -b;
            if(b != 0.0) {
                phi = (M_PI / 4.0) * (6.0 - (a / b));
            } else {
                phi = 0.0;
            }
        }
    }
    u = r * glm::cos(phi);
    v = r * glm::sin(phi);
    return glm::vec3(u, v, 0.0);
}


glm::vec3 WarpFunctions::squareToSphereUniform(const glm::vec2 &sample)
{
    float z = 1.0 - 2.0 * sample[0];
    float x = glm::cos(2.0 * M_PI * sample[1]) * glm::sqrt(1.0 - z * z);
    float y = glm::sin(2.0 * M_PI * sample[1]) * glm::sqrt(1.0 - z * z);
    return glm::vec3(x, y, z);
}


glm::vec3 WarpFunctions::squareToSphereCapUniform(const glm::vec2 &sample, float thetaMin)
{
    float z = (1.0 - 2.0 * sample[0]);
    z = z * (1.0 - thetaMin / 180.0) + thetaMin/180.0;
    float x = glm::cos(2.0 * M_PI * sample[1]) * glm::sqrt(1.0 - z * z);
    float y = glm::sin(2.0 * M_PI * sample[1]) * glm::sqrt(1.0 - z * z);
    return glm::vec3(x, y, z);
}


glm::vec3 WarpFunctions::squareToHemisphereUniform(const glm::vec2 &sample)
{
    glm::vec3 u = squareToDiskUniform(sample);
    float r = glm::sqrt(u[0] * u[0] + u[1] * u[1]);
    float x = u[0] * glm::sqrt(2.0 - r * r);
    float y = u[1] * glm::sqrt(2.0 - r * r);
    float z = 1.0 - r * r;
    return glm::vec3(x, y, z);

}


glm::vec3 WarpFunctions::squareToHemisphereCosine(const glm::vec2 &sample)
{
    glm::vec3 u = squareToDiskConcentric(sample);
    float z = glm::sqrt(std::max(0.0, 1.0 - u[0] * u[0] - u[1] * u[1]));
    return glm::vec3(u[0], u[1], z);
}


glm::vec3 WarpFunctions::UniformSampleCone(const glm::vec2 &xi, float cosThetaMax) {
    float cosTheta = (1.f - xi[0]) + xi[0] * cosThetaMax;
    float sinTheta = std::sqrt(1.f - cosTheta * cosTheta);
    float phi = xi[1] * 2.f * 3.14159;
    return glm::vec3(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta,
                    cosTheta);
}
