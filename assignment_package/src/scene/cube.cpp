#include "cube.h"
#include <iostream>


Cube::Cube() {}

int GetFaceIndex(const glm::vec3& P)
{
    int idx = 0;
    float val = -1;
    for(int i = 0; i < 3; i++){
        if(glm::abs(P[i]) > val){
            idx = i * glm::sign(P[i]);
            val = glm::abs(P[i]);
        }
    }
    return idx;
}

glm::vec3 GetCubeNormal(const glm::vec3& P)
{
    int idx = glm::abs(GetFaceIndex(glm::vec3(P)));
    glm::vec3 N(0,0,0);
    N[idx] = glm::sign(P[idx]);
    return N;
}


bool Cube::Intersect(const Ray& r) const
{
    //Transform the ray
    Ray r_loc = r.GetTransformedCopy(transform.invT());

    float t_n = -1000000;
    float t_f = 1000000;
    for(int i = 0; i < 3; i++){
        //Ray parallel to slab check
        if(r_loc.direction[i] == 0){
            if(r_loc.origin[i] < -0.5f || r_loc.origin[i] > 0.5f){
                return false;
            }
        }
        //If not parallel, do slab intersect check
        float t0 = (-0.5f - r_loc.origin[i])/r_loc.direction[i];
        float t1 = (0.5f - r_loc.origin[i])/r_loc.direction[i];
        if(t0 > t1){
            float temp = t1;
            t1 = t0;
            t0 = temp;
        }
        if(t0 > t_n){
            t_n = t0;
        }
        if(t1 < t_f){
            t_f = t1;
        }
    }
    if(t_n < t_f)
    {
        float t = t_n > 0 ? t_n : t_f;
        if (t < 0){
            return false;
        }
        //Lastly, transform the point found in object space by T
        glm::vec4 P = glm::vec4(r_loc.origin + t*r_loc.direction, 1);
        //InitializeIntersection(isect, t, glm::vec3(P));
        return true;
    }
    else{//If t_near was greater than t_far, we did not hit the cube
        return false;
    }
}


void Cube::ComputeTBN(const glm::vec3 &P, glm::vec3 *nor, glm::vec3 *tan, glm::vec3 *bit) const
{
    glm::vec3 normal = GetCubeNormal(P);
    *nor = glm::normalize(transform.invTransT() * normal);
    glm::vec3 tangent;
    glm::vec3 bitangent;
    if(normal[0] == 1) {
        tangent = glm::vec3(0, 0, -1);
        bitangent = glm::vec3(0, -1, 0);
    } else if (normal[0] == -1) {
        tangent = glm::vec3(0, 0, -1);
        bitangent = glm::vec3(0, 1, 0);
    } else if (normal[1] == 1) {
        tangent = glm::vec3(0, 0, -1);
        bitangent = glm::vec3(1, 0, 0);
    } else if (normal[1] == -1) {
        tangent = glm::vec3(0, 0, -1);
        bitangent = glm::vec3(-1, 0, 0);
    } else if (normal[2] == 1) {
        tangent = glm::vec3(0, 1, 0);
        bitangent = glm::vec3(1, 0, 0);
    } else if (normal[2] == -1) {
        tangent = glm::vec3(0, 1, 0);
        bitangent = glm::vec3(-1, 0, 0);
    }
    *tan = glm::normalize(transform.T3() * tangent);
    *bit = glm::normalize(transform.T3() * bitangent);
}
