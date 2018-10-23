#pragma once
#include "la.h"
#include "tinyobj/tiny_obj_loader.h"
#include "raytracing/ray.h"
#include "raytracing/intersection.h"
#include "scene/cube.h"


class Triangle
{
public:

    Triangle(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3);

    glm::vec3 points[3];
    glm::vec3 midPoint;
    glm::vec3 normals[3];
    glm::vec3 planeNormal;
    std::vector<glm::vec3> boundingBox;

   // Cube* boundingBox;
//    std::vector<glm::vec3> getBoundingBox();
    bool Intersect(const Ray& r, Intersection* isect) const;
};

// for the acceleration tree
class Node
{
public:
    Node();
    ~Node();

    Node* leftChild;
    Node* rightChild;
    unsigned int axis; // Which axis split this node represents
    glm::vec3 minCorner, maxCorner; // The world-space bounds of this node
    QList<Triangle*> triangles; // A collection of pointers to the particles contained in this node.
};


class ObjLoader
{
public:
    ObjLoader();

public:

    // Should return the sum of all triangles' areas
 //   virtual float Area() const;

    // TODO: make children nullptrs (initialize variables)
    void LoadOBJ(const QString &filepath);

    // parts that define the obj tree
    Node* root;
    void build(const QList<Triangle*> triangles);
    glm::vec3 minCorner, maxCorner; // For visualization purposes

    bool intersectsBoundingBox(Node* node, const Ray &r) const;
    bool triangleIntersection(Node* parent, const Ray& r, Intersection* isect, int *numIntersections) const;
    bool Intersect(const Ray& r, int *numIntersections);

    QList<Triangle*> faces;

};
