#include "objloader.h"
#include "iostream"
#include "QFileDialog"

Triangle::Triangle(const glm::vec3 &p1, const glm::vec3 &p2, const glm::vec3 &p3) {
    planeNormal = glm::normalize(glm::cross(p2 - p1, p3 - p2));
    points[0] = p1;
    points[1] = p2;
    points[2] = p3;

    // set the bounding box on initialization
    glm::vec3 minPoint = glm::vec3(0, 0, 0);
    minPoint[0] = glm::min(points[0][0], glm::min(points[1][0], points[2][0]));
    minPoint[1] = glm::min(points[0][1], glm::min(points[1][1], points[2][1]));
    minPoint[2] = glm::min(points[0][2], glm::min(points[1][2], points[2][2]));

    glm::vec3 maxPoint = glm::vec3(0, 0, 0);
    maxPoint[0] = glm::max(points[0][0], glm::max(points[1][0], points[2][0]));
    maxPoint[1] = glm::max(points[0][1], glm::max(points[1][1], points[2][1]));
    maxPoint[2] = glm::max(points[0][2], glm::max(points[1][2], points[2][2]));

    // set midpoint of the bounding box for sorting triangles
    midPoint = glm::vec3((minPoint[0] + maxPoint[0]) / 2.f, (minPoint[1] + maxPoint[1]) / 2.f,
            (minPoint[2] + maxPoint[2]) / 2.f);

    std::vector<glm::vec3> box = std::vector<glm::vec3>();

    //boundingBox = new Cube()
    //    boundingBox.push_back(minPoint);
    //    boundingBox.push_back(maxPoint);
}


bool Triangle::Intersect(const Ray& r, Intersection* isect) const
{
    // assumes untransformed mesh
    Ray r_loc = r.GetTransformedCopy(glm::mat4());
    //1. Ray-plane intersection
    if(planeNormal == glm::vec3(1, 0, 0)) {
        int k = 0;
    }
    float t =  glm::dot(planeNormal, (points[0] - r_loc.origin)) / glm::dot(planeNormal, r_loc.direction);
    if(t < 0) return false;

    glm::vec3 P = r_loc.origin + t * r_loc.direction;
    //2. Barycentric test
    float S = 0.5f * glm::length(glm::cross(points[0] - points[1], points[0] - points[2]));
    float s1 = 0.5f * glm::length(glm::cross(P - points[1], P - points[2]))/S;
    float s2 = 0.5f * glm::length(glm::cross(P - points[2], P - points[0]))/S;
    float s3 = 0.5f * glm::length(glm::cross(P - points[0], P - points[1]))/S;
    float sum = s1 + s2 + s3;

    if(s1 >= 0 && s1 <= 1 && s2 >= 0 && s2 <= 1 && s3 >= 0 && s3 <= 1 && (abs(1.0 - sum) > .0001)){
        isect->t = t;
        return true;
    }
    return false;
}

// obj loader class functions
ObjLoader::ObjLoader()
{}

void ObjLoader::LoadOBJ(const QString &filepath) {
    //QString filepath = QFileDialog::getOpenFileName(0, QString("Load Scene File"), QDir::currentPath().append(QString("../../")), QString("*.obj"));
    // QString filepath = local_path.toString(); filepath.append(filename);
    std::vector<tinyobj::shape_t> shapes; std::vector<tinyobj::material_t> materials;
    std::string errors = tinyobj::LoadObj(shapes, materials, filepath.toStdString().c_str());
    std::cout << errors << std::endl;
    if(errors.size() == 0)
    {
        //Read the information from the vector of shape_ts
        for(unsigned int i = 0; i < shapes.size(); i++)
        {
            std::vector<float> &positions = shapes[i].mesh.positions;
            std::vector<float> &normals = shapes[i].mesh.normals;
            std::vector<unsigned int> &indices = shapes[i].mesh.indices;
            for(unsigned int j = 0; j < indices.size(); j += 3)
            {
                glm::vec3 p1(positions[indices[j]*3], positions[indices[j]*3+1], positions[indices[j]*3+2]);
                glm::vec3 p2(positions[indices[j+1]*3], positions[indices[j+1]*3+1], positions[indices[j+1]*3+2]);
                glm::vec3 p3(positions[indices[j+2]*3], positions[indices[j+2]*3+1], positions[indices[j+2]*3+2]);

                Triangle* t = new Triangle(p1, p2, p3);
                if(normals.size() > 0)
                {
                    glm::vec3 n1(normals[indices[j]*3], normals[indices[j]*3+1], normals[indices[j]*3+2]);
                    glm::vec3 n2(normals[indices[j+1]*3], normals[indices[j+1]*3+1], normals[indices[j+1]*3+2]);
                    glm::vec3 n3(normals[indices[j+2]*3], normals[indices[j+2]*3+1], normals[indices[j+2]*3+2]);
                    t->normals[0] = n1;
                    t->normals[1] = n2;
                    t->normals[2] = n3;
                }

                // initialize bounding box
                t->boundingBox.push_back(glm::vec3());
                t->boundingBox.push_back(glm::vec3());
                t->boundingBox[0][0] = glm::min(p1[0], glm::min(p2[0], p3[0]));
                t->boundingBox[0][1] = glm::min(p1[1], glm::min(p2[1], p3[1]));
                t->boundingBox[0][2] = glm::min(p1[2], glm::min(p2[2], p3[2]));

                t->boundingBox[1][0] = glm::max(p1[0], glm::max(p2[0], p3[0]));
                t->boundingBox[1][1] = glm::max(p1[1], glm::max(p2[1], p3[1]));
                t->boundingBox[1][2] = glm::max(p1[2], glm::max(p2[2], p3[2]));
                this->faces.append(t);
            }
        }
        std::cout << "" << std::endl;
        //TODO: .mtl file loading
    }
    else
    {
        //An error loading the OBJ occurred!
        std::cout << errors << std::endl;
    }

    // put faces in acceleration structure
    build(this->faces);
}

// node functions
Node::Node()
    : leftChild(nullptr), rightChild(nullptr), axis(0), minCorner(), maxCorner(), triangles()
{}

Node::~Node()
{
    delete leftChild;
    delete rightChild;
}

// Comparator functions you can use with std::sort to sort vec3s along the cardinal axes
bool xSort(Triangle* a, Triangle* b) { return a->midPoint.x < b->midPoint.x; }
bool ySort(Triangle* a, Triangle* b) { return a->midPoint.y < b->midPoint.y; }
bool zSort(Triangle* a, Triangle* b) { return a->midPoint.z < b->midPoint.z; }

void fillChildren(Node* parent, int depth) {
    if(parent->triangles.size() < 2) {
        parent->leftChild = nullptr;
        parent->rightChild = nullptr;
        return;
    }
    int axis = depth % 3;
    parent->axis = axis;

    glm::vec3 lmin = parent->minCorner;
    glm::vec3 lmax = parent->maxCorner;
    glm::vec3 rmin = parent->minCorner;
    glm::vec3 rmax = parent->maxCorner;

    // sort parents particles based on calculated axis of division
    switch (axis) {
    case 0:
        std::sort(parent->triangles.begin(), parent->triangles.end(), xSort);
        break;
    case 1:
        std::sort(parent->triangles.begin(), parent->triangles.end(), ySort);
        break;
    case 2:
        std::sort(parent->triangles.begin(), parent->triangles.end(), zSort);
        break;
    default:
        break;
    }

    int middle = parent->triangles.size() / 2;
    QList<Triangle*> left = QList<Triangle*>();
    QList<Triangle*> right = QList<Triangle*>();

    // add lower points to left vector and upper to right
    // also check for min and max to set bounding box of parent
    for(int i = 0; i < middle; i++) {
        Triangle* p = parent->triangles[i];
        left.push_back(p);
    }
    for(int i = middle; i < parent->triangles.size(); i++) {
        Triangle* p = parent->triangles[i];
        right.push_back(p);
    }

    Triangle* midPoint = parent->triangles[middle];
    if(axis == 0) {
        rmin[0] = (midPoint->boundingBox[0])[0];
        lmax[0] = (midPoint->boundingBox[0])[0];
    } else if (axis== 1) {
        rmin[1] = (midPoint->boundingBox[0])[1];
        lmax[1] = (midPoint->boundingBox[0])[1];
    } else {
        rmin[2] = (midPoint->boundingBox[0])[2];
        lmax[2] = (midPoint->boundingBox[0])[2];
    }

    Node* leftChild = new Node();
    Node* rightChild = new Node();

    leftChild->triangles = left;
    leftChild->maxCorner = lmax;
    leftChild->minCorner = lmin;

    rightChild->triangles = right;
    rightChild->minCorner = rmin;
    rightChild->maxCorner = rmax;

    parent->leftChild = leftChild;
    parent->rightChild = rightChild;


    fillChildren(leftChild, depth+1);
    fillChildren(rightChild, depth+1);
}

void ObjLoader::build(const QList<Triangle*> triangles) {
    root = new Node();
    float minX = std::numeric_limits<int>::max();
    float minY = std::numeric_limits<int>::max();
    float minZ = std::numeric_limits<int>::max();
    float maxX = std::numeric_limits<int>::min();
    float maxY = std::numeric_limits<int>::min();
    float maxZ = std::numeric_limits<int>::min();


    for(Triangle* p: triangles) {
        minX = glm::min(minX, p->boundingBox[0][0]);
        minY = glm::min(minY, p->boundingBox[0][1]);
        minZ = glm::min(minZ, p->boundingBox[0][2]);

        maxX = glm::max(maxX, p->boundingBox[1][0]);
        maxY = glm::max(maxY, p->boundingBox[1][1]);
        maxZ = glm::max(maxZ, p->boundingBox[1][2]);
    }


    this->minCorner = glm::vec3(minX, minY, minZ);
    this->maxCorner = glm::vec3(maxX, maxY, maxZ);

    root->minCorner = glm::vec3(minX, minY, minZ);
    root->maxCorner = glm::vec3(maxX, maxY, maxZ);

    root->triangles = triangles;

    fillChildren(root, 0);
}

bool ObjLoader::intersectsBoundingBox(Node* node, const Ray& r) const{
    // initialize bounding box as a cube based on min and max values of the node
    Cube bBox = Cube();
    glm::vec3 scale = glm::vec3(node->maxCorner[0] - node->minCorner[0], node->maxCorner[1] - node->minCorner[1], node->maxCorner[2] - node->minCorner[2]);
    glm::vec3 translate = node->minCorner + (scale / glm::vec3(2.0, 2.0, 2.0));
    glm::vec3 rotate = glm::vec3();
    bBox.transform = Transform(translate, rotate, scale);

    return bBox.Intersect(r);
}

bool ObjLoader::triangleIntersection(Node* parent, const Ray &r, Intersection *isect, int* numIntersections) const{
    if(intersectsBoundingBox(parent, r)) {
        if(parent->leftChild == nullptr && parent->rightChild == nullptr) { // hit a leaf
            if(parent->triangles[0]->Intersect(r, isect)) {
                // initialize intersection and return true
                if(isect->t > 0) {
                    glm::vec3 P = glm::vec3(isect->t * r.direction + r.origin);
                    // make sure this changes correctly
                    (*numIntersections)++;
                    // do we need to initialize intersection? just need to know if intersects
                    return true;
                }
            }
            return false;
        }

        // keep recursing down appropriate sides
        bool hitLeft = triangleIntersection(parent->leftChild, r, isect, numIntersections);
        bool hitRight = triangleIntersection(parent->rightChild, r, isect, numIntersections);
        return hitLeft || hitRight;
    }

    /*
    if (hitLeft && !hitRight) {
        return triangleIntersection(parent->leftChild, r, isect, numIntersections);
    } else if (!hitLeft && hitRight) {
        return triangleIntersection(parent->rightChild, r, isect, numIntersections);
    } else if (!hitLeft && !hitRight) {
        // did not intersect with either of the bboxes
        return false;
    }

    Intersection* testLeft = new Intersection();
    Intersection* testRight = new Intersection();

    hitLeft = triangleIntersection(parent->leftChild, r, testLeft, numIntersections);
    hitRight = triangleIntersection(parent->rightChild, r, testRight, numIntersections);

    // checking for recursive bbox-to-triangle intersections
    if (hitLeft && !hitRight) {
        isect = testLeft;
        return true;
    } else if (!hitLeft && hitRight) {
        isect = testRight;
        return true;
    } else if (!hitLeft && !hitRight) {
        return false;
    }

    // else: both recursive bbox-to-triangle intersections returned true for the test intersections;
    *isect = (testLeft->t > testRight->t) ? *testRight : *testLeft;
    return true;
    */
}

bool ObjLoader::Intersect(const Ray &r, int* numIntersections)
{
    Intersection isect;
    return triangleIntersection(root, r, &isect, numIntersections);
}

