#include "boxcollider.h"

BoxCollider::BoxCollider(GLWidget277* context, glm::vec3 t, glm::vec3 r, glm::vec3 s) :
    Collider(context, t, r, s)
{
    worldTransform = glm::translate(glm::mat4(1.0f), translation)
            * glm::rotate(glm::mat4(1.0f), glm::radians(rotation[0]), glm::vec3(1,0,0))
            * glm::rotate(glm::mat4(1.0f), glm::radians(rotation[1]), glm::vec3(0,1,0))
            * glm::rotate(glm::mat4(1.0f), glm::radians(rotation[2]), glm::vec3(0,0,1))
            * glm::scale(glm::mat4(1.0f), scale);
    inverse_worldTransform = glm::inverse(worldTransform);
    inverse_transpose_worldTransform = glm::mat3(glm::inverse(glm::transpose(worldTransform)));
}

bool BoxCollider::isInside(glm::vec3 point) const {
    glm::vec4 localPoint = inverse_worldTransform * glm::vec4(point[0], point[1], point[2], 1);
    if(abs(localPoint[0]) <= 0.5 && abs(localPoint[1]) <= 0.5 && abs(localPoint[2]) <= 0.5) {
        return true;
    }
    return false;
}

glm::vec3 BoxCollider::getNormal(glm::vec3 point) const {
    glm::vec4 localPoint = inverse_worldTransform * glm::vec4(point[0], point[1], point[2], 1);

    float left = abs(localPoint[0] - (-0.5));
    float right = abs(localPoint[0] - (0.5));
    float down = abs(localPoint[1] - (-0.5));
    float up = abs(localPoint[1] - (0.5));
    float back = abs(localPoint[2] - (-0.5));
    float front = abs(localPoint[2] - (0.5));

    if(left < right && left < down && left < up && left < back && left < front) {
        return inverse_transpose_worldTransform * glm::vec3(-1, 0, 0);
    }
    if(right < left && right < down && right < up && right < back && right < front) {
        return inverse_transpose_worldTransform * glm::vec3(1, 0, 0);
    }

    if(up < right && up < down && up < left && up < back && up < front) {
        return inverse_transpose_worldTransform * glm::vec3(0, 1, 0);
    }
    if(down < right && down < left && down < up && down < back && down < front) {
        return inverse_transpose_worldTransform * glm::vec3(0, -1, 0);
    }

    if(front < right && front < down && front < up && front < back && front < left) {
        return inverse_transpose_worldTransform * glm::vec3(0, 0, 1);
    }
    if(back < right && back < down && back < up && back < left && back < front) {
        return inverse_transpose_worldTransform * glm::vec3(0, 0, -1);
    }

}

static const int CUB_IDX_COUNT = 36;
static const int CUB_VERT_COUNT = 24;

//These are functions that are only defined in this cpp file. They're used for organizational purposes
//when filling the arrays used to hold the vertex and index data.
void createCubeVertexPositions(glm::vec3 (&cub_vert_pos)[CUB_VERT_COUNT])
{
    int idx = 0;
    //Front face
    //UR
    cub_vert_pos[idx++] = glm::vec3(0.5f, 0.5f, 0.5f);
    //LR
    cub_vert_pos[idx++] = glm::vec3(0.5f, -0.5f, 0.5f);
    //LL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, -0.5f, 0.5f);
    //UL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, 0.5f, 0.5f);

    //Right face
    //UR
    cub_vert_pos[idx++] = glm::vec3(0.5f, 0.5f, -0.5f);
    //LR
    cub_vert_pos[idx++] = glm::vec3(0.5f, -0.5f, -0.5f);
    //LL
    cub_vert_pos[idx++] = glm::vec3(0.5f, -0.5f, 0.5f);
    //UL
    cub_vert_pos[idx++] = glm::vec3(0.5f, 0.5f, 0.5f);

    //Left face
    //UR
    cub_vert_pos[idx++] = glm::vec3(-0.5f, 0.5f, 0.5f);
    //LR
    cub_vert_pos[idx++] = glm::vec3(-0.5f, -0.5f, 0.5f);
    //LL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, -0.5f, -0.5f);
    //UL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, 0.5f, -0.5f);

    //Back face
    //UR
    cub_vert_pos[idx++] = glm::vec3(-0.5f, 0.5f, -0.5f);
    //LR
    cub_vert_pos[idx++] = glm::vec3(-0.5f, -0.5f, -0.5f);
    //LL
    cub_vert_pos[idx++] = glm::vec3(0.5f, -0.5f, -0.5f);
    //UL
    cub_vert_pos[idx++] = glm::vec3(0.5f, 0.5f, -0.5f);

    //Top face
    //UR
    cub_vert_pos[idx++] = glm::vec3(0.5f, 0.5f, -0.5f);
    //LR
    cub_vert_pos[idx++] = glm::vec3(0.5f, 0.5f, 0.5f);
    //LL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, 0.5f, 0.5f);
    //UL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, 0.5f, -0.5f);

    //Bottom face
    //UR
    cub_vert_pos[idx++] = glm::vec3(0.5f, -0.5f, 0.5f);
    //LR
    cub_vert_pos[idx++] = glm::vec3(0.5f, -0.5f, -0.5f);
    //LL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, -0.5f, -0.5f);
    //UL
    cub_vert_pos[idx++] = glm::vec3(-0.5f, -0.5f, 0.5f);
}


void createCubeVertexNormals(glm::vec3 (&cub_vert_nor)[CUB_VERT_COUNT])
{
    int idx = 0;
    //Front
    for(int i = 0; i < 4; i++){
        cub_vert_nor[idx++] = glm::vec3(0,0,1);
    }
    //Right
    for(int i = 0; i < 4; i++){
        cub_vert_nor[idx++] = glm::vec3(1,0,0);
    }
    //Left
    for(int i = 0; i < 4; i++){
        cub_vert_nor[idx++] = glm::vec3(-1,0,0);
    }
    //Back
    for(int i = 0; i < 4; i++){
        cub_vert_nor[idx++] = glm::vec3(0,0,-1);
    }
    //Top
    for(int i = 0; i < 4; i++){
        cub_vert_nor[idx++] = glm::vec3(0,1,0);
    }
    //Bottom
    for(int i = 0; i < 4; i++){
        cub_vert_nor[idx++] = glm::vec3(0,-1,0);
    }
}

void createCubeIndices(GLuint (&cub_idx)[CUB_IDX_COUNT])
{
    int idx = 0;
    for(int i = 0; i < 6; i++){
        cub_idx[idx++] = i*4;
        cub_idx[idx++] = i*4+1;
        cub_idx[idx++] = i*4+2;
        cub_idx[idx++] = i*4;
        cub_idx[idx++] = i*4+2;
        cub_idx[idx++] = i*4+3;
    }
}

void BoxCollider::create()
{
    GLuint cub_idx[CUB_IDX_COUNT];
    glm::vec3 cub_vert_pos[CUB_VERT_COUNT];
    glm::vec3 cub_vert_nor[CUB_VERT_COUNT];
    glm::vec3 cub_vert_col[CUB_VERT_COUNT];

    createCubeVertexPositions(cub_vert_pos);
    createCubeVertexNormals(cub_vert_nor);
    createCubeIndices(cub_idx);
std::vector<glm::vec3> col = std::vector<glm::vec3>();

    glm::vec3 color(255, 0, 255);

    for(int i = 0; i < CUB_VERT_COUNT; i++){
        col.push_back(color);
    }

    std::vector<glm::vec4> pos {//front face
                                glm::vec4(-0.5, -0.5, 0.5, 1),
                                glm::vec4(0.5, -0.5, 0.5, 1),
                                glm::vec4(0.5, 0.5, 0.5, 1),
                                glm::vec4(-0.5, 0.5, 0.5, 1),
                                //back face
                                glm::vec4(-0.5, -0.5, -0.5, 1),
                                glm::vec4(0.5, -0.5, -0.5, 1),
                                glm::vec4(0.5, 0.5, -0.5, 1),
                                glm::vec4(-0.5, 0.5, -0.5, 1),
                                //right face
                                glm::vec4(0.5, -0.5, 0.5, 1),
                                glm::vec4(0.5, -0.5, -0.5, 1),
                                glm::vec4(0.5, 0.5, -0.5, 1),
                                glm::vec4(0.5, 0.5, 0.5, 1),
                                //left face
                                glm::vec4(-0.5, -0.5, 0.5, 1),
                                glm::vec4(-0.5, -0.5, -0.5, 1),
                                glm::vec4(-0.5, 0.5, -0.5, 1),
                                glm::vec4(-0.5, 0.5, 0.5, 1),
                                //top face
                                glm::vec4(-0.5, 0.5, 0.5, 1), //16
                                glm::vec4(0.5, 0.5, 0.5, 1),
                                glm::vec4(0.5, 0.5, -0.5, 1),
                                glm::vec4(-0.5, 0.5, -0.5, 1), //19
                                //bottom face
                                glm::vec4(-0.5, -0.5, 0.5, 1),
                                glm::vec4(0.5, -0.5, 0.5, 1),
                                glm::vec4(0.5, -0.5, -0.5, 1),
                                glm::vec4(-0.5, -0.5, -0.5, 1)};

    std::vector<glm::vec4> nor {//front
                                glm::vec4(0, 0, 1, 0),
                                glm::vec4(0, 0, 1, 0),
                                glm::vec4(0, 0, 1, 0),
                                glm::vec4(0, 0, 1, 0),
                                //back
                                glm::vec4(0, 0, -1, 0),
                                glm::vec4(0, 0, -1, 0),
                                glm::vec4(0, 0, -1, 0),
                                glm::vec4(0, 0, -1, 0),
                                //right
                                glm::vec4(1, 0, 0, 0),
                                glm::vec4(1, 0, 0, 0),
                                glm::vec4(1, 0, 0, 0),
                                glm::vec4(1, 0, 0, 0),
                                //left
                                glm::vec4(-1, 0, 0, 0),
                                glm::vec4(-1, 0, 0, 0),
                                glm::vec4(-1, 0, 0, 0),
                                glm::vec4(-1, 0, 0, 0),
                                //top
                                glm::vec4(0, 1, 0, 0),
                                glm::vec4(0, 1, 0, 0),
                                glm::vec4(0, 1, 0, 0),
                                glm::vec4(0, 1, 0, 0),
                                //bottom
                                glm::vec4(0, -1, 0, 0),
                                glm::vec4(0, -1, 0, 0),
                                glm::vec4(0, -1, 0, 0),
                                glm::vec4(0, -1, 0, 0)};

    std::vector<GLuint> idx {0, 1, 2, 0, 2, 3,
                             4, 5, 6, 4, 6, 7,
                             8, 9, 10, 8, 10, 11,
                             12, 13, 14, 12, 14, 15,
                             16, 17, 18, 16, 18, 19,
                             20, 21, 22, 20, 22, 23 };

    count = 36;

    // Create a VBO on our GPU and store its handle in bufIdx
    generateIdx();
    // Tell OpenGL that we want to perform subsequent operations on the VBO referred to by bufIdx
    // and that it will be treated as an element array buffer (since it will contain triangle indices)
    mp_context->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufIdx);
    // Pass the data stored in cyl_idx into the bound buffer, reading a number of bytes equal to
    // SPH_IDX_COUNT multiplied by the size of a GLuint. This data is sent to the GPU to be read by shader programs.
    mp_context->glBufferData(GL_ELEMENT_ARRAY_BUFFER, CUB_IDX_COUNT * sizeof(GLuint), idx.data(), GL_STATIC_DRAW);

    // The next few sets of function calls are basically the same as above, except bufPos and bufNor are
    // array buffers rather than element array buffers, as they store vertex attributes like position.
    generatePos();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufPos);
    mp_context->glBufferData(GL_ARRAY_BUFFER, CUB_VERT_COUNT * sizeof(glm::vec4), pos.data(), GL_STATIC_DRAW);

    generateNor();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufNor);
    mp_context->glBufferData(GL_ARRAY_BUFFER, CUB_VERT_COUNT * sizeof(glm::vec4), nor.data(), GL_STATIC_DRAW);

    generateCol();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufCol);
    mp_context->glBufferData(GL_ARRAY_BUFFER, CUB_VERT_COUNT * sizeof(glm::vec3), col.data(), GL_STATIC_DRAW);

}
