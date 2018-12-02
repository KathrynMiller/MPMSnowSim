#include "gridboundary.h"

GridBoundary::GridBoundary(GLWidget277 *context, glm::vec3 min, glm::vec3 max) :
    Drawable(context), min(min), max(max) {}


void GridBoundary::create() {
    std::vector<glm::vec4> pos;
    std::vector<glm::vec3> color;
    std::vector<GLuint> idx;

    glm::vec4 p0 = glm::vec4(min[0], max[1], min[0], 1);
    glm::vec4 p1 = glm::vec4(min[0], min[1], min[0], 1);
    glm::vec4 p2 = glm::vec4(min[0], min[1], max[0], 1);
    glm::vec4 p3 = glm::vec4(min[0], max[1], max[0], 1);

    glm::vec4 p4 = glm::vec4(max[0], max[1], min[0], 1);
    glm::vec4 p5 = glm::vec4(max[0], max[1], max[0], 1);
    glm::vec4 p6 = glm::vec4(max[0], min[1], max[0], 1);
    glm::vec4 p7 = glm::vec4(max[0], min[1], min[0], 1);

    pos.push_back(p0);
    pos.push_back(p1);
    pos.push_back(p2);
    pos.push_back(p3);
    pos.push_back(p4);
    pos.push_back(p5);
    pos.push_back(p6);
    pos.push_back(p7);

    idx.push_back(0);
    idx.push_back(1);
    idx.push_back(1);
    idx.push_back(2);
    idx.push_back(2);
    idx.push_back(3);
    idx.push_back(3);
    idx.push_back(0);

    idx.push_back(0);
    idx.push_back(4);
    idx.push_back(4);
    idx.push_back(5);
    idx.push_back(5);
    idx.push_back(3);

    idx.push_back(4);
    idx.push_back(7);
    idx.push_back(7);
    idx.push_back(6);
    idx.push_back(6);
    idx.push_back(5);

    idx.push_back(6);
    idx.push_back(2);
    idx.push_back(7);
    idx.push_back(1);

glm::vec3 col = glm::vec3(0, 255, 255);
    color.push_back(col);
    color.push_back(col);
    color.push_back(col);
    color.push_back(col);
    color.push_back(col);
    color.push_back(col);
    color.push_back(col);
    color.push_back(col);

    count = idx.size();

    generateIdx();
    mp_context->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufIdx);
    mp_context->glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(GLuint), idx.data(), GL_STATIC_DRAW);

    generatePos();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufPos);
    mp_context->glBufferData(GL_ARRAY_BUFFER, pos.size() * sizeof(glm::vec4), pos.data(), GL_STATIC_DRAW);

    generateCol();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufCol);
    mp_context->glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), color.data(), GL_STATIC_DRAW);
}

GLenum GridBoundary::drawMode() {
    return GL_LINES;
}


