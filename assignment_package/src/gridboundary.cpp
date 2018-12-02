#include "gridboundary.h"

GridBoundary::GridBoundary(GLWidget277 *context, float cellSize, glm::vec3 min, glm::vec3 max) :
    Drawable(context), min(min), max(max), cellSize(cellSize) {}


void GridBoundary::create() {
    std::vector<glm::vec4> pos;
    std::vector<glm::vec3> color;
    std::vector<GLuint> idx;

    int idxNum = 0;
    //    for(int i = 0; i < numParticles; i++) {
    //        vert_pos.push_back(glm::vec4(positions.coeff(i, 0), positions.coeff(i, 1), positions.coeff(i, 2), 1));
    //        vertIdx.push_back(idxNum);
    //        idxNum ++;
    //        color.push_back(glm::vec3(255, 255, 255));
    //    }
    pos.push_back(glm::vec4(min[0] - 1, min[1] - 1, min[2] - 1, 1));
    pos.push_back(glm::vec4(max[0] + 1, max[1] + 1, max[2] + 1, 1));
    color.push_back(glm::vec3(255, 0, 255));
    color.push_back(glm::vec3(255, 0, 255));
    idx.push_back(0);
    idx.push_back(1);

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


