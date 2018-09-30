#include "particles.h"
#include "math.h"


Particles::Particles(GLWidget277 *context, int numParticles): Drawable(context), numParticles(numParticles),
positions(Eigen::MatrixXd(numParticles, 3)){

    int range = cbrt(numParticles);
    // build a cube from 1000 particles
    int index = 0;
    for(int i = -range/2; i < range/2; i++) {
        for(int j = -range/2; j < range/2; j++) {
            for(int k = -range/2; k < range/2; k++) {
                 positions(index, 0) = i / 10.0;
                 positions(index, 1) = j / 10.0;
                 positions(index, 2) = k / 10.0;
                 index++;
            }
        }
    }
}


void Particles::create() {
    std::vector<glm::vec4> vert_pos;
    std::vector<glm::vec3> color;
    std::vector<GLuint> vertIdx;

    int idxNum = 0;
    for(int i = 0; i < numParticles; i++) {
        vert_pos.push_back(glm::vec4(positions.coeff(i, 0), positions.coeff(i, 1), positions.coeff(i, 2), 1));
        vertIdx.push_back(idxNum);
        idxNum ++;
        color.push_back(glm::vec3(255, 255, 255));
    }

    count = numParticles * 4;

    generateIdx();

    mp_context->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufIdx);
    mp_context->glBufferData(GL_ELEMENT_ARRAY_BUFFER, vertIdx.size() * sizeof(GLuint), vertIdx.data(), GL_STATIC_DRAW);

    generatePos();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufPos);
    mp_context->glBufferData(GL_ARRAY_BUFFER, vert_pos.size() * sizeof(glm::vec4), vert_pos.data(), GL_STATIC_DRAW);

    generateCol();
    mp_context->glBindBuffer(GL_ARRAY_BUFFER, bufCol);
    mp_context->glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), color.data(), GL_STATIC_DRAW);

}

GLenum Particles::drawMode()
{
    return GL_POINTS;
}
