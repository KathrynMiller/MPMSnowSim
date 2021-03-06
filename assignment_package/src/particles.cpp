#include "particles.h"
#include "math.h"


Particles::Particles(GLWidget277 *context, int numParticles): Drawable(context), numParticles(numParticles),
positions(Eigen::MatrixXd(numParticles, 3)){

    // initialize other attributes of particles
    velocities = Eigen::MatrixXd::Zero(numParticles, 3);
    volumes = Eigen::MatrixXd::Zero(numParticles, 1);
    masses = Eigen::MatrixXd::Zero(numParticles, 1);

    for(int i = 0; i < numParticles; i++) {
        kernelWeights.push_back(new KernelWeights());
        deformations.push_back(new Deformation());
        deformations[i]->stress = Eigen::MatrixXd::Identity(3, 3);
        deformations[i]->F = Eigen::MatrixXd::Identity(3, 3);
        deformations[i]->Fe = Eigen::MatrixXd::Identity(3, 3);
        deformations[i]->Fp = Eigen::MatrixXd::Identity(3, 3);

// density = 1, mass = 10, volume = mass / density

        volumes(i) = 1.0 / numParticles;
        masses(i) = volumes(i) * density;
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

    count = numParticles;

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
