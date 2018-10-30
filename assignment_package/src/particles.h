#pragma once
#include <Partio.h>
#include <eigen-git-mirror/Eigen/Core>
#include <drawable.h>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <la.h>

struct KernelWeights {
    Eigen::Matrix3f N;
    Eigen::Matrix3f N_deriv;
};

class Particles: public Drawable
{
public:
    // number of particles in the simulation
    int numParticles;

    // numParticles x 3 matrix holding position data
    Eigen::MatrixXd positions;
    Eigen::MatrixXd masses;
    Eigen::MatrixXd volumes;
    Eigen::MatrixXd velocities;
    std::vector<KernelWeights*> kernelWeights;

    Particles(GLWidget277 *context, int numParticles);
    void create() override;
    GLenum drawMode() override;



};


