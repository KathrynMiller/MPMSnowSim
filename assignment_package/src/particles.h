#pragma once
#include <eigen-git-mirror/Eigen/Core>
#include <drawable.h>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <la.h>

struct KernelWeights {
    Eigen::Matrix3d N;
    Eigen::Matrix3d N_deriv;
};

struct Deformation {
    // stress on a particle (partial derivative of hyperelasticity model w.r.t F
    Eigen::Matrix3d stress;
    // deformation gradient
    Eigen::Matrix3d F;
    Eigen::Matrix3d Fe;
    Eigen::Matrix3d Fp;
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
    std::vector<Deformation*> deformations;

    Particles(GLWidget277 *context, int numParticles);
    void create() override;
    GLenum drawMode() override;

private:
    float density = 4.0 * pow(10, 2.0); // initial density in kg / m^3
};


