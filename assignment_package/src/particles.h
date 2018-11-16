#pragma once
#include <Partio.h>
#include <eigen-git-mirror/Eigen/Core>
#include <drawable.h>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <la.h>

struct KernelWeights {
    Eigen::MatrixXd N;
    Eigen::MatrixXd N_deriv;
};

struct Deformation {
    // stress on a particle (partial derivative of hyperelasticity model w.r.t F
    Eigen::Matrix3d stress;
    // deformation gradient
    Eigen::Matrix3d F;
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
    Eigen::MatrixXd mus;
    Eigen::MatrixXd lambdas;
    std::vector<KernelWeights*> kernelWeights;
    std::vector<Deformation*> deformations;

    Particles(GLWidget277 *context, int numParticles);
    void create() override;
    GLenum drawMode() override;

private:
    // constants that determine the consistency of the snow
    float hardeningCo = 10;
    float poissonsRatio = 0.3;
    float youngsMod = 1.0 * pow(10.0, 4.0);
    float density = 4.0 * pow(10, 2.0); // initial density in kg / m^3

    float thetaC = 2.5 * pow(10.0, -2.0);
    float thetaS = 7.5 * pow(10.0, -3.0);
};


