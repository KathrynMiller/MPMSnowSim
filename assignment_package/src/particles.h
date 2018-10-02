#pragma once
#include <Partio.h>
#include <eigen-git-mirror/Eigen/Core>
#include <drawable.h>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <la.h>

//template <unsigned int numParticles>
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

    Particles(GLWidget277 *context, int numParticles);
    void create() override;
    GLenum drawMode() override;

};
