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
private:
    int numParticles;
    Eigen::MatrixXd positions;
    Eigen::MatrixXd masses;
    Eigen::MatrixXd volumes;
    Eigen::MatrixXd velocities;
public:
    Particles(GLWidget277 *context, int numParticles);
    void create() override;
    GLenum drawMode() override;
};
