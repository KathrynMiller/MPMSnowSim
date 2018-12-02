#pragma once
#include <eigen-git-mirror/Eigen/Core>
#include <drawable.h>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <la.h>

class GridBoundary: public Drawable
{
public:
    GridBoundary(GLWidget277 *context, glm::vec3 min, glm::vec3 max);
    void create() override;
    GLenum drawMode() override;

    glm::vec3 min;
    glm::vec3 max;
};
