#ifndef MYGL_H
#define MYGL_H

#include <glwidget277.h>
#include <utils.h>
#include <shaderprogram.h>
#include <scene/cylinder.h>
#include <scene/sphere.h>
#include "camera.h"
#include "particles.h"
#include "simulation.h"

#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QObject>
#include <QFileDialog>
#include <QTimer>


class MyGL
    : public GLWidget277
{
    Q_OBJECT

private:
    Cylinder m_geomCylinder;// The instance of a unit cylinder we can use to render any cylinder
    Sphere m_geomSphere;// The instance of a unit sphere we can use to render any sphere
    ShaderProgram m_progLambert;// A shader program that uses lambertian reflection
    ShaderProgram m_progFlat;// A shader program that uses "flat" reflection (no shadowing at all)

    GLuint vao; // A handle for our vertex array object. This will store the VBOs created in our geometry classes.
                // Don't worry too much about this. Just know it is necessary in order to render geometry.

    /// Timer linked to timerUpdate(). Fires approx. 60 times per second
    QTimer timer;
    // stores time in milliseconds
    int time;

    Camera m_glCamera;

    Simulation* simulation;

public:
    explicit MyGL(QWidget *parent = 0);
    ~MyGL();

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void parseOBJ(const QString &fileName);

protected:
    void keyPressEvent(QKeyEvent *e);

private slots:
    /// Slot that gets called ~60 times per second
    void timerUpdate();
};


#endif // MYGL_H
