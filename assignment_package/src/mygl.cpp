#include "mygl.h"
#include <la.h>

#include <iostream>
#include <QApplication>
#include <QKeyEvent>
#include <mainwindow.h>
#include <algorithm>
#include <utility>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonValue>
#include <QDateTime>


MyGL::MyGL(QWidget *parent)
    : GLWidget277(parent),
      m_geomCylinder(this), m_geomSphere(this),
      m_progLambert(this), m_progFlat(this),
      m_glCamera(), simulation(new Simulation(new Particles(this, 1000))),
      time(QDateTime::currentMSecsSinceEpoch())
{
    // Connect the timer to a function so that when the timer ticks the function is executed
    connect(&timer, SIGNAL(timeout()), this, SLOT(timerUpdate()));
    // Tell the timer to redraw 60 times per second
    timer.start(16);

    setFocusPolicy(Qt::StrongFocus);
}

MyGL::~MyGL()
{
    makeCurrent();
    glDeleteVertexArrays(1, &vao);
    m_geomCylinder.destroy();
    m_geomSphere.destroy();
    simulation->getParticles()->destroy();
    simulation->~Simulation();
}

void MyGL::initializeGL()
{
    // Create an OpenGL context using Qt's QOpenGLFunctions_3_2_Core class
    // If you were programming in a non-Qt context you might use GLEW (GL Extension Wrangler)instead
    initializeOpenGLFunctions();
    // Print out some information about the current OpenGL context
    debugContextVersion();

    // Set a few settings/modes in OpenGL rendering
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    // Set the size with which points should be rendered
    glPointSize(5);
    // Set the color with which the screen is filled at the start of each render call.
    glClearColor(0.5, 0.5, 0.5, 1);

    printGLErrorLog();

    // Create a Vertex Attribute Object
    glGenVertexArrays(1, &vao);

    // Create and set up the diffuse shader
    m_progLambert.create(":/glsl/lambert.vert.glsl", ":/glsl/lambert.frag.glsl");
    // Create and set up the flat lighting shader
    m_progFlat.create(":/glsl/flat.vert.glsl", ":/glsl/flat.frag.glsl");

    // TODO: fill particle into a mesh
    simulation->getParticles()->create();

    // Set a color with which to draw geometry since you won't have one
    // defined until you implement the Node classes.
    // This makes your geometry render green.
    m_progLambert.setGeometryColor(glm::vec4(0,1,0,1));

    // We have to have a VAO bound in OpenGL 3.2 Core. But if we're not
    // using multiple VAOs, we can just bind one once.
    //    vao.bind();
    glBindVertexArray(vao);
}

void MyGL::resizeGL(int w, int h)
{
    //This code sets the concatenated view and perspective projection matrices used for
    //our scene's camera view.
    m_glCamera = Camera(w, h);
    glm::mat4 viewproj = m_glCamera.getViewProj();

    // Upload the view-projection matrix to our shaders (i.e. onto the graphics card)

    m_progLambert.setViewProjMatrix(viewproj);
    m_progFlat.setViewProjMatrix(viewproj);

    printGLErrorLog();
}

//This function is called by Qt any time your GL window is supposed to update
//For example, when the function updateGL is called, paintGL is called implicitly.
//DO NOT CONSTRUCT YOUR SCENE GRAPH IN THIS FUNCTION!
void MyGL::paintGL()
{
    // Clear the screen so that we only see newly drawn images
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_progFlat.setViewProjMatrix(m_glCamera.getViewProj());
    m_progLambert.setViewProjMatrix(m_glCamera.getViewProj());

    //#define NOPE
#ifndef NOPE

    glm::mat4 model = glm::translate(glm::mat4(1.0f), glm::vec3(0,0,0));
    //Send the geometry's transformation matrix to the shader
    // check if mesh is bound and draw with appropriate shader
        m_progLambert.setModelMatrix(model);
        m_progLambert.draw(*(simulation->getParticles()));


    this->glDisable(GL_DEPTH_TEST);

    this->glEnable(GL_DEPTH_TEST);
#endif
}


void MyGL::keyPressEvent(QKeyEvent *e)
{

    float amount = .10f;
    if(e->modifiers() & Qt::ShiftModifier){
        amount = .1f;
    }
    // http://doc.qt.io/qt-5/qt.html#Key-enum
    // This could all be much more efficient if a switch
    // statement were used, but I really dislike their
    // syntax so I chose to be lazy and use a long
    // chain of if statements instead
    if (e->key() == Qt::Key_Escape) {
        QApplication::quit();
    } else if (e->key() == Qt::Key_Right) {
        m_glCamera.RotatePhi(-amount);
    } else if (e->key() == Qt::Key_Left) {
        m_glCamera.RotatePhi(amount);
    } else if (e->key() == Qt::Key_Up) {
        m_glCamera.RotateTheta(-amount);
    } else if (e->key() == Qt::Key_Down) {
        m_glCamera.RotateTheta(amount);
    } else if (e->key() == Qt::Key_1) {
        m_glCamera.fovy += amount;
    } else if (e->key() == Qt::Key_2) {
        m_glCamera.fovy -= amount;
    } else if (e->key() == Qt::Key_W) {
        m_glCamera.TranslateAlongLook(amount);
    } else if (e->key() == Qt::Key_S) {
        m_glCamera.TranslateAlongLook(-amount);
    } else if (e->key() == Qt::Key_D) {
        m_glCamera.TranslateAlongRight(amount);
    } else if (e->key() == Qt::Key_A) {
        m_glCamera.TranslateAlongRight(-amount);
    } else if (e->key() == Qt::Key_Q) {
        m_glCamera.TranslateAlongUp(-amount);
    } else if (e->key() == Qt::Key_E) {
        m_glCamera.TranslateAlongUp(amount);
    } else if (e->key() == Qt::Key_R) {
        m_glCamera = Camera(this->width(), this->height());
    } else if (e->key() == Qt::Key_Plus) {
        m_glCamera.Zoom(amount);
    } else if (e->key() == Qt::Key_Minus) {
        m_glCamera.Zoom(-amount);
    } else if (e->key() == Qt::Key_P) {
        m_glCamera.PanRight(amount);
    } else if(e->key() == Qt::Key_O) {
        m_glCamera.PanRight(-amount);
    } else if (e->key() == Qt::Key_U) {
        m_glCamera.PanUp(amount);
    } else if (e->key() == Qt::Key_I) {
        m_glCamera.PanUp(-amount);
    }
    m_glCamera.RecomputeAttributes();
    update();  // Calls paintGL, among other things
}


void MyGL::parseOBJ(const QString &fileName) {

    qsrand(124324);
    QFile file(fileName);
    if(file.exists())
    {
        if(file.open(QFile::ReadOnly | QFile::Text))
        {
            std::vector<glm::vec3> v;
            while(!file.atEnd())
            {
                QString line = file.readLine().trimmed();
                QStringList lineParts = line.split(("\\s+"));
                if(lineParts.count() > 0)
                {

                    // if it’s a vertex position (v)
                    if(lineParts.at(0).compare("v", Qt::CaseInsensitive) == 0)
                    {
                        v.push_back(glm::vec3(lineParts.at(1).toFloat(),
                                              lineParts.at(2).toFloat(),
                                              lineParts.at(3).toFloat()));
                    }

                    // if it’s face data (f)
                    else if(lineParts.at(0).compare("f", Qt::CaseInsensitive) == 0)
                    {
                        // make random variations of pink
                        float r = (float) qrand() / (RAND_MAX);

                        for (int i = 1; i < lineParts.size(); i ++) {
                            // get points from v array
                            glm::vec3 pos = v[(lineParts.at(i).split("/").at(0).toInt() - 1)];
                        }
                    }

                }
            }

            file.close();
        }
    }
}

// MyGL's constructor links timerUpdate() to a timer that fires 60 times per second.
// We're treating MyGL as our game engine class, so we're going to use timerUpdate
void MyGL::timerUpdate()
{
    int64_t t = QDateTime::currentMSecsSinceEpoch();
    int64_t dt = t - time;
    //reset time
    time = t;

    simulation->getParticles()->destroy();
    simulation->RunSimulation();
    simulation->getParticles()->create();

    update();
}
