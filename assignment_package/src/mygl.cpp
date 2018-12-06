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
#include <QRegularExpression>
#include "boxcollider.h"
#include "spherecollider.h"


MyGL::MyGL(QWidget *parent)
    : GLWidget277(parent),
      m_geomCylinder(this), m_geomSphere(this),
      m_progLambert(this), m_progFlat(this),
      m_glCamera(), simulation(new Simulation(numSeconds, frameRate)),
      time(QDateTime::currentMSecsSinceEpoch()), poissonSampler(new PoissonSampler()), running(false),
      gridBoundary(nullptr)
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
    if(simulation != nullptr) {
        simulation->particles->destroy();
        simulation->~Simulation();
        for(Collider* c: simulation->colliders) {
            c->destroy();
        }
    }
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
    if(simulation != nullptr && simulation->particles != nullptr) {
        m_progLambert.setModelMatrix(model);
        m_progLambert.draw(*(simulation->particles));
        m_progLambert.draw(*gridBoundary);

        for(Collider* c: simulation->colliders) {
            model = glm::translate(glm::mat4(1.0f), c->translation)
                    * glm::rotate(glm::mat4(1.0f), glm::radians(c->rotation.x), glm::vec3(1,0,0))
                    * glm::rotate(glm::mat4(1.0f), glm::radians(c->rotation.y), glm::vec3(0,1,0))
                    * glm::rotate(glm::mat4(1.0f), glm::radians(c->rotation.z), glm::vec3(0,0,1))
                    * glm::scale(glm::mat4(1.0f), c->scale);
            m_progLambert.setModelMatrix(model);
            m_progLambert.draw(*c);
        }
    }


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

void MyGL::addColliders() {
    glm::vec3 center = glm::vec3(0, -1.5, 0);
    float radius = .5;
    // node: make sure center = translation and radius = scale for correct visualization
    SphereCollider* sphere = new SphereCollider(center, radius, this, center, glm::vec3(), glm::vec3(radius, radius, radius));
    //simulation->colliders.push_back(sphere);

    BoxCollider* box = new BoxCollider(this, glm::vec3(0, -1.5, 0), glm::vec3(0, 0, 45.0), glm::vec3(1, 1, 1));
    simulation->colliders.push_back(box);
    for(Collider* c: simulation->colliders) {
        c->create();
    }
}


void MyGL::generateNewParticleSet() {
    QString filename = QFileDialog::getOpenFileName(0, QString("Load Scene File"), QDir::currentPath().append(QString("../../")), QString("*.obj"));

    // clear current particles if any
    poissonSampler->activeValidSamples.clear();
    poissonSampler->validSamples.clear();

    // sample from new mesh
    poissonSampler->SampleMesh(filename);

    Particles* p = new Particles(this, poissonSampler->validSamples.size());
    glm::vec3 minCorner = glm::vec3(INFINITY);
    glm::vec3 maxCorner = glm::vec3(-INFINITY);
    // transfer positions to particles
    for(int i = 0; i < poissonSampler->validSamples.size(); i++) {
        p->positions(i, 0) = poissonSampler->validSamples[i]->pos[0];
        p->positions(i, 1) = poissonSampler->validSamples[i]->pos[1];
        p->positions(i, 2) = poissonSampler->validSamples[i]->pos[2];

        minCorner[0] = std::min(poissonSampler->validSamples[i]->pos[0], minCorner[0]);
        minCorner[1] = std::min(poissonSampler->validSamples[i]->pos[1], minCorner[1]);
        minCorner[2] = std::min(poissonSampler->validSamples[i]->pos[2], minCorner[2]);

        maxCorner[0] = std::max(poissonSampler->validSamples[i]->pos[0], maxCorner[0]);
        maxCorner[1] = std::max(poissonSampler->validSamples[i]->pos[1], maxCorner[1]);
        maxCorner[2] = std::max(poissonSampler->validSamples[i]->pos[2], maxCorner[2]);
    }
    simulation->setParticles(p);
    simulation->particles->create();
    simulation->minCorner = minCorner;
    simulation->maxCorner = maxCorner;
    minCorner += simulation->gridMinOffset * simulation->cellSize;
    maxCorner += simulation->gridMaxOffset * simulation->cellSize;
    gridBoundary = new GridBoundary(this, minCorner, maxCorner);
    gridBoundary->create();
    addColliders();

}

void MyGL::saveSet() {
    output_filepath = QFileDialog::getSaveFileName(0, QString("Save Image"), QString("../rendered_images"), tr("*.txt"));
    if(output_filepath.length() == 0)
    {
        return;
    }
    QFile file(output_filepath);
    if (file.open(QIODevice::ReadWrite)) {
        QTextStream stream(&file);

        for(int i = 0; i < simulation->particles->positions.rows(); i++) {
            stream << QString::number(simulation->particles->positions(i, 0)) << " "
                   << QString::number(simulation->particles->positions(i, 1)) << " "
                   << QString::number(simulation->particles->positions(i, 2)) << endl;
        }

    }
}

void MyGL::loadSet() {

    QString filepath = QFileDialog::getOpenFileName(0, QString("Load Particles"), QString("../rendered_images"), tr("*.txt"));
    if(filepath.length() == 0)
    {
        return;
    }

    QFile file(filepath);
    std::vector<glm::vec3> positions = std::vector<glm::vec3>();

    if(file.open(QIODevice::ReadOnly))
    {
        while(!file.atEnd())
        {
            QString line = file.readLine().trimmed();
            QStringList pos = line.split(QRegularExpression("\\s+"));
            if(pos.count() == 3)
            {
                positions.push_back(glm::vec3(pos.at(0).toFloat(),
                                              pos.at(1).toFloat(),
                                              pos.at(2).toFloat()));
            }
        }
        file.close();
    }

    //simulation = new Simulation(new Particles(this, positions.size()), numSeconds, frameRate);
    Particles* p = new Particles(this, positions.size());
    glm::vec3 minCorner = glm::vec3(INFINITY);
    glm::vec3 maxCorner = glm::vec3(-INFINITY);
    for(int i = 0; i < p->positions.rows(); i++) {
        p->positions(i, 0) = positions[i][0];
        p->positions(i, 1) = positions[i][1];
        p->positions(i, 2) = positions[i][2];

        minCorner[0] = std::min(positions[i][0], minCorner[0]);
        minCorner[1] = std::min(positions[i][1], minCorner[1]);
        minCorner[2] = std::min(positions[i][2], minCorner[2]);

        maxCorner[0] = std::max(positions[i][0], maxCorner[0]);
        maxCorner[1] = std::max(positions[i][1], maxCorner[1]);
        maxCorner[2] = std::max(positions[i][2], maxCorner[2]);
    }
    simulation->setParticles(p);
    simulation->minCorner = minCorner;
    simulation->maxCorner = maxCorner;
    simulation->particles->create();
    minCorner += simulation->gridMinOffset * simulation->cellSize;
    maxCorner += simulation->gridMaxOffset * simulation->cellSize;
    gridBoundary = new GridBoundary(this, minCorner, maxCorner);
    gridBoundary->create();
    addColliders();
}

void MyGL::runSim() {
    // select location to save obj files
    QString output_filepath = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                                "/home",
                                                                QFileDialog::ShowDirsOnly
                                                                | QFileDialog::DontResolveSymlinks);
    simulation->isRunning = true;
    simulation->RunSimulation(output_filepath, this);
}

// MyGL's constructor links timerUpdate() to a timer that fires 60 times per second.
// We're treating MyGL as our game engine class, so we're going to use timerUpdate
void MyGL::timerUpdate()
{
    int64_t t = QDateTime::currentMSecsSinceEpoch();
    int64_t dt = t - time;
    //reset time
    time = t;
    // set dt of sim?
    update();
}

int MyGL::getNumSeconds() {
    return numSeconds;
}

int MyGL::getFrameRate() {
    return frameRate;
}

void MyGL::updateNumSeconds(int n) {
    numSeconds = n;
}

void MyGL::updateFrameRate(int n) {
    frameRate = n;
}

void MyGL::setYoungsMod(double n) {
    this->simulation->youngsMod = n;
}

void MyGL::setThetaC(double n) {
    simulation->thetaC = n;
}

void MyGL::setThetaS(double n) {
    simulation->thetaS = n;
}

void MyGL::setHardeningCoeff(double n) {
    simulation->hardeningCoeff = n;
}

void MyGL::setMinX(int n) {
    simulation->gridMinOffset[0] = n;
    gridBoundary->min[0] = simulation->minCorner[0] + n * simulation->cellSize;
    gridBoundary->destroy();
    gridBoundary->create();
}

void MyGL::setMaxX(int n) {
    simulation->gridMaxOffset[0] = n;
    gridBoundary->max[0] = simulation->maxCorner[0] + n * simulation->cellSize;
    gridBoundary->destroy();
    gridBoundary->create();
}

void MyGL::setMinY(int n) {
    simulation->gridMinOffset[1] = n;
    gridBoundary->min[1] = simulation->minCorner[1] + n * simulation->cellSize;
    gridBoundary->destroy();
    gridBoundary->create();
}

void MyGL::setMaxY(int n) {
    simulation->gridMaxOffset[1] = n;
    gridBoundary->max[1] = simulation->maxCorner[1] + n * simulation->cellSize;
    gridBoundary->destroy();
    gridBoundary->create();
}

void MyGL::setMinZ(int n) {
    simulation->gridMinOffset[2] = n;
    gridBoundary->min[2] = simulation->minCorner[2] + n * simulation->cellSize;
    gridBoundary->destroy();
    gridBoundary->create();
}

void MyGL::setMaxZ(int n) {
    simulation->gridMaxOffset[2] = n;
    gridBoundary->max[2] = simulation->maxCorner[2] + n * simulation->cellSize;
    gridBoundary->destroy();
    gridBoundary->create();
}

void MyGL::setIsSnow(int i) {
    if(i > 0) {
        simulation->isSnow = true;
    } else {
        simulation->isSnow = false;
    }

}

void MyGL::setSamplerRadius(double n) {
    poissonSampler->radius = n;
}

double MyGL::getYoungsMod() {
    simulation->youngsMod;
}

double MyGL::getThetaC() {
    return simulation->thetaC;
}

double MyGL::getThetaS() {
    return simulation->thetaS;
}

double MyGL::getHardeningCoeff() {
    return simulation->hardeningCoeff;
}

glm::vec3 MyGL::getMinOffset() {
    return simulation->gridMinOffset;
}

glm::vec3 MyGL::getMaxOffset() {
    return simulation->gridMaxOffset;
}

bool MyGL::getIsSnow() {
    return simulation->isSnow;
}

double MyGL::getSamplerRadius() {
    return poissonSampler->radius;
}

void MyGL::rand(double d) {
    simulation->youngsMod = d;
}
