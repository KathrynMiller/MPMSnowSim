#include "mainwindow.h"
#include <ui_mainwindow.h>
#include "cameracontrolshelp.h"
#include <mygl.h>
#include <iostream>
#include <QListWidgetItem>
#include <QListWidget>
#include <QStringList>
#include <QString>
#include <QAbstractSpinBox>
#include <QRegularExpression>
#include <QMap>
#include <QJsonDocument>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mygl->setFocus();

    ui->frameRate->setValue(ui->mygl->getFrameRate());
    ui->numSeconds->setValue(ui->mygl->getNumSeconds());
    ui->samplerRadius->setValue(ui->mygl->getSamplerRadius());
    ui->isSnow->setChecked(ui->mygl->getIsSnow());
    ui->hardeningCo->setValue(ui->mygl->getHardeningCoeff());
    ui->youngsMod->setValue(ui->mygl->getYoungsMod());
    ui->thetaC->setValue(ui->mygl->getThetaC());
    ui->thetaS->setValue(ui->mygl->getThetaS());
    ui->minX->setValue(ui->mygl->getMinOffset()[0]);
    ui->minY->setValue(ui->mygl->getMinOffset()[1]);
    ui->minZ->setValue(ui->mygl->getMinOffset()[2]);
    ui->maxX->setValue(ui->mygl->getMaxOffset()[0]);
    ui->maxY->setValue(ui->mygl->getMaxOffset()[1]);
    ui->maxZ->setValue(ui->mygl->getMaxOffset()[2]);

    connect(ui->NewParticleSet, SIGNAL(clicked(bool)), ui->mygl, SLOT(generateNewParticleSet()));
    connect(ui->SaveSet, SIGNAL(clicked(bool)), ui->mygl, SLOT(saveSet()));
    connect(ui->LoadSet, SIGNAL(clicked(bool)), ui->mygl, SLOT(loadSet()));
    connect(ui->runSim, SIGNAL(clicked(bool)), ui->mygl, SLOT(runSim()));
    connect(ui->frameRate, SIGNAL(valueChanged(int)), ui->mygl, SLOT(updateFrameRate(int)));
    connect(ui->numSeconds, SIGNAL(valueChanged(int)), ui->mygl, SLOT(updateNumSeconds(int)));
    connect(ui->samplerRadius, SIGNAL(valueChanged(double)), ui->mygl, SLOT(setSamplerRadius(double)));
    connect(ui->isSnow, SIGNAL(stateChanged(int)), ui->mygl, SLOT(setIsSnow(int)));
    connect(ui->hardeningCo, SIGNAL(valueChanged(double)), ui->mygl, SLOT(setHardeningCoeff(double)));
    connect(ui->youngsMod, SIGNAL(valueChanged(double)), ui->mygl, SLOT(setYoungsMod(double)));
    connect(ui->thetaC, SIGNAL(valueChanged(double)), ui->mygl, SLOT(setThetaC(double)));
    connect(ui->thetaS, SIGNAL(valueChanged(double)), ui->mygl, SLOT(setThetaS(double)));
    connect(ui->minX, SIGNAL(valueChanged(int)), ui->mygl, SLOT(setMinX(int)));
    connect(ui->minY, SIGNAL(valueChanged(int)), ui->mygl, SLOT(setMinY(int)));
    connect(ui->minZ, SIGNAL(valueChanged(int)), ui->mygl, SLOT(setMinZ(int)));
    connect(ui->maxX, SIGNAL(valueChanged(int)), ui->mygl, SLOT(setMaxX(int)));
    connect(ui->maxY, SIGNAL(valueChanged(int)), ui->mygl, SLOT(setMaxY(int)));
    connect(ui->maxZ, SIGNAL(valueChanged(int)), ui->mygl, SLOT(setMaxZ(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionQuit_triggered()
{
    QApplication::exit();
}

void MainWindow::on_actionCamera_Controls_triggered()
{
    CameraControlsHelp* c = new CameraControlsHelp();
    c->show();
}
