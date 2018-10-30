#include "mainwindow.h"
#include <ui_mainwindow.h>
#include "cameracontrolshelp.h"
#include <mygl.h>
#include <iostream>
#include <QListWidgetItem>
#include <QListWidget>
#include <QStringList>
#include <QString>
#include <QRegularExpression>
#include <QMap>
#include <QJsonDocument>



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->mygl->setFocus();
    connect(ui->NewParticleSet, SIGNAL(clicked(bool)), ui->mygl, SLOT(generateNewParticleSet()));
    connect(ui->SaveSet, SIGNAL(clicked(bool)), ui->mygl, SLOT(saveSet()));
    connect(ui->LoadSet, SIGNAL(clicked(bool)), ui->mygl, SLOT(loadSet()));
    connect(ui->runSim, SIGNAL(clicked(bool)), ui->mygl, SLOT(runSim()));
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
