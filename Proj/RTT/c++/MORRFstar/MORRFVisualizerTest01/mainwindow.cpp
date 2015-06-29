#include "mainwindow.h"
#include <QFileDialog>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    mpViz = new MORRFVisualizer();

    createActions();
    createMenuBar();

    mpMap = NULL;

    setCentralWidget(mpViz);
}

MainWindow::~MainWindow()
{
    if(mpViz)
    {
        delete mpViz;
        mpViz = NULL;
    }
}

void MainWindow::createMenuBar()
{
     mpFileMenu = menuBar()->addMenu("&File");
     mpFileMenu->addAction(mpOpenAction);
     mpFileMenu->addAction(mpSaveAction);

     mpEditMenu = menuBar()->addMenu("&Edit");
     mpEditMenu->addAction(mpLoadMapAction);
     mpEditMenu->addAction(mpLoadObjAction);
     mpEditMenu->addAction(mpRunAction);

}

void MainWindow::createActions()
{
    mpOpenAction = new QAction("Open", this);
    mpSaveAction = new QAction("Save", this);
    mpLoadMapAction = new QAction("Load Map", this);
    mpLoadObjAction = new QAction("Load Objectives", this);
    mpRunAction = new QAction("Run", this);

    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
    connect(mpLoadMapAction, SIGNAL(triggered()), this, SLOT(onLoadMap()));
    connect(mpLoadObjAction, SIGNAL(triggered()), this, SLOT(onLoadObj()));
    connect(mpRunAction, SIGNAL(triggered()), this, SLOT(onRun()));
}

void MainWindow::onOpen()
{
}

void MainWindow::onSave()
{

}

void MainWindow::onLoadMap()
{

    mMapFilename = QFileDialog::getOpenFileName(this,
             tr("Open Map File 1"), "./", tr("Map Files (*.*)"));

    qDebug("OPENING ");
    qDebug(mMapFilename.toStdString().c_str());

    if(mpMap)
    {
        delete mpMap;
        mpMap = NULL;
    }
    mpMap = new QPixmap(mMapFilename);

    mpViz->setPixmap(*mpMap);
}

void MainWindow::onLoadObj()
{
    qDebug("onLoadObj()");
}

void MainWindow::onRun()
{

}
