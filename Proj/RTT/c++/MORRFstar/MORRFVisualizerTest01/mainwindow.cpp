#include "mainwindow.h"
#include <QFileDialog>
#include <configobjdialog.h>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    mpViz = new MORRFVisualizer();

    createActions();
    createMenuBar();

    mpMap = NULL;
    mpMORRF = NULL;

    mpConfigObjDialog = new ConfigObjDialog(this);
    mpConfigObjDialog->hide();

    setCentralWidget(mpViz);

    updateTitle();
}

MainWindow::~MainWindow()
{
    if(mpConfigObjDialog)
    {
        delete mpConfigObjDialog;
        mpConfigObjDialog = NULL;
    }
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

    mpContextMenu = new QMenu();
    setContextMenuPolicy(Qt::CustomContextMenu);

    mpContextMenu->addAction(mpAddStartAction);
    mpContextMenu->addAction(mpAddGoalAction);

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

    mpAddStartAction = new QAction("Add Start", this);
    mpAddGoalAction = new QAction("Add Goal", this);

    connect(mpAddStartAction, SIGNAL(triggered()), this, SLOT(onAddStart()));
    connect(mpAddGoalAction, SIGNAL(triggered()), this, SLOT(onAddGoal()));

    connect(this, SIGNAL(customContextMenuRequested(const QPoint)),this, SLOT(contextMenuRequested(QPoint)));
}

void MainWindow::onOpen()
{

}

void MainWindow::onSave()
{

}

void MainWindow::onLoadMap()
{

    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open Map File"), "./", tr("Map Files (*.*)"));

    QFileInfo fileInfo(tempFilename);
    QString filename(fileInfo.fileName());
    mpViz->mMOPPInfo.mMapFilename = filename;
    mpViz->mMOPPInfo.mMapFullpath = tempFilename;

    qDebug("OPENING ");
    qDebug(mpViz->mMOPPInfo.mMapFilename.toStdString().c_str());

    if(mpMap)
    {
        delete mpMap;
        mpMap = NULL;
    }
    mpMap = new QPixmap(mpViz->mMOPPInfo.mMapFullpath);
    if(mpMap)
    {
        mpViz->mMOPPInfo.mMapWidth = mpMap->width();
        mpViz->mMOPPInfo.mMapHeight = mpMap->height();
        mpViz->setPixmap(*mpMap);
    }

    updateTitle();
}

void MainWindow::onLoadObj()
{
    mpConfigObjDialog->exec();
    updateTitle();
}

void MainWindow::onRun()
{
    if (mpViz->mMOPPInfo.mMapWidth <= 0 || mpViz->mMOPPInfo.mMapHeight <= 0)
    {
        QMessageBox msgBox;
        msgBox.setText("Map is not initialized.");
        msgBox.exec();
        return;
    }

    if (mpViz->mMOPPInfo.mObjectiveNum < 2)
    {
        QMessageBox msgBox;
        msgBox.setText("Objective Number is less than 2.");
        msgBox.exec();
        return;
    }

    if(mpMORRF)
    {
        delete mpMORRF;
        mpMORRF = NULL;
    }
    mpViz->mMOPPInfo.initFuncsParams();
    mpMORRF = new MORRF(mpMap->width(), mpMap->height(), mpViz->mMOPPInfo.mObjectiveNum, mpViz->mMOPPInfo.mSubproblemNum);
    mpMORRF->addFuncs(mpViz->mMOPPInfo.mFuncs, mpViz->mMOPPInfo.mDistributions);
    POS2D start(mpViz->mMOPPInfo.mStart.x(), mpViz->mMOPPInfo.mStart.y());
    POS2D goal(mpViz->mMOPPInfo.mGoal.x(), mpViz->mMOPPInfo.mGoal.y());

    mpMORRF->init(start, goal);
    mpViz->setMORRF(mpMORRF);

    while(mpMORRF->getCurrentIteration() <= mpViz->mMOPPInfo.mMaxIterationNum)
    {
        mpMORRF->extend();
        repaint();
        QString msg = "CurrentIteration " + QString::number(mpMORRF->getCurrentIteration());
        qDebug(msg.toStdString().c_str());
    }
}

void MainWindow::onAddStart()
{
    mpViz->mMOPPInfo.mStart = mCursorPoint;
    repaint();
}

void MainWindow::onAddGoal()
{
    mpViz->mMOPPInfo.mGoal = mCursorPoint;
    repaint();
}

void MainWindow::contextMenuRequested(QPoint point)
{
    mCursorPoint = point;
    mpContextMenu->popup(mapToGlobal(point));
}

void MainWindow::updateTitle()
{
    QString title = "ObjNum( " + QString::number(mpViz->mMOPPInfo.mObjectiveNum) + " )";
    title += " ==> " + mpViz->mMOPPInfo.mMapFilename;
    setWindowTitle(title);
}
