#include "mainwindow.h"
#include <QFileDialog>
#include <configobjdialog.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    mpViz = new MORRFVisualizer();

    createActions();
    createMenuBar();



    mpMap = NULL;

    mpConfigObjDialog = new ConfigObjDialog(this);
    mpConfigObjDialog->hide();

    setCentralWidget(mpViz);
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

    mpViz->mMOPPInfo.mMapFilename = QFileDialog::getOpenFileName(this,
             tr("Open Map File"), "./", tr("Map Files (*.*)"));

    qDebug("OPENING ");
    qDebug(mpViz->mMOPPInfo.mMapFilename.toStdString().c_str());

    if(mpMap)
    {
        delete mpMap;
        mpMap = NULL;
    }
    mpMap = new QPixmap(mpViz->mMOPPInfo.mMapFilename);

    mpViz->setPixmap(*mpMap);
}

void MainWindow::onLoadObj()
{
    mpConfigObjDialog->exec();
}

void MainWindow::onRun()
{

}

void MainWindow::onAddStart()
{
    mpViz->mMOPPInfo.mStart = mCursorPoint;
    /*
    QString msg = "Add Start ";
    msg.append(QString::number(mpViz->mMOPPInfo.mStart.x()));
    msg.append(" ");
    msg.append(QString::number(mpViz->mMOPPInfo.mStart.y()));
    qDebug(msg.toStdString().c_str());
    */
    repaint();
}

void MainWindow::onAddGoal()
{
    mpViz->mMOPPInfo.mGoal = mCursorPoint;
    /*
    QString msg = "Add Goal ";
    msg.append(QString::number(mpViz->mMOPPInfo.mGoal.x()));
    msg.append(" ");
    msg.append(QString::number(mpViz->mMOPPInfo.mGoal.y()));
    qDebug(msg.toStdString().c_str());
    */
    repaint();
}

void MainWindow::contextMenuRequested(QPoint point)
{
    mCursorPoint = point;
    mpContextMenu->popup(mapToGlobal(point));
}
