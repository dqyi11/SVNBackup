#include "mainwindow.h"
#include <QFileDialog>
#include <configobjdialog.h>
#include <QMessageBox>
#include <QtDebug>
#include <QKeyEvent>
#include <QStatusBar>

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

    mpStatusLabel = new QLabel();
    mpStatusProgressBar = new QProgressBar();

    statusBar()->addWidget(mpStatusProgressBar);
    statusBar()->addWidget(mpStatusLabel);


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
    mpFileMenu->addAction(mpExportAction);

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
    mpExportAction = new QAction("Export", this);
    mpLoadMapAction = new QAction("Load Map", this);
    mpLoadObjAction = new QAction("Load Objectives", this);
    mpRunAction = new QAction("Run", this);

    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(onOpen()));
    connect(mpSaveAction, SIGNAL(triggered()), this, SLOT(onSave()));
    connect(mpExportAction, SIGNAL(triggered()), this, SLOT(onExport()));
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
    QString tempFilename = QFileDialog::getOpenFileName(this,
             tr("Open File"), "./", tr("Json Files (*.json)"));

    if(mpViz)
    {
        mpViz->mMOPPInfo.loadFromFile(tempFilename);
        openMap(mpViz->mMOPPInfo.mMapFullpath);
        if(mpConfigObjDialog)
        {
            mpConfigObjDialog->updateDisplay();
        }
        repaint();
    }
}

void MainWindow::onSave()
{
    QString tempFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Json Files (*.json)"));

    if(mpViz)
    {
        mpViz->mMOPPInfo.saveToFile(tempFilename);
    }
}

void MainWindow::onExport()
{
    QString pathFilename = QFileDialog::getSaveFileName(this, tr("Save File"), "./", tr("Txt Files (*.txt)"));

    if(mpViz)
    {
        mpViz->mMOPPInfo.exportPaths(pathFilename);
    }
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

    openMap(mpViz->mMOPPInfo.mMapFullpath);
}


bool MainWindow::openMap(QString filename)
{
    if(mpMap)
    {
        delete mpMap;
        mpMap = NULL;
    }
    mpMap = new QPixmap(filename);
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
    if(mpViz->mMOPPInfo.mStart.x()<0 || mpViz->mMOPPInfo.mStart.y()<0)
    {
        QMessageBox msgBox;
        msgBox.setText("Start is not set.");
        msgBox.exec();
        return;
    }
    if(mpViz->mMOPPInfo.mGoal.x()<0 || mpViz->mMOPPInfo.mGoal.y()<0)
    {
        QMessageBox msgBox;
        msgBox.setText("Goal is not set.");
        msgBox.exec();
        return;
    }
    if(mpMORRF)
    {
        delete mpMORRF;
        mpMORRF = NULL;
    }

    mpViz->mMOPPInfo.initFuncsParams();
    QString msg = "RUNNING MORRF ... \n";
    msg += "ObjNum( " + QString::number(mpViz->mMOPPInfo.mObjectiveNum) + " ) \n";
    msg += "SubproblemNum( " + QString::number(mpViz->mMOPPInfo.mSubproblemNum) + " ) \n";
    msg += "SegmentLen( " + QString::number(mpViz->mMOPPInfo.mSegmentLength) + " ) \n";
    msg += "MaxIterationNum( " + QString::number(mpViz->mMOPPInfo.mMaxIterationNum) + " ) \n";
    qDebug(msg.toStdString().c_str());

    mpMORRF = new MORRF(mpMap->width(), mpMap->height(), mpViz->mMOPPInfo.mObjectiveNum, mpViz->mMOPPInfo.mSubproblemNum, mpViz->mMOPPInfo.mSegmentLength, mpViz->mMOPPInfo.mMethodType);

    mpMORRF->addFuncs(mpViz->mMOPPInfo.mFuncs, mpViz->mMOPPInfo.mDistributions);
    POS2D start(mpViz->mMOPPInfo.mStart.x(), mpViz->mMOPPInfo.mStart.y());
    POS2D goal(mpViz->mMOPPInfo.mGoal.x(), mpViz->mMOPPInfo.mGoal.y());

    mpMORRF->init(start, goal);
    mpViz->mMOPPInfo.getObstacleInfo(mpMORRF->getMapInfo());
    mpViz->setMORRF(mpMORRF);

    //mpMORRF->dumpMapInfo("map.txt");

    while(mpMORRF->getCurrentIteration() <= mpViz->mMOPPInfo.mMaxIterationNum)
    {
        QString msg = "CurrentIteration " + QString::number(mpMORRF->getCurrentIteration()) + " ";
        if(true == mpMORRF->areReferenceStructuresCorrect())
        {
            msg += "R(T) ";
        }
        else
        {
            msg += "R(F) ";
        }
        if(true == mpMORRF->areSubproblemStructuresCorrect())
        {
            msg += "S(T) ";
        }
        else
        {
            msg += "S(F) ";
        }
        if(true == mpMORRF->areAllReferenceNodesTractable())
        {
            msg += "R(T) ";
        }
        else
        {
            msg += "R(F) ";
        }
        if(true == mpMORRF->areAllSubproblemNodesTractable())
        {
            msg += "S(T) ";
        }
        else
        {
            msg += "S(F) ";
        }
        if(true == mpMORRF->areAllReferenceNodesFitnessPositive())
        {
            msg += "R(T) ";
        }
        else
        {
            msg += "R(F) ";
        }
        if(true == mpMORRF->areAllSubproblemNodesFitnessPositive())
        {
            msg += "S(T) ";
        }
        else
        {
            msg += "S(F) ";
        }
        if(true == mpMORRF->isNodeNumberIdentical())
        {
            msg += "T ";
        }
        else
        {
            msg += "F ";
        }
        if(true == mpMORRF->isRefTreeMinCost())
        {
            msg += "T ";
        }
        else
        {
            msg += "F ";
        }
        for(int k=0;k<mpViz->mMOPPInfo.mObjectiveNum;k++)
        {
            std::list<RRTNode*> list = mpMORRF->getReferenceTree(k)->findAllChildren(mpMORRF->getReferenceTree(k)->mpRoot);
            int num = list.size();
            msg += QString::number(num) + " ";
        }
        msg += QString::number(mpMORRF->getBallRadius());
        qDebug(msg.toStdString().c_str());

        mpMORRF->extend();

        updateStatus();
        repaint();
    }

    std::vector<Path*> paths = mpMORRF->getPaths();
    mpViz->mMOPPInfo.loadPaths(paths);
    repaint();
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

void MainWindow::updateStatus()
{
    if(mpViz==NULL || mpMORRF==NULL)
    {
        return;
    }
    if(mpStatusProgressBar)
    {
        mpStatusProgressBar->setMinimum(0);
        mpStatusProgressBar->setMaximum(mpViz->mMOPPInfo.mMaxIterationNum);
        mpStatusProgressBar->setValue(mpMORRF->getCurrentIteration());
    }
    if(mpStatusLabel)
    {
        QString status = "";
        status += "TreeIdx: " + QString::number(mpViz->getCurrentTreeIndex());
        mpStatusLabel->setText(status);
    }
    repaint();
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Right)
    {
        if(mpViz)
        {
            mpViz->nextTree();
            updateStatus();
            repaint();
        }
    }
    else if(event->key() == Qt::Key_Left)
    {
        if(mpViz)
        {
            mpViz->prevTree();
            updateStatus();
            repaint();
        }
    }
}
