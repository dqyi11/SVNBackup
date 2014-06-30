#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QFileDialog>
#include <QBitmap>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mpMapViewer = new MapViewer(this);
    ui->gridLayout->addWidget(mpMapViewer);
    ui->gridLayout->setSizeConstraint(QLayout::SetMinimumSize);
    mCtrl = new PlayerCtrl();

    timerId = -1;
    mMapWidth = 0;
    mMapHeight = 0;

    mpPathLoader = new PathLoader();
    mMotionCtrl = NULL;
    mCtrlOn = false;
    mInterval = 100;
}

MainWindow::~MainWindow()
{
    killTimer(timerId);
    if(mCtrl!=NULL)
    {
        delete mCtrl;
        mCtrl = NULL;
    }
    if(mpPathLoader!=NULL)
    {
        delete mpPathLoader;
        mpPathLoader = NULL;
    }
    if(mMotionCtrl!=NULL)
    {
        delete mMotionCtrl;
        mMotionCtrl = NULL;
    }
    delete ui;
}

void MainWindow::on_action_Open_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open File"));
    if(filename.isEmpty())
    {
        return;
    }

    mpPathLoader->loadFile(filename);

    QPixmap pixmap(mpPathLoader->getMapFilepath());
    mpMapViewer->setPixmap(pixmap);
    mpMapViewer->adjustSize();
    mpMapViewer->showPoint(true);
    mpMapViewer->setPoint(40,40);

    mpMapViewer->loadPath(*(mpPathLoader->getPath()));

    mMapWidth = pixmap.width();
    mMapHeight = pixmap.height();

    qDebug ("pixmap w: %d h: %d", pixmap.width(), pixmap.height());
    qDebug("map viwer w: %d h: %d", mpMapViewer->width(), mpMapViewer->height());

    mCtrl->connect();
    mCtrl->setScale(40.0);

    if(mMotionCtrl!=NULL)
    {
        delete mMotionCtrl;
        mMotionCtrl = NULL;
    }
    mMotionCtrl = new MotionController(mpPathLoader->getPath());

    timerId = startTimer(mInterval);
}

void MainWindow::timerEvent(QTimerEvent *event)
{
    mCtrl->update();
    double x_pos = mCtrl->getXPos();
    double y_pos = mCtrl->getYPos();
    double yaw = mCtrl->getYaw();
    qDebug("point pos x %f y %f yaw %f", x_pos, y_pos, yaw);
    //qDebug("map width %d height %d", mMapWidth, mMapHeight);

    int disp_x_pos = transToDispX(x_pos);
    int disp_y_pos = transToDispY(y_pos);
    qDebug("paint pos x %d y %d", disp_x_pos, disp_y_pos);

    if(mCtrlOn==true)
    {
        if(mMotionCtrl->isFinished()==true)
        {
            mCtrlOn = false;
        }

        mMotionCtrl->updateVel(disp_x_pos, disp_y_pos, yaw, mInterval);
        double l_vel = mMotionCtrl->getLinearVelocity();
        double a_vel = mMotionCtrl->getAngularVelocity();

        mCtrl->setSpeed(l_vel, a_vel);

    }

    mpMapViewer->setPoint(disp_x_pos, disp_y_pos);
    mpMapViewer->setPointYaw(yaw);
    mpMapViewer->update();
    update();
}

double MainWindow::transFromDispX(double x)
{
    return x - mMapWidth/2;
}

double MainWindow::transFromDispY(double y)
{
    return - (y - mMapHeight/2);
}

double MainWindow::transToDispX(double x)
{
    return x + mMapWidth/2;
}

double MainWindow::transToDispY(double y)
{
    return - y + mMapHeight/2;
}


void MainWindow::on_action_Play_triggered()
{
    mCtrlOn = true;
}

