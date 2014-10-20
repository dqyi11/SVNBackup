#include "interactivewindow.h"
#include <QtGui>
#include "imagedatagraph.h"
#include "segmentation.h"
#include "configdialog.h"
#include "parametermanager.h"

InteractiveWindow::InteractiveWindow(QWidget *parent) :
    QMainWindow(parent)
{
    mpImageLabel = new InteractiveLabel();
    mpImageLabel->setBackgroundRole(QPalette::Base);
    mpImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    setCentralWidget(mpImageLabel);

    mpOpenAction = new QAction(tr("&Open"), this);
    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(on_open_clicked()));
    mpExportAction = new QAction(tr("&Export"), this);
    connect(mpExportAction, SIGNAL(triggered()), this, SLOT(on_export_clicked()));;

    mpClearAction = new QAction(tr("&Clear"), this);
    connect(mpClearAction, SIGNAL(triggered()), this, SLOT(on_clear_clicked()));
    mpSegmentAction = new QAction(tr("&Segment"), this);
    connect(mpSegmentAction, SIGNAL(triggered()), this, SLOT(on_segment_clicked()));
    mpConfigAction = new QAction(tr("&Config"), this);
    connect(mpConfigAction, SIGNAL(triggered()), this, SLOT(on_config_clicked()));

    mpWorkStateGroup = new QActionGroup(this);

    mpGraphCutAction = new QAction(tr("&Graph Cut"), this);
    mpGraphCutAction->setCheckable(true);
    mpGraphCutAction->setActionGroup(mpWorkStateGroup);
    connect(mpGraphCutAction, SIGNAL(triggered()), this, SLOT(on_graphcut_clicked()));
    mpGrabCutAction = new QAction(tr("&Grab Cut"), this);
    mpGrabCutAction->setCheckable(true);
    mpGrabCutAction->setActionGroup(mpWorkStateGroup);
    connect(mpGrabCutAction, SIGNAL(triggered()), this, SLOT(on_grabcut_clicked()));


    mpFileMenu = new QMenu(tr("&File"), this);
    mpFileMenu->addAction(mpOpenAction);
    mpFileMenu->addAction(mpExportAction);
    mpEditMenu = new QMenu(tr("&Edit"), this);
    mpEditMenu->addAction(mpClearAction);
    mpEditMenu->addAction(mpConfigAction);
    mpEditMenu->addAction(mpSegmentAction);
    mpModeMenu = new QMenu(tr("&Mode"), this);
    mpModeMenu->addAction(mpGraphCutAction);
    mpModeMenu->addAction(mpGrabCutAction);


    this->menuBar()->addMenu(mpFileMenu);
    this->menuBar()->addMenu(mpEditMenu);
    this->menuBar()->addMenu(mpModeMenu);

    this->setWindowTitle(tr("Interactive Window"));
    this->resize(200, 200);

    mpGraphCutAction->setChecked(true);
    mpImageLabel->mCurrentWorkingState = InteractiveLabel::GRAPH_CUT_SEGMENTATION;

    mpParamMgr = new ParameterManager();
    mpConfigDialog = new ConfigDialog(mpParamMgr, this);

    qDebug() << "UI Initalized";
}

InteractiveWindow::~InteractiveWindow()
{
    if(mpImageLabel)
    {
        delete mpImageLabel;
        mpImageLabel = NULL;
    }
    if(mpOpenAction)
    {
        delete mpOpenAction;
        mpOpenAction = NULL;
    }
    if(mpClearAction)
    {
        delete mpClearAction;
        mpClearAction = NULL;
    }
    if(mpSegmentAction)
    {
        delete mpSegmentAction;
        mpSegmentAction = NULL;
    }
    if(mpConfigAction)
    {
        delete mpConfigAction;
        mpConfigAction = NULL;
    }
    if(mpGraphCutAction)
    {
        delete mpGraphCutAction;
        mpGraphCutAction = NULL;
    }
    if(mpGrabCutAction)
    {
        delete mpGrabCutAction;
        mpGrabCutAction = NULL;
    }
    if(mpWorkStateGroup)
    {
        delete mpWorkStateGroup;
        mpWorkStateGroup = NULL;
    }
    if(mpFileMenu)
    {
        delete mpFileMenu;
        mpFileMenu = NULL;
    }
    if(mpEditMenu)
    {
        delete mpEditMenu;
        mpEditMenu = NULL;
    }
    if(mpConfigDialog)
    {
        delete mpConfigDialog;
        mpConfigDialog = NULL;
    }
}

void InteractiveWindow::on_open_clicked()
{
    mFilename = QFileDialog::getOpenFileName(this, tr("Open File"), QDir::currentPath());
    if (!mFilename.isEmpty())
    {
        QImage image(mFilename);
        if (image.isNull())
        {
            QMessageBox::information(this, tr("Image Viewer"), tr("Cannot load %1.").arg(mFilename));
            return;
        }
        mpImageLabel->mColorPixmap = QPixmap::fromImage(image);
        mpImageLabel->setPixmap(mpImageLabel->mColorPixmap);
        mpImageLabel->resize(image.width(), image.height());


        QImage grayImg = mpImageLabel->mColorPixmap.toImage();

        for(int i=0;i<image.width();i++)
        {
            for(int j=0;j<image.height();j++)
            {
                QRgb col = image.pixel(i,j);
                int gray = qGray(col);
                grayImg.setPixel(i,j,qRgb(gray,gray,gray));
            }
        }
        mpImageLabel->mGrayPixmap = QPixmap::fromImage(grayImg);

        setWindowTitle(mFilename);
        resize(mpImageLabel->width(), mpImageLabel->height()+menuBar()->height());
        update();
    }
}

void InteractiveWindow::on_export_clicked()
{
    QString saveFilename = QFileDialog::getSaveFileName(this, tr("Open File"), QDir::currentPath());
    if (!saveFilename.isEmpty())
    {
        mpImageLabel->saveLabeledImage(saveFilename);
    }
}

void InteractiveWindow::on_clear_clicked()
{
    if(mpImageLabel)
    {
        mpImageLabel->clearLabel();
    }
}

void InteractiveWindow::on_segment_clicked()
{
    if(mpImageLabel->mCurrentWorkingState == InteractiveLabel::GRAPH_CUT_SEGMENTATION)
    {
        mpConfigDialog->exec();
        qDebug() << "Generating Graph-cut based segmentation " << mpParamMgr->mSmoothnessRatio;
        GraphCutSegmentation seg(mFilename.toStdString().c_str(), mpImageLabel->width(), mpImageLabel->height(),
                                 mpImageLabel->mpForegroundSeedMgr, mpImageLabel->mpBackgroundSeedMgr);
        qDebug() << " KDE Sigma " << mpParamMgr->mKDEBandWidth;
        seg.mKDESigma = mpParamMgr->mKDEBandWidth;
        seg.mSmoothnessRatio = mpParamMgr->mSmoothnessRatio;
        seg.process();
        seg.visualize();
    }
    else if(mpImageLabel->mCurrentWorkingState == InteractiveLabel::GRAB_CUT_SEGMENTATION)
    {
        mpConfigDialog->exec();
        qDebug() << "Generating Grab-cut based segmentation" << mpParamMgr->mSmoothnessRatio;
        GrabCutSegmentation seg(mFilename.toStdString().c_str(), mpImageLabel->width(), mpImageLabel->height(),
                                mpImageLabel->mRectStartX, mpImageLabel->mRectStartY, mpImageLabel->mRectEndX, mpImageLabel->mRectEndY);
        seg.mIterationNum = mpParamMgr->mIterationNumber;
        qDebug() << " KDE Sigma " << mpParamMgr->mKDEBandWidth;
        seg.mKDESigma = mpParamMgr->mKDEBandWidth;
        seg.mSmoothnessRatio = mpParamMgr->mSmoothnessRatio;
        seg.process();
        seg.visualize();
    }
}

void InteractiveWindow::on_graphcut_clicked()
{
    mpImageLabel->setWorkingState(InteractiveLabel::GRAPH_CUT_SEGMENTATION);
}

void InteractiveWindow::on_grabcut_clicked()
{
    mpImageLabel->setWorkingState(InteractiveLabel::GRAB_CUT_SEGMENTATION);
}

void InteractiveWindow::on_config_clicked()
{
    mpConfigDialog->exec();
}
