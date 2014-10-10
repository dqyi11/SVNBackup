#include "interactivewindow.h"
#include "ui_interactivewindow.h"
#include <QtGui>
#include "imagedatagraph.h"

InteractiveWindow::InteractiveWindow(QWidget *parent) :
    QMainWindow(parent)
{
    mpImageLabel = new InteractiveLabel();
    mpImageLabel->setBackgroundRole(QPalette::Base);
    mpImageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    //mpImageLabel->setScaledContents(true);

    /*
    mpScrollArea = new QScrollArea;
    mpScrollArea->setBackgroundRole(QPalette::Dark);
    mpScrollArea->setWidget(mpImageLabel);
    setCentralWidget(mpScrollArea);
    */
    setCentralWidget(mpImageLabel);


    mpOpenAction = new QAction(tr("&Open"), this);
    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(on_open_clicked()));
    mpClearAction = new QAction(tr("&Clear"), this);
    connect(mpClearAction, SIGNAL(triggered()), this, SLOT(on_clear_clicked()));
    mpSegmentAction = new QAction(tr("&Segment"), this);
    connect(mpSegmentAction, SIGNAL(triggered()), this, SLOT(on_segment_clicked()));

    mpFileMenu = new QMenu(tr("&File"), this);
    mpFileMenu->addAction(mpOpenAction);
    mpEditMenu = new QMenu(tr("&Edit"), this);
    mpEditMenu->addAction(mpClearAction);
    mpEditMenu->addAction(mpSegmentAction);

    this->menuBar()->addMenu(mpFileMenu);
    this->menuBar()->addMenu(mpEditMenu);

    this->setWindowTitle(tr("Interactive Window"));
    this->resize(200, 200);

    //qDebug() << "Here";
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
}

void InteractiveWindow::on_open_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), QDir::currentPath());
    if (!fileName.isEmpty())
    {
        QImage image(fileName);
        if (image.isNull())
        {
            QMessageBox::information(this, tr("Image Viewer"), tr("Cannot load %1.").arg(fileName));
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

        setWindowTitle(fileName);
        resize(mpImageLabel->width(), mpImageLabel->height()+menuBar()->height());
        update();
    }

}


void InteractiveWindow::on_clear_clicked()
{
    if(mpImageLabel)
    {
        mpImageLabel->clear();
    }
}

void InteractiveWindow::on_segment_clicked()
{
    qDebug() << "Generating graph";

}


