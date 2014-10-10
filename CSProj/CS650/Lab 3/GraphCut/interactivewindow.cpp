#include "interactivewindow.h"
#include "ui_interactivewindow.h"
#include <QtGui>

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


    mpOpenAction = new QAction(tr("&Open..."), this);
    connect(mpOpenAction, SIGNAL(triggered()), this, SLOT(on_open_clicked()));
    mpClearAction = new QAction(tr("&Clear..."), this);
    connect(mpClearAction, SIGNAL(triggered()), this, SLOT(on_clear_clicked()));
    mpSegmentAction = new QAction(tr("&Segment..."), this);
    connect(mpSegmentAction, SIGNAL(triggered()), this, SLOT(on_segment_clicked()));

    mpFileMenu = new QMenu(tr("&File"), this);
    mpFileMenu->addAction(mpOpenAction);
    mpEditMenu = new QMenu(tr("&Edit"), this);
    mpEditMenu->addAction(mpClearAction);

    this->menuBar()->addMenu(mpFileMenu);
    this->menuBar()->addMenu(mpEditMenu);

    this->setWindowTitle(tr("Interactive Window"));
    this->resize(200, 200);

    //qDebug() << "Here";

}

InteractiveWindow::~InteractiveWindow()
{

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
        mpImageLabel->setPixmap(QPixmap::fromImage(image));
        mpImageLabel->resize(image.width(), image.height());

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

}


