#include "interactivewindow.h"
#include "ui_interactivewindow.h"
#include <QtGui>

InteractiveWindow::InteractiveWindow(QWidget *parent) :
    QMainWindow(parent)
{
    mpImageLabel = new QLabel();
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

    mCurrentState = NORMAL;

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

void InteractiveWindow::mouseReleaseEvent ( QMouseEvent * e )
{
    mCurrentState = NORMAL;
}

void InteractiveWindow::mousePressEvent ( QMouseEvent * e )
{
    if (e->button()==Qt::LeftButton)
    {
        qDebug() << "Left";
        mCurrentState = ADD_FOREGROUND;

    }
    else if(e->button()==Qt::RightButton)
    {
        qDebug() << "Right";
        mCurrentState = ADD_BACKGROUND;
    }
}

void InteractiveWindow::mouseMoveEvent( QMouseEvent * e )
{
    int x_bias = 0;
    int y_bias = menuBar()->height();

    if(mCurrentState == ADD_FOREGROUND)
    {
        //qDebug() << "F: " << e->pos();
        mForegroundSeeds.push_back(QPoint(e->x()-x_bias, e->y()-y_bias));

        update();
    }
    else if(mCurrentState == ADD_BACKGROUND)
    {
        //qDebug() << "B: " << e->pos();
        mBackgroundSeeds.push_back(QPoint(e->x()-x_bias, e->y()-y_bias));

        update();
    }

}

void InteractiveWindow::on_clear_clicked()
{
    mForegroundSeeds.clear();
    mBackgroundSeeds.clear();
}

void InteractiveWindow::on_segment_clicked()
{

}

void InteractiveWindow::paintEvent(QPaintEvent* e)
{
    QMainWindow::paintEvent(e);

    QPainter painter(this);
    QPen blue_pen(QColor(0,0,255));
    QBrush blue_brush(QColor(0,0,255));
    QPen red_pen(QColor(255,0,0));
    QBrush red_brush(QColor(255,0,0));

    int x_bias = 0;
    int y_bias = menuBar()->height();

    painter.setPen(red_pen);
    painter.setBrush(red_brush);
    for(std::list<QPoint>::iterator it=mForegroundSeeds.begin();it!=mForegroundSeeds.end();it++)
    {
        int x_pos = (*it).x() + x_bias;
        int y_pos = (*it).y() + y_bias;
        painter.drawPoint(x_pos, y_pos);
    }

    painter.setPen(blue_pen);
    painter.setBrush(blue_brush);
    for(std::list<QPoint>::iterator it=mBackgroundSeeds.begin();it!=mBackgroundSeeds.end();it++)
    {
        int x_pos = (*it).x() + x_bias;
        int y_pos = (*it).y() + y_bias;
        painter.drawPoint(x_pos, y_pos);
    }


}
