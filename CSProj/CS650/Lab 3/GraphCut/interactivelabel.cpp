#include "interactivelabel.h"
#include <QtGui>
#include <QIODevice>
#include "segmentation.h"

InteractiveLabel::InteractiveLabel(QWidget * parent) :
    QLabel(parent)
{
     mCurrentLabelingState = NORMAL;
     mCurrentWorkingState = GRAPH_CUT_SEGMENTATION;
     mPointerRadius = 2;

     mpForegroundSeedMgr = new SeedManager();
     mpBackgroundSeedMgr = new SeedManager();

     mpInitPoint = new QPoint(0,0);
     mpEndPoint = new QPoint(0,0);

     mDrawingRect = false;
     mRectStartX = 0;
     mRectStartY = 0;
     mRectEndX = 0;
     mRectEndY = 0;
}

InteractiveLabel::~InteractiveLabel()
{
    if(mpForegroundSeedMgr)
    {
        delete mpForegroundSeedMgr;
        mpForegroundSeedMgr = NULL;
    }
    if(mpBackgroundSeedMgr)
    {
        delete mpBackgroundSeedMgr;
        mpBackgroundSeedMgr = NULL;
    }
    if(mpInitPoint)
    {
        delete mpInitPoint;
        mpInitPoint = NULL;
    }
    if(mpEndPoint)
    {
        delete mpEndPoint;
        mpEndPoint = NULL;
    }
    QLabel::~QLabel();
}

void InteractiveLabel::mousePressEvent ( QMouseEvent * e )
{
    if (e->button()==Qt::LeftButton)
    {
        if(mCurrentWorkingState == GRAPH_CUT_SEGMENTATION)
        {
            mCurrentLabelingState = ADD_FOREGROUND;
        }
        else if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
        {
            mpInitPoint->setX(e->x());
            mpInitPoint->setY(e->y());
            mpEndPoint->setX(e->x());
            mpEndPoint->setY(e->y());
            mDrawingRect = true;
        }

        setPixmap(mGrayPixmap);
        update();
    }
    else if(e->button()==Qt::RightButton)
    {
        if(mCurrentWorkingState == GRAPH_CUT_SEGMENTATION)
        {
            mCurrentLabelingState = ADD_BACKGROUND;
            setPixmap(mGrayPixmap);
            update();
        }
    }
}

void InteractiveLabel::clearLabel()
{
    if(mpForegroundSeedMgr)
    {
        mpForegroundSeedMgr->clear();
    }
    if(mpBackgroundSeedMgr)
    {
        mpBackgroundSeedMgr->clear();
    }
    update();
}

void InteractiveLabel::mouseMoveEvent( QMouseEvent * e )
{
    if(mCurrentWorkingState == GRAPH_CUT_SEGMENTATION)
    {
        if(mCurrentLabelingState == ADD_FOREGROUND)
        {
            //qDebug() << "F: " << e->pos();
            int cursor_x = e->x();
            int cursor_y = e->y();
            for(int i=cursor_x - mPointerRadius;i<=cursor_x + mPointerRadius;i++)
            {
                for(int j=cursor_y - mPointerRadius;j<=cursor_y + mPointerRadius;j++)
                {
                    if(i>=0 && i<width() && j>=0 && j<height())
                    {
                        mpForegroundSeedMgr->addSeed(i, j);
                    }
                }
            }
            update();
        }
        else if(mCurrentLabelingState == ADD_BACKGROUND)
        {
            int cursor_x = e->x();
            int cursor_y = e->y();
            for(int i=cursor_x - mPointerRadius;i<=cursor_x + mPointerRadius;i++)
            {
                for(int j=cursor_y - mPointerRadius;j<=cursor_y + mPointerRadius;j++)
                {
                    if(i>=0 && i<width() && j>=0 && j<height())
                    {
                        mpBackgroundSeedMgr->addSeed(i, j);
                    }
                }
            }
            update();
        }
    }
    else if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
    {
        //if(e->button()==Qt::LeftButton)
        if(mDrawingRect==true)
        {
            mpEndPoint->setX(e->x());
            mpEndPoint->setY(e->y());
            update();
        }
    }
}

void InteractiveLabel::mouseReleaseEvent ( QMouseEvent * e )
{
    if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
    {
        if(e->button()==Qt::LeftButton)
        {
            mpEndPoint->setX(e->x());
            mpEndPoint->setY(e->y());

            mRectStartX = mpInitPoint->x()<= mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            mRectStartY = mpInitPoint->y()<= mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();
            mRectEndX   = mpInitPoint->x()> mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            mRectEndY   = mpInitPoint->y()> mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();

            mRectStartX = mRectStartX >= 0 ? mRectStartX : 0;
            mRectStartY = mRectStartY >= 0 ? mRectStartY : 0;
            mRectEndX   = mRectEndX < this->width() ?  mRectEndX : this->width()-1;
            mRectEndY   = mRectEndY < this->height() ?  mRectEndY : this->height()-1;
        }
    }
    setPixmap(mColorPixmap);
    mCurrentLabelingState = NORMAL;
    mDrawingRect = false;
    update();
}

void InteractiveLabel::paintEvent(QPaintEvent* e)
{
    QLabel::paintEvent(e);

    if(mCurrentWorkingState == GRAPH_CUT_SEGMENTATION)
    {
        QPainter painter(this);
        QPen blue_pen(QColor(0,0,255));
        QBrush blue_brush(QColor(0,0,255));
        QPen red_pen(QColor(255,0,0));
        QBrush red_brush(QColor(255,0,0));


        painter.setPen(red_pen);
        painter.setBrush(red_brush);
        for(std::list<PixelPosition>::iterator it=mpForegroundSeedMgr->mpSeeds->begin();it!=mpForegroundSeedMgr->mpSeeds->end();it++)
        {
            painter.drawPoint((*it).vals[0], (*it).vals[1]);
        }

        painter.setPen(blue_pen);
        painter.setBrush(blue_brush);
        for(std::list<PixelPosition>::iterator it=mpBackgroundSeedMgr->mpSeeds->begin();it!=mpBackgroundSeedMgr->mpSeeds->end();it++)
        {
            painter.drawPoint((*it).vals[0], (*it).vals[1]);
        }
    }
    else if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
    {
        if(mpInitPoint->x()!= mpEndPoint->x() || mpInitPoint->y()!=mpEndPoint->y())
        {
            QPainter painter(this);
            QPen green_pen(QColor(0,255,0));
            painter.setPen(green_pen);

            mRectStartX = mpInitPoint->x()<= mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            mRectStartY = mpInitPoint->y()<= mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();
            mRectEndX   = mpInitPoint->x()> mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            mRectEndY   = mpInitPoint->y()> mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();

            painter.drawRect(mRectStartX, mRectStartY, mRectEndX - mRectStartX, mRectEndY - mRectStartY);
        }
    }
}

void InteractiveLabel::setWorkingState(WorkingState state)
{
    if(mCurrentWorkingState == state)
    {
        return;
    }
    mCurrentWorkingState = state;

    if(mCurrentWorkingState == GRAPH_CUT_SEGMENTATION)
    {
        mCurrentLabelingState = NORMAL;
        mpForegroundSeedMgr->clear();
        mpBackgroundSeedMgr->clear();
        update();
    }
    else if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
    {
        mpInitPoint->setX(0);
        mpInitPoint->setY(0);
        mpEndPoint->setX(0);
        mpEndPoint->setY(0);
        update();
    }

}

void InteractiveLabel::saveLabeledImage(QString filename)
{
    QFile exportFile(filename);
    exportFile.open(QIODevice::WriteOnly);
    QPixmap exportPixmap = pixmap()->copy(0, 0, pixmap()->width(), pixmap()->height());
    QPainter painter(&exportPixmap);

    if(mCurrentWorkingState == GRAPH_CUT_SEGMENTATION)
    {
        QPen blue_pen(QColor(0,0,255));
        QBrush blue_brush(QColor(0,0,255));
        QPen red_pen(QColor(255,0,0));
        QBrush red_brush(QColor(255,0,0));


        painter.setPen(red_pen);
        painter.setBrush(red_brush);
        for(std::list<PixelPosition>::iterator it=mpForegroundSeedMgr->mpSeeds->begin();it!=mpForegroundSeedMgr->mpSeeds->end();it++)
        {
            painter.drawPoint((*it).vals[0], (*it).vals[1]);
        }

        painter.setPen(blue_pen);
        painter.setBrush(blue_brush);
        for(std::list<PixelPosition>::iterator it=mpBackgroundSeedMgr->mpSeeds->begin();it!=mpBackgroundSeedMgr->mpSeeds->end();it++)
        {
            painter.drawPoint((*it).vals[0], (*it).vals[1]);
        }
    }
    else if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
    {
        if(mpInitPoint->x()!= mpEndPoint->x() || mpInitPoint->y()!=mpEndPoint->y())
        {
            QPen green_pen(QColor(0,255,0));
            painter.setPen(green_pen);

            mRectStartX = mpInitPoint->x()<= mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            mRectStartY = mpInitPoint->y()<= mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();
            mRectEndX   = mpInitPoint->x()> mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            mRectEndY   = mpInitPoint->y()> mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();

            painter.drawRect(mRectStartX, mRectStartY, mRectEndX - mRectStartX, mRectEndY - mRectStartY);
        }
    }

    exportPixmap.save(&exportFile, "PNG");
}
