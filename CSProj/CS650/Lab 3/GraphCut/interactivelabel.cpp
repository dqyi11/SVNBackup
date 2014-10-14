#include "interactivelabel.h"
#include <QtGui>
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

void InteractiveLabel::mouseReleaseEvent ( QMouseEvent * e )
{
    if(mCurrentWorkingState == GRAB_CUT_SEGMENTATION)
    {
        if(e->button()==Qt::LeftButton)
        {
            mpEndPoint->setX(e->x());
            mpEndPoint->setY(e->y());
        }
    }
    setPixmap(mColorPixmap);
    mCurrentLabelingState = NORMAL;
    update();
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
        if(e->button()==Qt::LeftButton)
        {
            mpEndPoint->setX(e->x());
            mpEndPoint->setY(e->y());
            update();
        }
    }
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

            int start_x = mpInitPoint->x()<= mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            int start_y = mpInitPoint->y()<= mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();
            int end_x   = mpInitPoint->x()> mpEndPoint->x() ?  mpInitPoint->x(): mpEndPoint->x();
            int end_y   = mpInitPoint->y()> mpEndPoint->y() ?  mpInitPoint->y(): mpEndPoint->y();

            painter.drawRect(start_x, start_y, end_x - start_x, end_y - start_y);
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
