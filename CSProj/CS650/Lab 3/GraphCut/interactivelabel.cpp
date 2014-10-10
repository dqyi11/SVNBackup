#include "interactivelabel.h"
#include <QtGui>

SeedManager::SeedManager()
{
    mpSeeds = new std::list<QPoint>();
}

SeedManager::~SeedManager()
{
    if (mpSeeds)
    {
        mpSeeds->clear();
        delete mpSeeds;
        mpSeeds = NULL;
    }
}

void SeedManager::clear()
{
    if(mpSeeds)
    {
        mpSeeds->clear();
    }
}

bool SeedManager::hasSeed(int x, int y)
{
    for(std::list<QPoint>::iterator it=mpSeeds->begin();it!=mpSeeds->end();it++)
    {
        if(it->x()==x && it->y()==y)
        {
            return true;
        }
    }
    return false;
}

void SeedManager::addSeed(int x, int y)
{
    if(mpSeeds)
    {
        if(false == hasSeed(x,y))
        {
            mpSeeds->push_back(QPoint(x, y));
        }
    }
}

InteractiveLabel::InteractiveLabel(QWidget * parent) :
    QLabel(parent)
{
     mCurrentState = NORMAL;
     mPointerRadius = 2;

     mpForegroundSeedMgr = new SeedManager();
     mpBackgroundSeedMgr = new SeedManager();

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
    QLabel::~QLabel();
}

void InteractiveLabel::mouseReleaseEvent ( QMouseEvent * e )
{
    //qDebug() << "Release";
    setPixmap(mColorPixmap);
    mCurrentState = NORMAL;
    update();
}

void InteractiveLabel::mousePressEvent ( QMouseEvent * e )
{
    if (e->button()==Qt::LeftButton)
    {
        //qDebug() << "Left";
        mCurrentState = ADD_FOREGROUND;
        setPixmap(mGrayPixmap);
        update();

    }
    else if(e->button()==Qt::RightButton)
    {
        //qDebug() << "Right";
        mCurrentState = ADD_BACKGROUND;
        setPixmap(mGrayPixmap);
        update();
    }
}

void InteractiveLabel::mouseMoveEvent( QMouseEvent * e )
{
    if(mCurrentState == ADD_FOREGROUND)
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
    else if(mCurrentState == ADD_BACKGROUND)
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

void InteractiveLabel::paintEvent(QPaintEvent* e)
{
    QLabel::paintEvent(e);

    QPainter painter(this);
    QPen blue_pen(QColor(0,0,255));
    QBrush blue_brush(QColor(0,0,255));
    QPen red_pen(QColor(255,0,0));
    QBrush red_brush(QColor(255,0,0));


    painter.setPen(red_pen);
    painter.setBrush(red_brush);
    for(std::list<QPoint>::iterator it=mpForegroundSeedMgr->mpSeeds->begin();it!=mpForegroundSeedMgr->mpSeeds->end();it++)
    {
        int x_pos = (*it).x();
        int y_pos = (*it).y();
        painter.drawPoint(x_pos, y_pos);
    }

    painter.setPen(blue_pen);
    painter.setBrush(blue_brush);
    for(std::list<QPoint>::iterator it=mpBackgroundSeedMgr->mpSeeds->begin();it!=mpBackgroundSeedMgr->mpSeeds->end();it++)
    {
        int x_pos = (*it).x();
        int y_pos = (*it).y();
        painter.drawPoint(x_pos, y_pos);
    }

}
