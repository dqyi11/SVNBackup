#include "mapviewer.h"
#include <QPainter>
#include <QtCore/qmath.h>

MapViewer::MapViewer(QWidget *parent) :
    QLabel(parent)
{
   mPoint = new QPoint();
   mShowPoint = false;
   mpPath = new DiscretePath();
}


void MapViewer::paintEvent(QPaintEvent * e)
{
    QLabel::paintEvent(e);

    if(mShowPoint)
    {
        QPainter painter(this);
        painter.setPen(QPen(Qt::red));
        painter.setBrush(QBrush(Qt::red, Qt::SolidPattern));
        painter.drawEllipse(*mPoint, 5 ,5);
        qDebug("paint x %d y %d", mPoint->x(), mPoint->y());
        double orientX = mPoint->x() + 10.0 * qCos(mPointYaw * 3.141592/180.0);
        double orientY = mPoint->y() + 10.0 * qSin(mPointYaw * 3.141592/180.0);
        QPointF orientPoint(orientX, orientY);
        painter.setPen(QPen(Qt::blue));
        painter.drawLine(*mPoint, orientPoint);
    }

    if(mpPath)
    {
        for(int i=0;i< mpPath->length();i++)
        {
            QPointF pointf = mpPath->mSequence[i];
            QPoint point((int)pointf.x(), (int)pointf.y());
            QPainter painter(this);
            painter.setPen(QPen(Qt::green));
            painter.setBrush(QBrush(Qt::green, Qt::SolidPattern));
            painter.drawEllipse(point, 2 ,2);
            //qDebug("point x %d y %d", point.x(), point.y());
        }
    }

}

void MapViewer::setPoint(int x, int y)
{
    if(mPoint)
    {
        mPoint->setX(x);
        mPoint->setY(y);
    }
}

void MapViewer::showPoint(bool show)
{
    mShowPoint = show;
}

void MapViewer::loadPath(DiscretePath path)
{
    *mpPath = path;
}

void MapViewer::setPointYaw(double yaw)
{
    mPointYaw = yaw;
}
