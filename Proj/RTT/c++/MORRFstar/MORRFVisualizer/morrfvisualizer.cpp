#include "morrfvisualizer.h"
#include <QtGui>

MORRFVisualizer::MORRFVisualizer(QWidget *parent) :
    QLabel(parent)
{
    mpMORRF = NULL;

}

void MORRFVisualizer::setMORRF(MORRF* pMorrf)
{

}

void MORRFVisualizer::paintEvent(QPaintEvent * e)
{
    QLabel::paintEvent(e);

    if(mMOPPInfo.mStart.x() >= 0 && mMOPPInfo.mStart.y() >= 0)
    {
        QPainter painter(this);
        QPen paintpen(QColor(255,0,0));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(mMOPPInfo.mStart);
    }

    if(mMOPPInfo.mGoal.x() >= 0 && mMOPPInfo.mGoal.y() >= 0)
    {
        QPainter painter(this);
        QPen paintpen(QColor(0,0,255));
        paintpen.setWidth(10);
        painter.setPen(paintpen);
        painter.drawPoint(mMOPPInfo.mGoal);
    }
}
