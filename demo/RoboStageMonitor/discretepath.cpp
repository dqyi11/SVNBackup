#include "discretepath.h"

DiscretePath::DiscretePath()
{
    mSequence = QList<QPointF> ();
}

int DiscretePath::length()
{
    return mSequence.length();
}

void DiscretePath::addPoint(double x, double y)
{
    QPointF * pPoint = new QPointF(x, y);
    mSequence.append(*pPoint);
}

DiscretePath & DiscretePath::operator=(const DiscretePath & a)
{
    mSequence.clear();
    for(int i=0;i<a.mSequence.length();i++)
    {
        QPointF * pPoint = new QPointF(a.mSequence[i]);
        this->mSequence.append(*pPoint);
    }
}
