#ifndef DISCRETEPATH_H
#define DISCRETEPATH_H

#include <QList>
#include <QPointF>

class DiscretePath
{
public:
    DiscretePath();
    int length();
    void addPoint(double x, double y);

    DiscretePath & operator=(const DiscretePath & a);

    QList<QPointF> mSequence;
};

#endif // DISCRETEPATH_H
