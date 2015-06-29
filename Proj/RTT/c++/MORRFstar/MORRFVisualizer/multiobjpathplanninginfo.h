#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>

class MultiObjPathPlanningInfo
{
public:
    MultiObjPathPlanningInfo();

    QString mInfoFilename;
    QString mMapFilename;
    int mObjectiveNum;
    QPoint mStart;
    QPoint mGoal;
};

#endif // MULTIOBJPATHPLANNINGINFO_H
