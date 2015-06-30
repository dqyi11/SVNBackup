#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>
#include <list>

class MultiObjPathPlanningInfo
{
public:
    MultiObjPathPlanningInfo();

    int** getObstacleInfo();

    int** getPixInfo(QString filename);

    QString mInfoFilename;
    QString mMapFilename;
    int mObjectiveNum;
    QPoint mStart;
    QPoint mGoal;

    bool mMinDistEnabled;
    std::list<QString> mObjectiveFiles;
};

#endif // MULTIOBJPATHPLANNINGINFO_H
