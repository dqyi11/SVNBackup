#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>
#include <list>
#include <vector>

class MultiObjPathPlanningInfo
{
public:
    MultiObjPathPlanningInfo();

    int** getObstacleInfo();
    std::vector<int**> getFitnessDistributions();

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
