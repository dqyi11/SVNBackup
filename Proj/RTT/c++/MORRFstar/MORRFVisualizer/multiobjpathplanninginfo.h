#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>
#include <list>
#include <vector>

#include "morrf.h"

class MultiObjPathPlanningInfo
{
public:
    MultiObjPathPlanningInfo();

    int** getObstacleInfo();
    std::vector<int**> getFitnessDistributions();

    int** getPixInfo(QString filename);
    void initFuncsParams();

    static double calcDist(POS2D pos_a, POS2D pos_b, int** distribution)
    {
        return 0.0;
    }

    static double calcFitness(POS2D pos_a, POS2D pos_b, int** distribution)
    {
        return 0.0;
    }

    QString mInfoFilename;
    QString mMapFilename;
    QString mMapFullpath;
    int mMapWidth;
    int mMapHeight;

    int mObjectiveNum;
    QPoint mStart;
    QPoint mGoal;

    bool mMinDistEnabled;
    std::list<QString> mObjectiveFiles;

    std::vector<COST_FUNC_PTR> mFuncs;
    std::vector<int**>         mDistributions;

    int mSubproblemNum;
    int mMaxIterationNum;
};

#endif // MULTIOBJPATHPLANNINGINFO_H
