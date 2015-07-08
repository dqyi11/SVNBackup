#include "multiobjpathplanninginfo.h"
#include <QPixmap>

MultiObjPathPlanningInfo::MultiObjPathPlanningInfo()
{
    mInfoFilename = "";
    mMapFilename = "";
    mObjectiveNum = 0;

    mStart.setX(-1);
    mStart.setY(-1);
    mGoal.setX(-1);
    mGoal.setY(-1);

    mMinDistEnabled = false;

    mSubproblemNum = 30;
    mMaxIterationNum = 100;
    mSegmentLength = 5.0;

    mMapWidth = 0;
    mMapHeight = 0;
}

bool MultiObjPathPlanningInfo::getObstacleInfo(int** obstacleInfo)
{
    if(obstacleInfo==NULL)
    {
        return false;
    }
    return getPixInfo(mMapFilename, obstacleInfo);
}

std::vector<int**> MultiObjPathPlanningInfo::getFitnessDistributions()
{
    std::vector<int**> fitnessDistributions;
    for(std::list<QString>::iterator it=mObjectiveFiles.begin();it!=mObjectiveFiles.end();it++)
    {
        QString fitnessName = (*it);
        int** fitness = new int*[mMapWidth];
        for(int i=0;i<mMapWidth;i++)
        {
            fitness[i] = new int[mMapHeight];
        }
        getPixInfo(fitnessName, fitness);
        fitnessDistributions.push_back(fitness);
    }
    return fitnessDistributions;
}

bool MultiObjPathPlanningInfo::getPixInfo(QString filename, int ** pixInfo)
{
    if(pixInfo==NULL)
    {
        return false;
    }
    QPixmap map(filename);
    QImage grayImg = map.toImage();
    int width = map.width();
    int height = map.height();

    for(int i=0;i<width;i++)
    {
        for(int j=0;j<height;j++)
        {
            QRgb col = grayImg.pixel(i,j);
            pixInfo[i][j] = qGray(col);
        }
    }
    return true;
}

void MultiObjPathPlanningInfo::initFuncsParams()
{
    mFuncs.clear();
    mDistributions.clear();

    std::vector<int**> fitnessDistributions = getFitnessDistributions();

    if(mMinDistEnabled==true)
    {
        mFuncs.push_back(MultiObjPathPlanningInfo::calcDist);
        mDistributions.push_back(NULL);

        for(int k=0;k<mObjectiveNum-1;k++)
        {
            mFuncs.push_back(MultiObjPathPlanningInfo::calcCost);
            mDistributions.push_back(fitnessDistributions[k]);
        }
    }
    else
    {
        for(int k=0;k<mObjectiveNum;k++)
        {
            mFuncs.push_back(MultiObjPathPlanningInfo::calcCost);
            mDistributions.push_back(fitnessDistributions[k]);
        }
    }
}


