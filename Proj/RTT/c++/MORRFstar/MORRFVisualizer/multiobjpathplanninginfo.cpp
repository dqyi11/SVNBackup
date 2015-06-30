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

    mSubproblemNum = 10;
    mMaxIterationNum = 0;

    mMapWidth = 0;
    mMapHeight = 0;
}

int** MultiObjPathPlanningInfo::getObstacleInfo()
{
    return getPixInfo(mMapFilename);
}

std::vector<int**> MultiObjPathPlanningInfo::getFitnessDistributions()
{
    std::vector<int**> fitnessDistributions;
    for(std::list<QString>::iterator it=mObjectiveFiles.begin();it!=mObjectiveFiles.end();it++)
    {
        QString fitnessName = (*it);
        int** fitness = getPixInfo(fitnessName);
        fitnessDistributions.push_back(fitness);
    }
    return fitnessDistributions;
}

int** MultiObjPathPlanningInfo::getPixInfo(QString filename)
{
    QPixmap map(filename);
    QImage grayImg = map.toImage();
    int width = map.width();
    int height = map.height();
    int ** pValueArray = new int*[width];
    for(int i=0;i<width;i++)
    {
        pValueArray[i] = new int[height];
    }
    for(int i=0;i<width;i++)
    {
        for(int j=0;j<height;j++)
        {
            QRgb col = grayImg.pixel(i,j);
            pValueArray[i][j] = qGray(col);
        }
    }
    return pValueArray;
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
            mFuncs.push_back(MultiObjPathPlanningInfo::calcFitness);
            mDistributions.push_back(fitnessDistributions[k]);
        }
    }
    else
    {
        for(int k=0;k<mObjectiveNum;k++)
        {
            mFuncs.push_back(MultiObjPathPlanningInfo::calcFitness);
            mDistributions.push_back(fitnessDistributions[k]);
        }
    }
}


