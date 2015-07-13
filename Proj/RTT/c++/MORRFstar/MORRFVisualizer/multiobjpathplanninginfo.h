#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>
#include <QJsonObject>
#include <list>
#include <vector>

#include "morrf.h"

class MultiObjPathPlanningInfo
{
public:
    MultiObjPathPlanningInfo();

    bool getObstacleInfo(int** obstacleInfo);
    std::vector<int**> getFitnessDistributions();

    bool getPixInfo(QString filename, int** pixInfo);
    void initFuncsParams();

    bool saveToFile(QString filename);
    bool loadFromFile(QString filename);

    void read(const QJsonObject &json);
    void write(QJsonObject &json) const;

    void loadPaths(std::vector<Path*> paths);

    static double calcDist(POS2D pos_a, POS2D pos_b, int** distribution)
    {
        double dist = 0.0;
        if (pos_a[0]==pos_b[0] || pos_a[1]==pos_b[1])
            return dist;
        dist = std::sqrt(std::pow(pos_a[0]-pos_b[0],2)+std::pow(pos_a[1]-pos_b[1],2));
        return dist;
    }

    static double calcCost(POS2D pos_a, POS2D pos_b, int** distribution)
    {
        double cost = 0.0;
        if (pos_a[0]==pos_b[0] || pos_a[1]==pos_b[1])
            return cost;
        if(distribution==NULL)
            return cost;

        int width = sizeof(distribution)/sizeof(distribution[0]);
        int height = sizeof(distribution[0])/sizeof(int);

        double x_dist = pos_a[0]-pos_b[0];
        double y_dist = pos_a[1]-pos_b[1];
        if (std::abs(x_dist) > std::abs(y_dist))
        {
            double startX = 0.0, endX = 0.0, startY = 0.0;
            double k = y_dist / x_dist;
            if (pos_a[0] < pos_b[0])
            {
                startX = pos_a[0];
                endX = pos_b[0];
                startY = pos_a[1];
            }
            else
            {
                startX = pos_b[0];
                endX = pos_a[0];
                startY = pos_b[1];
            }
            for(int coordX =(int)startX; coordX <(int)endX; coordX++)
            {
                int coordY = (int)(k*(coordX-startX)+startY);
                if (coordX >= width || coordY >= height)
                {
                    cost += distribution[coordX][coordY]/255.0;
                }
            }
        }
        else
        {
            double startY = 0.0, endY = 0.0, startX = 0.0;
            double k = x_dist / y_dist;
            if (pos_a[0] < pos_b[0])
            {
                startY = pos_a[1];
                endY = pos_b[1];
                startX = pos_a[0];
            }
            else
            {
                startY = pos_b[1];
                endY = pos_a[1];
                startX = pos_b[0];
            }
            for(int coordY =(int)startY; coordY <(int)endY; coordY++)
            {
                int coordX = (int)(k*(coordY-startY)+startX);
                if (coordX >= width || coordY >= height)
                {
                    cost += distribution[coordX][coordY]/255.0;
                }
            }
        }
        return cost;
    }

    /* Member variables */
    QString mInfoFilename;
    QString mMapFilename;
    QString mMapFullpath;
    int mMapWidth;
    int mMapHeight;

    int mObjectiveNum;
    QPoint mStart;
    QPoint mGoal;

    bool mMinDistEnabled;
    std::vector<QString> mObjectiveFiles;

    std::vector<COST_FUNC_PTR> mFuncs;
    std::vector<int**>         mDistributions;

    int mSubproblemNum;
    int mMaxIterationNum;
    double mSegmentLength;

    std::vector<Path*> mFoundPaths;
};

#endif // MULTIOBJPATHPLANNINGINFO_H
