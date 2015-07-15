#ifndef MULTIOBJPATHPLANNINGINFO_H
#define MULTIOBJPATHPLANNINGINFO_H

#include <QString>
#include <QPoint>
#include <QJsonObject>
#include <list>
#include <vector>
#include <qDebug>
#include <math.h>

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
        if (pos_a == pos_b)
            return dist;
        double delta_x = abs(pos_a[0]-pos_b[0]);
        double delta_y = abs(pos_a[1]-pos_b[1]);
        dist = sqrt(delta_x*delta_x+delta_y*delta_y);
        //dist = (delta_x*delta_x+delta_y*delta_y);
        if(dist < 0.0)
        {
            qWarning() << "Dist negative " << dist ;
        }
        return dist;
    }

    static double calcCost(POS2D pos_a, POS2D pos_b, int** distribution)
    {
        double cost = 0.0;
        if (pos_a == pos_b)
            return cost;
        if(distribution==NULL)
            return cost;

        //int width = sizeof(distribution)/sizeof(distribution[0]);
        //int height = sizeof(distribution[0])/sizeof(int);
        //int width = sizeof(distribution)/sizeof(*distribution);
        //int height = sizeof(*distribution)/sizeof(*distribution[0]);

        double x_dist = pos_a[0]-pos_b[0];
        double y_dist = pos_a[1]-pos_b[1];
        if (abs(x_dist) > abs(y_dist))
        {
            int startX = 0, endX = 0, startY = 0, endY = 0;
            double k = y_dist / x_dist;
            if (pos_a[0] < pos_b[0])
            {
                startX = (int)floor(pos_a[0]);
                endX = (int)floor(pos_b[0]);
                startY = (int)floor(pos_a[1]);
                endY = (int)floor(pos_b[1]);
            }
            else
            {
                startX = (int)floor(pos_b[0]);
                endX = (int)floor(pos_a[0]);
                startY = (int)floor(pos_b[1]);
                endY = (int)floor(pos_a[1]);
            }
            for(int coordX = startX; coordX < endX; coordX++)
            {
                int coordY = (int)floor(k*(coordX-startX)+startY);
                //if (coordX < pos || coordY >= height)
                {
                    double fitnessVal = (double)distribution[coordX][coordY];
                    if(fitnessVal < 0)
                    {
                        qWarning() << "Cost negative " << fitnessVal;
                    }
                    cost += fitnessVal/255.0;
                }
            }
        }
        else
        {
            int startY = 0, endY = 0, startX = 0, endX = 0;
            double k = x_dist / y_dist;
            if (pos_a[0] < pos_b[0])
            {
                startY = (int)floor(pos_a[1]);
                endY = (int)floor(pos_b[1]);
                startX = (int)floor(pos_a[0]);
                endX = (int)floor(pos_b[0]);
            }
            else
            {
                startY = (int)floor(pos_b[1]);
                endY = (int)floor(pos_a[1]);
                startX = (int)floor(pos_b[0]);
                endX = (int)floor(pos_a[0]);
            }
            for(int coordY = startY; coordY < endY; coordY++)
            {
                int coordX = (int)floor(k*(coordY-startY)+startX);
                //if (coordX >= width || coordY >= height)
                {
                    double fitnessVal = (double)distribution[coordX][coordY];
                    if(fitnessVal < 0)
                    {
                        qWarning() << "Cost negative " << fitnessVal;
                    }
                    cost += fitnessVal/255.0;
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

    MORRF::MORRF_TYPE mMethodType;

    std::vector<Path*> mFoundPaths;
};

#endif // MULTIOBJPATHPLANNINGINFO_H
