#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <list>
//#include <QPoint>
#include "globaldef.h"

typedef struct
{
    int vals[2];
}PixelPosition;

typedef struct
{
    int vals[3];
}PixelColor;

class SeedManager
{
public:
    SeedManager();
    ~SeedManager();

    void clear();
    void addSeed(int x, int y);
    bool hasSeed(int x, int y);

    std::list<PixelPosition> * mpSeeds;
};

class Segmentation
{
public:
    Segmentation(const char* filename, int width, int height);
    ~Segmentation();

    void visualize(bool includeMask=true);
    virtual void process(EstimatorType type=KDE) = 0;
protected:
    char * mpFilename;
    int mImgWidth;
    int mImgHeight;
    std::list<PixelPosition> mForegroundSet;
    std::list<PixelPosition> mBackgroundSet;
    int * mpTrimap;
};



class GraphCutSegmentation : public Segmentation
{
public:
    GraphCutSegmentation(const char* filename, int width, int height, SeedManager * foreground, SeedManager * background);

    virtual void process(EstimatorType type=KDE);

    float mKDESigma;
};

class GrabCutSegmentation : public Segmentation
{
public:
    GrabCutSegmentation(const char* filename, int width, int height, int rect_min_x, int rect_min_y, int rect_max_x, int rect_max_y);

    virtual void process(EstimatorType type=KDE);

    int mRectUpperLeftX;
    int mRectUpperLeftY;
    int mRectLowerRightX;
    int mRectLowerRightY;

    float mKDESigma;
    int mIterationNum;

protected:
    void initalizeSeeds(int img_width, int img_height, int rect_min_x, int rect_min_y, int rect_max_x, int rect_max_y, float inner_ratio, float outer_ratio);
};

#endif // SEGMENTATION_H
