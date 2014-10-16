#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <list>
#include <QPoint>

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
    Segmentation(const char* filename, int width, int height, float regionImportance);
    ~Segmentation();

    void visualize(bool includeMask=true);
    virtual void process(float sigma_nb, float sigma_kde) = 0;
protected:
    char * mpFilename;
    int mImgWidth;
    int mImgHeight;
    std::list<PixelPosition> mForegroundSet;
    std::list<PixelPosition> mBackgroundSet;
    int * mpTrimap;
    float mRegionImportance;
};

class GraphCutSegmentation : public Segmentation
{
public:
    GraphCutSegmentation(const char* filename, int width, int height, float regionImportance, SeedManager * foreground, SeedManager * background);

    virtual void process(float sigma_nb, float sigma_kde);
};

class GrabCutSegmentation : public Segmentation
{
public:
    GrabCutSegmentation(const char* filename, int width, int height, float regionImportance, int rect_x, int rect_y, int rect_w, int rect_h);

    virtual void process(float sigma_nb, float sigma_kde);

    int mRectUpperLeftX;
    int mRectUpperLeftY;
    int mRectLowerRightX;
    int mRectLowerRightY;

    int mIterationNum;
};

#endif // SEGMENTATION_H
