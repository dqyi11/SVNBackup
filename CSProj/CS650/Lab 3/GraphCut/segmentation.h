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
    Segmentation(const char* filename);
    ~Segmentation();

    void visualize();
    virtual void process() = 0;
protected:
    char * mpFilename;
    std::list<PixelPosition> mForegroundSet;
    std::list<PixelPosition> mBackgroundSet;
};

class GraphCutSegmentation : public Segmentation
{
public:
    GraphCutSegmentation(const char* filename, SeedManager * foreground, SeedManager * background);

    virtual void process();
};

class GrabCutSegmentation : public Segmentation
{
public:
    GrabCutSegmentation(const char* filename, int rect_x, int rect_y, int rect_w, int rect_h);

    virtual void process();

    int mRectUpperLeftX;
    int mRectUpperLeftY;
    int mRectLowerRightX;
    int mRectLowerRightY;
};

#endif // SEGMENTATION_H
