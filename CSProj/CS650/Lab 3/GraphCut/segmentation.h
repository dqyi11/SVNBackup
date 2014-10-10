#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <list>
#include <QPoint>

typedef struct
{
  int x;
  int y;
}PixelPosition;

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
    Segmentation(SeedManager * foreground, SeedManager * background);
    ~Segmentation();

    std::list<PixelPosition> * mpForegroundSet;
    std::list<PixelPosition> * mpBackgroundSet;
};

#endif // SEGMENTATION_H
