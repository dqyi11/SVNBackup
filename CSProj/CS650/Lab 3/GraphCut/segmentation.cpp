#include "segmentation.h"

SeedManager::SeedManager()
{
    mpSeeds = new std::list<PixelPosition>();
}

SeedManager::~SeedManager()
{
    if (mpSeeds)
    {
        mpSeeds->clear();
        delete mpSeeds;
        mpSeeds = NULL;
    }
}

void SeedManager::clear()
{
    if(mpSeeds)
    {
        mpSeeds->clear();
    }
}

bool SeedManager::hasSeed(int x, int y)
{
    for(std::list<PixelPosition>::iterator it=mpSeeds->begin();it!=mpSeeds->end();it++)
    {
        if(it->x==x && it->y==y)
        {
            return true;
        }
    }
    return false;
}

void SeedManager::addSeed(int x, int y)
{
    if(mpSeeds)
    {
        if(false == hasSeed(x,y))
        {
            PixelPosition pos;
            pos.x = x;
            pos.y = y;
            mpSeeds->push_back(pos);
        }
    }
}

Segmentation::Segmentation(SeedManager * foreground, SeedManager * background)
{
    mpForegroundSet = new std::list<PixelPosition>();
    mpBackgroundSet = new std::list<PixelPosition>();

    for(std::list<PixelPosition>::iterator it=foreground->mpSeeds->begin();it!=foreground->mpSeeds->end();it++)
    {
        mpForegroundSet->push_back(*it);
    }
    for(std::list<PixelPosition>::iterator it=background->mpSeeds->begin();it!=background->mpSeeds->end();it++)
    {
        mpBackgroundSet->push_back(*it);
    }
}

Segmentation::~Segmentation()
{
    if(mpForegroundSet)
    {
        delete mpForegroundSet;
        mpForegroundSet = NULL;
    }
    if(mpBackgroundSet)
    {
        delete mpBackgroundSet;
        mpBackgroundSet = NULL;
    }
}
