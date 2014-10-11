#include "segmentation.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <iostream>

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

Segmentation::Segmentation(const char* filename, SeedManager * foreground, SeedManager * background)
{
    std::cout << "Assigning ... " << filename << std::endl;
    mpFilename = new char[strlen(filename)+1];
    strcpy(mpFilename, filename);

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

void Segmentation:: visualize()
{

    std::cout << "Loading ... " << mpFilename << std::endl;

    IplImage* img = cvLoadImage(mpFilename);
    if(img->imageData==NULL)
    {
        std::cout << "Return " << std::endl;
        return;
    }
    std::cout << "generating Matrix ... " << img->width << " and "  << img->height << std::endl;

    IplImage * imgData = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);

    for(std::list<PixelPosition>::iterator it=mpForegroundSet->begin();it!=mpForegroundSet->end();it++)
    {
        //std::cout << imgData.at<uchar>(it->x, it->y) << std::endl;
        //std::cout << it->x << " " << it->y << " " << (int)imgData.at<uchar>(it->x, it->y) << std::endl;
        ((uchar *)(imgData->imageData + it->y*imgData->widthStep))[it->x*imgData->nChannels + 0] = 255;
        ((uchar *)(imgData->imageData + it->y*imgData->widthStep))[it->x*imgData->nChannels + 1] = 255;
        ((uchar *)(imgData->imageData + it->y*imgData->widthStep))[it->x*imgData->nChannels + 2] = 255;
    }
    for(std::list<PixelPosition>::iterator it=mpBackgroundSet->begin();it!=mpBackgroundSet->end();it++)
    {
        // imgData.at<uchar>(it->x, it->y) = (uchar)122;
        //std::cout << imgData.at<uchar>(it->x, it->y) << std::endl;
        //std::cout << it->x << " " << it->y << " " << imgData.at<uchar>(it->x, it->y) << std::endl;
        ((uchar *)(imgData->imageData + it->y*imgData->widthStep))[it->x*imgData->nChannels + 1] = 255;
        ((uchar *)(imgData->imageData + it->y*imgData->widthStep))[it->x*imgData->nChannels + 2] = 255;
    }

    std::string newFilename(mpFilename);
    newFilename += "-vis.png";
    cvSaveImage(newFilename.c_str(), imgData);

    std::cout << "Writing file ... " << std::endl;

    cvReleaseImage(&img);
    cvReleaseImage(&imgData);

}
