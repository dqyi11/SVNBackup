#include "segmentation.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <iostream>
#include "imagedatagraph.h"
#include "qdebug.h"

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
        if(it->vals[0]==x && it->vals[1]==y)
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
            PixelPosition pixel_pos;
            pixel_pos.vals[0] = x;
            pixel_pos.vals[1] = y;
            mpSeeds->push_back(pixel_pos);
        }
    }
}

Segmentation::Segmentation(const char* filename, SeedManager * foreground, SeedManager * background)
{
    std::cout << "Assigning ... " << filename << std::endl;
    mpFilename = new char[strlen(filename)+1];
    strcpy(mpFilename, filename);

    mForegroundSet.clear();
    mBackgroundSet.clear();

    for(std::list<PixelPosition>::iterator it=foreground->mpSeeds->begin();it!=foreground->mpSeeds->end();it++)
    {
        mForegroundSet.push_back(*it);
    }
    for(std::list<PixelPosition>::iterator it=background->mpSeeds->begin();it!=background->mpSeeds->end();it++)
    {
        mBackgroundSet.push_back(*it);
    }
}

Segmentation::~Segmentation()
{
}

void Segmentation::process()
{
    ImageDataGraph * pGraph = new ImageDataGraph(mpFilename);

    pGraph->importPrior(mForegroundSet, mBackgroundSet);

    int flow =  pGraph->maxFlowCut();
    qDebug() << "Flow: " << flow;

    mForegroundSet.clear();
    mBackgroundSet.clear();
    for(int j=0;j<pGraph->mImgHeight;j++)
    {
        for(int i=0;i<pGraph->mImgWidth;i++)
        {
            PixelPosition pos;
            pos.vals[0] = i;
            pos.vals[1] = j;
            int node_id = i + j * pGraph->mImgWidth;
            if (pGraph->mpGraph->what_segment(node_id) == PixelGraph::SOURCE)
            {
                mForegroundSet.push_back(pos);
            }
            else
            {
                mBackgroundSet.push_back(pos);
            }
        }
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

    for(std::list<PixelPosition>::iterator it=mForegroundSet.begin();it!=mForegroundSet.end();it++)
    {
        ((uchar *)(imgData->imageData + it->vals[1]*imgData->widthStep))[it->vals[0]*imgData->nChannels + 0] = 255;
        ((uchar *)(imgData->imageData + it->vals[1]*imgData->widthStep))[it->vals[0]*imgData->nChannels + 1] = 255;
        ((uchar *)(imgData->imageData + it->vals[1]*imgData->widthStep))[it->vals[0]*imgData->nChannels + 2] = 255;
    }
    for(std::list<PixelPosition>::iterator it=mBackgroundSet.begin();it!=mBackgroundSet.end();it++)
    {
        ((uchar *)(imgData->imageData + it->vals[1]*imgData->widthStep))[it->vals[0]*imgData->nChannels + 1] = 255;
        ((uchar *)(imgData->imageData + it->vals[1]*imgData->widthStep))[it->vals[0]*imgData->nChannels + 2] = 255;
    }

    std::string newFilename(mpFilename);
    newFilename += "-vis.png";
    //cvSaveImage(newFilename.c_str(), imgData);
    cvNamedWindow(newFilename.c_str()); //create a window with the name "MyWindow"
    cvShowImage(newFilename.c_str(), imgData);

    std::cout << "Writing file ... " << std::endl;

    cvReleaseImage(&img);
    cvReleaseImage(&imgData);

}
