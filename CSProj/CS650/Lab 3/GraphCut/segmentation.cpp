#include "segmentation.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <iostream>
#include "imagedatagraph.h"
#include "qdebug.h"

#define GET_MAX(a, b) a>b?a:b
#define GET_MIN(a, b) a<=b?a:b

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

Segmentation::Segmentation(const char* filename, int width, int height)
{
    //std::cout << "Assigning ... " << filename << std::endl;
    mpFilename = new char[strlen(filename)+1];
    strcpy(mpFilename, filename);

    mForegroundSet.clear();
    mBackgroundSet.clear();

    mImgWidth = width;
    mImgHeight = height;

    mpTrimap = new int[mImgWidth * mImgHeight];
 }

Segmentation::~Segmentation()
{
    if(mpFilename)
    {
        delete mpFilename;
        mpFilename = NULL;
    }
    mForegroundSet.clear();
    mBackgroundSet.clear();
}

void Segmentation:: visualize(bool includeMask)
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

    for(int j=0;j<imgData->height;j++)
    {
        for(int i=0;i<imgData->width;i++)
        {
            int node_id = i + j * imgData->width;
            if (mpTrimap[node_id] == ImageDataGraph::FOREGROUND_PIXEL)
            {
                ((uchar *)(imgData->imageData + j*imgData->widthStep))[i*imgData->nChannels + 0] = ((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 0];
                ((uchar *)(imgData->imageData + j*imgData->widthStep))[i*imgData->nChannels + 1] = ((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 1];
                ((uchar *)(imgData->imageData + j*imgData->widthStep))[i*imgData->nChannels + 2] = ((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 2];
            }
            else
            {
                ((uchar *)(imgData->imageData + j*imgData->widthStep))[i*imgData->nChannels + 0] = 255;
                ((uchar *)(imgData->imageData + j*imgData->widthStep))[i*imgData->nChannels + 1] = 255;
                ((uchar *)(imgData->imageData + j*imgData->widthStep))[i*imgData->nChannels + 2] = 255;
            }
        }
    }

    if(includeMask==true)
    {
        IplImage * imgMaskData = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);

        for(int j=0;j<imgMaskData->height;j++)
        {
            for(int i=0;i<imgMaskData->width;i++)
            {
                int node_id = i + j * imgData->width;
                if (mpTrimap[node_id] == ImageDataGraph::FOREGROUND_PIXEL)
                {
                    ((uchar *)(imgMaskData->imageData + j*imgMaskData->widthStep))[i*imgMaskData->nChannels + 0] = 153;
                    ((uchar *)(imgMaskData->imageData + j*imgMaskData->widthStep))[i*imgMaskData->nChannels + 1] = 255;
                    ((uchar *)(imgMaskData->imageData + j*imgMaskData->widthStep))[i*imgMaskData->nChannels + 2] = 51;

                }
                else if (mpTrimap[node_id] == ImageDataGraph::BACKGROUND_PIXEL)
                {
                    ((uchar *)(imgMaskData->imageData + j*imgMaskData->widthStep))[i*imgMaskData->nChannels + 0] = 102;
                    ((uchar *)(imgMaskData->imageData + j*imgMaskData->widthStep))[i*imgMaskData->nChannels + 1] = 102;
                    ((uchar *)(imgMaskData->imageData + j*imgMaskData->widthStep))[i*imgMaskData->nChannels + 2] = 255;
                }
            }
        }

        std::string newMaskFilename(mpFilename);
        newMaskFilename += "-mask.png";

        cvNamedWindow(newMaskFilename.c_str());
        cvShowImage(newMaskFilename.c_str(), imgMaskData);
        cvSaveImage(newMaskFilename.c_str(), imgMaskData);

        cvReleaseImage(&imgMaskData);
    }

    std::string newFilename(mpFilename);
    newFilename += "-foreground.png";

    cvNamedWindow(newFilename.c_str());
    cvShowImage(newFilename.c_str(), imgData);
    cvSaveImage(newFilename.c_str(), imgData);

    std::cout << "Writing file ... " << std::endl;

    cvReleaseImage(&img);
    cvReleaseImage(&imgData);

}

GraphCutSegmentation::GraphCutSegmentation(const char* filename, int width, int height, SeedManager * foreground, SeedManager * background) : Segmentation(filename, width, height)
{
    for(std::list<PixelPosition>::iterator it=foreground->mpSeeds->begin();it!=foreground->mpSeeds->end();it++)
    {
        mForegroundSet.push_back(*it);
        int index = it->vals[0] + it->vals[1]* mImgWidth;
        mpTrimap[index] = ImageDataGraph::FOREGROUND_PIXEL;
    }
    for(std::list<PixelPosition>::iterator it=background->mpSeeds->begin();it!=background->mpSeeds->end();it++)
    {
        mBackgroundSet.push_back(*it);
        int index = it->vals[0] + it->vals[1]* mImgWidth;
        mpTrimap[index] = ImageDataGraph::BACKGROUND_PIXEL;
    }
}


void GraphCutSegmentation::process(EstimatorType type)
{
    qDebug() << "Create graph from " << mpFilename;
    ImageDataGraph * pGraph = new ImageDataGraph(mpFilename, type);
    if(type==KDE)
    {
        pGraph->mSigmaKDE = mKDESigma;
    }
    pGraph->mpGridPrior = mpTrimap;
    qDebug() << "Import prior, foreground num " << mForegroundSet.size() << " and background num " << mBackgroundSet.size();

    pGraph->initalizeType(type);
    pGraph->importPrior(mForegroundSet, mBackgroundSet);

    pGraph->initializeGraph();

    qDebug() << "Graph cutting ";

    int flow =  pGraph->maxFlowCut();
    qDebug() << "Flow: " << flow;

    for(int j=0;j<pGraph->mImgHeight;j++)
    {
        for(int i=0;i<pGraph->mImgWidth;i++)
        {
            int node_id = i + j * pGraph->mImgWidth;
            if (pGraph->mpGraph->what_segment(node_id) == PixelGraph::SOURCE)
            {
                mpTrimap[node_id] = ImageDataGraph::FOREGROUND_PIXEL;
            }
            else
            {
                mpTrimap[node_id] = ImageDataGraph::BACKGROUND_PIXEL;
            }
        }
    }

    if(pGraph)
    {
        delete pGraph;
        pGraph = NULL;
    }
}

GrabCutSegmentation::GrabCutSegmentation(const char* filename, int width, int height, int rect_x, int rect_y, int rect_w, int rect_h) : Segmentation(filename, width, height)
{
    mRectUpperLeftX = rect_x;
    mRectUpperLeftY = rect_y;
    mRectLowerRightX = rect_x + rect_w;
    mRectLowerRightY = rect_y + rect_h;

    mIterationNum = 5;
}

void GrabCutSegmentation::initalizeSeeds(int img_width, int img_height, int rect_min_x, int rect_min_y, int rect_max_x, int rect_max_y, float inner_ratio, float outer_ratio)
{
    int rect_w = rect_max_x - rect_min_x;
    int rect_h = rect_max_y - rect_min_y;
    int center_x = (int)((rect_min_x + rect_max_x) / 2);
    int center_y = (int)((rect_min_y + rect_max_y) / 2);

    int foreground_seed_min_x = center_x - (int)(rect_w * inner_ratio);
    int foreground_seed_max_x = center_x + (int)(rect_w * inner_ratio);
    int foreground_seed_min_y = center_y - (int)(rect_h * inner_ratio);
    int foreground_seed_max_y = center_y + (int)(rect_h * inner_ratio);

    int background_seed_min_x = GET_MIN(0, rect_min_x - (int)(rect_w * outer_ratio));
    int background_seed_max_x = GET_MAX(mImgWidth, rect_max_x + (int)(rect_w * outer_ratio));
    int background_seed_min_y = GET_MIN(0, rect_min_y - (int)(rect_h * outer_ratio));
    int background_seed_max_y = GET_MAX(mImgHeight-1, rect_max_y + (int)(rect_h * outer_ratio));
    for(int j=0;j<img_height;j++)
    {
        for(int i=0;i<img_width;i++)
        {
            PixelPosition pos;
            pos.vals[0] = i;
            pos.vals[1] = j;

            if(i>=foreground_seed_min_x  && i<foreground_seed_max_x && j>=foreground_seed_min_y && j<foreground_seed_max_y)
            {
                mForegroundSet.push_back(pos);
            }
            else if( ( (i>=background_seed_min_x && i<rect_min_x) || (i>=rect_max_x && i<background_seed_max_x) )
                     && ( (j>=background_seed_min_y && j<rect_min_y) || (j>=rect_max_y && j<background_seed_max_y) )  )
            {
                mBackgroundSet.push_back(pos);
            }
        }
    }
}

void GrabCutSegmentation::process(EstimatorType type)
{
    qDebug() << "Create graph from " << mpFilename;
    ImageDataGraph * pGraph = new ImageDataGraph(mpFilename, type);
    if(type==KDE)
    {
        pGraph->mSigmaKDE = mKDESigma;
    }

    mpTrimap = new int[pGraph->mImgWidth*pGraph->mImgHeight];
    for(int j=0;j<pGraph->mImgHeight;j++)
    {
        for(int i=0;i<pGraph->mImgWidth;i++)
        {
            if(i>=mRectUpperLeftX && i<mRectLowerRightX && j>=mRectUpperLeftY && j<mRectLowerRightY)
            {
                mpTrimap[i+j*pGraph->mImgWidth] = ImageDataGraph::UNKNOWN_PIXEL;
            }
            else
            {
                mpTrimap[i+j*pGraph->mImgWidth] = ImageDataGraph::BACKGROUND_PIXEL;
            }
        }
    }

    pGraph->initalizeType(type);
    initalizeSeeds(pGraph->mImgWidth, pGraph->mImgHeight, mRectUpperLeftX, mRectUpperLeftY, mRectLowerRightX, mRectLowerRightY, 0.1, 0.1);

    int iterationCnt = 0;
    while(iterationCnt < mIterationNum)
    {
        if(pGraph)
        {
            delete pGraph;
            pGraph = NULL;
        }
        pGraph = new ImageDataGraph(mpFilename);
        pGraph->mpGridPrior = mpTrimap;
        pGraph->importPrior(mForegroundSet, mBackgroundSet);
        pGraph->initializeGraph();

        qDebug() << "Iteration " << iterationCnt << " T: " << pGraph->getGibbsEnergy() << " D: " << pGraph->getDataEnergy() << " S: " << pGraph->getSmoothnessEnergy();

        int flow =  pGraph->maxFlowCut();

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
                if (pGraph->mpGraph->what_segment(node_id) == PixelGraph::SINK)
                {
                    mpTrimap[node_id] = ImageDataGraph::FOREGROUND_PIXEL;
                    mForegroundSet.push_back(pos);
                }
                else
                {
                    mpTrimap[node_id] = ImageDataGraph::BACKGROUND_PIXEL;
                    mBackgroundSet.push_back(pos);
                }
            }
        }

        iterationCnt ++;
    }

    if(pGraph)
    {
        delete pGraph;
        pGraph = NULL;
    }

}
