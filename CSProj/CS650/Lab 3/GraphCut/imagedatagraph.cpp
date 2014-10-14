#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <math.h>

#include "imagedatagraph.h"
#include "qdebug.h"

ImageDataGraph::ImageDataGraph(const char* filename, float sigma_nb, float sigma_kde)
{
    mpFilename = const_cast<char *>(filename);

    IplImage * img_rgb = cvLoadImage(mpFilename);
    IplImage * img = cvCreateImage(cvGetSize(img_rgb), img_rgb->depth, img_rgb->nChannels);
    cvCvtColor(img_rgb, img, CV_RGB2Luv);
    mImgWidth = img->width;
    mImgHeight = img->height;
    mConnectNum = 4;

    mSigmaNeighborhood = sigma_nb;
    mSigmaKDE = sigma_kde;
    mMaxNeighborhood = 0.0;

    mpForegroundEstimator = new GaussianKernelDensityEstimator(mSigmaKDE);
    mpBackgroundEstimator = new GaussianKernelDensityEstimator(mSigmaKDE);

    mpRVals = new int[mImgWidth*mImgHeight];
    mpGVals = new int[mImgWidth*mImgHeight];
    mpBVals = new int[mImgWidth*mImgHeight];

    mpGridPrior = NULL;

    mpGraph = new PixelGraph(mImgWidth*mImgHeight, mImgWidth*mImgHeight*mConnectNum);

    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            mpRVals[j*mImgWidth+i] = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 0];
            mpGVals[j*mImgWidth+i] = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 1];
            mpBVals[j*mImgWidth+i] = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 2];
        }
    }

    cvReleaseImage(&img);
    cvReleaseImage(&img_rgb);
}

ImageDataGraph::~ImageDataGraph()
{
    mpFilename = NULL;
    if(mpGraph)
    {
        delete mpGraph;
        mpGraph = NULL;
    }
}


float ImageDataGraph::getNeighborhoodWeight(PixelPosition p, PixelPosition q)
{
    double weight = 0.0;

    double spatial_distance = sqrt( pow((double)(p.vals[0]-q.vals[0]), 2) + pow((double)(p.vals[1]-q.vals[1]), 2) );
    int p_r = mpRVals[p.vals[1]*mImgWidth+p.vals[0]];
    int p_g = mpGVals[p.vals[1]*mImgWidth+p.vals[0]];
    int p_b = mpBVals[p.vals[1]*mImgWidth+p.vals[0]];
    int q_r = mpRVals[q.vals[1]*mImgWidth+q.vals[0]];
    int q_g = mpGVals[q.vals[1]*mImgWidth+q.vals[0]];
    int q_b = mpBVals[q.vals[1]*mImgWidth+q.vals[0]];
    double color_distance = sqrt( pow((double)(p_r - q_r), 2) + pow((double)(p_g - q_g), 2) + pow((double)(p_b - q_b), 2) );

    weight = exp(-pow(color_distance, 2)/(2*pow((double)mSigmaNeighborhood, 2))) / spatial_distance;
    return (float)weight;
}

void ImageDataGraph::importPrior(std::list<PixelPosition> foreground_set, std::list<PixelPosition> background_set)
{
    for(std::list<PixelPosition>::iterator it=foreground_set.begin();it!=foreground_set.end();it++)
    {
        PixelPosition pos = *it;
        PixelColor color;
        int index = pos.vals[1]*mImgWidth+pos.vals[0];
        color.vals[0] = mpRVals[index];
        color.vals[1] = mpGVals[index];
        color.vals[2] = mpBVals[index];

        mpForegroundEstimator->addSample(pos, color);
    }

    for(std::list<PixelPosition>::iterator it=background_set.begin();it!=background_set.end();it++)
    {
        PixelPosition pos = *it;
        PixelColor color;
        int index = pos.vals[1]*mImgWidth+pos.vals[0];
        color.vals[0] = mpRVals[index];
        color.vals[1] = mpGVals[index];
        color.vals[2] = mpBVals[index];

        mpBackgroundEstimator->addSample(pos, color);
    }
}

void ImageDataGraph::initializeGraph()
{
    qDebug() << "Adding nodes";
    // Add nodes
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            int node_id = mpGraph->add_node();
        }
    }

    qDebug() << "Adding neighborhood links";
    // Add neighborhood links
    mMaxNeighborhood = 0.0;
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            int node_id = i+j*mImgWidth;

            std::list<int> neighbor_ids = std::list<int>();

            if(j>0 && i>0)
            {
                neighbor_ids.push_back(node_id-mImgWidth);
                neighbor_ids.push_back(node_id-1);
                neighbor_ids.push_back(node_id-mImgWidth-1);
            }
            else if(j>0)
            {
                neighbor_ids.push_back(node_id-mImgWidth);
            }
            else if(i>0)
            {
                neighbor_ids.push_back(node_id-1);
            }

            if(j<mImgHeight-1 && i<mImgWidth-1)
            {
                neighbor_ids.push_back(node_id+mImgWidth);
                neighbor_ids.push_back(node_id+1);
                neighbor_ids.push_back(node_id+mImgWidth+1);
            }
            else if(j<mImgHeight-1)
            {
                neighbor_ids.push_back(node_id+mImgWidth);
            }
            else if(i<mImgWidth-1)
            {
                neighbor_ids.push_back(node_id+1);
            }

            double neighbor_weight_sum = 0.0;
            PixelPosition p;
            p.vals[0] = i;
            p.vals[1] = j;
            for (std::list<int>::iterator it = neighbor_ids.begin(); it != neighbor_ids.end(); it++)
            {
                PixelPosition q;
                q.vals[0] = (*it)%mImgWidth;
                q.vals[1] = (int)(*it/mImgWidth);
                float neighbor_weight = getNeighborhoodWeight(p, q);
                mpGraph->add_edge(node_id, *it, neighbor_weight, neighbor_weight);
                neighbor_weight_sum += neighbor_weight;
            }
            if (neighbor_weight_sum > mMaxNeighborhood)
            {
                mMaxNeighborhood = neighbor_weight_sum;
            }
        }
    }

    qDebug() << "Adding terminal links";
    //Add terminal links
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            int node_id = i + j*mImgWidth;

            if(mpGridPrior[node_id]==BACKGROUND_PIXEL)
            {
                mpGraph->add_tweights( node_id, 1+mMaxNeighborhood , 0.0 );
            }
            else if(mpGridPrior[node_id]==FOREGROUND_PIXEL)
            {
                mpGraph->add_tweights( node_id, 0.0 , 1+mMaxNeighborhood );
            }
            else
            {
                PixelColor color;
                color.vals[0] = mpRVals[node_id];
                color.vals[1] = mpGVals[node_id];
                color.vals[2] = mpBVals[node_id];
                double foregroundWeight = mpForegroundEstimator->getEstimation(color);
                double backgroundWeight = mpBackgroundEstimator->getEstimation(color);
                mpGraph->add_tweights( node_id, foregroundWeight , backgroundWeight );
            }
        }
    }

    qDebug() << "Finish initialization";
}

int ImageDataGraph::maxFlowCut()
{
    qDebug() << "max flow cut";
    return mpGraph->maxflow();
}
