#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <math.h>

#include "imagedatagraph.h"
#include "qdebug.h"

ImageDataGraph::ImageDataGraph(const char* filename, float sigma_kde, float neighborhood_gamma)
{
    mpFilename = const_cast<char *>(filename);

    IplImage * img_rgb = cvLoadImage(mpFilename);
    IplImage * img = cvCreateImage(cvGetSize(img_rgb), img_rgb->depth, img_rgb->nChannels);
    cvCvtColor(img_rgb, img, CV_RGB2Luv);
    mImgWidth = img->width;
    mImgHeight = img->height;
    mConnectNum = 8;
    mTotalGibbsEnergy = 0.0;
    mDataGibbsEnergy = 0.0;
    mSmoothnessGibbsEnergy = 0.0;

    mpNeighborhoodWeights = new float * [mConnectNum];
    for(int k=0;k<mConnectNum;k++)
    {
        mpNeighborhoodWeights[k] = new float[mImgWidth*mImgHeight];
    }

    mSigmaKDE = sigma_kde;

    mNeighborhoodGamma = neighborhood_gamma;
    mNeighborhoodBeta = 0.0;

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
    for(int k=0;k<mConnectNum;k++)
    {
        if(mpNeighborhoodWeights[k])
        {
            delete mpNeighborhoodWeights[k];
            mpNeighborhoodWeights[k] = NULL;
        }
    }
    if(mpNeighborhoodWeights)
    {
        delete mpNeighborhoodWeights;
        mpNeighborhoodWeights = NULL;
    }
    if(mpForegroundEstimator)
    {
        delete mpForegroundEstimator;
        mpForegroundEstimator = NULL;
    }
    if(mpBackgroundEstimator)
    {
        delete mpBackgroundEstimator;
        mpBackgroundEstimator = NULL;
    }
    if(mpRVals)
    {
        delete mpRVals;
        mpRVals = NULL;
    }
    if(mpGVals)
    {
        delete mpGVals;
        mpGVals = NULL;
    }
    if(mpBVals)
    {
        delete mpBVals;
        mpBVals = NULL;
    }
}


float ImageDataGraph::getNeighborhoodWeight(PixelPosition p, PixelPosition q, float gamma, float beta, float spatial_distance)
{
    float weight = 0.0;
    double color_distance_square = getColorSquaredDistance(p.vals[1]*mImgWidth+p.vals[0], q.vals[1]*mImgWidth+q.vals[0]);

    weight = gamma * exp(- color_distance_square * beta ) / spatial_distance;
    return weight;
}

float ImageDataGraph::getColorSquaredDistance(int p_idx, int q_idx)
{
    float distance = 0.0;
    //qDebug() << " P " << p_idx << " + Q " << q_idx;
    distance = pow((double)(mpRVals[p_idx] - mpRVals[q_idx]), 2) + pow((double)(mpGVals[p_idx] - mpGVals[q_idx]), 2) + pow((double)(mpBVals[p_idx] - mpBVals[q_idx]), 2) ;
    return distance;
}

float ImageDataGraph::calcNeighborhoodBeta()
{
    int pixelCnt = 0;
    double diff = 0.0;
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            //qDebug() << i << " + " << j;
            int p_idx = i + j*mImgWidth;
            if( i > 0)
            {
                int q_idx = p_idx - 1; // W
                diff += getColorSquaredDistance(p_idx, q_idx);
                pixelCnt ++;
            }
            if( i > 0 && j > 0)
            {
                int q_idx = p_idx - mImgWidth - 1; // NW
                diff += getColorSquaredDistance(p_idx, q_idx);
                pixelCnt ++;
            }
            if( j > 0 )
            {
                int q_idx = p_idx - mImgWidth; // N
                diff += getColorSquaredDistance(p_idx, q_idx);
                pixelCnt ++;
            }
            if( j > 0 && i < mImgWidth-1 )
            {
                int q_idx = p_idx - mImgWidth + 1; // NE
                diff += getColorSquaredDistance(p_idx, q_idx);
                pixelCnt ++;
            }
        }
    }
    //qDebug() << "diff " << diff;
    return 1.0 / (2 *diff / pixelCnt);
}

void ImageDataGraph::initializeNeighborhoodWeights()
{
    mNeighborhoodBeta = calcNeighborhoodBeta();
    qDebug() << "Neighborhood beta " << mNeighborhoodBeta;
    calcNeighborhoodWeights(mNeighborhoodGamma, mNeighborhoodBeta);
}

void ImageDataGraph::calcNeighborhoodWeights(float gamma, float beta)
{
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            int idx = i+j*mImgWidth;
            for(int k=0;k<mConnectNum;k++)
            {
                switch(k)
                {
                case 0:    //N
                default:
                    if(j>0)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i;
                        q.vals[1] = j-1;
                        mpNeighborhoodWeights[0][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.0);
                    }
                    break;
                case 1:    //NE
                    if(j>0 && i<mImgWidth-1)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i+1;
                        q.vals[1] = j-1;
                        mpNeighborhoodWeights[1][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.414);
                    }
                    break;
                case 2:    //E
                    if(i<mImgWidth-1)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i+1;
                        q.vals[1] = j;
                        mpNeighborhoodWeights[2][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.0);
                    }
                    break;
                case 3:    //SE
                    if(j<mImgHeight-1 && i<mImgWidth-1)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i+1;
                        q.vals[1] = j+1;
                        mpNeighborhoodWeights[3][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.414);
                    }
                    break;
                case 4:    //S
                    if(j<mImgHeight-1)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i;
                        q.vals[1] = j+1;
                        mpNeighborhoodWeights[4][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.0);
                    }
                    break;
                case 5:    //SW
                    if(i> 0 && j<mImgHeight-1)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i-1;
                        q.vals[1] = j+1;
                        mpNeighborhoodWeights[5][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.414);
                    }
                    break;
                case 6:    //W
                    if(i> 0)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i-1;
                        q.vals[1] = j;
                        mpNeighborhoodWeights[6][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.0);
                    }
                    break;
                case 7:    //NW
                    if(i> 0 && j>0)
                    {
                        PixelPosition p, q;
                        p.vals[0] = i;
                        p.vals[1] = j;
                        q.vals[0] = i-1;
                        q.vals[1] = j-1;
                        mpNeighborhoodWeights[7][idx] = getNeighborhoodWeight(p, q, gamma, beta, 1.414);
                    }
                    break;
                }
            }
        }
    }
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
    qDebug() << "Initialize neighborhood weights";
    initializeNeighborhoodWeights();

    qDebug() << "Adding nodes";
    // Add nodes
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            //int node_id = mpGraph->add_node();
            mpGraph->add_node();
        }
    }

    mTotalGibbsEnergy = 0.0;
    mDataGibbsEnergy = 0.0;
    mSmoothnessGibbsEnergy = 0.0;

    qDebug() << "Adding neighborhood links";
    // Add neighborhood links
    for(int j=0;j<mImgHeight;j++)
    {
        for(int i=0;i<mImgWidth;i++)
        {
            int node_id = i+j*mImgWidth;
            if( i > 0)
            {
                int neighbor_id = node_id - 1; // W
                mpGraph->add_edge(node_id, neighbor_id, mpNeighborhoodWeights[W][node_id], mpNeighborhoodWeights[W][node_id]);

            }
            if( i > 0 && j > 0)
            {
                int neighbor_id = node_id - mImgWidth - 1; // NW
                mpGraph->add_edge(node_id, neighbor_id, mpNeighborhoodWeights[NW][node_id], mpNeighborhoodWeights[NW][node_id]);
            }
            if( j > 0 )
            {
                int neighbor_id = node_id - mImgWidth; // N
                mpGraph->add_edge(node_id, neighbor_id, mpNeighborhoodWeights[N][node_id], mpNeighborhoodWeights[N][node_id]);

            }
            if( j > 0 && i < mImgWidth-1 )
            {
                int neighbor_id = node_id - mImgWidth + 1; // NE
                mpGraph->add_edge(node_id, neighbor_id, mpNeighborhoodWeights[NE][node_id], mpNeighborhoodWeights[NE][node_id]);
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

            /*
            if(mpGridPrior[node_id]==BACKGROUND_PIXEL)
            {
                mpGraph->add_tweights( node_id, 0.0, 1+mMaxNeighborhood );
            }
            else if(mpGridPrior[node_id]==FOREGROUND_PIXEL)
            {
                mpGraph->add_tweights( node_id, 1+mMaxNeighborhood, 0.0 );
            }
            else */
            {
                PixelColor color;
                color.vals[0] = mpRVals[node_id];
                color.vals[1] = mpGVals[node_id];
                color.vals[2] = mpBVals[node_id];
                double foregroundWeight = mpForegroundEstimator->getEstimation(color);
                double backgroundWeight = mpBackgroundEstimator->getEstimation(color);
                double normalizedForegroundWeight = - log( foregroundWeight / (foregroundWeight+backgroundWeight) );
                double normalizedBackgroundWeight = - log( backgroundWeight / (foregroundWeight+backgroundWeight) );
                mpGraph->add_tweights( node_id, normalizedForegroundWeight , normalizedBackgroundWeight );
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
