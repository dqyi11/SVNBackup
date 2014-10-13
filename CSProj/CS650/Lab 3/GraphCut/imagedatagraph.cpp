#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <math.h>

#include "imagedatagraph.h"

ImageDataGraph::ImageDataGraph(const char* filename, float sigma_nb)
{
    mpFilename = const_cast<char *>(filename);

    IplImage  * img = cvLoadImage(mpFilename);
    img_width = img->width;
    img_height = img->height;
    connect_num = 4;

    mpForegroundEstimator = new GaussianKernelDensityEstimator();
    mpBackgroundEstimator = new GaussianKernelDensityEstimator();

    mSigmaNeighborhood = sigma_nb;

    mpRVals = new int[img_width*img_height];
    mpGVals = new int[img_width*img_height];
    mpBVals = new int[img_width*img_height];

    mpGraph = new PixelGraph(img_width*img_height, img_width*img_height*connect_num);

    for(int j=0;j<img_height;j++)
    {
        for(int i=0;i<img_width;i++)
        {
            mpRVals[j*img_width+i] = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 0];
            mpGVals[j*img_width+i] = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 1];
            mpBVals[j*img_width+i] = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 2];
        }
    }

    cvReleaseImage(&img);
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
    int p_r = mpRVals[p.vals[1]*img_width+p.vals[0]];
    int p_g = mpGVals[p.vals[1]*img_width+p.vals[0]];
    int p_b = mpBVals[p.vals[1]*img_width+p.vals[0]];
    int q_r = mpRVals[q.vals[1]*img_width+q.vals[0]];
    int q_g = mpGVals[q.vals[1]*img_width+q.vals[0]];
    int q_b = mpBVals[q.vals[1]*img_width+q.vals[0]];
    double color_distance = sqrt( pow((double)(p_r - q_r), 2) + pow((double)(p_g - q_g), 2) + pow((double)(p_b - q_b), 2) );

    weight = exp(-pow(color_distance, 2)/(2*pow((double)mSigmaNeighborhood, 2))) / spatial_distance;
    return (float)weight;
}
