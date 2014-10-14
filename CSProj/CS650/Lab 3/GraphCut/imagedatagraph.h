#ifndef PIXELGRAPH_H
#define PIXELGRAPH_H

#include "maxflow/graph.h"
#include "segmentation.h"
#include "kerneldensityestimator.h"

typedef Graph<int,int,int> PixelGraph;

class ImageDataGraph
{
public:
    ImageDataGraph(const char* filename, float sigma_nb=2.0, float sigma_kde=2.0);
    ~ImageDataGraph();

    float getNeighborhoodWeight(PixelPosition p, PixelPosition q);
    void importPrior(std::list<PixelPosition> foreground_set, std::list<PixelPosition> background_set);

private:
    char * mpFilename;
    PixelGraph * mpGraph;

    int * mpRVals;
    int * mpGVals;
    int * mpBVals;

    int mImgWidth;
    int mImgHeight;
    int mConnectNum;

    int * mpGridPrior;

    float mSigmaNeighborhood;
    float mSigmaKDE;

    GaussianKernelDensityEstimator * mpForegroundEstimator;
    GaussianKernelDensityEstimator * mpBackgroundEstimator;

};

#endif // PIXELGRAPH_H
