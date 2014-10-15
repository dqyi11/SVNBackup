#ifndef PIXELGRAPH_H
#define PIXELGRAPH_H

#include "maxflow/graph.h"
#include "segmentation.h"
#include "kerneldensityestimator.h"

typedef Graph<int,int,int> PixelGraph;

class ImageDataGraph
{
    friend class Segmentation;
    friend class GraphCutSegmentation;
    friend class GrabCutSegmentation;
public:
    ImageDataGraph(const char* filename, float regionImportance, float sigma_nb=2.0, float sigma_kde=2.0);
    ~ImageDataGraph();

    enum PixelClass {UNKNOWN_PIXEL = 0, FOREGROUND_PIXEL = 1, BACKGROUND_PIXEL = 2};

    float getNeighborhoodWeight(PixelPosition p, PixelPosition q);
    void importPrior(std::list<PixelPosition> foreground_set, std::list<PixelPosition> background_set);
    void initializeGraph();

    int maxFlowCut();

protected:
    char * mpFilename;
    PixelGraph * mpGraph;
    float mRegionImportance;

    int * mpRVals;
    int * mpGVals;
    int * mpBVals;

    int mImgWidth;
    int mImgHeight;
    int mConnectNum;

    int * mpGridPrior;

    float mSigmaNeighborhood;
    float mSigmaKDE;

    double mMaxNeighborhood;

    GaussianKernelDensityEstimator * mpForegroundEstimator;
    GaussianKernelDensityEstimator * mpBackgroundEstimator;

};

#endif // PIXELGRAPH_H
