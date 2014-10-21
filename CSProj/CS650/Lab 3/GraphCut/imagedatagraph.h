#ifndef PIXELGRAPH_H
#define PIXELGRAPH_H

#include "maxflow/graph.h"
#include "segmentation.h"
#include "densityestimator.h"
#include "globaldef.h"

typedef Graph<int,int,int> PixelGraph;

class ImageDataGraph
{
    friend class Segmentation;
    friend class GraphCutSegmentation;
    friend class GrabCutSegmentation;
public:
    enum Neighborhood { N = 0, NE, E, SE, S, SW, W, NW };

    enum PixelClass {UNKNOWN_PIXEL = 0, FOREGROUND_PIXEL = 1, BACKGROUND_PIXEL = 2};

    ImageDataGraph(const char* filename);
    ~ImageDataGraph();

    void initalizeType(EstimatorType type = KDE);
    float getNeighborhoodWeight(PixelPosition p, PixelPosition q);
    void importPrior(std::list<PixelPosition> foreground_set, std::list<PixelPosition> background_set);
    void initializeGraph();
    void initializeNeighborhoodWeights();

    int maxFlowCut();

    double getGibbsEnergy() { return mTotalGibbsEnergy; }
    double getDataEnergy() { return mDataGibbsEnergy; }
    double getSmoothnessEnergy() { return mSmoothnessGibbsEnergy; }

protected:

    float getColorSquaredDistance(int p_idx, int q_idx);
    float getNeighborhoodWeight(PixelPosition p, PixelPosition q, float gamma, float beta, float spatial_distance);
    float calcNeighborhoodBeta();
    void calcNeighborhoodWeights(float gamma, float beta);

    EstimatorType mEstimatorType;
    char * mpFilename;
    PixelGraph * mpGraph;
    float mRegionImportance;

    int * mpRVals;
    int * mpGVals;
    int * mpBVals;

    float ** mpNeighborhoodWeights;

    double mMaxNeighborhoodWeights;

    int mImgWidth;
    int mImgHeight;
    int mConnectNum;

    int * mpGridPrior;

    float mSigmaKDE;
    float mNeighborhoodBeta;
    float mNeighborhoodGamma;

    double mTotalGibbsEnergy;
    double mDataGibbsEnergy;
    double mSmoothnessGibbsEnergy;

    DensityEstimator * mpForegroundEstimator;
    DensityEstimator * mpBackgroundEstimator;

};

#endif // PIXELGRAPH_H
