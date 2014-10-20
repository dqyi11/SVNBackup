#ifndef KERNELDENSITYESTIMATOR_H
#define KERNELDENSITYESTIMATOR_H
#include "densityestimator.h"
#include "segmentation.h"
#include <list>

class KernelDensityEstimator : public DensityEstimator
{
public:
    KernelDensityEstimator(float bandwidth);
    ~KernelDensityEstimator();

    void addSample(PixelPosition pos, PixelColor color);

protected:
    int mSampleNumber;
    float mBandWidth;

    std::list<PixelPosition> mSamplePositions;
    std::list<PixelColor>    mSampleColors;
};

class GaussianKernelDensityEstimator : public KernelDensityEstimator
{
public:
    GaussianKernelDensityEstimator(float bandwith);
    float getEstimation(PixelColor color);

    float estimateOptimalBandwidth();
};

#endif // KERNELDENSITYESTIMATOR_H
