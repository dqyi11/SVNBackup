#ifndef KERNELDENSITYESTIMATOR_H
#define KERNELDENSITYESTIMATOR_H

#include "segmentation.h"
#include <list>

class KernelDensityEstimator
{
public:
    KernelDensityEstimator(float bandwidth);
    ~KernelDensityEstimator();

    void addSample(PixelPosition pos, PixelColor color);

    virtual float getEstimation(PixelColor color) = 0;

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
};

#endif // KERNELDENSITYESTIMATOR_H
