#ifndef KERNELDENSITYESTIMATOR_H
#define KERNELDENSITYESTIMATOR_H

#include "segmentation.h"
#include <list>

class KernelDensityEstimator
{
public:
    KernelDensityEstimator();
    ~KernelDensityEstimator();

    void addSample(PixelPosition pos, PixelColor color);

    virtual float getEstimation(PixelColor color) = 0;

protected:
    int mSampleNumber;
    std::list<PixelPosition> mSamplePositions;
    std::list<PixelColor>    mSampleColors;
};

class GaussianKernelDensityEstimator : public KernelDensityEstimator
{
public:
    GaussianKernelDensityEstimator(float sigma);
    float getEstimation(PixelColor color);

    float mSigma;
};

#endif // KERNELDENSITYESTIMATOR_H
