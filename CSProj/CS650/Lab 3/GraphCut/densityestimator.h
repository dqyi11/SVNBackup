#ifndef DENSITYESTIMATOR_H
#define DENSITYESTIMATOR_H

#include "segmentation.h"

class DensityEstimator
{
public:
    DensityEstimator();

    virtual void addSample(PixelPosition pos, PixelColor color) = 0;
    virtual float getEstimation(PixelColor color) = 0;
};

#endif // DENSITYESTIMATOR_H
