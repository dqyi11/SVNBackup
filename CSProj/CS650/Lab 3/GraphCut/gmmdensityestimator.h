#ifndef GMMDENSITYESTIMATOR_H
#define GMMDENSITYESTIMATOR_H

#include "densityestimator.h"

class GMMDensityEstimator : public DensityEstimator
{
public:
    GMMDensityEstimator();
    ~GMMDensityEstimator();

    void addSample(PixelPosition pos, PixelColor color);
    float getEstimation(PixelColor color);

    int mSampleNumber;

    std::list<PixelPosition> mSamplePositions;
    std::list<PixelColor>    mSampleColors;
};

#endif // GMMDENSITYESTIMATOR_H
