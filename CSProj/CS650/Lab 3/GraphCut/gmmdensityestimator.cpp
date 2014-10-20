#include "gmmdensityestimator.h"

GMMDensityEstimator::GMMDensityEstimator()
{
}

GMMDensityEstimator::~GMMDensityEstimator()
{
    mSampleColors.clear();
    mSamplePositions.clear();
}

void GMMDensityEstimator::addSample(PixelPosition pos, PixelColor color)
{

}

float GMMDensityEstimator::getEstimation(PixelColor color)
{
    return 0.0;
}
