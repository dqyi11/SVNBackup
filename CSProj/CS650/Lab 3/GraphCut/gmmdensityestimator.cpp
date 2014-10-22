#include "gmmdensityestimator.h"
#include "gmm.h"

GMMDensityEstimator::GMMDensityEstimator(int componentNum)
{
    mComponentNumber = componentNum;
}

GMMDensityEstimator::~GMMDensityEstimator()
{
    mSampleColors.clear();
    mSamplePositions.clear();
}

void GMMDensityEstimator::addSample(PixelPosition pos, PixelColor color)
{
    mSamplePositions.push_back(pos);
    mSampleColors.push_back(color);
    mSampleNumber++;
}

float GMMDensityEstimator::getEstimation(PixelColor color)
{
    return 0.0;
}

void GMMDensityEstimator::learningModel()
{

}
