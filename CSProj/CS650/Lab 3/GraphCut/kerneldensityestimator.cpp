#include "kerneldensityestimator.h"
#include <math.h>

KernelDensityEstimator::KernelDensityEstimator(float bandwidth)
{
    mSampleNumber = 0;
    mBandWidth = bandwidth;
}

KernelDensityEstimator::~KernelDensityEstimator()
{
     mSampleColors.clear();
}

void KernelDensityEstimator::addSample(PixelPosition pos, PixelColor color)
{
    mSamplePositions.push_back(pos);
    mSampleColors.push_back(color);
    mSampleNumber++;
}

GaussianKernelDensityEstimator::GaussianKernelDensityEstimator(float bandwidth) : KernelDensityEstimator(bandwidth)
{
}

float GaussianKernelDensityEstimator::getEstimation(PixelColor color)
{
    double estimation = 0.0;
    float norm_term = 1 / sqrt(2 * 3.1415926);

    for(std::list<PixelColor>::iterator it=mSampleColors.begin();it!=mSampleColors.end();it++)
    {
        float squared_distance = pow((double)(color.vals[0]-it->vals[0])/mBandWidth,2)
                +pow((double)(color.vals[1]-it->vals[1])/mBandWidth,2)
                +pow((double)(color.vals[2]-it->vals[2])/mBandWidth,2);
        estimation += exp(squared_distance / 2 );
    }

    estimation *= norm_term/(mSampleNumber * mBandWidth);
    //estimation /= mSampleNumber;

    return (float)estimation;
}
