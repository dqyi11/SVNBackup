#include "kerneldensityestimator.h"
#include <math.h>

KernelDensityEstimator::KernelDensityEstimator()
{
    mSampleNumber = 0;
}


void KernelDensityEstimator::addSample(PixelPosition pos, PixelColor color)
{
    mSamplePositions.push_back(pos);
    mSampleColors.push_back(color);
    mSampleNumber++;
}

GaussianKernelDensityEstimator::GaussianKernelDensityEstimator(float sigma)
{
    mSigma = sigma;
}

float GaussianKernelDensityEstimator::getEstimation(PixelColor color)
{
    double estimation = 0.0;

    for(std::list<PixelColor>::iterator it=mSampleColors.begin();it!=mSampleColors.end();it++)
    {
        float distance = sqrt( pow((double)(color.vals[0]-it->vals[0]),2)
                +pow((double)(color.vals[1]-it->vals[1]),2)
                +pow((double)(color.vals[2]-it->vals[2]),2) );
        estimation += exp(distance / (2 * pow((double)mSigma,2)) );
    }

    estimation /= mSampleNumber;

    return (float)estimation;
}
