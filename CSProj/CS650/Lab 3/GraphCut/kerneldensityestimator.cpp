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
    float estimation = 0.0;

    for(std::list<PixelColor>::iterator it=mSampleColors.begin();it!=mSampleColors.end();it++)
    {
        float distance = sqrt( pow((float)(color.vals[0]-it->vals[0]),2)
                +pow((float)(color.vals[1]-it->vals[1]),2)
                +pow((float)(color.vals[2]-it->vals[2]),2) );
        estimation += exp(distance / (2 * pow((float)mSigma,2)) );
    }

    return estimation;
}
