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
    float norm_term = sqrt(2 * 3.1415926);

    for(std::list<PixelColor>::iterator it=mSampleColors.begin();it!=mSampleColors.end();it++)
    {
        float squared_distance = pow((double)(color.vals[0]-it->vals[0])/mBandWidth,2)
                +pow((double)(color.vals[1]-it->vals[1])/mBandWidth,2)
                +pow((double)(color.vals[2]-it->vals[2])/mBandWidth,2);
        estimation += exp(squared_distance / 2 );
    }

    float normScale = (mSampleNumber * mBandWidth) / norm_term;
    estimation /= normScale;
    //estimation /= mSampleNumber;

    return (float)estimation;
}

float GaussianKernelDensityEstimator::estimateOptimalBandwidth()
{
    double meanVal0=0.0, meanVal1=0.0, meanVal2=0.0;
    for(std::list<PixelColor>::iterator it=mSampleColors.begin();it!=mSampleColors.end();it++)
    {
        meanVal0 += it->vals[0];
        meanVal1 += it->vals[1];
        meanVal2 += it->vals[2];
    }
    meanVal0 /=  mSampleColors.size();
    meanVal1 /=  mSampleColors.size();
    meanVal2 /=  mSampleColors.size();

    double var = 0.0;
    for(std::list<PixelColor>::iterator it=mSampleColors.begin();it!=mSampleColors.end();it++)
    {
        var += pow((double)(it->vals[0]-meanVal0),2)
                +pow((double)(it->vals[1]-meanVal1),2)
                +pow((double)(it->vals[2]-meanVal2),2);

    }
    var /= mSampleColors.size();

    return 1.06*sqrt(var)*pow((double)mSampleColors.size(), -0.2);

}
