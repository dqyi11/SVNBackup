#include "gmmdensityestimator.h"
#include "gmm.h"

GMMDensityEstimator::GMMDensityEstimator(int componentNum)
{
    mComponentNumber = componentNum;
    mpGMM = new GaussianMixtureModel(3, mComponentNumber);
}

GMMDensityEstimator::~GMMDensityEstimator()
{
    mSampleColors.clear();
    mSamplePositions.clear();

    if(mpGMM)
    {
        delete mpGMM;
        mpGMM = NULL;
    }
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
    double * train_data = new double [mSampleNumber*3];
    int idx = 0;
    for(std::list<PixelColor>::iterator it=mSampleColors.begin(); it!=mSampleColors.end();it++)
    {
        PixelColor pix_color = *it;
        train_data[idx*3] = pix_color.vals[0];
        train_data[idx*3+1] = pix_color.vals[1];
        train_data[idx*3+2] = pix_color.vals[2];
        idx ++;
    }

    mpGMM->train(train_data, mSampleNumber);
}
