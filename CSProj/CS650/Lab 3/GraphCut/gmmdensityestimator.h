#ifndef GMMDENSITYESTIMATOR_H
#define GMMDENSITYESTIMATOR_H

#include "densityestimator.h"

class GaussianMixtureModel;

class GMMDensityEstimator : public DensityEstimator
{
public:
    GMMDensityEstimator(int componentNum);
    ~GMMDensityEstimator();

    void addSample(PixelPosition pos, PixelColor color);
    float getEstimation(PixelColor color);

    void learningModel();

    int mSampleNumber;
    std::list<PixelPosition> mSamplePositions;
    std::list<PixelColor>    mSampleColors;

private:

    GaussianMixtureModel * mpGMM;

    int mComponentNumber;

};

#endif // GMMDENSITYESTIMATOR_H
