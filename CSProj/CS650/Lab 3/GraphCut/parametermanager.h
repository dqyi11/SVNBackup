#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H

#include "globaldef.h"

class ParameterManager
{
public:
    ParameterManager();

    int mIterationNumber;
    float mSmoothnessRatio;
    float mKDEBandWidth;

    EstimatorType mDEType;
};

#endif // PARAMETERMANAGER_H
