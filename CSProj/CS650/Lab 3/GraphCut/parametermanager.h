#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H

class ParameterManager
{
public:
    ParameterManager();

    int mIterationNumber;
    float mRegionImportance;
    float mNeighborhoodSigma;
    float mKDESigma;
};

#endif // PARAMETERMANAGER_H
