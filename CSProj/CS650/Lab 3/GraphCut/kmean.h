#ifndef KMEAN_H
#define KMEAN_H

class KMean
{
public:
    KMean(int data_dim, int cluster_num);
    ~KMean();

    int * cluster(double * data, int data_size);
    double* getMean(int i)	{ return mMeans[i]; }

private:

    void init(double * data, int data_size);

    double getLabel(const double* sample, int* label);
    double getDistance(const double* p, const double* q);

    double ** mMeans;
    int mDataDim;
    int mClusterNum;

    int mMaxIterationCnt;
    double mStopError;
};

#endif // KMEAN_H
