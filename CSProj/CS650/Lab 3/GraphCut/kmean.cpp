#include "kmean.h"
#include <iostream>


KMean::KMean(int data_dim, int cluster_num)
{
    mDataDim = data_dim;
    mClusterNum = cluster_num;

    mMeans = new double*[mClusterNum];
    for (int i = 0; i < mClusterNum; i++)
    {
        mMeans[i] = new double[mDataDim];
    }

    mMaxIterationCnt = 100;
    mStopError = 0.001;
}

KMean::~KMean()
{
    for (int i = 0; i < mClusterNum; i++)
    {
        delete[] mMeans[i];
    }
    delete[] mMeans;
}

int * KMean::cluster(double * data, int data_size)
{
    int * labels = new int[data_size];

    double * x = new double[mDataDim];
    int label = -1;
    int iterationCnt = 0;
    double lastCost = 0;
    double currentCost = 0;
    int unchanged = 0;
    bool loop = true;
    int* counts = new int[mClusterNum];
    double** next_means = new double*[mClusterNum];	// New model for re-estimation
    for (int i = 0; i < mClusterNum; i++)
    {
        next_means[i] = new double[mDataDim];
    }

    while (loop)
    {
        memset(counts, 0, sizeof(int) * mClusterNum);
        for (int i = 0; i < mClusterNum; i++)
        {
            memset(next_means[i], 0, sizeof(double) * mDataDim);
        }

        lastCost = currentCost;
        currentCost = 0;

        // Classification
        for (int i = 0; i < data_size; i++)
        {
            for(int j = 0; j < mDataDim; j++)
                x[j] = data[i*mDataDim+j];

            currentCost += getLabel(x, &label);

            counts[label]++;
            for (int d = 0; d < mDataDim; d++)
            {
                next_means[label][d] += x[d];
            }
        }
        currentCost /= data_size;

        // Re-estimation
        for (int i = 0; i < mClusterNum; i++)
        {
            if (counts[i] > 0)
            {
                for (int d = 0; d < mDataDim; d++)
                {
                    next_means[i][d] /= counts[i];
                }
                memcpy(mMeans[i], next_means[i], sizeof(double) * mDataDim);
            }
        }

        // Terminal conditions
        iterationCnt++;
        if (fabs(lastCost - currentCost) < mStopError * lastCost)
        {
            unchanged++;
        }
        if (iterationCnt >= mMaxIterationCnt || unchanged >= 3)
        {
            loop = false;
        }
    }

    // Output the label file
    for (int i = 0; i < data_size; i++)
    {
        for(int j = 0; j < mDataDim; j++)
        {
            x[j] = data[i*mDataDim+j];
        }
        getLabel(x, &label);
        labels[i] = label;
    }
    delete[] counts;
    delete[] x;
    for (int i = 0; i < mClusterNum; i++)
    {
        delete[] next_means[i];
    }
    delete[] next_means;


    return labels;
}

void KMean::init(double *data, int data_size)
{
    double* sample = new double[mDataDim];

    for (int i = 0; i < mClusterNum; i++)
    {
        int select = i * data_size / mClusterNum;
        for(int j = 0; j < mDataDim; j++)
            sample[j] = data[select*mDataDim+j];
        memcpy(mMeans[i], sample, sizeof(double) * mDataDim);
    }

    delete[] sample;
}

double KMean::getLabel(const double* sample, int* label)
{
    double dist = -1;
    for (int i = 0; i < mClusterNum; i++)
    {
        double temp = getDistance(sample, mMeans[i]);
        if (temp < dist || dist == -1)
        {
            dist = temp;
            *label = i;
        }
    }
    return dist;
}

double KMean::getDistance(const double* p, const double* q)
{
    double temp = 0;
    for (int d = 0; d < mDataDim; d++)
    {
        temp += (p[d] - q[d]) * (p[d] - q[d]);
    }
    return sqrt(temp);
}
