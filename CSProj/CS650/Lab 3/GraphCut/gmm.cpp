#include "gmm.h"
#include "kmean.h"
#include <iostream>
#include <math.h>

GaussianMixtureModel::GaussianMixtureModel(int data_dim, int component_num)
{
    mDataDim = data_dim;
    mComponentNum = component_num;

    mPriors = new double[mComponentNum];
    mMeans = new double*[mComponentNum];
    mVars = new double*[mComponentNum];
    for (int i = 0; i < mComponentNum; i++)
    {
        mPriors[i] = 1.0 / mComponentNum;

        mMeans[i] = new double[mDataDim];
        mVars[i] = new double[mDataDim];
        for (int d = 0; d < mDataDim; d++)
        {
            mMeans[i][d] = 0;
            mVars[i][d] = 1;
        }
    }

    mMinVars = new double[mDataDim];

    mMaxIterationCnt = 100;
    mStopError = 0.001;
}

GaussianMixtureModel::~GaussianMixtureModel()
{
    delete[] mPriors;

    for (int i = 0; i < mComponentNum; i++)
    {
        delete[] mMeans[i];
        delete[] mVars[i];
    }
    delete[] mMeans;
    delete[] mVars;

    delete[] mMinVars;
}

void GaussianMixtureModel::init(double *data, int data_size)
{
    const double MIN_VAR = 1E-10;

    KMean* kmean = new KMean(mDataDim, mComponentNum);
    int * labels = kmean->cluster(data,data_size);

    int* counts = new int[mComponentNum];
    double* overMeans = new double[mDataDim];	// Overall mean of training data
    for (int i = 0; i < mComponentNum; i++)
    {
        counts[i] = 0;
        mPriors[i] = 0;
        memcpy(mMeans[i], kmean->getMean(i), sizeof(double) * mDataDim);
        memset(mVars[i], 0, sizeof(double) * mDataDim);
    }
    memset(overMeans, 0, sizeof(double) * mDataDim);
    memset(mMinVars, 0, sizeof(double) * mDataDim);

    double* x = new double[mDataDim];
    int label = -1;

    for (int i = 0; i < data_size; i++)
    {
        for(int j=0;j<mDataDim;j++)
            x[j]=data[i*mDataDim+j];
        label=labels[i];

        // Count each Gaussian
        counts[label]++;
        double* m = kmean->getMean(label);
        for (int d = 0; d < mDataDim; d++)
        {
            mVars[label][d] += (x[d] - m[d]) * (x[d] - m[d]);
        }

        // Count the overall mean and variance.
        for (int d = 0; d < mDataDim; d++)
        {
            overMeans[d] += x[d];
            mMinVars[d] += x[d] * x[d];
        }
    }

    // Compute the overall variance (* 0.01) as the minimum variance.
    for (int d = 0; d < mDataDim; d++)
    {
        overMeans[d] /= data_size;
        mMinVars[d] = std::max(MIN_VAR, 0.01 * (mMinVars[d] / data_size - overMeans[d] * overMeans[d]));
    }

    // Initialize each Gaussian.
    for (int i = 0; i < mComponentNum; i++)
    {
        mPriors[i] = 1.0 * counts[i] / data_size;

        if (mPriors[i] > 0)
        {
            for (int d = 0; d < mDataDim; d++)
            {
                mVars[i][d] = mVars[i][d] / counts[i];

                // A minimum variance for each dimension is required.
                if (mVars[i][d] < mMinVars[d])
                {
                    mVars[i][d] = mMinVars[d];
                }
            }
        }
    }
    delete kmean;
    delete[] x;
    delete[] counts;
    delete[] overMeans;
    delete[] labels;
}

void GaussianMixtureModel::train(double *data, int data_size)
{
    init(data, data_size);

    // Re-estimation
    bool loop = true;
    double iterationCnt = 0;
    double lastL = 0;
    double currentL = 0;
    int unchanged = 0;
    double* x = new double[mDataDim];
    double* next_priors = new double[mComponentNum];
    double** next_vars = new double*[mComponentNum];
    double** next_means = new double*[mComponentNum];

    for (int i = 0; i < mComponentNum; i++)
    {
        next_means[i] = new double[mDataDim];
        next_vars[i] = new double[mDataDim];
    }

    while (loop)
    {
        // Clear buffer for reestimation
        memset(next_priors, 0, sizeof(double) * mComponentNum);
        for (int i = 0; i < mComponentNum; i++)
        {
            memset(next_vars[i], 0, sizeof(double) * mDataDim);
            memset(next_means[i], 0, sizeof(double) * mDataDim);
        }

        lastL = currentL;
        currentL = 0;

        // Predict
        for (int k = 0; k < data_size; k++)
        {
            for(int j=0;j<mDataDim;j++)
                x[j]=data[k*mDataDim+j];
            double p = getValue(x);

            for (int j = 0; j < mComponentNum; j++)
            {
                double pj = getValue(x, j) * mPriors[j] / p;

                next_priors[j] += pj;

                for (int d = 0; d < mDataDim; d++)
                {
                    next_means[j][d] += pj * x[d];
                    next_vars[j][d] += pj* x[d] * x[d];
                }
            }

            currentL += (p > 1E-20) ? log10(p) : -20;
        }
        currentL /= data_size;

        // Reestimation: generate new priors, means and variances.
        for (int j = 0; j < mComponentNum; j++)
        {
            mPriors[j] = next_priors[j] / data_size;

            if (mPriors[j] > 0)
            {
                for (int d = 0; d < mDataDim; d++)
                {
                    mMeans[j][d] = next_means[j][d] / next_priors[j];
                    mVars[j][d] = next_vars[j][d] / next_priors[j] - mMeans[j][d] * mMeans[j][d];
                    if (mVars[j][d] < mMinVars[d])
                    {
                        mVars[j][d] = mMinVars[d];
                    }
                }
            }
        }

        // Terminal conditions
        iterationCnt++;
        if (fabs(currentL - lastL) < mStopError * fabs(lastL))
        {
            unchanged++;
        }
        if (iterationCnt >= mMaxIterationCnt || unchanged >= 3)
        {
            loop = false;
        }

    }
    delete[] next_priors;
    for (int i = 0; i < mComponentNum; i++)
    {
        delete[] next_means[i];
        delete[] next_vars[i];
    }
    delete[] next_means;
    delete[] next_vars;
    delete[] x;
}

double GaussianMixtureModel::getValue(const double* x, int j)
{
    double val = 1;
    for (int d = 0; d < mDataDim; d++)
    {
        val *= 1 / sqrt(2 * 3.14159 * mVars[j][d]);
        val *= exp(-0.5 * (x[d] - mMeans[j][d]) * (x[d] - mMeans[j][d]) / mVars[j][d]);
    }
    return val;

}

double GaussianMixtureModel::getValue(const double* x)
{
    double val = 0;
    for (int i = 0; i < mComponentNum; i++)
    {
        val += mPriors[i] * getValue(x, i);
    }
    return val;
}
