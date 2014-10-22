#ifndef GMM_H
#define GMM_H

class GaussianMixtureModel
{
public:
    GaussianMixtureModel(int data_dim, int component_num);
    ~GaussianMixtureModel();


    void train(double *data, int data_size);

    double getValue(const double* x);

private:
    void init(double *data, int data_size);
    double getValue(const double* x, int j);

    int mDataDim;
    int mComponentNum;

    double* mPriors;
    double** mMeans;
    double** mVars;

    double* mMinVars;

    int mMaxIterationCnt;
    double mStopError;
};

#endif // GMM_H
