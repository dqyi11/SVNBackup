#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class MeanShiftTracker 
{

public:
	MeanShiftTracker();
	
    Mat buildModel(const Mat& inputImg, Rect rect);
    void setTargetModel(Mat model) { mTargetModel = model; }

    bool findTarget(const Mat& referenceImg, const Rect& referenceRect, const Mat& candidateImg, Rect& candidateRect);
    bool findTarget(const Mat& candidateImg, Rect& candidateRect);

private:
    Mat calcWeight(const Mat& prevHistROI, const Mat& nextHistROI, const Mat& bb, const Mat& bg, const Mat& br);

    Rect calcLocationUpdate(const Mat& wiROI, const Rect& rect);

    Mat calcNormalizedColorHist(const Mat& image, const Mat& histWeights);
    Mat getBinIndices(const Mat& src, int binNum);

    Mat create2dGaussianKernel(int sizeX, int sizeY, float sigmaX=1.0, float sigmaY=1.0);
    Mat createEpanechnikovKernel(int sizeX, int sizeY);

    Mat mTargetModel;

    int mMaxIteration;
    int mBinNum;
    float mBinWidth;
    int mColorDimension;
    int mPixelDimension;
    int mColorRange;
};
