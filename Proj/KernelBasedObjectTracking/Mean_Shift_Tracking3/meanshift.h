#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class MeanShiftTracker 
{

public:
	MeanShiftTracker();

    MeanShiftTracker(const Mat& referenceImg, Rect referenceRect);
	
    Mat buildModel(const Mat& inputImg, Rect rect);
    void setTargetModel(Mat model) { mTargetModel = model; }

    bool findTarget(const Mat& referenceImg, const Rect& referenceRect, const Mat& candidateImg, Rect& candidateRect);

private:

	Mat computeWeights(const Mat& hist_prev_roi, const Mat& hist_next_roi, const Mat& bg, const Mat& br);

	Point2f computeNextLocation(const Mat& wi_roi, int x, int y);

    Mat computeNormalizedColorHist(const Mat& image, const Mat& mHistWeights);

	void normalizeHistogram(Mat& hist);

	Mat getBinIndices(const Mat& src, const int binSize);

	Mat create2dGaussianKernel(const int sizeX, const float sigmaX, const int sizeY, const float sigmaY);

    Mat mCandidateImg;
    Mat mReferenceImg;

    Mat mTargetModel;
    Mat mCandidateModel;

    Rect mReferenceRect;
    Rect mCandidateRect;

    //Mat mHistWeights;

    int mMaxIteration;

    int mBinNum;
    float mBinWidth;
    int mColorDimension;
    int mPixelDimension;
    int mColorRange;

};
