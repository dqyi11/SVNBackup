#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

class MeanShiftTracker 
{

public:
	MeanShiftTracker();

	MeanShiftTracker(const Mat& initialFrame, cv::Rect target_initial);
	
	Mat buildTargetModel(const Mat& initialFrame, cv::Rect target_initial);

	int run(const Mat& currentFrame, const int Nmax);

	cv::Mat getTargetModel() const { return targetModel; }
	void setTargetModel(cv::Mat val) { targetModel = val; }

	cv::Mat getInitialFrame() const { return initialFrame; }
	void setInitialFrame(cv::Mat val) { initialFrame = val; }

	cv::Mat getCurrentFrame() const { return currentFrame; }
	void setCurrentFrame(cv::Mat val) { currentFrame = val; }

	cv::Rect getTarget_initial() const { return target_initial; }
	void setTarget_initial(cv::Rect val) { target_initial = val; }

	cv::Rect getTarget_current() const { return target_current; }
	void setTarget_current(cv::Rect val) { target_current = val; }

	int getMaxIterNum() const { return Nmax; }
	void setMaxIterNum(int val) { Nmax = val; }

	cv::Mat getHistWeights() const { return histWeights; }
	void setHistWeights(cv::Mat val) { histWeights = val; }

private:

	Mat computeWeights(const Mat& hist_prev_roi, const Mat& hist_next_roi, const Mat& bg, const Mat& br);

	Point2f computeNextLocation(const Mat& wi_roi, int x, int y);

	void computeNormalizedColorHist(const Mat& image, const Mat& histWeights, Mat& hist, int N);

	void normalizeHistogram(Mat& hist);

	Mat getBinIndices(const Mat& src, const int binSize);

	Mat create2dGaussianKernel(const int sizeX, const float sigmaX, const int sizeY, const float sigmaY);

	// current frame
	Mat currentFrame;

	// initial frame
	Mat initialFrame;

	// target model histogram
	// this will be computed once, for the initial frame
	Mat targetModel;

	// target location in initial frame
	Rect target_initial;

	// target location to be computed in the next frame
	Rect target_current;

	Mat histWeights;

	// maximum number of meanshift iterations
	int Nmax;

};
