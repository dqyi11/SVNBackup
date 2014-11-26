#include "meanshift.h"
#include <iostream>

using namespace cv;
using namespace std;


MeanShiftTracker::MeanShiftTracker()
{

}


MeanShiftTracker::MeanShiftTracker(const Mat& initialFrame, cv::Rect target_initial)
{

	setInitialFrame(initialFrame);
	setTarget_initial(target_initial);
	setTarget_current(target_initial);

	buildTargetModel(initialFrame, target_initial);

}


cv::Mat MeanShiftTracker::buildTargetModel(const Mat& initialFrame, cv::Rect target_initial)
{

	// simdiki kareden ilgi alani icindeki bolgeyi al
	Mat initial_roi = initialFrame(target_initial);

	// ilgi alaninin histogrami
	Mat hist_initial_roi;

	// number of histogram bins
	int nbins = 16;

	Mat histWeights = create2dGaussianKernel(target_initial.width, 1, target_initial.height, 1);
	setHistWeights(histWeights);

	computeNormalizedColorHist(initial_roi, histWeights, hist_initial_roi, nbins);


	computeBackgroundWeightedHistogram(initialFrame, target_initial, hist_initial_roi, nbins);
	//


	setTargetModel(hist_initial_roi);

	return hist_initial_roi;

}


void MeanShiftTracker::computeNormalizedBackgroundHist(const Mat& backgroundRegion, 
													   const Rect& foregroundRegion, 
													   Mat& hist, int N)
{
	const int histSize[] = {N, N};

	// make sure that the histogram has a proper size and type
	hist = Mat(2, histSize, CV_32F, Scalar::all(0));


	// the loop below assumes that the image
	// is a 8-bit 3-channel. check it.
	CV_Assert(backgroundRegion.type() == CV_8UC3);


	for (int y=0; y<backgroundRegion.size().height; y++)
	{
		for (int x=0; x<backgroundRegion.size().width; x++)
		{

			if (foregroundRegion.contains(Point(x,y)))
			{
				continue;
			}

			hist.at<float>( 
				backgroundRegion.at<cv::Vec3b>(y,x)[1]*N/256, 
				backgroundRegion.at<cv::Vec3b>(y,x)[2]*N/256) += 1.0f;

		}
	}


	double s = sum(hist)[0];
	hist.convertTo(hist, hist.type(), 1./s, 0);

}


void MeanShiftTracker::computeNormalizedColorHist(const Mat& image, const Mat& histWeights, Mat& hist, int N)
{
	const int histSize[] = {N, N};

	// make sure that the histogram has a proper size and type
	hist = Mat(2, histSize, CV_32F, Scalar::all(0));
	
	
	// the loop below assumes that the image
	// is a 8-bit 3-channel. check it.
	CV_Assert(image.type() == CV_8UC3);


	for (int y=0; y<image.size().height; y++)
	{
		for (int x=0; x<image.size().width; x++)
		{

			hist.at<float>( 
				image.at<cv::Vec3b>(y,x)[1]*N/256, 
				image.at<cv::Vec3b>(y,x)[2]*N/256) += histWeights.at<float>(y,x);

		}
	}


	// normalize histogram to sum up to 1
	double s = sum(hist)[0];
	hist.convertTo(hist, hist.type(), 1./s, 0);

}


void MeanShiftTracker::computeBackgroundWeights(const Mat& hist_background, Mat backgroundWeights, const int numBins)
{

	const int weightsSize[] = {numBins, numBins};

	Mat sorted = hist_background.clone();

	// assuming histogram is 2D
	// find the smallest nonzero element in histogram
	float minNonzero = 1000.f;
	for (int y=0; y<numBins; y++)
	{
		for (int x=0; x<numBins; x++)
		{

			if ( hist_background.at<float>(y, x) > 0  && 
				hist_background.at<float>(y, x) < minNonzero )
			{
				minNonzero = hist_background.at<float>(y, x);
			}	

		}
	}



	//cv::sort(hist_background, sorted, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
	//cv::sort(sorted, sorted, CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

	//float minNonzero = -1.f;
	//for (int y=0; y<numBins; y++)
	//{
	//	for (int x=0; x<numBins; x++)
	//	{

	//		if ( sorted.at<float>(y, x) > 0 )
	//		{
	//			minNonzero = sorted.at<float>(y, x);
	//			break;
	//		}	

	//	}

	//	if (minNonzero > 0)
	//	{
	//		break;
	//	}
	//}


	// trying to implement equation 17 of the paper
	Mat minValMat = Mat(2, weightsSize, CV_32F, Scalar::all(minNonzero));

	divide(minValMat, hist_background, minValMat);

	min(minValMat, 1, backgroundWeights);

}


void MeanShiftTracker::computeBackgroundWeightedHistogram(const Mat& frame, const Rect& foregroundRect, Mat srcDst, const int numBins)
{

	// perform background-weighted histogram computation
	int x, y, width, height;

	// background area is supposed to be 3 times the foreground area
	x = foregroundRect.x - foregroundRect.width;
	y = foregroundRect.y - foregroundRect.height;
	if (x < 0)
		x = 0;
	if (y < 0)
		y = 0;
	width = 3*foregroundRect.width;
	height = 3*foregroundRect.height;

	if( x + width > frame.size().width-1 )
		width -= x + width - (frame.size().width-1);

	if( y + height > frame.size().height-1 )
		height -= y + height - (frame.size().height-1);

	Rect backgroundRect(x, y, width, height);


	Mat backgroundRegion =  frame(backgroundRect);

	Mat hist_background;


	// compute normalized histogram of the background region
	computeNormalizedBackgroundHist(backgroundRegion, foregroundRect, hist_background, numBins);


	Mat backgroundWeights;

	const int weightsSize[] = {numBins, numBins};

	// assuming a 2D histogram
	// weights have the same dimensions with the histogram
	backgroundWeights = Mat(2, weightsSize, CV_32F, Scalar::all(0));

	computeBackgroundWeights(hist_background, backgroundWeights, numBins);

	
	// weight histogram bins with background weights
	// element-wise multiplication
	multiply(srcDst, backgroundWeights, srcDst);


	// normalize the histogram after weighting
	// by dividing with the sum of bg weights
	double s = sum(backgroundWeights)[0];
	srcDst.convertTo(srcDst, srcDst.type(), 1./s, 0);

	//

}


// compute the coordinate of the next pixel location according to equation 13
// given the upper-left x coordinate of roi
// given the upper-left y coordinate of roi
cv::Point2f MeanShiftTracker::computeNextLocation(const Mat& wi_roi, int x, int y)
{

	Mat x2D, y2D;


	// pixel locations as 1D arrays
	// x = [x0,..,xw]  (row vector)
	// y = [y0,..,yh]' (column vector)
	Mat xi, yi;

	// fill the array of x locations for current roi
	xi.create(1, wi_roi.size().width, CV_32F);
	for (int i = 0; i < wi_roi.size().width; i++)
	{
		xi.at<float>(i) = (float) x+i;
	}

	// fill the array of y locations for current roi
	yi.create(wi_roi.size().height, 1, CV_32F);
	for (int j = 0; j < wi_roi.size().height; j++)
	{
		yi.at<float>(j) = (float) y+j;
	}

	// convert 1D array of locations to 2D matrix, by repeating values
	// along horizontal dims for x, and vertical dims for y.
	repeat(xi, wi_roi.size().height, 1, x2D);

	repeat(yi, 1, wi_roi.size().width, y2D);

	xi.release();
	yi.release();


	// weighting x coordinates
	Mat wi_roi_X;
	multiply(wi_roi, x2D, wi_roi_X);	//element-wise multiplication
	double s1 = sum(wi_roi_X)[0];
	double s2 = sum(wi_roi)[0];
	
	float newX = (float)(s1/s2);


	// repeat the same for y coordinates
	Mat wi_roi_Y;
	multiply(wi_roi, y2D, wi_roi_Y);
	s1 = sum(wi_roi_Y)[0];

	float newY = (float)(s1/s2);


	return Point2f(newX, newY);
}


// compute weights for the current target location according to equation 10
cv::Mat MeanShiftTracker::computeWeights(const Mat& hist_prev_roi, const Mat& hist_next_roi, const Mat& bg, const Mat& br)
{
	Mat wi_roi;
	wi_roi.create(bg.size(), CV_32F);

	float q,p;
	int histIdx_G, histIdx_R;

	for (int row = 0; row < bg.size().height; row++)
	{
		for (int col = 0; col < bg.size().width; col++)
		{

			try
			{
				histIdx_G = bg.at<uchar>(row, col);
				histIdx_R = br.at<uchar>(row, col);

				q = hist_prev_roi.at<float>(histIdx_G, histIdx_R);
				p = hist_next_roi.at<float>(histIdx_G, histIdx_R);

				wi_roi.at<float>(row,col) = sqrtf( q / (p+0.000001) );
			}
		 	catch (cv::Exception e)
			{
				//cout << e.what() << endl;
				wi_roi.at<float>(row,col) = 0.f;
			}

		}
	}


	return wi_roi;
}


Mat MeanShiftTracker::create2dGaussianKernel(const int sizeX, const float sigmaX, const int sizeY, const float sigmaY)
{
	
	Mat kernelX = getGaussianKernel(sizeX, sigmaX, CV_32F); 
	Mat kernelY = getGaussianKernel(sizeY, sigmaY, CV_32F); 
	Mat kernel = kernelY * kernelX.t();

	return kernel;

}


// normalizes histogram so that it sums up to 1
cv::Mat MeanShiftTracker::getBinIndices(const Mat& src, const int binSize)
{
	Mat dst;
	dst.create(src.size(), CV_8UC1);
	dst = src / binSize;

	return dst;
}


// normalizes histogram so that it sums up to 1
void MeanShiftTracker::normalizeHistogram(Mat& hist)
{

	double s = sum(hist)[0];
	hist.convertTo(hist, hist.type(), 1./s, 0);

}


int MeanShiftTracker::run( const Mat& currentFrame, const int Nmax )
{

	if (target_initial.area() <= 1 || target_current.area() <= 1
		|| initialFrame.empty() || currentFrame.empty())
	{
		return EXIT_FAILURE;
	}


	int nbins = 16;

	Mat current_roi,
		//bb,		// will hold histogram bin indices of b plane 
		bg,		// will hold histogram bin indices of g plane
		br;		// will hold histogram bin indices of r plane

	Mat hist_roi;

	vector<Mat> bgr_planes;


	int x = target_current.x, 
		y = target_current.y,
		width = target_current.width,
		height = target_current.height;


	// initial mean locations
	int mx = x + width/2;
	int my = y + height/2;

	Point2f newTargetLoc;


	float diff = 10000.0f;
	float epsilon = 0.1f;

	int iter = 0;

	float prevDiff = -1.0f;

	int prevWidth = width, prevHeight = height;

	// repeat until the difference convergences
	while ( (diff != prevDiff || diff > epsilon) && iter < Nmax)
	{

		// obtain the current region of interest
		current_roi = currentFrame(Rect(x, y, width, height));


		// compute histogram bin indices of the current region of interest
		split(current_roi, bgr_planes);

		
		bg = getBinIndices(bgr_planes[1], nbins);

		br = getBinIndices(bgr_planes[2], nbins);


		// compute the rgb histogram of the region of interest in current frame
		computeNormalizedColorHist(current_roi, histWeights, hist_roi, nbins);
		

		// perform background weighting, section 6.1 of the paper
		computeBackgroundWeightedHistogram(currentFrame, Rect(x, y, width, height), hist_roi, nbins);


		// compute weights for each pixel location in current roi
		Mat wi_roi = computeWeights(targetModel, hist_roi, bg, br);


		// compute new mean location based on previous locations and weights.
		Point2f newTargetLoc = computeNextLocation(wi_roi, x, y);


		prevDiff = diff;

		// obtain the difference btw the previous target center and calculated target candidate's center location
		diff = sqrtf( (newTargetLoc.x-mx) * (newTargetLoc.x-mx) + 
			(newTargetLoc.y-my) * (newTargetLoc.y-my) ); 

		if (diff > prevDiff)
		{
			break;
		}

		// update mean locations
		mx = (int) newTargetLoc.x;
		my = (int) newTargetLoc.y;


		// update pixel locations and rectangle dimensions, and continue iterations if needed
		x = cvFloor( mx - width/2 );
		y = cvFloor( my - height/2 );

		// ensure that boundaries are not violated 
		if (x < 0)
			x = 0;
		if (y < 0)
			y = 0;
		if( x + width > currentFrame.size().width-1 )
			x -= x + width - (currentFrame.size().width-1);

		if( y + height > currentFrame.size().height-1 )
			y -= y + height - (currentFrame.size().height-1);


		// update current hist weights if width or height become different
		// then previous values
		if (width != prevWidth || height != prevHeight)
		{
			setHistWeights( create2dGaussianKernel(width, 1, height, 1) );

			// update prev width height values
			prevWidth = width;
			prevHeight = height;
		}

		
		// update mean location considering a possible update in
		// x, y, width or height values
		mx = x + width/2;
		my = y + height/2;

		setTarget_current(Rect(x, y, width, height));

		iter++;

	}

	return EXIT_SUCCESS;

}
