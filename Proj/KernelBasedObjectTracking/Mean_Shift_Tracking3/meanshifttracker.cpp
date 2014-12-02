#include "meanshifttracker.h"
#include <iostream>

using namespace std;


MeanShiftTracker::MeanShiftTracker()
{
    mColorDimension = 3;
    mPixelDimension = 2;
    mBinNum = 16;
    mColorRange = 256;
    mBinWidth = cvRound(float(mColorRange)/mBinNum);
    mMaxIteration = 20;

}


Mat MeanShiftTracker::computeNormalizedColorHist(const Mat& image, const Mat& histWeights)
{
    const int histSize[] = {mBinNum, mBinNum, mBinNum};

	// make sure that the histogram has a proper size and type
    Mat hist = Mat(3, histSize, CV_32F, Scalar::all(0));
	
	// the loop below assumes that the image
	// is a 8-bit 3-channel. check it.
	CV_Assert(image.type() == CV_8UC3);

	for (int y=0; y<image.size().height; y++)
	{
		for (int x=0; x<image.size().width; x++)
		{
			hist.at<float>( 
                image.at<cv::Vec3b>(y,x)[1]/mBinWidth,
                image.at<cv::Vec3b>(y,x)[2]/mBinWidth,
                image.at<cv::Vec3b>(y,x)[3]/mBinWidth) += histWeights.at<float>(y,x);
		}
	}

	double s = sum(hist)[0];
	hist.convertTo(hist, hist.type(), 1./s, 0);

    return hist;
}

cv::Mat MeanShiftTracker::buildModel(const Mat& inputImg, cv::Rect rect)
{
    Mat imgROI = inputImg(rect);
    //Mat histWeights = create2dGaussianKernel(rect.width, rect.height, 1, 1);
    Mat histWeights = createEpanechnikovKernel(rect.width, rect.height);
    Mat ROI_hist = computeNormalizedColorHist(imgROI, histWeights);

    return ROI_hist;
}

// compute weights for the current target location according to equation 10
cv::Mat MeanShiftTracker::computeWeights(const Mat& hist_prev_roi, const Mat& hist_next_roi, const Mat& bb, const Mat& bg, const Mat& br)
{
	Mat wi_roi;
	wi_roi.create(bg.size(), CV_32F);

	float q,p;
    int histIdx_B, histIdx_G, histIdx_R;

	for (int row = 0; row < bg.size().height; row++)
	{
		for (int col = 0; col < bg.size().width; col++)
		{
			try
			{
                histIdx_B = bb.at<uchar>(row, col);
				histIdx_G = bg.at<uchar>(row, col);
				histIdx_R = br.at<uchar>(row, col);

                q = hist_prev_roi.at<float>(histIdx_B, histIdx_G, histIdx_R);
                p = hist_next_roi.at<float>(histIdx_B, histIdx_G, histIdx_R);

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


// normalizes histogram so that it sums up to 1
cv::Mat MeanShiftTracker::getBinIndices(const Mat& src, const int binSize)
{
	Mat dst;
	dst.create(src.size(), CV_8UC1);
	dst = src / binSize;

	return dst;
}

bool MeanShiftTracker::findTarget( const Mat& referenceImg, const Rect& referenceRect, const Mat& candidateImg, Rect& candidateRect)
{
    if (referenceRect.area() <= 1 || candidateRect.area() <= 1
        || referenceImg.empty() || candidateImg.empty())
    {
        return false;
    }
    mTargetModel = buildModel(referenceImg, referenceRect);
    findTarget(candidateImg, candidateRect);
    return true;
}

bool MeanShiftTracker::findTarget(const Mat& candidateImg, Rect& candidateRect)
{
    if (candidateRect.area() <= 1 || candidateImg.empty())
	{
        return false;
	}

    Mat current_roi, hist_roi, bb, bg, br;
	vector<Mat> bgr_planes;

    int x = candidateRect.x,
        y = candidateRect.y,
        width = candidateRect.width,
        height = candidateRect.height;

	// initial mean locations
	int mx = x + width/2;
	int my = y + height/2;

	float diff = 10000.0f;
	float epsilon = 0.1f;
	int iter = 0;
	float prevDiff = -1.0f;

	// repeat until the difference convergences
    while ( (diff != prevDiff || diff > epsilon) && iter < mMaxIteration)
	{
		// obtain the current region of interest
        current_roi = candidateImg(Rect(x, y, width, height));

		// compute histogram bin indices of the current region of interest
		split(current_roi, bgr_planes);
		
        bb = getBinIndices(bgr_planes[0], mBinNum);
        bg = getBinIndices(bgr_planes[1], mBinNum);
        br = getBinIndices(bgr_planes[2], mBinNum);

		// compute the rgb histogram of the region of interest in current frame
        //hist_roi = computeNormalizedColorHist(current_roi, mHistWeights);
        hist_roi = buildModel(candidateImg, candidateRect);
		
		// compute weights for each pixel location in current roi
        Mat wi_roi = computeWeights(mTargetModel, hist_roi, bb, bg, br);

		// compute new mean location based on previous locations and weights.
		Point2f newTargetLoc = computeNextLocation(wi_roi, x, y);

		prevDiff = diff;

		// obtain the difference btw the previous target center and calculated target candidate's center location
        diff = sqrtf( (newTargetLoc.x-mx) * (newTargetLoc.x-mx) + (newTargetLoc.y-my) * (newTargetLoc.y-my) );

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

		// ensure that boundaries are ok 
		if (x < 0)
			x = 0;
		if (y < 0)
			y = 0;
        if( x + width > candidateImg.size().width-1 )
            x -= x + width - (candidateImg.size().width-1);
        if( y + height > candidateImg.size().height-1 )
            y -= y + height - (candidateImg.size().height-1);

		// update mean location considering a possible update in
		// x, y, width or height values
		mx = x + width/2;
		my = y + height/2;

		iter++;

        candidateRect.x = x;
        candidateRect.y = y;
        candidateRect.width = width;
        candidateRect.height = height;
	}

    return true;
}

Mat MeanShiftTracker::create2dGaussianKernel(const int sizeX, const int sizeY, const float sigmaX=1.0, const float sigmaY=1.0)
{
	Mat kernelX = getGaussianKernel(sizeX, sigmaX, CV_32F); 
	Mat kernelY = getGaussianKernel(sizeY, sigmaY, CV_32F); 
	Mat kernel = kernelY * kernelX.t();

	return kernel;
}

Mat MeanShiftTracker::createEpanechnikovKernel(const int sizeX, const int sizeY)
{
    const int kernel_size[] = {sizeX, sizeY};
    Mat kernel(2, kernel_size, CV_32F, Scalar::all(0));
    double dist = 0.0;
    for(int i=0;i<sizeX;i++)
    {
        for(int j=0;j<sizeY;j++)
        {
            dist = (i-sizeX/2)*(i-sizeX/2)+(j-sizeY/2)*(j-sizeY/2);
            kernel.at<float>(i,j) = 3*(1-dist)/4;
        }
    }
    return kernel;
}



