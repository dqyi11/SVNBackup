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

Mat MeanShiftTracker::calcNormalizedColorHist(const Mat& image, const Mat& histWeights)
{
    const int histSize[] = {mBinNum, mBinNum, mBinNum};

    Mat hist = Mat(3, histSize, CV_32F, Scalar::all(0));
	
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

Mat MeanShiftTracker::buildModel(const Mat& inputImg, cv::Rect rect)
{
    Mat imgROI = inputImg(rect);
    Mat histWeights = create2dGaussianKernel(rect.width, rect.height, 1, 1);
    //Mat histWeights = createEpanechnikovKernel(rect.width, rect.height);
    Mat ROI_hist = calcNormalizedColorHist(imgROI, histWeights);

    return ROI_hist;
}

Mat MeanShiftTracker::calcWeight(const Mat& prevHistROI, const Mat& nextHistROI, const Mat& bb, const Mat& bg, const Mat& br)
{
    Mat wiROI;
    wiROI.create(bg.size(), CV_32F);

	float q,p;
    int histIdxB, histIdxG, histIdxR;
	for (int row = 0; row < bg.size().height; row++)
	{
		for (int col = 0; col < bg.size().width; col++)
		{
			try
			{
                histIdxB = bb.at<uchar>(row, col);
                histIdxG = bg.at<uchar>(row, col);
                histIdxR = br.at<uchar>(row, col);

                q = prevHistROI.at<float>(histIdxB, histIdxG, histIdxR);
                p = nextHistROI.at<float>(histIdxB, histIdxG, histIdxR);
                wiROI.at<float>(row,col) = sqrtf( q / (p+0.000001) );
			}
		 	catch (cv::Exception e)
			{
                wiROI.at<float>(row,col) = 0.f;
			}
		}
	}
    return wiROI;
}


Rect MeanShiftTracker::calcLocationUpdate(const Mat& wiROI, const Rect& rect)
{
    int x = rect.x, y = rect.y;
    Mat x2D, y2D, xi, yi;
    xi.create(1, wiROI.size().width, CV_32F);
    for (int i = 0; i < wiROI.size().width; i++)
	{
		xi.at<float>(i) = (float) x+i;
	}
    yi.create(wiROI.size().height, 1, CV_32F);
    for (int j = 0; j < wiROI.size().height; j++)
	{
		yi.at<float>(j) = (float) y+j;
	}

    repeat(xi, wiROI.size().height, 1, x2D);
    repeat(yi, 1, wiROI.size().width, y2D);
	xi.release();
	yi.release();

    Mat wi_roi_X, wi_roi_Y;
    multiply(wiROI, x2D, wi_roi_X);
    multiply(wiROI, y2D, wi_roi_Y);
    double sW = sum(wiROI)[0];
    double sWX = sum(wi_roi_X)[0];
    double sWY = sum(wi_roi_Y)[0];

    int newX = (int)(sWX/sW);
    int newY = (int)(sWY/sW);
    return Rect(newX, newY, rect.width, rect.height);
}


Mat MeanShiftTracker::getBinIndices(const Mat& src, int binNum)
{
	Mat dst;
	dst.create(src.size(), CV_8UC1);
    dst = src / binNum;
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

	int mx = x + width/2;
	int my = y + height/2;

	float diff = 10000.0f;
	float epsilon = 0.1f;
	int iter = 0;
	float prevDiff = -1.0f;

    while ( (diff != prevDiff || diff > epsilon) && iter < mMaxIteration)
	{
        current_roi = candidateImg(Rect(x, y, width, height));
		split(current_roi, bgr_planes);		
        bb = getBinIndices(bgr_planes[0], mBinNum);
        bg = getBinIndices(bgr_planes[1], mBinNum);
        br = getBinIndices(bgr_planes[2], mBinNum);

        hist_roi = buildModel(candidateImg, candidateRect);
		
        Mat wi_roi = calcWeight(mTargetModel, hist_roi, bb, bg, br);

        Rect newCandidateRect = calcLocationUpdate(wi_roi, candidateRect);

		prevDiff = diff;

        diff = sqrtf( (newCandidateRect.x-mx)*(newCandidateRect.x-mx) + (newCandidateRect.y-my)*(newCandidateRect.y-my) );

		if (diff > prevDiff)
		{
			break;
		}

        mx = newCandidateRect.x;
        my = newCandidateRect.y;

		x = cvFloor( mx - width/2 );
		y = cvFloor( my - height/2 );

		if (x < 0)
			x = 0;
		if (y < 0)
			y = 0;
        if( x + width > candidateImg.size().width-1 )
            x -= x + width - (candidateImg.size().width-1);
        if( y + height > candidateImg.size().height-1 )
            y -= y + height - (candidateImg.size().height-1);

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

Mat MeanShiftTracker::create2dGaussianKernel(int sizeX, int sizeY, float sigmaX, float sigmaY)
{
	Mat kernelX = getGaussianKernel(sizeX, sigmaX, CV_32F); 
	Mat kernelY = getGaussianKernel(sizeY, sigmaY, CV_32F); 
	Mat kernel = kernelY * kernelX.t();
	return kernel;
}

Mat MeanShiftTracker::createEpanechnikovKernel(int sizeX, int sizeY)
{
    const int kernel_size[] = {sizeX, sizeY};
    float norm_x = 0.0, norm_y = 0.0;
    Mat kernel(2, kernel_size, CV_32F, Scalar::all(0));
    double dist = 0.0;
    for(int i=0;i<sizeX;i++)
    {
        for(int j=0;j<sizeY;j++)
        {
            norm_x = (i-sizeX/2)/(float)sizeX;
            norm_y = (j-sizeY/2)/(float)sizeY;
            dist = norm_x*norm_x + norm_y*norm_y;
            kernel.at<float>(i,j) = 3*(1-dist)/4;
        }
    }
    return kernel;
}



