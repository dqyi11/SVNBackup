#ifndef TARGETDETECTOR_H
#define TARGETDETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>
using namespace cv;


class TargetDetector
{
public:
    TargetDetector();

    Rect findTarget(const Mat & referenceImg, Rect referenceRect, const Mat & candidateImg);
    Mat getROIHist(const Mat & inputImage, Rect box);
 private:
    float calcBhattacharya(const Mat & referenceROIHist, const Mat & candidateROIHist);

    Mat calcWeight(const Mat & referenceROIHist, const Mat & targetROIHist, Rect box, const Mat & candidateImg);
    float calcEpanechnivkovKernel(std::vector<float> x, float cd);

    Rect calcLocationUpdate(const Mat& weights, Rect candidateRect);

private:
    double mEpsilon;
    int mDataDimension;
    int mColorDimension;

    int mBinNum;
    int mColorRange;
    int mBinWidth;

    int mIterationMax;

};

#endif // TARGETDETECTOR_H
