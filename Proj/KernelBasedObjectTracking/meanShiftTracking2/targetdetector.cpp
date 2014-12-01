#include "targetdetector.h"
#include <math.h>
#include <QDebug>

#define PI 3.1415926

using namespace std;

TargetDetector::TargetDetector()
{
    mEpsilon = 0.1;
    mDataDimension = 2;
    mColorDimension = 3;

    mBinNum = 16;
    mColorRange = 256;
    mBinWidth = cvRound(float(mColorRange)/float(mBinNum));

    mIterationMax = 5;
}

Rect TargetDetector::findTarget(const Mat & referenceImg, Rect referenceRect, const Mat & candidateImg)
{
    double new_rho = 0.0, rho = 0.0;
    bool leave = false;

    Rect candidate_rect = referenceRect;
    Mat reference_roi_hist = getROIHist(referenceImg, referenceRect);
    //Mat candidate_roi_hist = getROIHist(candidateImg, referenceRect);
    Rect new_candidate_rect = candidate_rect;


    for(int it=0;it<mIterationMax;it++)
    {
        qDebug() << "In loop @ " << it;

        Mat candidate_roi_hist = getROIHist(candidateImg, candidate_rect);

        // rho = calc Bhattacharyya Coefficient
        rho = calcBhattacharya(reference_roi_hist, candidate_roi_hist);

        // calc weight w
        Mat weights = calcWeight(reference_roi_hist, candidate_roi_hist, candidate_rect, candidateImg);

        // update candiate_rect
        new_candidate_rect = calcLocationUpdate(weights, candidate_rect);

        if(abs(new_candidate_rect.x-candidate_rect.x)<1 && abs(new_candidate_rect.y-candidate_rect.y)<1)
        {
            break;
        }
        else
        {
            candidate_rect = new_candidate_rect;
        }

        /*
        if(new_candidate_rect.x < 0)
        {
            new_candidate_rect.x = 0;
        }
        if(new_candidate_rect.y < 0)
        {
            new_candidate_rect.y = 0;
        }
        if(new_candidate_rect.x+new_candidate_rect.width>=referenceImg.cols)
        {
            new_candidate_rect.x = referenceImg.cols-new_candidate_rect.width-1;
        }
        if(new_candidate_rect.y+new_candidate_rect.height>=referenceImg.rows)
        {
            new_candidate_rect.y = referenceImg.rows-new_candidate_rect.height-1;
        }
        */

        qDebug() << "Candiate " << candidate_rect.x << " , " << candidate_rect.y << " new Candidate " << new_candidate_rect.x << " , " << new_candidate_rect.y;

        // get new rho
        new_rho = calcBhattacharya(reference_roi_hist, candidate_roi_hist);
        qDebug() << "RHO " << rho << " , new RHO " << new_rho;
        rho = new_rho;

    }

    return candidate_rect;
}

Rect TargetDetector::calcLocationUpdate(const Mat& weights, Rect candidateRect)
{
    Rect new_candidate_rect = candidateRect;
    float delta_x = 0.0, delta_y = 0.0;
    float center_i = static_cast<float>(weights.rows/2.0);
    float center_j = static_cast<float>(weights.cols/2.0);
    float num_x = 0.0, num_y = 0.0;
    float norm_i = 0.0, norm_j = 0.0;
    double mult = 0.0;
    float den = 0.0;
    for(int i=0;i<weights.rows;i++)
    {
        for(int j=0;j<weights.cols;j++)
        {
            norm_i = static_cast<float>(i-center_i)/center_i;
            norm_j = static_cast<float>(j-center_j)/center_j;
            mult = pow(norm_i,2)+pow(norm_j,2)>1.0?0.0:1.0;
            num_x += static_cast<float>(norm_j*weights.at<float>(i,j)*mult);
            num_y += static_cast<float>(norm_i*weights.at<float>(i,j)*mult);
            den += static_cast<float>(weights.at<float>(i,j)*mult);
        }
    }

    delta_x = (num_x/den)*center_j;
    delta_y = (num_y/den)*center_i;

    qDebug()  << "Location update X:" << delta_x << " Y:" << delta_y;
    new_candidate_rect.x += delta_x;
    new_candidate_rect.y += delta_y;

    return new_candidate_rect;
}

Mat TargetDetector::getROIHist(const Mat & inputImg, Rect box)
{
    int num_of_rows = box.width;
    int num_of_cols = box.height;
    Mat kernel(num_of_rows,num_of_cols,CV_32F,cv::Scalar(0));

    float parameter_cd = 0.1*PI*num_of_rows*num_of_cols;
    float kernel_sum = 0.0;

    float center_i = static_cast<float>(num_of_cols/2.0);
    float center_j = static_cast<float>(num_of_rows/2.0);

    for(int i=0;i<num_of_rows;i++)
    {
        for(int j=0;j<num_of_cols;j++)
        {
            std::vector<float> pos(mDataDimension);
            pos[0] = fabs((float)i - center_i);
            pos[1] = fabs((float)j - center_j);

            float val = static_cast<float>(parameter_cd*(1.0-((pos[0]*pos[0]+pos[1]*pos[1])/(center_i*center_j))));
            kernel.at<float>(i,j) = val<0?0:val;
            kernel_sum += kernel.at<float>(i,j);
        }
    }


    Mat roi_hist(mColorDimension,mBinNum,CV_32F,cv::Scalar(1e-10));
    int i_img = box.y;
    for(int i=0;i<kernel.rows;i++)
    {
        int j_img = box.x;
        for(int j=0;j<kernel.cols;j++)
        {
            Vec3f curr_pixel_val = inputImg.at<Vec3b>(i_img, j_img);
            for(int k=0;k<mColorDimension;k++)
            {
                int bin_idx = static_cast<int>(curr_pixel_val[k]/mBinWidth);
                roi_hist.at<float>(k,bin_idx) += kernel.at<float>(i,j)/kernel_sum;
            }
            j_img ++;
        }
        i_img ++;
    }

    return roi_hist;
}

float TargetDetector::calcBhattacharya(const Mat & referenceROIHist, const Mat & candidateROIHist)
{
    float rho = 0.0;
    float q = 0.0, p = 0.0;
    for(int k=0;k<mColorDimension;k++)
    {
        for(int i=0;i<mBinNum;i++)
        {
            q = referenceROIHist.at<float>(k, i);
            p = candidateROIHist.at<float>(k, i);
            rho += sqrt(q*p);
        }
    }
    return rho;
}

Mat TargetDetector::calcWeight(const Mat& referenceROIHist, const Mat& candidateROIHist, Rect box, const Mat & candidateImg)
{
    int num_of_rows = box.height>box.width?box.width:box.height;
    int num_of_cols = box.height>box.width?box.width:box.height;
    Mat weight(num_of_rows,num_of_cols,CV_32F,cv::Scalar(1.0000));

    int bin_idx = 0;
    int pix_val = 0;
    float p = 0.0, q = 0.0;

    std::vector<cv::Mat> bgr_planes;
    split(candidateImg, bgr_planes);

    int i_img = box.y, j_img = box.x;
    for(int k=0;k<mColorDimension;k++)
    {
        i_img = box.y;
        for(int i=0;i<num_of_rows;i++)
        {
            j_img = box.x;
            for(int j=0;j<num_of_cols;j++)
            {
                pix_val = static_cast<int>(bgr_planes[k].at<uchar>(i_img, j_img));
                bin_idx = pix_val / mBinWidth;
                q = referenceROIHist.at<float>(k, bin_idx);
                p = candidateROIHist.at<float>(k, bin_idx);
                // weight.at<float>(i,j) *= sqrt(q/p);
                weight.at<float>(i,j) *= sqrt(q/p);
                j_img ++;
            }
            i_img ++;
        }
    }
    return weight;
}

float TargetDetector::calcEpanechnivkovKernel(std::vector<float> x, float cd)
{
    int dimension = x.size();
    float x_square = 0.0;
    for(int i=0;i<dimension;i++)
    {
        x_square += x[i]*x[i];
    }
    return 0.5*(dimension+2)*(1.0-x_square)/cd;
}
