#include "meanshift.h"

RAList::RAList( void )
{
    label			= -1;
    next			= 0;	//NULL
}

RAList::~RAList( void )
{}

int RAList::Insert(RAList *entry)
{
    if(!next)
    {
        next		= entry;
        entry->next = 0;
        return 0;
    }
    if(next->label > entry->label)
    {
        entry->next	= next;
        next		= entry;
        return 0;
    }
    exists	= 0;
    cur		= next;
    while(cur)
    {
        if(entry->label == cur->label)
        {
            exists = 1;
            break;
        }
        else if((!(cur->next))||(cur->next->label > entry->label))
        {
            entry->next	= cur->next;
            cur->next	= entry;
            break;
        }
        cur = cur->next;
    }
    return (int)(exists);
}

IplImage * mean_shift_filter2(const IplImage* img, int spatial_radius, int range_radius, int iteration)
{
    double range_radius2=range_radius*range_radius;

    IplImage *result = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
    //cvCvtColor(img, result, CV_RGB2Lab);
    cvCvtColor(img, result, CV_RGB2Luv);
    //cvCopy(img, result);

    // Step One. Filtering stage of meanshift segmentation
    // http://rsbweb.nih.gov/ij/plugins/download/Mean_Shift.java
    for(int i=0;i<img->height;i++)
    {
        for(int j=0;j<img->width;j++)
        {
            int ic = i;
            int jc = j;
            int icOld, jcOld;
            float LOld, UOld, VOld;
            float L = (float)((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 0];
            float U = (float)((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 1];
            float V = (float)((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 2];
            // in the case of 8-bit and 16-bit images R, G and B are converted to floating-point format and scaled to fit 0 to 1 range
            // http://opencv.willowgarage.com/documentation/c/miscellaneous_image_transformations.html
            L = 100*L/255;
            U = 354*U/255 - 134;
            V = 256*V/255 - 140;

            double shift = 5;
            for (int iters=0;shift > 3 && iters < iteration;iters++)
            {
                icOld = ic;
                jcOld = jc;
                LOld = L;
                UOld = U;
                VOld = V;

                float mi = 0;
                float mj = 0;
                float mL = 0;
                float mU = 0;
                float mV = 0;
                int num=0;

                // deteremine i2, j2 size
                int i2from = max(0,i-spatial_radius), i2to = min(img->height, i+spatial_radius+1);
                int j2from = max(0,j-spatial_radius), j2to = min(img->width, j+spatial_radius+1);
                for (int i2=i2from; i2 < i2to;i2++) {
                    for (int j2=j2from; j2 < j2to; j2++) {
                        float L2 = (float)((uchar *)(result->imageData + i2*img->widthStep))[j2*result->nChannels + 0];
                        float U2 = (float)((uchar *)(result->imageData + i2*img->widthStep))[j2*result->nChannels + 1];
                        float V2 = (float)((uchar *)(result->imageData + i2*img->widthStep))[j2*result->nChannels + 2];

                        L2 = 100*L2/255;
                        U2 = 354*U2/255 - 134;
                        V2 = 256*V2/255 - 140;


                        double dL = L2 - L;
                        double dU = U2 - U;
                        double dV = V2 - V;
                        if (dL*dL+dU*dU+dV*dV <= range_radius2) {
                            mi += i2;
                            mj += j2;
                            mL += L2;
                            mU += U2;
                            mV += V2;
                            num++;
                        }
                    }
                }
                float num_ = 1.f/num;
                L = mL*num_;
                U = mU*num_;
                V = mV*num_;
                ic = (int) (mi*num_+0.5);
                jc = (int) (mj*num_+0.5);
                int di = ic-icOld;
                int dj = jc-jcOld;
                double dL = L-LOld;
                double dU = U-UOld;
                double dV = V-VOld;

                shift = di*di+dj*dj+dL*dL+dU*dU+dV*dV;
            }

            L = 255*L/100;
            U = 255*(U+134)/354;
            V = 255*(V+140)/256;

            ((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 0] = (uchar)L;
            ((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 1] = (uchar)U;
            ((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 2] = (uchar)V;
        }
    }

    IplImage *tobeshow = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
    //cvCvtColor(result, tobeshow, CV_Lab2RGB);
    cvCvtColor(result, tobeshow, CV_Luv2RGB);
    //cvCopy(result, tobeshow);

    return tobeshow;
}

IplImage * mean_shift_filter(const IplImage* img, int spatial_radius, int range_radius, int iteration)
{
    double range_radius2=range_radius*range_radius;

    IplImage *result = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
    cvCvtColor(img, result, CV_RGB2Lab);

    // Step One. Filtering stage of meanshift segmentation
    // http://rsbweb.nih.gov/ij/plugins/download/Mean_Shift.java
    for(int i=0;i<img->height;i++)
    {
        for(int j=0;j<img->width;j++)
        {
            int ic = i;
            int jc = j;
            int icOld, jcOld;
            float LOld, UOld, VOld;
            float L = (float)((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 0];
            float U = (float)((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 1];
            float V = (float)((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 2];
            // in the case of 8-bit and 16-bit images R, G and B are converted to floating-point format and scaled to fit 0 to 1 range
            // http://opencv.willowgarage.com/documentation/c/miscellaneous_image_transformations.html
            L = L*100/255;
            U = U-128;
            V = V-128;
            double shift = 5;
            for (int iters=0;shift > 3 && iters < iteration;iters++)
            {
                icOld = ic;
                jcOld = jc;
                LOld = L;
                UOld = U;
                VOld = V;

                float mi = 0;
                float mj = 0;
                float mL = 0;
                float mU = 0;
                float mV = 0;
                int num=0;

                // deteremine i2, j2 size
                int i2from = max(0,i-spatial_radius), i2to = min(img->height, i+spatial_radius+1);
                int j2from = max(0,j-spatial_radius), j2to = min(img->width, j+spatial_radius+1);
                for (int i2=i2from; i2 < i2to;i2++) {
                    for (int j2=j2from; j2 < j2to; j2++) {
                        float L2 = (float)((uchar *)(result->imageData + i2*img->widthStep))[j2*result->nChannels + 0];
                        float U2 = (float)((uchar *)(result->imageData + i2*img->widthStep))[j2*result->nChannels + 1];
                        float V2 = (float)((uchar *)(result->imageData + i2*img->widthStep))[j2*result->nChannels + 2];
                        L2 = L2*100/255;
                        U2 = U2-128;
                        V2 = V2-128;

                        double dL = L2 - L;
                        double dU = U2 - U;
                        double dV = V2 - V;
                        if (dL*dL+dU*dU+dV*dV <= range_radius2) {
                            mi += i2;
                            mj += j2;
                            mL += L2;
                            mU += U2;
                            mV += V2;
                            num++;
                        }
                    }
                }
                float num_ = 1.f/num;
                L = mL*num_;
                U = mU*num_;
                V = mV*num_;
                ic = (int) (mi*num_+0.5);
                jc = (int) (mj*num_+0.5);
                int di = ic-icOld;
                int dj = jc-jcOld;
                double dL = L-LOld;
                double dU = U-UOld;
                double dV = V-VOld;

                shift = di*di+dj*dj+dL*dL+dU*dU+dV*dV;
            }

            L = L*255/100;
            U = U+128;
            V = V+128;
            ((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 0] = (uchar)L;
            ((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 1] = (uchar)U;
            ((uchar *)(result->imageData + i*img->widthStep))[j*result->nChannels + 2] = (uchar)V;
        }
    }

    IplImage *tobeshow = cvCreateImage(cvGetSize(img),img->depth,img->nChannels);
    cvCvtColor(result, tobeshow, CV_Lab2RGB);

    return tobeshow;
}
