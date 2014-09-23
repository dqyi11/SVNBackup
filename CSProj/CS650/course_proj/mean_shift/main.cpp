#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"

//#include <QtCore/QCoreApplication>
#include <QtDebug>

#include "meanshift.h"

int main(int argc, char *argv[])
{
    //QCoreApplication a(argc, argv);

    if (argc < 2)
    {
        qDebug() << "No file";
    }

    char * filename = new char[strlen(argv[1])];
    strcpy(filename, argv[1]);

    qDebug() << filename;
    
    //return a.exec();
    IplImage *img = cvLoadImage(filename);

    // Mean shift
    IplImage *img_filtered = mean_shift_filter(img, 10, 7.5);

    cvNamedWindow("Mean Shift Filter",CV_WINDOW_AUTOSIZE);
    cvShowImage("Mean Shift Filter",img_filtered);

    cvWaitKey();
    cvDestroyWindow("Mean Shift Filter");
    cvReleaseImage(&img);

}
