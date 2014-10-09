#include <QCoreApplication>
#include "maxflow/graph.h"
#include <QtDebug>
#include <list>

/*
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    return a.exec();
}*/

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
typedef Graph<int,int,int> GraphType;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        qDebug() << "No file";
    }

    QString filename = QString::fromStdString(argv[1]);

    qDebug() << filename;

    //return a.exec();

    IplImage* im_gray = cvLoadImage(filename.toStdString().c_str(),CV_LOAD_IMAGE_GRAYSCALE);
    int im_width = im_gray->width;
    int im_height = im_gray->height;
    qDebug() << "Width " << im_width;
    qDebug() << "Height " << im_height;


    GraphType *g = new GraphType(im_height*im_width, 4*im_height*im_width);

    for(int j=0;j<im_height;j++)
    {
        for(int i=0;i<im_width;i++)
        {
            int node_id = g -> add_node();
            int color_val = (int)((uchar *)(im_gray->imageData + j*im_gray->widthStep))[i*im_gray->nChannels + 0];
            //qDebug() << i << " , " << j << " : " << node_id << " - " << color_val;
            g -> add_tweights( node_id, color_val , 255-color_val );
        }
    }

    //qDebug() << "Hello!";

    for(int j=0;j<im_height;j++)
    {
        for(int i=0;i<im_width;i++)
        {
            int node_id = i+j*im_width;
            //qDebug() << node_id;
            std::list<int> neighbor_ids = std::list<int>();
            if(j>0 && i>0)
            {
                neighbor_ids.push_back(node_id-im_width);
                neighbor_ids.push_back(node_id-1);
                neighbor_ids.push_back(node_id-im_width-1);
            }
            else if(j>0)
            {
                neighbor_ids.push_back(node_id-im_width);
            }
            else if(i>0)
            {
                neighbor_ids.push_back(node_id-1);
            }

            if(j<im_height-1 && i<im_width-1)
            {
                neighbor_ids.push_back(node_id+im_width);
                neighbor_ids.push_back(node_id+1);
                neighbor_ids.push_back(node_id+im_width+1);
            }
            else if(j<im_height-1)
            {
                neighbor_ids.push_back(node_id+im_width);
            }
            else if(i<im_width-1)
            {
                neighbor_ids.push_back(node_id+1);
            }

            for (std::list<int>::iterator it = neighbor_ids.begin(); it != neighbor_ids.end(); it++)
            {
                g->add_edge(node_id, *it, 30, 30);
            }
        }
    }


    int flow = g -> maxflow();

    //printf("Flow = %d\n", flow);

    IplImage * segImg = cvCreateImage(cvGetSize(im_gray), im_gray->depth, im_gray->nChannels);

    for(int j=0;j<im_height;j++)
    {
        for(int i=0;i<im_width;i++)
        {
            int node_id = i+j*im_width;
            int color_val = 0;
            if (g->what_segment(node_id) == GraphType::SOURCE)
            {
                color_val = 255;
            }

            ((uchar *)(segImg->imageData + j*segImg->widthStep))[i*segImg->nChannels + 0] = color_val;

        }
    }

    cvNamedWindow("Segmentation",CV_WINDOW_AUTOSIZE);
    cvShowImage("Segmentation",segImg);
    cvWaitKey();

    cvDestroyWindow("Segmentation");
    cvReleaseImage(&segImg);
    cvReleaseImage(&im_gray);

    delete g;

    return 0;


}
