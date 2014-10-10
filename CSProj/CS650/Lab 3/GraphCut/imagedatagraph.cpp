#include "imagedatagraph.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"

ImageDataGraph::ImageDataGraph(const char* filename)
{
    mpFilename = const_cast<char *>(filename);

    IplImage* img = cvLoadImage(mpFilename);
    img_width = img->width;
    img_height = img->height;
    connect_num = 4;

    mpGraph = new PixelGraph(img_width*img_height, img_width*img_height*connect_num);

    for(int j=0;j<img_height;j++)
    {
        for(int i=0;i<img_width;i++)
        {
            //int node_id = g -> add_node();
            int color_r = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 0];
            int color_g = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 1];
            int color_b = (int)((uchar *)(img->imageData + j*img->widthStep))[i*img->nChannels + 2];
            //g -> add_tweights( node_id, color_val , 255-color_val );
        }
    }
}

ImageDataGraph::~ImageDataGraph()
{
    mpFilename = NULL;
    if(mpGraph)
    {
        delete mpGraph;
        mpGraph = NULL;
    }
}
