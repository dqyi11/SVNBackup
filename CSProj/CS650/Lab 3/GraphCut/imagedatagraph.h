#ifndef PIXELGRAPH_H
#define PIXELGRAPH_H

#include "maxflow/graph.h"
#include "segmentation.h"

typedef Graph<int,int,int> PixelGraph;

class ImageDataGraph
{
public:
    ImageDataGraph(const char* filename, float sigma_nb=2.0);
    ~ImageDataGraph();

    float getNeighborhoodWeight(PixelPosition p, PixelPosition q);

private:
    char * mpFilename;
    PixelGraph * mpGraph;

    int * mpRVals;
    int * mpGVals;
    int * mpBVals;

    int img_width;
    int img_height;
    int connect_num;

    float mSigmaNeighborhood;

};

#endif // PIXELGRAPH_H
