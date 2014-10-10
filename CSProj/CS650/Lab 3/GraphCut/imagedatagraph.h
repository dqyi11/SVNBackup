#ifndef PIXELGRAPH_H
#define PIXELGRAPH_H

#include "maxflow/graph.h"

typedef Graph<int,int,int> PixelGraph;

class ImageDataGraph
{
public:
    ImageDataGraph(const char* filename);
    ~ImageDataGraph();

private:
    char * mpFilename;
    PixelGraph * mpGraph;

    int img_width;
    int img_height;
    int connect_num;

};

#endif // PIXELGRAPH_H
