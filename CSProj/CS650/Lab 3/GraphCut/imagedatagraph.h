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

};

#endif // PIXELGRAPH_H
