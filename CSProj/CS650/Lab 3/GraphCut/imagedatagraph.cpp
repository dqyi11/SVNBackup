#include "imagedatagraph.h"

ImageDataGraph::ImageDataGraph(const char* filename)
{
    mpFilename = const_cast<char *>(filename);
    mpGraph = new PixelGraph();
}

ImageDataGraph::~ImageDataGraph()
{
}
