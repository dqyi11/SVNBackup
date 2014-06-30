/*
 * CMapGraphVisualizer.h
 *
 *  Created on: Jan 8, 2013
 *      Author: walter
 */

#ifndef CMAPGRAPHVISUALIZER_H_
#define CMAPGRAPHVISUALIZER_H_

#include "../Base/CHexaGridGraph.h"
#include "igraph.h"
#include <gvc.h>

class CVisualVertex
{
public:
	CVisualVertex();
	virtual ~CVisualVertex();

	CHexaVertex * mpVertex;
	Agnode_t    * mpVisualHandle;
};

class CVisualEdge
{
public:
	CVisualEdge();
	virtual ~CVisualEdge();

    CEdge    * mpEdge;
    Agedge_t * mpVisualHandle;
};

class CMapGraphVisualizer {
public:
	CMapGraphVisualizer(CHexaGridGraph * graph);
	virtual ~CMapGraphVisualizer();

	CVisualVertex * findVisualVertex(CHexaVertex * vertex);

	void dataPreparation();
	void draw();

private:
	CHexaGridGraph * mpGraph;
	igraph_t * mpIGraph;

	list<CVisualVertex*> mVisualVertexList;
	list<CVisualEdge*>   mVisualEdgeList;

};

#endif /* CMAPGRAPHVISUALIZER_H_ */
